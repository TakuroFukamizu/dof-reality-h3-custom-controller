//! HTTP / WebSocket API (DESIGN.md §7). The API never drives the rig by
//! itself: poses go to a latest-value watch, admin commands to the control
//! loop over a bounded queue with a reply, status comes from a watch.
//!
//! # Decisions not spelled out in DESIGN.md
//!
//! - `GET /v1/motion` always upgrades, whatever `?session=` says. Each message
//!   is checked against the current session by [`PoseIntake`], so a stale or
//!   unknown session gets `stale_session` acks. When `?session=` is given, a
//!   message naming another session is also acked `stale_session`.
//! - Issuing a session clears the pose watch: a pose of the previous session
//!   never drives after the new session exists. Watch updates are made while
//!   holding the session lock, so they are ordered with session changes. The
//!   supervisor also binds a game drive to the session current at START
//!   (`AcceptedPose::session_gen`): a new session during ACTIVE stops the rig.
//! - `/v1/stop` needs no token and is not rate limited (stop must always get
//!   through; the bounded admin queue and the 1 s timeout are its backpressure).
//!   `/v1/disarm` and `/v1/fault` need the token but are not rate limited, and
//!   `/v1/fault` latches whatever its body (unparsable text becomes the note).
//! - Token-protected routes check the token first, then the rate limit, so
//!   unauthenticated requests cannot use up the operator's budget.
//! - 503 `control_loop_unavailable`: the loop did not answer within
//!   [`ADMIN_TIMEOUT`]. A command it had not taken by then is never executed
//!   later, except Stop / Disarm / Fault ([`runs_when_abandoned`]), which still
//!   run when the loop gets to them (outcome unknown to the caller).
//! - With `server.allow_non_loopback`, peers that are not loopback may only use
//!   `GET /v1/status` and `POST /v1/stop` (403 `loopback_only` otherwise): the
//!   game input and the admin token never travel over the LAN.
//! - A known path with the wrong method is a JSON 405 `method_not_allowed`.
//! - `GET /v1/events[?n=&after=]` reads the control loop's event ring (the
//!   context before and after a fault). No token, loopback only (not in
//!   [`lan_allowed`]), and it keeps answering after the control loop died, so
//!   the events before a crash can still be read.
//! - A WS text message longer than `max_body_bytes`, a binary message or
//!   malformed JSON gets a `bad_message` ack (seq null); the connection stays
//!   open. Frames above [`ws_hard_limit`] close the connection.
//! - While the latest status snapshot lists an `axis_conflict` (two MCUs
//!   selecting the same axis, DESIGN.md §10), every pose / gain message is
//!   acked `axis_conflict` before it reaches [`PoseIntake`]: nothing is stored
//!   and freshness is not extended. `/v1/arm`, `/v1/start`, `/v1/center` get
//!   the supervisor's 409 `axis_conflict`; stop / disarm / fault are unaffected.
//! - `GET /` and `GET /index.html` serve the embedded operator test page
//!   ([`TEST_PAGE`], `static/test_page.html`, no external resources). It is
//!   not in [`lan_allowed`]: the page carries the admin token, which never
//!   travels over the LAN.
//! - Origin check ([`check_origin`]): an `Origin` header must be in
//!   `allowed_origins`, **or** be the page's own origin: exactly
//!   `http://<Host header>` where the Host's hostname is `localhost` or a
//!   loopback IP literal ([`same_origin_loopback`]). The loopback condition is
//!   what keeps DNS rebinding out (`Host: evil.example:8892` with
//!   `Origin: http://evil.example:8892` still gets 403); every other foreign
//!   Origin is refused as before, and requests without Origin are unchanged.
//! - Calibration (DESIGN.md §11): `POST /v1/calibrate` (token, rate limited;
//!   body `{"axes":[1,2,3]}` optional, empty = all) and
//!   `POST /v1/calibrate/cancel` (token, not rate limited: it only stops
//!   motion) are plain admin commands. `POST /v1/calibration/export` (token,
//!   rate limited) writes `<rig_id>.calibration-<YYYYmmdd-HHMMSS>.toml` next
//!   to the server's config file from the latest status snapshot
//!   ([`crate::calibration_export`]): 409 `no_calibration_result` without an
//!   OK result, 409 `exists` instead of overwriting, 500 `no_config_path`
//!   when the server was not started from a file, 500 `invalid_result` when
//!   the produced file does not validate (nothing is written then). None of
//!   the three is in [`lan_allowed`].

use std::net::SocketAddr;
use std::sync::atomic::{AtomicU8, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;

use axum::body::Bytes;
use axum::extract::ws::{Message, WebSocket, WebSocketUpgrade};
use axum::extract::connect_info::{ConnectInfo, MockConnectInfo};
use axum::extract::rejection::QueryRejection;
use axum::extract::{DefaultBodyLimit, Query, Request, State};
use axum::http::{header, HeaderMap, HeaderValue, Method, StatusCode};
use axum::middleware::{self, Next};
use axum::response::{Html, IntoResponse, Response};
use axum::routing::{get, post};
use axum::{Json, Router};
use serde::Deserialize;
use serde_json::{json, Value};
use tokio::sync::{mpsc, oneshot, watch};
use tokio::time::{Instant, MissedTickBehavior};
use tower_http::limit::RequestBodyLimitLayer;

use crate::calibration_export::{self, AxisExport};
use crate::config::{bounded_ms, Config};
use crate::limits::{
    self as motion_limits, Ceilings, LimitsPatch, LimitsRequest, MotionLimits, PatchError, SharedLimits, Source,
};
use crate::protocol::CalResultCode;
use crate::session::{AckMsg, AcceptedPose, ClientMsg, PoseIntake, RejectReason, PROTOCOL_VERSION};
use crate::supervisor::{AdminCmd, AdminError, AdminOk, StatusSnapshot};
use crate::telemetry::{lock_events, shared_event_log, SharedEventLog};
use crate::types::{AxisId, Counts};

/// How long an admin request may wait for the control loop (→ 503).
pub const ADMIN_TIMEOUT: Duration = Duration::from_secs(1);
/// Capacity of the admin command queue to the control loop.
pub const ADMIN_QUEUE_LEN: usize = 16;

/// Operator test page (DESIGN.md §7.4), embedded so the binary works offline.
pub const TEST_PAGE: &str = include_str!("../static/test_page.html");

pub type AdminReply = Result<AdminOk, AdminError>;
/// One admin command and where to send its result.
pub type AdminRequest = (AdminCmd, AdminReplyTx);

const CLAIM_WAITING: u8 = 0;
const CLAIM_TAKEN: u8 = 1;
const CLAIM_ABANDONED: u8 = 2;

/// Reply side of an admin request (held by the control loop). The loop calls
/// [`Self::claim`] before executing, so a request the API has already answered
/// with 503 is never executed late (see [`runs_when_abandoned`]).
#[derive(Debug)]
pub struct AdminReplyTx {
    tx: oneshot::Sender<AdminReply>,
    claim: Arc<AtomicU8>,
}

impl AdminReplyTx {
    /// Take the request for execution. `false`: the requester gave up (it got
    /// 503) and will not see the result. Exactly one of the loop's claim and
    /// the API's give-up wins.
    pub fn claim(&self) -> bool {
        self.claim
            .compare_exchange(CLAIM_WAITING, CLAIM_TAKEN, Ordering::SeqCst, Ordering::SeqCst)
            .is_ok()
    }

    /// Send the result (`Err` if the requester is gone).
    pub fn send(self, r: AdminReply) -> Result<(), AdminReply> {
        self.tx.send(r)
    }
}

/// Commands that run even after the requester gave up: they only make the rig
/// safer, so a late execution is better than none.
pub fn runs_when_abandoned(cmd: &AdminCmd) -> bool {
    matches!(cmd, AdminCmd::Stop | AdminCmd::Disarm | AdminCmd::Fault { .. })
}

/// Hard per-frame/message cap of the WS transport. Larger messages close
/// the connection; messages between `max_body_bytes` and this get `bad_message`.
pub fn ws_hard_limit(max_body_bytes: usize) -> usize {
    max_body_bytes.saturating_mul(4).max(64 * 1024)
}

/// Shared state of all handlers.
#[derive(Clone)]
pub struct AppState {
    inner: Arc<Inner>,
    events: SharedEventLog,
    /// The config file the server was started from (export target directory).
    config_path: Option<Arc<std::path::PathBuf>>,
}

struct Inner {
    cfg: Arc<Config>,
    /// Requested motion limits shared with the control loop (design
    /// 2026-11-01 §5.2). Reset to `cfg.limits` on every new session.
    limits: SharedLimits,
    session: Arc<Mutex<PoseIntake>>,
    pose_tx: Arc<watch::Sender<Option<AcceptedPose>>>,
    admin_tx: mpsc::Sender<AdminRequest>,
    status_rx: watch::Receiver<Arc<StatusSnapshot>>,
    admin_token: String,
    bucket: Mutex<TokenBucket>,
    shutdown_rx: watch::Receiver<bool>,
}

impl AppState {
    /// `shutdown_rx` turning `true` closes open WebSockets.
    pub fn new(
        cfg: Arc<Config>,
        session: Arc<Mutex<PoseIntake>>,
        pose_tx: Arc<watch::Sender<Option<AcceptedPose>>>,
        admin_tx: mpsc::Sender<AdminRequest>,
        status_rx: watch::Receiver<Arc<StatusSnapshot>>,
        admin_token: String,
        shutdown_rx: watch::Receiver<bool>,
    ) -> Self {
        let bucket = Mutex::new(TokenBucket::new(cfg.server.admin_rate_limit_per_s, Instant::now()));
        let limits = motion_limits::shared(MotionLimits::from_config(&cfg.limits));
        Self {
            inner: Arc::new(Inner { cfg, limits, session, pose_tx, admin_tx, status_rx, admin_token, bucket, shutdown_rx }),
            events: shared_event_log(Instant::now()),
            config_path: None,
        }
    }

    /// Share the requested motion limits with the control loop (the server
    /// passes the same `Arc` to both). Without it the state keeps a private
    /// copy (tests of the API alone).
    pub fn with_limits(self, limits: SharedLimits) -> Self {
        let inner = Arc::try_unwrap(self.inner).unwrap_or_else(|arc| Inner {
            cfg: arc.cfg.clone(),
            limits: arc.limits.clone(),
            session: arc.session.clone(),
            pose_tx: arc.pose_tx.clone(),
            admin_tx: arc.admin_tx.clone(),
            status_rx: arc.status_rx.clone(),
            admin_token: arc.admin_token.clone(),
            bucket: Mutex::new(TokenBucket::new(arc.cfg.server.admin_rate_limit_per_s, Instant::now())),
            shutdown_rx: arc.shutdown_rx.clone(),
        });
        Self { inner: Arc::new(Inner { limits, ..inner }), ..self }
    }

    /// The `[limits]` values (what a new session resets to).
    fn limits_defaults(&self) -> MotionLimits {
        MotionLimits::from_config(&self.inner.cfg.limits)
    }

    /// Ceilings as the control loop last published them (fallback constants
    /// before the first LIMITS from an axis).
    fn ceilings(&self) -> Ceilings {
        self.inner.status_rx.borrow().motion_limits.ceiling
    }

    /// The requested set clamped as the control loop will clamp it (same
    /// ceilings, same operator cap on the game-written fields, same stop
    /// deadline).
    fn clamped(&self) -> motion_limits::Clamped {
        let req = motion_limits::read(&self.inner.limits);
        motion_limits::clamp(
            &req.requested,
            &self.ceilings(),
            &req.operator_cap(self.limits_defaults()),
            motion_limits::stop_deadline(&self.inner.cfg.timing),
        )
    }

    /// The gain ceiling in effect for pose / gain messages.
    fn current_max_gain(&self) -> f64 {
        self.clamped().effective.max_gain
    }

    /// `GET /v1/limits` body (also the `POST` answer): the requested set and
    /// what it clamps to right now (`effective` here is the clamped request;
    /// whether the control loop has applied it yet is status
    /// `motion_limits.pending`).
    fn limits_view(&self) -> Value {
        let c = self.clamped();
        let req = motion_limits::read(&self.inner.limits);
        json!({
            "ok": true,
            "effective": c.effective,
            "requested": req.requested,
            "ceiling": self.ceilings(),
            "defaults": self.limits_defaults(),
            "operator_max": self.limits_defaults(),
            "game_fields": req.game_fields,
            "limited": c.limited,
            "reasons": c.reasons,
        })
    }

    /// Where the running config was read from: `POST /v1/calibration/export`
    /// writes next to it. Without it the export answers 500 `no_config_path`.
    pub fn with_config_path(self, path: Option<std::path::PathBuf>) -> Self {
        Self { config_path: path.map(Arc::new), ..self }
    }

    /// Serve `GET /v1/events` from this log (the control loop's). Without it
    /// the endpoint serves an empty log of its own.
    pub fn with_event_log(self, events: SharedEventLog) -> Self {
        Self { events, ..self }
    }

    fn session(&self) -> MutexGuard<'_, PoseIntake> {
        lock(&self.inner.session)
    }
}

pub(crate) fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(|e| e.into_inner())
}

/// The full API router (DESIGN.md §7).
pub fn router(state: AppState) -> Router {
    let max_body = state.inner.cfg.server.max_body_bytes;
    let limited = Router::new()
        .route("/v1/arm", post(|s: State<AppState>| admin_simple(s, AdminCmd::Arm)))
        .route("/v1/start", post(|s: State<AppState>| admin_simple(s, AdminCmd::Start)))
        .route("/v1/fault/reset", post(|s: State<AppState>| admin_simple(s, AdminCmd::FaultReset)))
        .route("/v1/center", post(|s: State<AppState>| admin_simple(s, AdminCmd::Center)))
        .route("/v1/reference", post(reference))
        .route("/v1/limits", post(set_limits))
        .route("/v1/calibrate", post(calibrate))
        .route("/v1/calibration/export", post(calibration_export))
        .route_layer(middleware::from_fn_with_state(state.clone(), rate_limit));
    // Safety direction: token, but never rate limited.
    let admin = Router::new()
        .route("/v1/disarm", post(|s: State<AppState>| admin_simple(s, AdminCmd::Disarm)))
        .route("/v1/fault", post(fault))
        .route("/v1/calibrate/cancel", post(|s: State<AppState>| admin_simple(s, AdminCmd::CalibrateCancel)))
        .merge(limited)
        // Outermost: auth before the rate limit.
        .route_layer(middleware::from_fn_with_state(state.clone(), require_token));
    Router::new()
        .route("/", get(test_page))
        .route("/index.html", get(test_page))
        .route("/v1/session", post(new_session))
        .route("/v1/motion", get(motion))
        .route("/v1/status", get(status))
        .route("/v1/limits", get(get_limits))
        .route("/v1/events", get(events))
        .route("/v1/stop", post(|s: State<AppState>| admin_simple(s, AdminCmd::Stop)))
        .merge(admin)
        .method_not_allowed_fallback(|| async { error(StatusCode::METHOD_NOT_ALLOWED, "method_not_allowed") })
        .fallback(|| async { error(StatusCode::NOT_FOUND, "not_found") })
        .layer(DefaultBodyLimit::max(max_body))
        .layer(RequestBodyLimitLayer::new(max_body))
        .layer(middleware::from_fn_with_state(state.clone(), check_origin))
        .layer(middleware::from_fn_with_state(state.clone(), check_peer))
        .with_state(state)
}

fn error(code: StatusCode, err: &str) -> Response {
    (code, Json(json!({"ok": false, "error": err}))).into_response()
}

// ------------------------------------------------------------------ guards

/// Routes a non-loopback peer may use when `allow_non_loopback` is set.
fn lan_allowed(method: &Method, path: &str) -> bool {
    (method == Method::GET && path == "/v1/status") || (method == Method::POST && path == "/v1/stop")
}

fn peer_addr(req: &Request) -> Option<SocketAddr> {
    let ext = req.extensions();
    ext.get::<ConnectInfo<SocketAddr>>()
        .map(|c| c.0)
        .or_else(|| ext.get::<MockConnectInfo<SocketAddr>>().map(|m| m.0))
}

async fn check_peer(State(st): State<AppState>, req: Request, next: Next) -> Response {
    if st.inner.cfg.server.allow_non_loopback {
        // Unknown peer counts as non-loopback (fail closed).
        let loopback = peer_addr(&req).is_some_and(|a| a.ip().to_canonical().is_loopback());
        if !loopback && !lan_allowed(req.method(), req.uri().path()) {
            return error(StatusCode::FORBIDDEN, "loopback_only");
        }
    }
    next.run(req).await
}

/// `Origin` is the test page's own origin on a loopback host: exactly
/// `http://<host>` where `host` is the request's Host (with port) and its
/// hostname is `localhost` or a loopback IP literal (`127.0.0.1`, `[::1]`,
/// `127.x.y.z`). Compared case-insensitively (hostnames are), no trailing
/// slash (browsers never send one). A Host naming any other hostname never
/// matches, so a rebinding DNS name pointing at 127.0.0.1 gains nothing.
pub fn same_origin_loopback(origin: &str, host: Option<&str>) -> bool {
    let Some(host) = host.map(str::trim).filter(|h| !h.is_empty()) else {
        return false;
    };
    let hostname = if let Some(rest) = host.strip_prefix('[') {
        // `[::1]:8892` → `::1`
        match rest.split_once(']') {
            Some((v6, _)) => v6,
            None => return false,
        }
    } else {
        let h = host.rsplit_once(':').map_or(host, |(h, _)| h);
        if h.contains(':') {
            return false; // unbracketed IPv6 is not a valid Host
        }
        h
    };
    let loopback = hostname.eq_ignore_ascii_case("localhost")
        || hostname.parse::<std::net::IpAddr>().is_ok_and(|ip| ip.to_canonical().is_loopback());
    if !loopback {
        return false;
    }
    let Some(rest) = origin.strip_prefix("http://").or_else(|| origin.strip_prefix("HTTP://")) else {
        return false;
    };
    rest.eq_ignore_ascii_case(host)
}

fn request_host(req: &Request) -> Option<&str> {
    req.headers()
        .get(header::HOST)
        .and_then(|v| v.to_str().ok())
        .or_else(|| req.uri().authority().map(|a| a.as_str()))
}

async fn check_origin(State(st): State<AppState>, req: Request, next: Next) -> Response {
    if let Some(origin) = req.headers().get(header::ORIGIN) {
        let allowed = origin.to_str().is_ok_and(|o| {
            st.inner.cfg.server.allowed_origins.iter().any(|a| a == o) || same_origin_loopback(o, request_host(&req))
        });
        if !allowed {
            return error(StatusCode::FORBIDDEN, "forbidden_origin");
        }
    }
    let resp = next.run(req).await;
    // RequestBodyLimitLayer answers 413 with a plain-text body; keep JSON.
    if resp.status() == StatusCode::PAYLOAD_TOO_LARGE {
        return error(StatusCode::PAYLOAD_TOO_LARGE, "body_too_large");
    }
    resp
}

async fn require_token(State(st): State<AppState>, req: Request, next: Next) -> Response {
    // Empty token = `server.require_admin_token = false` (loopback-only setup).
    if st.inner.admin_token.is_empty() || bearer_ok(req.headers(), &st.inner.admin_token) {
        return next.run(req).await;
    }
    let mut resp = error(StatusCode::UNAUTHORIZED, "unauthorized");
    resp.headers_mut().insert(header::WWW_AUTHENTICATE, HeaderValue::from_static("Bearer"));
    resp
}

fn bearer_ok(headers: &HeaderMap, token: &str) -> bool {
    let Some(given) = headers
        .get(header::AUTHORIZATION)
        .and_then(|v| v.to_str().ok())
        .and_then(|v| v.strip_prefix("Bearer "))
    else {
        return false;
    };
    !token.is_empty() && constant_time_eq(given.trim().as_bytes(), token.as_bytes())
}

/// Compare without an early exit on the first differing byte (length leaks).
pub fn constant_time_eq(a: &[u8], b: &[u8]) -> bool {
    if a.len() != b.len() {
        return false;
    }
    let diff = a.iter().zip(b).fold(0u8, |acc, (x, y)| acc | (x ^ y));
    std::hint::black_box(diff) == 0
}

async fn rate_limit(State(st): State<AppState>, req: Request, next: Next) -> Response {
    if !lock(&st.inner.bucket).take(Instant::now()) {
        return error(StatusCode::TOO_MANY_REQUESTS, "rate_limited");
    }
    next.run(req).await
}

/// Token bucket: `rate` tokens per second, burst `rate`. Rate 0 disables.
#[derive(Debug, Clone)]
pub struct TokenBucket {
    rate: f64,
    tokens: f64,
    last: Instant,
}

impl TokenBucket {
    pub fn new(rate_per_s: u32, now: Instant) -> Self {
        let rate = f64::from(rate_per_s);
        Self { rate, tokens: rate, last: now }
    }

    /// Take one token; false when exhausted.
    pub fn take(&mut self, now: Instant) -> bool {
        if self.rate <= 0.0 {
            return true;
        }
        let dt = now.saturating_duration_since(self.last).as_secs_f64();
        self.last = now;
        self.tokens = (self.tokens + dt * self.rate).min(self.rate);
        if self.tokens >= 1.0 {
            self.tokens -= 1.0;
            true
        } else {
            false
        }
    }
}

// ------------------------------------------------------------------ test page

async fn test_page() -> Response {
    ([(header::CACHE_CONTROL, HeaderValue::from_static("no-store"))], Html(TEST_PAGE)).into_response()
}

// ------------------------------------------------------------------ admin

async fn send_admin(st: &AppState, cmd: AdminCmd) -> Response {
    let (tx, mut rx) = oneshot::channel();
    let claim = Arc::new(AtomicU8::new(CLAIM_WAITING));
    let reply = AdminReplyTx { tx, claim: claim.clone() };
    let fut = async {
        st.inner.admin_tx.send((cmd, reply)).await.ok()?;
        (&mut rx).await.ok()
    };
    let answer = match tokio::time::timeout(ADMIN_TIMEOUT, fut).await {
        Ok(a) => a,
        Err(_) => {
            let gave_up = claim
                .compare_exchange(CLAIM_WAITING, CLAIM_ABANDONED, Ordering::SeqCst, Ordering::SeqCst)
                .is_ok();
            if gave_up {
                None
            } else {
                // The loop took it just now and is executing it: wait for the answer.
                tokio::time::timeout(ADMIN_TIMEOUT, rx).await.ok().and_then(Result::ok)
            }
        }
    };
    match answer {
        Some(Ok(ok)) => {
            (StatusCode::OK, Json(json!({"ok": true, "rig_state": ok.rig_state, "note": ok.note}))).into_response()
        }
        Some(Err(e)) => (
            StatusCode::CONFLICT,
            Json(json!({"ok": false, "error": e.error, "blockers": e.blockers})),
        )
            .into_response(),
        _ => error(StatusCode::SERVICE_UNAVAILABLE, "control_loop_unavailable"),
    }
}

async fn admin_simple(State(st): State<AppState>, cmd: AdminCmd) -> Response {
    send_admin(&st, cmd).await
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct ReferenceBody {
    axis: u8,
    count: Counts,
}

async fn reference(State(st): State<AppState>, body: Bytes) -> Response {
    let Ok(b) = serde_json::from_slice::<ReferenceBody>(&body) else {
        return error(StatusCode::BAD_REQUEST, "bad_request");
    };
    let Some(axis) = AxisId::new(b.axis) else {
        return error(StatusCode::BAD_REQUEST, "bad_axis");
    };
    send_admin(&st, AdminCmd::Reference { axis, count: b.count }).await
}

/// Longest note kept from an unparsable /v1/fault body (characters).
const FAULT_NOTE_MAX_CHARS: usize = 200;

#[derive(Deserialize)]
struct FaultBody {
    #[serde(default)]
    note: String,
}

/// Note for an operator fault. Never fails: a fault must not be lost to a
/// body typo, so anything but `{"note": ...}` is kept as (truncated) text.
pub fn fault_note(body: &[u8]) -> String {
    if body.iter().all(u8::is_ascii_whitespace) {
        return String::new();
    }
    match serde_json::from_slice::<FaultBody>(body) {
        Ok(b) => b.note,
        Err(_) => {
            let text: String = String::from_utf8_lossy(body).chars().take(FAULT_NOTE_MAX_CHARS).collect();
            format!("unparsed body: {text}")
        }
    }
}

async fn fault(State(st): State<AppState>, body: Bytes) -> Response {
    send_admin(&st, AdminCmd::Fault { note: fault_note(&body) }).await
}

// ------------------------------------------------------------------ calibration

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct CalibrateBody {
    #[serde(default)]
    axes: Vec<u8>,
}

/// `POST /v1/calibrate` with an optional `{"axes":[…]}` body (empty = all axes).
async fn calibrate(State(st): State<AppState>, body: Bytes) -> Response {
    let axes = if body.iter().all(u8::is_ascii_whitespace) {
        Vec::new()
    } else {
        let Ok(b) = serde_json::from_slice::<CalibrateBody>(&body) else {
            return error(StatusCode::BAD_REQUEST, "bad_request");
        };
        let mut axes = Vec::with_capacity(b.axes.len());
        for a in b.axes {
            let Some(id) = AxisId::new(a) else {
                return error(StatusCode::BAD_REQUEST, "bad_axis");
            };
            axes.push(id);
        }
        axes
    };
    send_admin(&st, AdminCmd::Calibrate { axes }).await
}

fn error_detail(code: StatusCode, err: &str, detail: String) -> Response {
    (code, Json(json!({"ok": false, "error": err, "detail": detail}))).into_response()
}

/// `POST /v1/calibration/export`: see the module docs and
/// [`crate::calibration_export`].
async fn calibration_export(State(st): State<AppState>) -> Response {
    if !loop_alive(&st) {
        return error(StatusCode::SERVICE_UNAVAILABLE, "control_loop_unavailable");
    }
    let Some(path) = st.config_path.as_ref().map(|p| p.to_path_buf()) else {
        return error(StatusCode::INTERNAL_SERVER_ERROR, "no_config_path");
    };
    let cfg = st.inner.cfg.clone();
    let snap = st.inner.status_rx.borrow().clone();
    let mut axes: Vec<AxisExport> = Vec::new();
    for a in &snap.axes {
        let Some(r) = a.calibration.as_ref().filter(|r| r.result == CalResultCode::Ok) else { continue };
        let Some(ac) = cfg.axis(a.axis_id) else { continue };
        match calibration_export::derive(a.axis_id, r.device_id, ac.decel_zone_count, r) {
            Ok(e) => axes.push(e),
            Err(e) => return error_detail(StatusCode::INTERNAL_SERVER_ERROR, "invalid_result", e),
        }
    }
    if axes.is_empty() {
        return (
            StatusCode::CONFLICT,
            Json(json!({"ok": false, "error": "no_calibration_result", "blockers": ["no_calibration_result: no axis has an OK CAL_RESULT in this boot"]})),
        )
            .into_response();
    }
    // File I/O off the async executor.
    let rig_id = cfg.rig_id.clone();
    let result = tokio::task::spawn_blocking(move || write_export(&path, &rig_id, &axes)).await;
    match result {
        Ok(Ok((written, axes))) => {
            tracing::info!(path = %written.display(), "calibration exported");
            Json(json!({"ok": true, "path": written, "axes": axes})).into_response()
        }
        Ok(Err((code, err, detail))) => error_detail(code, err, detail),
        Err(e) => error_detail(StatusCode::INTERNAL_SERVER_ERROR, "export_failed", e.to_string()),
    }
}

type ExportError = (StatusCode, &'static str, String);

fn write_export(
    config_path: &std::path::Path,
    rig_id: &str,
    axes: &[AxisExport],
) -> Result<(std::path::PathBuf, Vec<AxisExport>), ExportError> {
    let text = std::fs::read_to_string(config_path)
        .map_err(|e| (StatusCode::INTERNAL_SERVER_ERROR, "config_read_failed", format!("{}: {e}", config_path.display())))?;
    let rewrite = calibration_export::rewrite_toml(&text, axes)
        .map_err(|e| (StatusCode::INTERNAL_SERVER_ERROR, "rewrite_failed", e))?;
    let ts = calibration_export::utc_timestamp(std::time::SystemTime::now());
    let head = calibration_export::header(&ts, &config_path.display().to_string(), axes, &rewrite.replaced);
    let full = format!("{head}{}", rewrite.text);
    Config::from_toml_str(&full).map_err(|e| (StatusCode::INTERNAL_SERVER_ERROR, "invalid_result", e.to_string()))?;
    let dir = config_path.parent().filter(|d| !d.as_os_str().is_empty()).unwrap_or(std::path::Path::new("."));
    let out = dir.join(calibration_export::file_name(rig_id, &ts));
    let mut f = match std::fs::OpenOptions::new().write(true).create_new(true).open(&out) {
        Ok(f) => f,
        Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => {
            return Err((StatusCode::CONFLICT, "exists", out.display().to_string()))
        }
        Err(e) => return Err((StatusCode::INTERNAL_SERVER_ERROR, "write_failed", format!("{}: {e}", out.display()))),
    };
    use std::io::Write as _;
    f.write_all(full.as_bytes())
        .and_then(|()| f.sync_all())
        .map_err(|e| (StatusCode::INTERNAL_SERVER_ERROR, "write_failed", format!("{}: {e}", out.display())))?;
    Ok((out, axes.to_vec()))
}

// ------------------------------------------------------------------ game

async fn new_session(State(st): State<AppState>) -> Response {
    let cfg = &st.inner.cfg;
    let defaults = st.limits_defaults();
    let id = {
        let mut s = st.session();
        let id = s.new_session();
        // Under the lock: no pose of the old session can land after this.
        st.inner.pose_tx.send_replace(None);
        // A new session starts from the configured limits (design §5.2),
        // with no field under the operator cap yet (nothing game-written).
        motion_limits::write(&st.inner.limits, LimitsRequest::new(defaults));
        id
    };
    tracing::info!("new game session issued");
    Json(json!({
        "source_session": id,
        "protocol": PROTOCOL_VERSION,
        "max_gain": st.current_max_gain(),
        "source_freshness_ms": cfg.timing.source_freshness_ms,
        "require_frame_seq": !cfg.game.allow_missing_frame_seq,
    }))
    .into_response()
}

/// `GET /v1/limits`: the shared motion limits (design 2026-11-01 §5.3).
async fn get_limits(State(st): State<AppState>) -> Response {
    if !loop_alive(&st) {
        return error(StatusCode::SERVICE_UNAVAILABLE, "control_loop_unavailable");
    }
    Json(st.limits_view()).into_response()
}

/// `POST /v1/limits` (admin): apply a [`LimitsPatch`] to the requested set.
/// Values above the firmware ceilings are clamped (reported in `limited`),
/// invalid ones are 400. Unlike the game channel, the admin may raise a
/// field above the operator's `[limits]` value (and releases it from the
/// operator cap).
async fn set_limits(State(st): State<AppState>, body: Bytes) -> Response {
    if !loop_alive(&st) {
        return error(StatusCode::SERVICE_UNAVAILABLE, "control_loop_unavailable");
    }
    let Ok(patch) = serde_json::from_slice::<LimitsPatch>(&body) else {
        return error(StatusCode::BAD_REQUEST, "bad_request");
    };
    match apply_limits_patch(&st, &patch, Source::Admin) {
        Ok(()) => {
            // Same as the WS path: a lowered gain ceiling reaches the tracked pose now.
            let max_gain = st.current_max_gain();
            let mut s = st.session();
            if let Some(p) = s.reapply_gain_ceiling(&st.inner.cfg.game, max_gain) {
                st.inner.pose_tx.send_replace(Some(p));
            }
            drop(s);
            Json(st.limits_view()).into_response()
        }
        Err(PatchError::NonFinite) => error(StatusCode::BAD_REQUEST, "non_finite"),
        Err(PatchError::OutOfRange) => error(StatusCode::BAD_REQUEST, "out_of_range"),
    }
}

/// Replace the present fields of the shared requested set, recording who
/// wrote them (`Source::Game` → held to the operator's values by the clamp).
fn apply_limits_patch(st: &AppState, patch: &LimitsPatch, source: Source) -> Result<(), PatchError> {
    let mut guard = st.inner.limits.lock().unwrap_or_else(|e| e.into_inner());
    guard.apply(patch, source)
}

/// False once the control loop has ended (its status sender is gone).
fn loop_alive(st: &AppState) -> bool {
    st.inner.status_rx.has_changed().is_ok()
}

async fn status(State(st): State<AppState>) -> Response {
    // A dead loop must not keep reporting its last state (e.g. ACTIVE).
    if !loop_alive(&st) {
        return error(StatusCode::SERVICE_UNAVAILABLE, "control_loop_unavailable");
    }
    let mut v = serde_json::to_value(&*st.inner.status_rx.borrow().clone()).unwrap_or(Value::Null);
    if let Value::Object(m) = &mut v {
        m.insert("admin_token_required".into(), Value::from(!st.inner.admin_token.is_empty()));
    }
    Json(v).into_response()
}

/// Default and maximum `n` of `GET /v1/events`.
pub const EVENTS_DEFAULT_N: usize = 100;

#[derive(Deserialize)]
struct EventsQuery {
    n: Option<usize>,
    after: Option<u64>,
}

/// `GET /v1/events?n=&after=`: the newest `n` events (default
/// [`EVENTS_DEFAULT_N`], at most the ring size) with `id > after`, oldest first.
async fn events(State(st): State<AppState>, q: Result<Query<EventsQuery>, QueryRejection>) -> Response {
    let Ok(Query(q)) = q else {
        return error(StatusCode::BAD_REQUEST, "bad_request");
    };
    let log = lock_events(&st.events);
    let n = q.n.unwrap_or(EVENTS_DEFAULT_N).min(log.capacity());
    let body = json!({
        "ok": true,
        "now_ms": crate::telemetry::ms_since(log.start(), Instant::now()),
        "next_id": log.next_id(),
        "capacity": log.capacity(),
        "dropped": log.dropped(),
        "events": log.recent_after(q.after.unwrap_or(0), n),
    });
    drop(log);
    Json(body).into_response()
}

#[derive(Deserialize)]
struct MotionQuery {
    session: Option<String>,
}

async fn motion(State(st): State<AppState>, q: Query<MotionQuery>, ws: WebSocketUpgrade) -> Response {
    let limit = ws_hard_limit(st.inner.cfg.server.max_body_bytes);
    let pinned = q.0.session;
    ws.max_message_size(limit)
        .max_frame_size(limit)
        .on_upgrade(move |socket| motion_ws(st, socket, pinned))
}

async fn motion_ws(st: AppState, mut socket: WebSocket, pinned: Option<String>) {
    let period = bounded_ms(st.inner.cfg.server.status_period_ms.max(1));
    let mut status_tick = tokio::time::interval(period);
    status_tick.set_missed_tick_behavior(MissedTickBehavior::Skip);
    let mut shutdown = st.inner.shutdown_rx.clone();
    if *shutdown.borrow() {
        return;
    }
    loop {
        let out = tokio::select! {
            m = socket.recv() => match m {
                Some(Ok(Message::Text(t))) => ack_json(&handle_text(&st, t.as_str(), pinned.as_deref())),
                Some(Ok(Message::Binary(_))) => ack_json(&bad_message(&st)),
                Some(Ok(Message::Close(_))) | None | Some(Err(_)) => break,
                Some(Ok(_)) => continue, // ping/pong are answered by the transport
            },
            _ = status_tick.tick() => {
                if !loop_alive(&st) {
                    let _ = socket.send(Message::Close(None)).await;
                    break;
                }
                status_json(&st)
            }
            r = shutdown.changed() => {
                if r.is_err() || *shutdown.borrow() {
                    let _ = socket.send(Message::Close(None)).await;
                    break;
                }
                continue;
            }
        };
        if socket.send(Message::Text(out.into())).await.is_err() {
            break;
        }
    }
}

fn bad_message(st: &AppState) -> AckMsg {
    AckMsg::rejected(None, RejectReason::BadMessage, st.session().is_fresh(Instant::now()))
}

fn ack_json(a: &AckMsg) -> String {
    serde_json::to_string(a).unwrap_or_else(|_| r#"{"type":"ack","accepted":false,"reason":"bad_message"}"#.into())
}

fn status_json(st: &AppState) -> String {
    let snap = st.inner.status_rx.borrow().clone();
    let mut v = serde_json::to_value(&*snap).unwrap_or(Value::Null);
    if let Value::Object(m) = &mut v {
        m.insert("type".into(), Value::from("status"));
        m.insert("admin_token_required".into(), Value::from(!st.inner.admin_token.is_empty()));
    }
    v.to_string()
}

/// Handle one WS text message and build its ack.
fn handle_text(st: &AppState, text: &str, pinned: Option<&str>) -> AckMsg {
    let cfg = &st.inner.cfg;
    let recv_at = Instant::now();
    if text.len() > cfg.server.max_body_bytes {
        return bad_message(st);
    }
    let msg = match ClientMsg::parse(text) {
        Ok(m) => m,
        Err(r) => return AckMsg::rejected(None, r, st.session().is_fresh(recv_at)),
    };
    let seq = msg.seq();
    let mut s = st.session();
    if pinned.is_some_and(|p| p != msg.source_session()) {
        return AckMsg::rejected(Some(seq), RejectReason::StaleSession, false);
    }
    if !st.inner.status_rx.borrow().axis_conflict.is_empty() {
        // Not handed to the intake: no pose stored, no freshness extension.
        return AckMsg::rejected(Some(seq), RejectReason::AxisConflict, s.is_fresh(recv_at));
    }
    if let ClientMsg::Limits(l) = &msg {
        // Header rules, then the patch; the seq is consumed only on success.
        // Freshness is never extended by a limits message. The patch is
        // applied whole before clamping, so velocity and acceleration are
        // judged together by the stop-time rule. Written as the game: the
        // fields are held to the operator's `[limits]` values (Issue #15).
        let r = s.accept_limits(l, || {
            apply_limits_patch(st, &l.patch, Source::Game).map_err(|e| match e {
                PatchError::NonFinite => RejectReason::NonFinite,
                PatchError::OutOfRange => RejectReason::OutOfRange,
            })?;
            Ok(st.clamped())
        });
        return match r {
            Ok(c) => {
                // The new gain ceiling applies to the pose being tracked now
                // (freshness unchanged), not only to the next one.
                if let Some(p) = s.reapply_gain_ceiling(&cfg.game, c.effective.max_gain) {
                    st.inner.pose_tx.send_replace(Some(p));
                }
                AckMsg::accepted_limits(seq, c.effective, c.reasons, s.is_fresh(recv_at))
            }
            Err(r) => {
                let fresh = r != RejectReason::StaleSession && s.is_fresh(recv_at);
                AckMsg::rejected(Some(seq), r, fresh)
            }
        };
    }
    let max_gain = st.current_max_gain();
    let result = match &msg {
        ClientMsg::Pose(p) => s
            .accept_pose(p, recv_at, &cfg.game, max_gain, cfg.timing.source_freshness_ms)
            .map(|a| (a.checked.effective_gain, a.checked.limits.clone(), Some(a))),
        ClientMsg::Gain(g) => s.accept_gain(g, &cfg.game, max_gain).map(|u| (u.effective_gain, u.limits, u.pose)),
        ClientMsg::Limits(_) => unreachable!("handled above"),
    };
    match result {
        Ok((gain, limits, pose)) => {
            if let Some(p) = pose {
                // Still under the session lock (see module docs).
                st.inner.pose_tx.send_replace(Some(p));
            }
            AckMsg::accepted(seq, gain, limits, s.is_fresh(recv_at))
        }
        Err(r) => {
            let fresh = r != RejectReason::StaleSession && s.is_fresh(recv_at);
            AckMsg::rejected(Some(seq), r, fresh)
        }
    }
}
