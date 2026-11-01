//! api: guards and handlers (DESIGN.md §7). Router-level tests use
//! `tower::ServiceExt::oneshot` with a stub control loop; WebSocket tests run
//! a real server with the sim backend on 127.0.0.1:0.

mod common;

use std::sync::{Arc, Mutex};
use std::time::Duration;

use axum::body::Body;
use axum::http::{Request, StatusCode};
use axum::Router;
use common::*;
use futures_util::{SinkExt, StreamExt};
use h3cs::api::{self, AdminRequest, AppState, TokenBucket};
use h3cs::config::Config;
use h3cs::session::{AcceptedPose, PoseIntake};
use h3cs::supervisor::{AdminCmd, AdminError, AdminOk, Supervisor};
use h3cs::types::RigState;
use http_body_util::BodyExt;
use serde_json::Value;
use tokio::sync::{mpsc, watch};
use tokio::time::Instant;
use tokio_tungstenite::tungstenite::Message;
use tower::ServiceExt;

// ------------------------------------------------------------------ stub harness

struct Stub {
    app: Router,
    pose_rx: watch::Receiver<Option<AcceptedPose>>,
    seen: Arc<Mutex<Vec<AdminCmd>>>,
    _shutdown: watch::Sender<bool>,
}

/// Router with a stub control loop: `reply` decides each admin answer;
/// `None` never answers (the request times out).
type Reply = fn(&AdminCmd) -> Result<AdminOk, AdminError>;

fn stub(cfg: Config, reply: Option<Reply>) -> Stub {
    let cfg = Arc::new(cfg);
    let (admin_tx, mut admin_rx) = mpsc::channel::<AdminRequest>(api::ADMIN_QUEUE_LEN);
    let (pose_tx, pose_rx) = watch::channel(None);
    let (shutdown_tx, shutdown_rx) = watch::channel(false);
    let snap = Supervisor::new(&cfg).snapshot(Instant::now());
    let (_status_tx, status_rx) = watch::channel(Arc::new(snap));
    let seen = Arc::new(Mutex::new(Vec::new()));
    let seen2 = seen.clone();
    tokio::spawn(async move {
        let _keep = _status_tx;
        let mut held = Vec::new();
        while let Some((cmd, tx)) = admin_rx.recv().await {
            seen2.lock().unwrap().push(cmd.clone());
            match reply {
                Some(f) => {
                    let _ = tx.send(f(&cmd));
                }
                None => held.push(tx),
            }
        }
    });
    let state = AppState::new(
        cfg,
        Arc::new(Mutex::new(PoseIntake::new())),
        Arc::new(pose_tx),
        admin_tx,
        status_rx,
        TOKEN.into(),
        shutdown_rx,
    );
    Stub { app: api::router(state), pose_rx, seen, _shutdown: shutdown_tx }
}

fn ok_reply(_: &AdminCmd) -> Result<AdminOk, AdminError> {
    Ok(AdminOk { rig_state: RigState::ReadyToArm, note: None })
}

fn not_ready(_: &AdminCmd) -> Result<AdminOk, AdminError> {
    Err(AdminError { error: "not_ready".into(), blockers: vec!["not_connected: axis2".into()] })
}

fn req(method: &str, path: &str, headers: &[(&str, &str)], body: &str) -> Request<Body> {
    let mut b = Request::builder().method(method).uri(path);
    for (k, v) in headers {
        b = b.header(*k, *v);
    }
    if !body.is_empty() {
        b = b.header("content-type", "application/json").header("content-length", body.len().to_string());
    }
    b.body(Body::from(body.to_string())).unwrap()
}

async fn call(app: &Router, r: Request<Body>) -> (StatusCode, Value) {
    let resp = app.clone().oneshot(r).await.unwrap();
    let status = resp.status();
    let bytes = resp.into_body().collect().await.unwrap().to_bytes();
    (status, serde_json::from_slice(&bytes).unwrap_or(Value::Null))
}

fn bearer() -> String {
    format!("Bearer {TOKEN}")
}

// ------------------------------------------------------------------ guards

#[tokio::test]
async fn foreign_origin_is_forbidden() {
    let s = stub(sim_cfg(), Some(ok_reply));
    let (st, body) = call(&s.app, req("GET", "/v1/status", &[("origin", "http://evil.example")], "")).await;
    assert_eq!(st, StatusCode::FORBIDDEN);
    assert_eq!(body["error"], "forbidden_origin");
    // Even /v1/stop and the WS route are behind the Origin check.
    let (st, _) = call(&s.app, req("POST", "/v1/stop", &[("origin", "http://evil.example")], "")).await;
    assert_eq!(st, StatusCode::FORBIDDEN);
    let (st, _) = call(&s.app, req("GET", "/v1/motion", &[("origin", "null")], "")).await;
    assert_eq!(st, StatusCode::FORBIDDEN);
    assert!(s.seen.lock().unwrap().is_empty(), "nothing reached the control loop");

    let (st, _) = call(&s.app, req("GET", "/v1/status", &[("origin", "http://localhost:8891")], "")).await;
    assert_eq!(st, StatusCode::OK);
    let (st, _) = call(&s.app, req("GET", "/v1/status", &[], "")).await;
    assert_eq!(st, StatusCode::OK, "no Origin (non-browser client) is allowed");
}

#[tokio::test]
async fn test_page_is_served_at_root_and_index_html() {
    let s = stub(sim_cfg(), Some(ok_reply));
    for path in ["/", "/index.html"] {
        let resp = s.app.clone().oneshot(req("GET", path, &[], "")).await.unwrap();
        assert_eq!(resp.status(), StatusCode::OK, "{path}");
        let ct = resp.headers().get("content-type").unwrap().to_str().unwrap().to_string();
        assert!(ct.starts_with("text/html"), "{path}: {ct}");
        assert_eq!(resp.headers().get("cache-control").unwrap(), "no-store");
        let body = resp.into_body().collect().await.unwrap().to_bytes();
        let html = std::str::from_utf8(&body).unwrap();
        assert_eq!(html, api::TEST_PAGE);
        assert!(html.starts_with("<!DOCTYPE html>"), "{path}");
        assert!(html.contains("h3-control-server 試験ページ") && html.contains("/v1/motion"), "{path}");
    }
    // No external resources: the page must work offline.
    for needle in ["src=\"http", "href=\"http", "https://", "@import"] {
        assert!(!api::TEST_PAGE.contains(needle), "external resource in the page: {needle}");
    }
    // POST / is a known path with the wrong method.
    let (st, body) = call(&s.app, req("POST", "/", &[], "")).await;
    assert_eq!(st, StatusCode::METHOD_NOT_ALLOWED);
    assert_eq!(body["error"], "method_not_allowed");
    assert!(s.seen.lock().unwrap().is_empty());
}

#[tokio::test]
async fn same_origin_loopback_requests_pass_the_origin_check() {
    let s = stub(sim_cfg(), Some(ok_reply));
    // The page's own origin (browser: Origin = scheme://Host) on a loopback host.
    for (host, origin) in [
        ("127.0.0.1:8892", "http://127.0.0.1:8892"),
        ("localhost:8892", "http://localhost:8892"),
        ("LOCALHOST:8892", "http://localhost:8892"),
        ("[::1]:8892", "http://[::1]:8892"),
        ("127.0.0.5:18892", "http://127.0.0.5:18892"),
        ("localhost", "http://localhost"),
    ] {
        let (st, body) = call(&s.app, req("POST", "/v1/stop", &[("host", host), ("origin", origin)], "")).await;
        assert_eq!(st, StatusCode::OK, "host {host} origin {origin}: {body}");
        let (st, _) = call(&s.app, req("GET", "/v1/status", &[("host", host), ("origin", origin)], "")).await;
        assert_eq!(st, StatusCode::OK, "host {host} origin {origin}");
        let h = [("host", host), ("origin", origin), ("authorization", &bearer())];
        let (st, _) = call(&s.app, req("POST", "/v1/arm", &h, "")).await;
        assert_eq!(st, StatusCode::OK, "host {host} origin {origin}");
    }
    // The configured bridge origins still pass whatever the Host is.
    let (st, _) = call(&s.app, req("POST", "/v1/stop", &[("host", "127.0.0.1:8892"), ("origin", "http://localhost:8891")], "")).await;
    assert_eq!(st, StatusCode::OK);
    // No Origin (curl, the Python client) is unchanged.
    let (st, _) = call(&s.app, req("POST", "/v1/stop", &[("host", "127.0.0.1:8892")], "")).await;
    assert_eq!(st, StatusCode::OK);
}

#[tokio::test]
async fn foreign_origins_stay_forbidden_even_when_they_match_the_host() {
    let s = stub(sim_cfg(), Some(ok_reply));
    let cases: [(&str, &str, &str); 9] = [
        // DNS rebinding: a name that resolves to 127.0.0.1 is still not loopback by name.
        ("evil.example:8892", "http://evil.example:8892", "rebinding"),
        ("evil.example", "http://evil.example", "rebinding, no port"),
        // Same host, different port / scheme / shape.
        ("127.0.0.1:8892", "http://127.0.0.1:9999", "other port"),
        ("127.0.0.1:8892", "http://127.0.0.1", "missing port"),
        ("127.0.0.1:8892", "https://127.0.0.1:8892", "https"),
        ("127.0.0.1:8892", "http://127.0.0.1:8892/", "trailing slash"),
        ("127.0.0.1:8892", "http://localhost:8892", "host literal vs name"),
        ("127.0.0.1:8892", "null", "opaque origin"),
        ("192.168.1.10:8892", "http://192.168.1.10:8892", "LAN address"),
    ];
    for (host, origin, why) in cases {
        let (st, body) = call(&s.app, req("POST", "/v1/stop", &[("host", host), ("origin", origin)], "")).await;
        assert_eq!(st, StatusCode::FORBIDDEN, "{why}: host {host} origin {origin}");
        assert_eq!(body["error"], "forbidden_origin", "{why}");
        let (st, _) = call(&s.app, req("GET", "/v1/motion", &[("host", host), ("origin", origin)], "")).await;
        assert_eq!(st, StatusCode::FORBIDDEN, "{why} (ws)");
    }
    // Without a Host header the same-origin rule cannot apply.
    let (st, _) = call(&s.app, req("POST", "/v1/stop", &[("origin", "http://127.0.0.1:8892")], "")).await;
    assert_eq!(st, StatusCode::FORBIDDEN, "no Host");
    assert!(s.seen.lock().unwrap().is_empty(), "nothing reached the control loop");
}

#[test]
fn same_origin_loopback_rule() {
    use api::same_origin_loopback as same;
    assert!(same("http://127.0.0.1:8892", Some("127.0.0.1:8892")));
    assert!(same("http://[::1]:8892", Some("[::1]:8892")));
    assert!(same("http://localhost:8892", Some("localhost:8892")));
    assert!(same("http://Localhost:8892", Some("localhost:8892")));
    assert!(!same("http://127.0.0.1:8892", None));
    assert!(!same("http://127.0.0.1:8892", Some("")));
    assert!(!same("http://127.0.0.1:8892", Some("127.0.0.1:8893")));
    assert!(!same("http://evil.example:8892", Some("evil.example:8892")));
    assert!(!same("http://10.0.0.1:8892", Some("10.0.0.1:8892")));
    assert!(!same("https://127.0.0.1:8892", Some("127.0.0.1:8892")));
    assert!(!same("http://127.0.0.1:8892/", Some("127.0.0.1:8892")));
    assert!(!same("http://[::1]:8892", Some("[::1:8892")), "unterminated bracket");
    assert!(!same("http://::1:8892", Some("::1:8892")), "unbracketed v6 is not a valid Host");
    // IPv4-mapped loopback: canonical form is loopback.
    assert!(same("http://[::ffff:127.0.0.1]:8892", Some("[::ffff:127.0.0.1]:8892")));
}

#[tokio::test]
async fn same_origin_requests_keep_body_limit_and_rate_limit() {
    let mut cfg = sim_cfg();
    cfg.server.max_body_bytes = 64;
    cfg.server.admin_rate_limit_per_s = 2;
    let s = stub(cfg, Some(ok_reply));
    let same = [("host", "127.0.0.1:8892"), ("origin", "http://127.0.0.1:8892"), ("authorization", &bearer())];
    let big = "x".repeat(65);
    let (st, body) = call(&s.app, req("POST", "/v1/fault", &same, &big)).await;
    assert_eq!(st, StatusCode::PAYLOAD_TOO_LARGE);
    assert_eq!(body["error"], "body_too_large");
    let mut codes = Vec::new();
    for _ in 0..3 {
        let (st, _) = call(&s.app, req("POST", "/v1/arm", &same, "")).await;
        codes.push(st);
    }
    assert_eq!(codes, [StatusCode::OK, StatusCode::OK, StatusCode::TOO_MANY_REQUESTS]);
}

#[tokio::test]
async fn admin_routes_need_the_bearer_token() {
    let s = stub(sim_cfg(), Some(ok_reply));
    for path in ["/v1/arm", "/v1/start", "/v1/disarm", "/v1/fault/reset", "/v1/center", "/v1/reference", "/v1/fault"] {
        let (st, body) = call(&s.app, req("POST", path, &[], "")).await;
        assert_eq!(st, StatusCode::UNAUTHORIZED, "{path} without token");
        assert_eq!(body["error"], "unauthorized");
        let (st, _) = call(&s.app, req("POST", path, &[("authorization", "Bearer wrong-token")], "")).await;
        assert_eq!(st, StatusCode::UNAUTHORIZED, "{path} wrong token");
        let (st, _) = call(&s.app, req("POST", path, &[("authorization", TOKEN)], "")).await;
        assert_eq!(st, StatusCode::UNAUTHORIZED, "{path} without Bearer prefix");
    }
    assert!(s.seen.lock().unwrap().is_empty());
    let (st, body) = call(&s.app, req("POST", "/v1/arm", &[("authorization", &bearer())], "")).await;
    assert_eq!(st, StatusCode::OK);
    assert_eq!(body["ok"], true);
    assert_eq!(body["rig_state"], "READY_TO_ARM");
    assert_eq!(s.seen.lock().unwrap().as_slice(), &[AdminCmd::Arm]);
}

#[tokio::test]
async fn stop_needs_no_token_and_is_not_rate_limited() {
    let mut cfg = sim_cfg();
    cfg.server.admin_rate_limit_per_s = 1;
    let s = stub(cfg, Some(ok_reply));
    // Use up the admin budget.
    let (st, _) = call(&s.app, req("POST", "/v1/arm", &[("authorization", &bearer())], "")).await;
    assert_eq!(st, StatusCode::OK);
    let (st, _) = call(&s.app, req("POST", "/v1/arm", &[("authorization", &bearer())], "")).await;
    assert_eq!(st, StatusCode::TOO_MANY_REQUESTS);
    for _ in 0..5 {
        let (st, body) = call(&s.app, req("POST", "/v1/stop", &[], "")).await;
        assert_eq!(st, StatusCode::OK);
        assert_eq!(body["ok"], true);
    }
    assert_eq!(s.seen.lock().unwrap().iter().filter(|c| **c == AdminCmd::Stop).count(), 5);
}

#[tokio::test]
async fn oversize_body_is_413() {
    let s = stub(sim_cfg(), Some(ok_reply));
    let big = format!("{{\"note\":\"{}\"}}", "x".repeat(5000));
    let (st, body) = call(&s.app, req("POST", "/v1/fault", &[("authorization", &bearer())], &big)).await;
    assert_eq!(st, StatusCode::PAYLOAD_TOO_LARGE);
    assert_eq!(body["ok"], false);
    let big = format!("{{\"axis\":1,\"count\":0,\"pad\":\"{}\"}}", "x".repeat(5000));
    let (st, _) = call(&s.app, req("POST", "/v1/reference", &[("authorization", &bearer())], &big)).await;
    assert_eq!(st, StatusCode::PAYLOAD_TOO_LARGE);
    assert!(s.seen.lock().unwrap().is_empty());
}

#[tokio::test(start_paused = true)]
async fn admin_rate_limit_is_429_and_refills() {
    let mut cfg = sim_cfg();
    cfg.server.admin_rate_limit_per_s = 3;
    let s = stub(cfg, Some(ok_reply));
    let arm = || req("POST", "/v1/arm", &[("authorization", &bearer())], "");
    for _ in 0..3 {
        assert_eq!(call(&s.app, arm()).await.0, StatusCode::OK);
    }
    let (st, body) = call(&s.app, arm()).await;
    assert_eq!(st, StatusCode::TOO_MANY_REQUESTS);
    assert_eq!(body["error"], "rate_limited");
    tokio::time::advance(Duration::from_millis(400)).await;
    assert_eq!(call(&s.app, arm()).await.0, StatusCode::OK, "refilled at 3/s");
    assert_eq!(call(&s.app, arm()).await.0, StatusCode::TOO_MANY_REQUESTS);
    // Unauthenticated requests do not spend the budget.
    tokio::time::advance(Duration::from_secs(2)).await;
    for _ in 0..10 {
        let (st, _) = call(&s.app, req("POST", "/v1/arm", &[], "")).await;
        assert_eq!(st, StatusCode::UNAUTHORIZED);
    }
    assert_eq!(call(&s.app, arm()).await.0, StatusCode::OK);
}

#[test]
fn token_bucket_zero_rate_disables() {
    let t0 = Instant::now();
    let mut b = TokenBucket::new(0, t0);
    assert!((0..1000).all(|_| b.take(t0)));
    let mut b = TokenBucket::new(2, t0);
    assert!(b.take(t0) && b.take(t0) && !b.take(t0));
}

#[test]
fn constant_time_eq_basics() {
    assert!(api::constant_time_eq(b"abc", b"abc"));
    assert!(!api::constant_time_eq(b"abc", b"abd"));
    assert!(!api::constant_time_eq(b"abc", b"abcd"));
}

#[tokio::test]
async fn refused_admin_command_is_409_with_blockers() {
    let s = stub(sim_cfg(), Some(not_ready));
    let (st, body) = call(&s.app, req("POST", "/v1/arm", &[("authorization", &bearer())], "")).await;
    assert_eq!(st, StatusCode::CONFLICT);
    assert_eq!(body["ok"], false);
    assert_eq!(body["error"], "not_ready");
    assert_eq!(body["blockers"][0], "not_connected: axis2");
}

#[tokio::test(start_paused = true)]
async fn admin_without_control_loop_answer_is_503_after_1s() {
    let s = stub(sim_cfg(), None);
    let t = Instant::now();
    let (st, body) = call(&s.app, req("POST", "/v1/stop", &[], "")).await;
    assert_eq!(st, StatusCode::SERVICE_UNAVAILABLE);
    assert_eq!(body["error"], "control_loop_unavailable");
    assert!(t.elapsed() >= Duration::from_secs(1));
}

#[tokio::test]
async fn reference_and_fault_bodies() {
    let s = stub(sim_cfg(), Some(ok_reply));
    let h = [("authorization", bearer())];
    let h: Vec<(&str, &str)> = h.iter().map(|(k, v)| (*k, v.as_str())).collect();
    let (st, _) = call(&s.app, req("POST", "/v1/reference", &h, r#"{"axis":2,"count":-15}"#)).await;
    assert_eq!(st, StatusCode::OK);
    let (st, body) = call(&s.app, req("POST", "/v1/reference", &h, r#"{"axis":4,"count":0}"#)).await;
    assert_eq!(st, StatusCode::BAD_REQUEST);
    assert_eq!(body["error"], "bad_axis");
    let (st, _) = call(&s.app, req("POST", "/v1/reference", &h, r#"{"axis":1}"#)).await;
    assert_eq!(st, StatusCode::BAD_REQUEST);
    let (st, _) = call(&s.app, req("POST", "/v1/fault", &h, r#"{"note":"operator test"}"#)).await;
    assert_eq!(st, StatusCode::OK);
    let (st, _) = call(&s.app, req("POST", "/v1/fault", &h, "")).await;
    assert_eq!(st, StatusCode::OK, "empty fault body is still a fault");
    let seen = s.seen.lock().unwrap().clone();
    assert_eq!(
        seen,
        vec![
            AdminCmd::Reference { axis: h3cs::types::AxisId::new(2).unwrap(), count: -15 },
            AdminCmd::Fault { note: "operator test".into() },
            AdminCmd::Fault { note: String::new() },
        ]
    );
}

#[tokio::test]
async fn session_response_and_pose_watch_cleared() {
    let s = stub(sim_cfg(), Some(ok_reply));
    let (st, body) = call(&s.app, req("POST", "/v1/session", &[], "")).await;
    assert_eq!(st, StatusCode::OK);
    let id = body["source_session"].as_str().unwrap();
    assert_eq!(id.len(), 32);
    assert!(id.chars().all(|c| c.is_ascii_hexdigit()));
    assert_eq!(body["protocol"], 1);
    assert_eq!(body["max_gain"], 0.5);
    assert_eq!(body["source_freshness_ms"], 250);
    assert_eq!(body["require_frame_seq"], true);
    let (_, body2) = call(&s.app, req("POST", "/v1/session", &[], "")).await;
    assert_ne!(body2["source_session"], body["source_session"]);
    assert!(s.pose_rx.borrow().is_none());
}

#[tokio::test]
async fn unknown_route_is_json_404() {
    let s = stub(sim_cfg(), Some(ok_reply));
    let (st, body) = call(&s.app, req("GET", "/v1/nope", &[], "")).await;
    assert_eq!(st, StatusCode::NOT_FOUND);
    assert_eq!(body["ok"], false);
}

// ------------------------------------------------------------------ WS (real server)

#[tokio::test]
async fn new_session_makes_the_old_websocket_stale() {
    let srv = start_sim(sim_cfg()).await;
    let a = new_session(srv.local_addr).await;
    let mut ws_a = ws_connect(srv.local_addr, &a).await;
    ws_send(&mut ws_a, pose_json(&a, 1, 1, 0.1, 0.0, 0.0, 0.35)).await;
    let ack = next_ack(&mut ws_a).await;
    assert_eq!(ack["accepted"], true, "{ack}");
    assert_eq!(ack["seq"], 1);
    assert_eq!(ack["fresh"], true);

    let b = new_session(srv.local_addr).await;
    ws_send(&mut ws_a, pose_json(&a, 2, 2, 0.1, 0.0, 0.0, 0.35)).await;
    let ack = next_ack(&mut ws_a).await;
    assert_eq!(ack["accepted"], false);
    assert_eq!(ack["reason"], "stale_session");
    assert_eq!(ack["seq"], 2);

    // A WS pinned to session B rejects messages naming another session.
    let mut ws_b = ws_connect(srv.local_addr, &b).await;
    ws_send(&mut ws_b, pose_json(&a, 3, 3, 0.1, 0.0, 0.0, 0.35)).await;
    assert_eq!(next_ack(&mut ws_b).await["reason"], "stale_session");
    ws_send(&mut ws_b, pose_json(&b, 1, 1, 0.1, 0.0, 0.0, 0.6)).await;
    let ack = next_ack(&mut ws_b).await;
    assert_eq!(ack["accepted"], true);
    assert_eq!(ack["effective_gain"], 0.5); // limits.max_gain
    assert_eq!(ack["limits"][0], "gain_limited");
    srv.shutdown().await;
}

#[tokio::test]
async fn malformed_ws_messages_get_bad_message_and_the_socket_stays_open() {
    let srv = start_sim(sim_cfg()).await;
    let id = new_session(srv.local_addr).await;
    let mut ws = ws_connect(srv.local_addr, &id).await;

    for bad in ["not json".to_string(), r#"{"type":"teleport"}"#.into(), r#"{"type":"pose","seq":1}"#.into()] {
        ws_send(&mut ws, bad).await;
        let ack = next_ack(&mut ws).await;
        assert_eq!(ack["accepted"], false);
        assert_eq!(ack["reason"], "bad_message");
        assert!(ack["seq"].is_null());
    }
    // Over max_body_bytes (4096) but under the transport cap.
    let huge = format!("{{\"type\":\"pose\",\"pad\":\"{}\"}}", "x".repeat(6000));
    ws_send(&mut ws, huge).await;
    assert_eq!(next_ack(&mut ws).await["reason"], "bad_message");
    ws.send(Message::Binary(vec![1u8, 2, 3].into())).await.unwrap();
    assert_eq!(next_ack(&mut ws).await["reason"], "bad_message");

    // Non-finite and protocol errors are acked with their seq.
    let nan = pose_json(&id, 1, 1, 0.0, 0.0, 0.0, 0.35).replace("\"pitch\":0.0", "\"pitch\":1e999");
    ws_send(&mut ws, nan).await;
    let ack = next_ack(&mut ws).await;
    assert_eq!(ack["accepted"], false, "{ack}");

    ws_send(&mut ws, pose_json(&id, 5, 1, 0.2, 0.1, 0.0, 0.35)).await;
    let ack = next_ack(&mut ws).await;
    assert_eq!(ack["accepted"], true, "socket still usable: {ack}");
    ws_send(&mut ws, pose_json(&id, 5, 2, 0.2, 0.1, 0.0, 0.35)).await;
    assert_eq!(next_ack(&mut ws).await["reason"], "old_seq");
    let gain = serde_json::json!({"type":"gain","protocol":1,"source_session":id,"seq":6,"gain":0.2}).to_string();
    ws_send(&mut ws, gain).await;
    let ack = next_ack(&mut ws).await;
    assert_eq!(ack["accepted"], true);
    assert_eq!(ack["effective_gain"], 0.2);
    srv.shutdown().await;
}

fn assert_keys(v: &Value, keys: &[&str], what: &str) {
    let obj = v.as_object().unwrap_or_else(|| panic!("{what} is not an object: {v}"));
    for k in keys {
        assert!(obj.contains_key(*k), "{what} lacks key {k:?}: {v}");
    }
}

const STATUS_KEYS: &[&str] = &[
    "protocol", "rig_state", "armed", "active_source", "calibration_valid", "fault", "fault_history",
    "blockers", "session", "requested_pose", "requested_gain", "effective_gain", "limits",
    "virtual_targets", "target_counts", "session_epoch", "group_frame_id", "axes", "axis_conflict", "measured_angles",
    "physical_estop", "telemetry", "motion_limits",
];
const AXIS_KEYS: &[&str] = &[
    "axis_id", "name", "connected", "port", "device_id", "expected_device_id", "assignment", "claimants",
    "reference_stale", "boot_id", "mcu_state", "fault_code", "configured",
    "referenced", "calibration_valid", "actual_count", "target_count", "velocity_count_s",
    "following_error_count", "pwm_permille", "power_on_cmd", "brake_release_cmd", "power_controller", "current_a",
    "last_applied_frame", "status_age_ms", "rx_error_count", "stale_drop_count", "decode_errors", "fw_limits",
];
const SESSION_KEYS: &[&str] = &["source_session_prefix", "fresh", "freshness_mode", "source_age_ms", "last_seq", "last_frame_seq"];
const TELEMETRY_KEYS: &[&str] =
    &["tick_count", "tick_overruns", "tick_dt_ms", "pose_rx_to_tick_ms", "status_age_ms", "frame_skew", "log_dropped"];

fn check_status_shape(v: &Value) {
    assert_keys(v, STATUS_KEYS, "status");
    assert_eq!(v["protocol"], 1);
    assert_eq!(v["physical_estop"], "not_monitored");
    assert!(v["measured_angles"].is_null());
    let axes = v["axes"].as_array().expect("axes");
    assert_eq!(axes.len(), 3);
    for a in axes {
        assert_keys(a, AXIS_KEYS, "axis");
    }
    assert_keys(&v["session"], SESSION_KEYS, "session");
    assert_keys(&v["telemetry"], TELEMETRY_KEYS, "telemetry");
    assert_keys(&v["motion_limits"], &["effective", "requested", "ceiling", "defaults", "limited"], "motion_limits");
    for k in ["effective", "requested", "ceiling", "defaults"] {
        assert_keys(
            &v["motion_limits"][k],
            &["max_gain", "max_vel_count_s", "max_acc_count_s2", "duty_max_percent"],
            k,
        );
    }
    for k in ["tick_dt_ms", "pose_rx_to_tick_ms", "status_age_ms", "frame_skew"] {
        assert_keys(&v["telemetry"][k], &["p50", "p95", "p99", "max"], k);
    }
}

#[tokio::test]
async fn status_json_has_every_documented_key() {
    let srv = start_sim(sim_cfg()).await;
    wait_state(&srv, RigState::ReadyToArm, Duration::from_secs(3)).await;
    let r = http(srv.local_addr, "GET", "/v1/status", &[], None).await;
    assert_eq!(r.status, 200);
    let v = r.json();
    check_status_shape(&v);
    assert_eq!(v["rig_state"], "READY_TO_ARM");
    assert_eq!(v["armed"], false);
    assert!(v["telemetry"]["tick_count"].as_u64().unwrap() > 0);
    assert!(v["axes"][0]["current_a"].is_null(), "no current sensor: null, not 0");

    // WS pushes the same shape with type = status, without any pose sent.
    let id = new_session(srv.local_addr).await;
    let mut ws = ws_connect(srv.local_addr, &id).await;
    let st = next_status(&mut ws).await;
    check_status_shape(&st);
    // The first push may predate the session (published at the last tick).
    let mut st = st;
    for _ in 0..5 {
        if st["session"]["source_session_prefix"] == id[..8] {
            break;
        }
        st = next_status(&mut ws).await;
    }
    // Only a prefix is public: the full id gates pose input.
    assert_eq!(st["session"]["source_session_prefix"], &id[..8]);
    assert!(!st.to_string().contains(id.as_str()), "full session id leaked in status");
    let st2 = next_status(&mut ws).await;
    assert_eq!(st2["type"], "status");
    let _ = ws.next().await;
    srv.shutdown().await;
}

// ------------------------------------------------------------------ review fixes (stage B)

/// Router whose admin queue and status sender the test holds directly.
fn manual(cfg: Config) -> (Router, mpsc::Receiver<AdminRequest>, watch::Sender<Arc<h3cs::supervisor::StatusSnapshot>>) {
    manual_with_path(cfg, None)
}

/// [`manual`] with the config path the export writes next to.
fn manual_with_path(
    cfg: Config,
    config_path: Option<std::path::PathBuf>,
) -> (Router, mpsc::Receiver<AdminRequest>, watch::Sender<Arc<h3cs::supervisor::StatusSnapshot>>) {
    let cfg = Arc::new(cfg);
    let (admin_tx, admin_rx) = mpsc::channel::<AdminRequest>(api::ADMIN_QUEUE_LEN);
    let (pose_tx, _pose_rx) = watch::channel(None);
    let (_shutdown_tx, shutdown_rx) = watch::channel(false);
    let snap = Supervisor::new(&cfg).snapshot(Instant::now());
    let (status_tx, status_rx) = watch::channel(Arc::new(snap));
    let state = AppState::new(
        cfg,
        Arc::new(Mutex::new(PoseIntake::new())),
        Arc::new(pose_tx),
        admin_tx,
        status_rx,
        TOKEN.into(),
        shutdown_rx,
    )
    .with_config_path(config_path);
    (api::router(state), admin_rx, status_tx)
}

#[tokio::test]
async fn fault_latches_whatever_the_body() {
    let s = stub(sim_cfg(), Some(ok_reply));
    let h = [("authorization", bearer())];
    let h: Vec<(&str, &str)> = h.iter().map(|(k, v)| (*k, v.as_str())).collect();
    for body in [r#"{"note":"x","why":1}"#, "note=x", "{not json", "  "] {
        let (st, b) = call(&s.app, req("POST", "/v1/fault", &h, body)).await;
        assert_eq!(st, StatusCode::OK, "{body:?}: {b}");
    }
    let seen = s.seen.lock().unwrap().clone();
    assert_eq!(
        seen,
        vec![
            AdminCmd::Fault { note: "x".into() },
            AdminCmd::Fault { note: "unparsed body: note=x".into() },
            AdminCmd::Fault { note: "unparsed body: {not json".into() },
            AdminCmd::Fault { note: String::new() },
        ]
    );
    let long = "y".repeat(1000);
    assert_eq!(api::fault_note(long.as_bytes()).chars().count(), "unparsed body: ".len() + 200);
}

#[tokio::test]
async fn disarm_and_fault_are_not_rate_limited() {
    let mut cfg = sim_cfg();
    cfg.server.admin_rate_limit_per_s = 1;
    let s = stub(cfg, Some(ok_reply));
    let auth = [("authorization", bearer())];
    let auth: Vec<(&str, &str)> = auth.iter().map(|(k, v)| (*k, v.as_str())).collect();
    assert_eq!(call(&s.app, req("POST", "/v1/arm", &auth, "")).await.0, StatusCode::OK);
    assert_eq!(call(&s.app, req("POST", "/v1/arm", &auth, "")).await.0, StatusCode::TOO_MANY_REQUESTS);
    for _ in 0..5 {
        assert_eq!(call(&s.app, req("POST", "/v1/disarm", &auth, "")).await.0, StatusCode::OK);
        assert_eq!(call(&s.app, req("POST", "/v1/fault", &auth, "")).await.0, StatusCode::OK);
    }
    // Still behind the token.
    assert_eq!(call(&s.app, req("POST", "/v1/fault", &[], "")).await.0, StatusCode::UNAUTHORIZED);
    assert_eq!(call(&s.app, req("POST", "/v1/fault/reset", &auth, "")).await.0, StatusCode::TOO_MANY_REQUESTS);
}

#[tokio::test]
async fn wrong_method_is_json_405() {
    let s = stub(sim_cfg(), Some(ok_reply));
    let (st, body) = call(&s.app, req("GET", "/v1/session", &[], "")).await;
    assert_eq!(st, StatusCode::METHOD_NOT_ALLOWED);
    assert_eq!(body["error"], "method_not_allowed");
    assert_eq!(body["ok"], false);
    let (st, body) = call(&s.app, req("POST", "/v1/status", &[], "")).await;
    assert_eq!(st, StatusCode::METHOD_NOT_ALLOWED);
    assert_eq!(body["error"], "method_not_allowed");
}

#[tokio::test(start_paused = true)]
async fn admin_command_answered_503_is_never_executed_late() {
    let (app, mut admin_rx, _status) = manual(sim_cfg());
    let auth = bearer();
    let t = Instant::now();
    let (st, body) = call(&app, req("POST", "/v1/arm", &[("authorization", &auth)], "")).await;
    assert_eq!(st, StatusCode::SERVICE_UNAVAILABLE);
    assert_eq!(body["error"], "control_loop_unavailable");
    assert!(t.elapsed() >= api::ADMIN_TIMEOUT);
    // The loop catches up: the request is still queued but cannot be claimed.
    let (cmd, reply) = admin_rx.try_recv().expect("queued");
    assert_eq!(cmd, AdminCmd::Arm);
    assert!(!reply.claim(), "requester gave up: must not run");
    assert!(!api::runs_when_abandoned(&cmd));
    // Safety commands still run when abandoned.
    let (st, _) = call(&app, req("POST", "/v1/stop", &[], "")).await;
    assert_eq!(st, StatusCode::SERVICE_UNAVAILABLE);
    let (cmd, reply) = admin_rx.try_recv().expect("queued");
    assert!(!reply.claim() && api::runs_when_abandoned(&cmd));
    for c in [AdminCmd::Disarm, AdminCmd::Fault { note: String::new() }] {
        assert!(api::runs_when_abandoned(&c));
    }
    for c in [AdminCmd::Start, AdminCmd::Center, AdminCmd::FaultReset] {
        assert!(!api::runs_when_abandoned(&c));
    }
}

#[tokio::test(start_paused = true)]
async fn claimed_admin_command_is_answered_even_after_the_timeout() {
    let (app, mut admin_rx, _status) = manual(sim_cfg());
    let loop_task = tokio::spawn(async move {
        let (_, reply) = admin_rx.recv().await.unwrap();
        tokio::time::sleep(Duration::from_millis(900)).await;
        assert!(reply.claim());
        // Executing takes past the API timeout.
        tokio::time::sleep(Duration::from_millis(300)).await;
        let _ = reply.send(Ok(AdminOk { rig_state: RigState::Arming, note: None }));
    });
    let (st, body) = call(&app, req("POST", "/v1/arm", &[("authorization", &bearer())], "")).await;
    assert_eq!(st, StatusCode::OK, "{body}");
    assert_eq!(body["rig_state"], "ARMING");
    loop_task.await.unwrap();
}

#[tokio::test]
async fn non_loopback_peers_get_only_status_and_stop() {
    use axum::extract::connect_info::MockConnectInfo;
    let mut cfg = sim_cfg();
    cfg.server.allow_non_loopback = true;
    let lan: std::net::SocketAddr = "192.168.1.20:50000".parse().unwrap();
    let s = stub(cfg.clone(), Some(ok_reply));
    let app = s.app.clone().layer(MockConnectInfo(lan));
    let auth = bearer();
    // The test page (it holds the admin token) is closed to the LAN too.
    for (m, p) in [("POST", "/v1/session"), ("GET", "/v1/motion"), ("POST", "/v1/arm"), ("POST", "/v1/fault"), ("GET", "/"), ("GET", "/index.html")] {
        let (st, body) = call(&app, req(m, p, &[("authorization", &auth)], "")).await;
        assert_eq!(st, StatusCode::FORBIDDEN, "{m} {p}");
        assert_eq!(body["error"], "loopback_only");
    }
    assert_eq!(call(&app, req("GET", "/v1/status", &[], "")).await.0, StatusCode::OK);
    assert_eq!(call(&app, req("POST", "/v1/stop", &[], "")).await.0, StatusCode::OK);
    assert_eq!(s.seen.lock().unwrap().as_slice(), &[AdminCmd::Stop]);
    // Loopback peers (also IPv4-mapped) keep everything.
    for local in ["127.0.0.1:50000", "[::ffff:127.0.0.1]:50000", "[::1]:50000"] {
        let app = s.app.clone().layer(MockConnectInfo(local.parse::<std::net::SocketAddr>().unwrap()));
        assert_eq!(call(&app, req("POST", "/v1/session", &[], "")).await.0, StatusCode::OK, "{local}");
    }
    // Unknown peer: fail closed.
    assert_eq!(call(&s.app, req("POST", "/v1/session", &[], "")).await.0, StatusCode::FORBIDDEN);
    // Without the knob the bind is loopback-only and nothing is filtered.
    let s = stub(sim_cfg(), Some(ok_reply));
    assert_eq!(call(&s.app, req("POST", "/v1/session", &[], "")).await.0, StatusCode::OK);
}

#[tokio::test]
async fn status_is_503_once_the_control_loop_is_gone() {
    let (app, _admin_rx, status_tx) = manual(sim_cfg());
    assert_eq!(call(&app, req("GET", "/v1/status", &[], "")).await.0, StatusCode::OK);
    drop(status_tx);
    let (st, body) = call(&app, req("GET", "/v1/status", &[], "")).await;
    assert_eq!(st, StatusCode::SERVICE_UNAVAILABLE);
    assert_eq!(body["error"], "control_loop_unavailable");
}

#[tokio::test]
async fn events_endpoint_reads_the_shared_ring_even_after_the_loop_died() {
    use axum::extract::connect_info::MockConnectInfo;
    use h3cs::telemetry::{lock_events, shared_event_log};
    let mut cfg = sim_cfg();
    cfg.server.allow_non_loopback = true;
    let cfg = Arc::new(cfg);
    let (admin_tx, _admin_rx) = mpsc::channel::<AdminRequest>(api::ADMIN_QUEUE_LEN);
    let (pose_tx, _pose_rx) = watch::channel(None);
    let (_shutdown_tx, shutdown_rx) = watch::channel(false);
    let (status_tx, status_rx) = watch::channel(Arc::new(Supervisor::new(&cfg).snapshot(Instant::now())));
    let log = shared_event_log(Instant::now());
    let state = AppState::new(
        cfg,
        Arc::new(Mutex::new(PoseIntake::new())),
        Arc::new(pose_tx),
        admin_tx,
        status_rx,
        TOKEN.into(),
        shutdown_rx,
    )
    .with_event_log(log.clone());
    let app = api::router(state).layer(MockConnectInfo("127.0.0.1:50000".parse::<std::net::SocketAddr>().unwrap()));
    for k in 1..=5 {
        lock_events(&log).push(Instant::now(), "probe", serde_json::json!({ "k": k }));
    }

    let (st, body) = call(&app, req("GET", "/v1/events", &[], "")).await;
    assert_eq!(st, StatusCode::OK, "no token needed: {body}");
    assert_eq!(body["next_id"], 6);
    assert_eq!(body["dropped"], 0);
    let ks = |b: &Value| b["events"].as_array().unwrap().iter().map(|e| e["detail"]["k"].as_i64().unwrap()).collect::<Vec<_>>();
    assert_eq!(ks(&body), [1, 2, 3, 4, 5], "oldest first");
    assert_eq!(ks(&call(&app, req("GET", "/v1/events?n=2", &[], "")).await.1), [4, 5]);
    assert_eq!(ks(&call(&app, req("GET", "/v1/events?after=3", &[], "")).await.1), [4, 5]);
    assert_eq!(ks(&call(&app, req("GET", "/v1/events?after=1&n=2", &[], "")).await.1), [4, 5]);
    assert!(ks(&call(&app, req("GET", "/v1/events?after=5", &[], "")).await.1).is_empty());
    let (st, body) = call(&app, req("GET", "/v1/events?n=lots", &[], "")).await;
    assert_eq!((st, body["error"].as_str()), (StatusCode::BAD_REQUEST, Some("bad_request")));

    // Post-mortem: the loop is gone (status 503) but its events stay readable.
    drop(status_tx);
    assert_eq!(call(&app, req("GET", "/v1/status", &[], "")).await.0, StatusCode::SERVICE_UNAVAILABLE);
    let (st, body) = call(&app, req("GET", "/v1/events", &[], "")).await;
    assert_eq!(st, StatusCode::OK);
    assert_eq!(ks(&body).len(), 5);

    // Not for the LAN, like the rest of the operator surface.
    let lan = api::router(AppState::new(
        Arc::new({
            let mut c = sim_cfg();
            c.server.allow_non_loopback = true;
            c
        }),
        Arc::new(Mutex::new(PoseIntake::new())),
        Arc::new(watch::channel(None).0),
        mpsc::channel::<AdminRequest>(1).0,
        watch::channel(Arc::new(Supervisor::new(&sim_cfg()).snapshot(Instant::now()))).1,
        TOKEN.into(),
        watch::channel(false).1,
    ))
    .layer(MockConnectInfo("192.168.1.20:50000".parse::<std::net::SocketAddr>().unwrap()));
    let (st, body) = call(&lan, req("GET", "/v1/events", &[], "")).await;
    assert_eq!((st, body["error"].as_str()), (StatusCode::FORBIDDEN, Some("loopback_only")));
}

// ------------------------------------------------------------------ axis conflict (DESIGN §10)

/// Serve a router on a loopback port (WebSocket tests need real TCP).
async fn serve(app: Router) -> std::net::SocketAddr {
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move {
        let _ = axum::serve(listener, app.into_make_service_with_connect_info::<std::net::SocketAddr>()).await;
    });
    addr
}

#[tokio::test]
async fn pose_and_gain_are_rejected_axis_conflict_without_reaching_the_intake() {
    let cfg = Arc::new(sim_cfg());
    let (admin_tx, _admin_rx) = mpsc::channel::<AdminRequest>(api::ADMIN_QUEUE_LEN);
    let (pose_tx, pose_rx) = watch::channel(None);
    let (_shutdown_tx, shutdown_rx) = watch::channel(false);
    let mut snap = Supervisor::new(&cfg).snapshot(Instant::now());
    snap.axis_conflict = vec![h3cs::types::AxisId::new(2).unwrap()];
    let (status_tx, status_rx) = watch::channel(Arc::new(snap.clone()));
    let session = Arc::new(Mutex::new(PoseIntake::new()));
    let state = AppState::new(cfg, session.clone(), Arc::new(pose_tx), admin_tx, status_rx, TOKEN.into(), shutdown_rx);
    let addr = serve(api::router(state)).await;

    let id = new_session(addr).await;
    let mut ws = ws_connect(addr, &id).await;
    ws_send(&mut ws, pose_json(&id, 1, 1, 0.1, 0.0, 0.0, 0.3)).await;
    let ack = next_ack(&mut ws).await;
    assert_eq!(ack["accepted"], false);
    assert_eq!(ack["reason"], "axis_conflict");
    assert_eq!(ack["seq"], 1);
    assert_eq!(ack["fresh"], false, "freshness never extended");
    let gain = serde_json::json!({"type": "gain", "protocol": 1, "source_session": id, "seq": 2, "gain": 0.2}).to_string();
    ws_send(&mut ws, gain).await;
    let ack = next_ack(&mut ws).await;
    assert_eq!((ack["accepted"].as_bool(), ack["reason"].as_str()), (Some(false), Some("axis_conflict")));
    assert!(pose_rx.borrow().is_none(), "nothing stored for the supervisor");
    {
        let s = session.lock().unwrap();
        assert!(s.latest().is_none());
        assert!(!s.is_fresh(Instant::now()));
    }
    // The status pushed on the socket shows the conflict.
    let st = next_status(&mut ws).await;
    assert_eq!(st["axis_conflict"], serde_json::json!([2]));

    // Conflict resolved: the same client is accepted again (seq continues).
    snap.axis_conflict.clear();
    status_tx.send_replace(Arc::new(snap));
    ws_send(&mut ws, pose_json(&id, 3, 1, 0.1, 0.0, 0.0, 0.3)).await;
    let ack = next_ack(&mut ws).await;
    assert_eq!(ack["accepted"], true, "{ack}");
    assert!(pose_rx.borrow().is_some());
}

#[tokio::test]
async fn arm_start_center_pass_the_supervisors_axis_conflict_as_409() {
    fn conflict(cmd: &AdminCmd) -> Result<AdminOk, AdminError> {
        match cmd {
            AdminCmd::Arm | AdminCmd::Start | AdminCmd::Center => Err(AdminError {
                error: "axis_conflict".into(),
                blockers: vec!["axis_conflict: axis2 claimed by sim:3(5131000000000002), sim:2(5131000000000001)".into()],
            }),
            _ => Ok(AdminOk { rig_state: RigState::Offline, note: Some("not driving".into()) }),
        }
    }
    let s = stub(sim_cfg(), Some(conflict));
    let b = bearer();
    for path in ["/v1/arm", "/v1/start", "/v1/center"] {
        let (st, body) = call(&s.app, req("POST", path, &[("authorization", &b)], "")).await;
        assert_eq!(st, StatusCode::CONFLICT, "{path}: {body}");
        assert_eq!(body["ok"], false);
        assert_eq!(body["error"], "axis_conflict");
        assert!(body["blockers"][0].as_str().unwrap().starts_with("axis_conflict: axis2"), "{body}");
    }
    // Stop (no token) is unaffected.
    let (st, body) = call(&s.app, req("POST", "/v1/stop", &[], "")).await;
    assert_eq!((st, body["ok"].as_bool()), (StatusCode::OK, Some(true)));
}

// ------------------------------------------------------------------ calibration (DESIGN.md §11)

fn auth_headers() -> Vec<(&'static str, String)> {
    vec![("authorization", bearer())]
}

#[tokio::test]
async fn calibrate_routes_need_the_token_and_carry_the_axes() {
    let s = stub(sim_cfg(), Some(ok_reply));
    let h = auth_headers();
    let h: Vec<(&str, &str)> = h.iter().map(|(k, v)| (*k, v.as_str())).collect();
    for p in ["/v1/calibrate", "/v1/calibrate/cancel", "/v1/calibration/export"] {
        let (st, body) = call(&s.app, req("POST", p, &[], "")).await;
        assert_eq!(st, StatusCode::UNAUTHORIZED, "{p}");
        assert_eq!(body["error"], "unauthorized");
        let (st, _) = call(&s.app, req("GET", p, &h, "")).await;
        assert_eq!(st, StatusCode::METHOD_NOT_ALLOWED, "{p}");
        let (st, body) = call(&s.app, req("POST", p, &[("origin", "http://evil.example")], "")).await;
        assert_eq!(st, StatusCode::FORBIDDEN, "{p}");
        assert_eq!(body["error"], "forbidden_origin");
    }
    assert!(s.seen.lock().unwrap().is_empty());
    let (st, body) = call(&s.app, req("POST", "/v1/calibrate", &h, "")).await;
    assert_eq!(st, StatusCode::OK, "{body}");
    let (st, _) = call(&s.app, req("POST", "/v1/calibrate", &h, r#"{"axes":[3,1]}"#)).await;
    assert_eq!(st, StatusCode::OK);
    let (st, body) = call(&s.app, req("POST", "/v1/calibrate", &h, r#"{"axes":[4]}"#)).await;
    assert_eq!(st, StatusCode::BAD_REQUEST);
    assert_eq!(body["error"], "bad_axis");
    let (st, body) = call(&s.app, req("POST", "/v1/calibrate", &h, "{not json")).await;
    assert_eq!(st, StatusCode::BAD_REQUEST);
    assert_eq!(body["error"], "bad_request");
    let (st, body) = call(&s.app, req("POST", "/v1/calibrate", &h, r#"{"axes":[1],"mode":0}"#)).await;
    assert_eq!(st, StatusCode::BAD_REQUEST, "unknown fields are refused: {body}");
    let (st, _) = call(&s.app, req("POST", "/v1/calibrate/cancel", &h, "")).await;
    assert_eq!(st, StatusCode::OK);
    let ax = |n: u8| h3cs::types::AxisId::new(n).unwrap();
    assert_eq!(
        s.seen.lock().unwrap().clone(),
        vec![
            AdminCmd::Calibrate { axes: vec![] },
            AdminCmd::Calibrate { axes: vec![ax(3), ax(1)] },
            AdminCmd::CalibrateCancel,
        ]
    );
    // Supervisor refusal → 409 with the code and blockers.
    let s = stub(sim_cfg(), Some(|_: &AdminCmd| {
        Err(AdminError { error: "power_axis_required".into(), blockers: vec!["power_axis_required: axis1 drives the main power".into()] })
    }));
    let (st, body) = call(&s.app, req("POST", "/v1/calibrate", &h, r#"{"axes":[2]}"#)).await;
    assert_eq!(st, StatusCode::CONFLICT);
    assert_eq!(body["error"], "power_axis_required");
    assert_eq!(body["blockers"][0], "power_axis_required: axis1 drives the main power");
}

#[tokio::test]
async fn calibration_routes_are_closed_to_the_lan() {
    use axum::extract::connect_info::MockConnectInfo;
    let mut cfg = sim_cfg();
    cfg.server.allow_non_loopback = true;
    let lan: std::net::SocketAddr = "192.168.1.20:50000".parse().unwrap();
    let s = stub(cfg, Some(ok_reply));
    let app = s.app.clone().layer(MockConnectInfo(lan));
    let auth = bearer();
    for p in ["/v1/calibrate", "/v1/calibrate/cancel", "/v1/calibration/export"] {
        let (st, body) = call(&app, req("POST", p, &[("authorization", &auth)], "")).await;
        assert_eq!(st, StatusCode::FORBIDDEN, "{p}");
        assert_eq!(body["error"], "loopback_only");
    }
    assert!(s.seen.lock().unwrap().is_empty());
}

fn scratch_dir(name: &str) -> std::path::PathBuf {
    let base = std::env::var_os("H3CS_TEST_TMP")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(std::env::temp_dir);
    let dir = base.join(format!("h3cs-api-{name}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

fn cal_ok(n_end: i32) -> h3cs::supervisor::CalibrationResult {
    h3cs::supervisor::CalibrationResult {
        device_id: h3cs::types::DeviceId(0x5131_1cdb_d4ba_8d58),
        result: h3cs::protocol::CalResultCode::Ok,
        step: 3,
        n_end_count: n_end,
        neutral_count: n_end / 2,
        sign: if n_end < 0 { -1 } else { 1 },
        reason: "ok".into(),
        boot_id: 7,
        at_ms: 1.0,
    }
}

#[tokio::test]
async fn calibration_export_writes_next_to_the_config_and_never_overwrites() {
    use h3cs::calibration_export::{file_name, utc_timestamp};
    let dir = scratch_dir("export");
    let cfg_path = dir.join("sim.toml");
    std::fs::copy(SIM, &cfg_path).unwrap();
    let cfg = sim_cfg();
    let (app, _admin_rx, status_tx) = manual_with_path(cfg.clone(), Some(cfg_path.clone()));
    let h = auth_headers();
    let h: Vec<(&str, &str)> = h.iter().map(|(k, v)| (*k, v.as_str())).collect();

    // No OK result yet: 409.
    let (st, body) = call(&app, req("POST", "/v1/calibration/export", &h, "")).await;
    assert_eq!(st, StatusCode::CONFLICT, "{body}");
    assert_eq!(body["error"], "no_calibration_result");
    assert!(std::fs::read_dir(&dir).unwrap().count() == 1, "nothing written");

    // Axis 1 OK (sign +1), axis 3 OK (sign −1), axis 2 FAILED (left alone).
    let mut snap = Supervisor::new(&cfg).snapshot(Instant::now());
    snap.axes[0].calibration = Some(cal_ok(6000));
    // The result carries the unit that measured it; the snapshot's own
    // device_id (here sim.toml's) is not what gets written.
    assert_ne!(snap.axes[0].device_id, h3cs::types::DeviceId(0x5131_1cdb_d4ba_8d58));
    snap.axes[1].calibration = Some(h3cs::supervisor::CalibrationResult {
        result: h3cs::protocol::CalResultCode::Failed,
        reason: "stall".into(),
        ..cal_ok(0)
    });
    snap.axes[2].calibration = Some(h3cs::supervisor::CalibrationResult { device_id: h3cs::types::DeviceId(0x5131_30ed_a00b_2040), ..cal_ok(-6138) });
    status_tx.send_replace(Arc::new(snap));

    // Overwrite refusal: the names of this second and the next already exist.
    let now = std::time::SystemTime::now();
    let taken: Vec<std::path::PathBuf> = [now, now + Duration::from_secs(1)]
        .iter()
        .map(|t| dir.join(file_name(&cfg.rig_id, &utc_timestamp(*t))))
        .collect();
    for p in &taken {
        std::fs::write(p, "placeholder").unwrap();
    }
    let (st, body) = call(&app, req("POST", "/v1/calibration/export", &h, "")).await;
    assert_eq!(st, StatusCode::CONFLICT, "{body}");
    assert_eq!(body["error"], "exists");
    for p in &taken {
        assert_eq!(std::fs::read_to_string(p).unwrap(), "placeholder", "not overwritten");
        std::fs::remove_file(p).unwrap();
    }

    let (st, body) = call(&app, req("POST", "/v1/calibration/export", &h, "")).await;
    assert_eq!(st, StatusCode::OK, "{body}");
    assert_eq!(body["ok"], true);
    let path = std::path::PathBuf::from(body["path"].as_str().unwrap());
    assert_eq!(path.parent(), Some(dir.as_path()));
    let name = path.file_name().unwrap().to_str().unwrap();
    assert!(name.starts_with("h3-sim.calibration-") && name.ends_with(".toml"), "{name}");
    let axes = body["axes"].as_array().unwrap();
    assert_eq!(axes.len(), 2);
    assert_eq!(axes[0]["axis_id"], 1);
    assert_eq!(axes[0]["device_id"], "51311cdbd4ba8d58");
    assert_eq!((axes[0]["mechanical_min_count"].as_i64(), axes[0]["mechanical_max_count"].as_i64()), (Some(0), Some(6000)));
    assert_eq!((axes[0]["soft_min_count"].as_i64(), axes[0]["soft_max_count"].as_i64()), (Some(300), Some(5700)));
    assert_eq!(
        (axes[0]["count_at_virtual_0"].as_i64(), axes[0]["count_at_virtual_512"].as_i64(), axes[0]["count_at_virtual_1023"].as_i64()),
        (Some(300), Some(3000), Some(5700))
    );
    assert_eq!(axes[1]["axis_id"], 3);
    assert_eq!(axes[1]["device_id"], "513130eda00b2040");
    assert_eq!((axes[1]["mechanical_min_count"].as_i64(), axes[1]["mechanical_max_count"].as_i64()), (Some(-6138), Some(0)));
    assert_eq!(
        (axes[1]["count_at_virtual_0"].as_i64(), axes[1]["count_at_virtual_512"].as_i64(), axes[1]["count_at_virtual_1023"].as_i64()),
        (Some(-300), Some(-3069), Some(-5838)),
        "sign −1: virtual still grows from the P end (0) toward the N end"
    );

    // The file is a valid config with those numbers, the untouched axis, the
    // original comments and a header.
    let text = std::fs::read_to_string(&path).unwrap();
    assert!(text.starts_with("# h3-control-server が校正結果から書き出した設定"), "{text}");
    assert!(text.contains("（UTC）"));
    assert!(text.contains("# axis 1: device_id 51311cdbd4ba8d58 / CAL_RESULT n_end_count 6000 neutral_count 3000 sign +1"));
    assert!(text.contains("axis 1 mechanical_max_count: 2814 -> 6000"), "previous value in the header: {text}");
    assert!(text.contains("calibration_valid = false   # 模擬軸のみ"), "value replaced, comment kept: {text}");
    assert!(text.contains("# 模擬軸 N は selected_axis = N を名乗る"), "other comments kept");
    let out = Config::load(&path).unwrap();
    let a1 = out.axis(h3cs::types::AxisId::new(1).unwrap()).unwrap();
    assert_eq!(a1.device_id, h3cs::types::DeviceId(0x5131_1cdb_d4ba_8d58));
    assert!(!a1.calibration_valid);
    assert_eq!((a1.mechanical_min_count, a1.mechanical_max_count, a1.soft_min_count, a1.soft_max_count), (0, 6000, 300, 5700));
    assert_eq!((a1.count_at_virtual_0, a1.count_at_virtual_512, a1.count_at_virtual_1023), (300, 3000, 5700));
    assert_eq!(a1.decel_zone_count, 300);
    let a2 = out.axis(h3cs::types::AxisId::new(2).unwrap()).unwrap();
    assert_eq!(a2, cfg.axis(h3cs::types::AxisId::new(2).unwrap()).unwrap(), "FAILED axis left as it was");
    let a3 = out.axis(h3cs::types::AxisId::new(3).unwrap()).unwrap();
    assert_eq!((a3.mechanical_min_count, a3.mechanical_max_count, a3.soft_min_count, a3.soft_max_count), (-6138, 0, -5838, -300));
    assert_eq!((a3.count_at_virtual_0, a3.count_at_virtual_512, a3.count_at_virtual_1023), (-300, -3069, -5838));
    assert!(!a3.calibration_valid);
    // Everything but the axis values is the running config.
    assert_eq!(out.rig_id, cfg.rig_id);
    assert_eq!(out.server, cfg.server);
    assert_eq!(out.timing, cfg.timing);
    assert_eq!(out.backend, cfg.backend);

    // An invalid outcome is refused and nothing is written.
    let mut snap = Supervisor::new(&cfg).snapshot(Instant::now());
    snap.axes[0].calibration = Some(cal_ok(100)); // stroke smaller than 2 × decel
    status_tx.send_replace(Arc::new(snap));
    let before = std::fs::read_dir(&dir).unwrap().count();
    let (st, body) = call(&app, req("POST", "/v1/calibration/export", &h, "")).await;
    assert_eq!(st, StatusCode::INTERNAL_SERVER_ERROR, "{body}");
    assert_eq!(body["error"], "invalid_result");
    assert!(body["detail"].as_str().unwrap().contains("soft_min_count"), "{body}");
    assert_eq!(std::fs::read_dir(&dir).unwrap().count(), before);

    // Without a config path (server not started from a file): 500.
    let (app, _rx, tx) = manual_with_path(cfg.clone(), None);
    let mut snap = Supervisor::new(&cfg).snapshot(Instant::now());
    snap.axes[0].calibration = Some(cal_ok(6000));
    tx.send_replace(Arc::new(snap));
    let (st, body) = call(&app, req("POST", "/v1/calibration/export", &h, "")).await;
    assert_eq!(st, StatusCode::INTERNAL_SERVER_ERROR);
    assert_eq!(body["error"], "no_config_path");
    let _ = std::fs::remove_dir_all(&dir);
}

// ------------------------------------------------------------------ motion limits (design 2026-11-01 §5.3)

fn limits_json(session: &str, seq: u64, fields: Value) -> String {
    let mut m = serde_json::json!({"type": "limits", "protocol": 1, "source_session": session, "seq": seq});
    for (k, v) in fields.as_object().unwrap() {
        m[k] = v.clone();
    }
    m.to_string()
}

#[tokio::test]
async fn limits_over_ws_and_rest() {
    let srv = start_sim(sim_cfg()).await;
    let addr = srv.local_addr;
    wait_state(&srv, RigState::ReadyToArm, Duration::from_secs(3)).await;

    // GET: defaults in effect, ceilings from the (simulated) firmware.
    let r = http(addr, "GET", "/v1/limits", &[], None).await;
    assert_eq!(r.status, 200, "{}", r.body);
    let v = r.json();
    assert_eq!(v["effective"]["max_gain"], 0.5);
    assert_eq!(v["effective"]["max_vel_count_s"], 3000);
    assert_eq!(v["ceiling"]["duty_max_percent"], 60);
    assert_eq!(v["ceiling"]["max_vel_count_s"], 6000);
    assert_eq!(v["defaults"]["max_acc_count_s2"], 15000);
    assert_eq!(v["limited"], serde_json::json!([]));

    // WS: raise the gain ceiling, ask for more velocity than the firmware allows.
    let id = new_session(addr).await;
    let mut ws = ws_connect(addr, &id).await;
    ws_send(&mut ws, limits_json(&id, 1, serde_json::json!({"max_gain": 0.8, "max_vel_count_s": 7000}))).await;
    let ack = next_ack(&mut ws).await;
    assert_eq!(ack["accepted"], true, "{ack}");
    assert_eq!(ack["seq"], 1);
    assert_eq!(ack["effective_gain"], 0.8);
    assert_eq!(ack["effective_limits"]["max_gain"], 0.8);
    assert_eq!(ack["effective_limits"]["max_vel_count_s"], 6000);
    assert_eq!(ack["effective_limits"]["max_acc_count_s2"], 15000);
    assert_eq!(ack["limits"], serde_json::json!(["ceiling_clamped"]));
    assert_eq!(ack["fresh"], false, "a limits message never extends freshness");
    // The next pose is clamped to the new ceiling, not the old 0.5.
    ws_send(&mut ws, pose_json(&id, 2, 1, 0.1, 0.0, 0.0, 0.7)).await;
    let ack = next_ack(&mut ws).await;
    assert_eq!(ack["accepted"], true, "{ack}");
    assert_eq!(ack["effective_gain"], 0.7);
    assert_eq!(ack["limits"], serde_json::json!([]));
    assert_eq!(ack["fresh"], true);
    // Invalid value: rejected, seq not consumed.
    ws_send(&mut ws, limits_json(&id, 3, serde_json::json!({"duty_max_percent": 0}))).await;
    let ack = next_ack(&mut ws).await;
    assert_eq!((ack["accepted"].as_bool(), ack["reason"].as_str()), (Some(false), Some("out_of_range")));
    ws_send(&mut ws, limits_json(&id, 3, serde_json::json!({"duty_max_percent": 50}))).await;
    let ack = next_ack(&mut ws).await;
    assert_eq!(ack["accepted"], true, "{ack}");
    assert_eq!(ack["effective_limits"]["duty_max_percent"], 50);
    ws_send(&mut ws, limits_json(&id, 3, serde_json::json!({}))).await;
    assert_eq!(next_ack(&mut ws).await["reason"], "old_seq");
    // Status (WS push and GET) shows the change once the control loop took it.
    let mut st = next_status(&mut ws).await;
    for _ in 0..20 {
        if st["motion_limits"]["effective"]["duty_max_percent"] == 50 {
            break;
        }
        st = next_status(&mut ws).await;
    }
    assert_eq!(st["motion_limits"]["effective"]["max_gain"], 0.8);
    assert_eq!(st["motion_limits"]["effective"]["max_vel_count_s"], 6000);
    assert_eq!(st["motion_limits"]["requested"]["max_vel_count_s"], 7000);
    assert_eq!(st["motion_limits"]["limited"], serde_json::json!(["max_vel_count_s"]));
    // The simulated axes received the new duty cap.
    let mut ok = false;
    for _ in 0..30 {
        let r = http(addr, "GET", "/v1/status", &[], None).await;
        let v = r.json();
        if v["axes"].as_array().unwrap().iter().all(|a| a["fw_limits"]["duty_max_percent"] == 50) {
            ok = true;
            break;
        }
        tokio::time::sleep(Duration::from_millis(50)).await;
    }
    assert!(ok, "fw_limits did not follow SET_LIMITS");

    // REST: token required; applies the same patch rules.
    let bearer = format!("Bearer {TOKEN}");
    let r = http(addr, "POST", "/v1/limits", &[], Some(r#"{"max_acc_count_s2": 1000}"#)).await;
    assert_eq!(r.status, 401);
    let r = http(addr, "POST", "/v1/limits", &[("Authorization", &bearer)], Some("not json")).await;
    assert_eq!(r.status, 400);
    let r = http(addr, "POST", "/v1/limits", &[("Authorization", &bearer)], Some(r#"{"max_vel_count_s": -1}"#)).await;
    assert_eq!(r.status, 400);
    assert_eq!(r.json()["error"], "out_of_range");
    let r = http(addr, "POST", "/v1/limits", &[("Authorization", &bearer)], Some(r#"{"max_acc_count_s2": 1000}"#)).await;
    assert_eq!(r.status, 200, "{}", r.body);
    let v = r.json();
    assert_eq!(v["effective"]["max_acc_count_s2"], 1000);
    assert_eq!(v["effective"]["max_gain"], 0.8, "other fields kept");

    // A new session resets everything to the configured defaults.
    let _id2 = new_session(addr).await;
    let v = http(addr, "GET", "/v1/limits", &[], None).await.json();
    assert_eq!(v["effective"], v["defaults"]);
    assert_eq!(v["requested"]["max_vel_count_s"], 3000);
    let _ = ws.close(None).await;
    srv.shutdown().await;
}

#[tokio::test]
async fn limits_message_is_rejected_during_an_axis_conflict() {
    let cfg = Arc::new(sim_cfg());
    let (admin_tx, _admin_rx) = mpsc::channel::<AdminRequest>(api::ADMIN_QUEUE_LEN);
    let (pose_tx, _pose_rx) = watch::channel(None);
    let (_shutdown_tx, shutdown_rx) = watch::channel(false);
    let mut snap = Supervisor::new(&cfg).snapshot(Instant::now());
    snap.axis_conflict = vec![h3cs::types::AxisId::new(2).unwrap()];
    let (_status_tx, status_rx) = watch::channel(Arc::new(snap));
    let session = Arc::new(Mutex::new(PoseIntake::new()));
    let state = AppState::new(cfg, session, Arc::new(pose_tx), admin_tx, status_rx, TOKEN.into(), shutdown_rx);
    let addr = serve(api::router(state)).await;
    let id = new_session(addr).await;
    let mut ws = ws_connect(addr, &id).await;
    ws_send(&mut ws, limits_json(&id, 1, serde_json::json!({"max_gain": 0.9}))).await;
    let ack = next_ack(&mut ws).await;
    assert_eq!((ack["accepted"].as_bool(), ack["reason"].as_str()), (Some(false), Some("axis_conflict")));
    let v = http(addr, "GET", "/v1/limits", &[], None).await.json();
    assert_eq!(v["requested"]["max_gain"], 0.5, "not applied");
}
