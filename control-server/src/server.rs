//! Wiring (DESIGN.md §1, §2): backends (sim / serial), axis links, the
//! control loop and the HTTP API.
//!
//! # Control loop (one task)
//!
//! Every `planner_period_ms` (interval with `MissedTickBehavior::Skip`):
//! take new links → drain link events (routed by port) → drain admin commands
//! → `Supervisor::tick` with the latest pose → deliver admin messages, then
//! SET_POSITION → telemetry → publish the status snapshot.
//!
//! # Decisions not spelled out in DESIGN.md
//!
//! - Status / Ack / FaultEvent from a port that is not `Supervisor::axis_port`
//!   (a second unit claiming the axis, or any port of an axis in conflict) are
//!   dropped; its `Disconnected` becomes `PortClosed`, which removes it from
//!   the axis's claimants (DESIGN.md §10.3).
//! - A failed send to an axis link latches a fault through `AdminCmd::Fault`
//!   (`RigFault::Operator` with a `link_send_failed:` note; there is no
//!   dedicated variant): always when the admin queue is full, and for a
//!   closed link only while armed (a link dying while disarmed is a plain
//!   disconnect, reported by the link's own `Disconnected`). A failed
//!   HOST_FAULT never re-latches, and nothing re-latches while FAULT_LATCHED.
//! - Shutdown (`RunningServer::shutdown`, or dropping it): DISARM if armed
//!   (controlled, never center), keep ticking until disarmed or
//!   [`SHUTDOWN_DISARM_WAIT`], then close the links. While shutting down only
//!   Stop / Disarm / Fault admin commands are accepted. Signal handling
//!   (SIGINT / SIGTERM) lives in `main.rs` and calls `shutdown()`.
//! - The control task signals "ended" through a drop guard, so a panic in it
//!   also ends `RunningServer::closed()` (main then exits) and GET /v1/status
//!   answers 503 instead of the last snapshot. The MCU leases stop the axes.
//! - Admin commands the API already answered with 503 are skipped unless
//!   [`api::runs_when_abandoned`] (Stop / Disarm / Fault).
//! - Sim backend: axis N's simulator is on a scrambled port (axis 3 on
//!   "sim:1", axis 1 on "sim:2", axis 2 on "sim:3") so mapping never depends
//!   on port order. With `report_selected_axis` (default) it reports
//!   `selected_axis = N` and the assignment comes from that; with it off it
//!   reports 0 and only the configured `device_id` maps it (DESIGN.md §10.3).
//!   Like a serial port, a sim port is reopened [`SERIAL_RETRY`] after its
//!   link closed (e.g. `status_timeout` after a host sleep), on the same
//!   simulated unit (state kept); `RunningServer::sims` holds views of them.
//! - Serial backend: one task per configured port opens it (retry every
//!   [`SERIAL_RETRY`], never fatal) and forwards the link's events. After an
//!   unknown device the port is reopened only once it has disappeared from
//!   the port list; `PortClosed` is sent then.

use std::net::SocketAddr;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use tokio::sync::{mpsc, watch};
use tokio::task::JoinHandle;
use tokio::time::{Instant, Interval, MissedTickBehavior};

use crate::api::{self, lock, AdminRequest, AppState};
use crate::axis_link::{self, LinkHandle, LinkSendError};
use crate::config::{BackendConfig, Config, ConfigError, SerialBackendConfig};
use crate::protocol::HostMsg;
use crate::limits::{self as motion_limits, MotionLimits, SharedLimits};
use crate::session::{AcceptedPose, PoseIntake};
use crate::sim_axis::{sim_detached, SimHandle, SimParams};
use crate::supervisor::{AdminCmd, AdminError, LinkEvent, StatusSnapshot, Supervisor, TickOutput};
use crate::telemetry::{lock_events, shared_event_log, Telemetry, TelemetrySummary};
use crate::types::{AxisId, DeviceId, RigState};

/// Longest wait for DISARMED during shutdown.
pub const SHUTDOWN_DISARM_WAIT: Duration = Duration::from_secs(3);
/// Serial open retry period.
pub const SERIAL_RETRY: Duration = Duration::from_secs(3);
/// Link event queue (all links → control loop).
pub const EVENT_QUEUE_LEN: usize = 1024;
/// At most this many link events are taken per tick.
const MAX_EVENTS_PER_TICK: usize = 1024;
/// Telemetry percentiles are recomputed every this many ticks.
const TELEMETRY_SUMMARY_EVERY: u64 = 10;
/// Sim backend: port index (0-based) → axis id on that port.
const SIM_PORT_AXES: [u8; 3] = [3, 1, 2];

#[derive(Debug, Clone, Default)]
pub struct ServerOptions {
    /// Admin token; overrides the `server.admin_token_env` variable.
    pub admin_token: Option<String>,
    /// Overrides `server.bind` (same loopback rule).
    pub bind_override: Option<SocketAddr>,
    /// The file `cfg` was read from; `POST /v1/calibration/export` writes
    /// next to it (refused with 500 `no_config_path` when unset).
    pub config_path: Option<std::path::PathBuf>,
}

#[derive(Debug, thiserror::Error)]
pub enum ServerError {
    #[error(transparent)]
    Config(#[from] ConfigError),
    #[error("server.bind {0:?} is not an IP:port literal")]
    BadBind(String),
    #[error("refusing to bind non-loopback address {0} (set server.allow_non_loopback)")]
    NonLoopback(SocketAddr),
    #[error("bind {addr}: {source}")]
    Bind { addr: SocketAddr, source: std::io::Error },
}

pub type Result<T> = std::result::Result<T, ServerError>;

/// A started server. Dropping it starts the same shutdown as [`Self::shutdown`]
/// without waiting (and stops the simulators).
pub struct RunningServer {
    pub local_addr: SocketAddr,
    pub admin_token: String,
    /// Sim backend only, sorted by axis id (fault injection in tests): views
    /// of the simulated units, which the sim port tasks run.
    pub sims: Vec<(AxisId, SimHandle)>,
    status_rx: watch::Receiver<Arc<StatusSnapshot>>,
    shutdown_tx: watch::Sender<bool>,
    done_rx: watch::Receiver<bool>,
    control: Option<JoinHandle<()>>,
    http: Option<JoinHandle<()>>,
    port_tasks: Vec<JoinHandle<()>>,
}

impl RunningServer {
    /// Latest published status snapshot.
    pub fn status(&self) -> Arc<StatusSnapshot> {
        self.status_rx.borrow().clone()
    }

    pub fn status_watch(&self) -> watch::Receiver<Arc<StatusSnapshot>> {
        self.status_rx.clone()
    }

    /// Resolves when the control loop or the HTTP server has ended on its own.
    pub async fn closed(&self) {
        let mut rx = self.done_rx.clone();
        let _ = rx.wait_for(|d| *d).await;
    }

    /// Graceful shutdown: DISARM armed axes (no center), close links and the
    /// HTTP server, then return.
    pub async fn shutdown(mut self) {
        let _ = self.shutdown_tx.send(true);
        if let Some(c) = self.control.take() {
            let abort = c.abort_handle();
            if tokio::time::timeout(SHUTDOWN_DISARM_WAIT + Duration::from_secs(2), c).await.is_err() {
                tracing::warn!("control loop did not stop in time; aborted");
                abort.abort();
            }
        }
        if let Some(h) = self.http.take() {
            let abort = h.abort_handle();
            if tokio::time::timeout(Duration::from_secs(2), h).await.is_err() {
                abort.abort();
            }
        }
        for t in self.port_tasks.drain(..) {
            t.abort();
        }
    }
}

impl Drop for RunningServer {
    fn drop(&mut self) {
        let _ = self.shutdown_tx.send(true);
        for t in &self.port_tasks {
            t.abort();
        }
    }
}

/// Validate, bind, start the backend, the control loop and the API.
pub async fn start(cfg: Config, opts: ServerOptions) -> Result<RunningServer> {
    cfg.validate()?;
    let addr = match opts.bind_override {
        Some(a) => a,
        None => cfg.server.bind.parse().map_err(|_| ServerError::BadBind(cfg.server.bind.clone()))?,
    };
    if !addr.ip().is_loopback() && !cfg.server.allow_non_loopback {
        return Err(ServerError::NonLoopback(addr));
    }
    let admin_token = resolve_token(&cfg, opts.admin_token);
    let listener = tokio::net::TcpListener::bind(addr).await.map_err(|source| ServerError::Bind { addr, source })?;
    let local_addr = listener.local_addr().map_err(|source| ServerError::Bind { addr, source })?;
    let cfg = Arc::new(cfg);

    let (events_tx, events_rx) = mpsc::channel(EVENT_QUEUE_LEN);
    let (links_tx, links_rx) = mpsc::channel::<LinkHandle>(16);
    let (admin_tx, admin_rx) = mpsc::channel::<AdminRequest>(api::ADMIN_QUEUE_LEN);
    let (pose_tx, pose_rx) = watch::channel::<Option<AcceptedPose>>(None);
    let (shutdown_tx, shutdown_rx) = watch::channel(false);
    let (done_tx, done_rx) = watch::channel(false);
    let done_tx = Arc::new(done_tx);
    let session = Arc::new(Mutex::new(PoseIntake::new()));
    let limits = motion_limits::shared(MotionLimits::from_config(&cfg.limits));

    let sup = Supervisor::new(&cfg);
    tracing::info!(session_epoch = sup.session_epoch(), "supervisor ready");
    let event_log = shared_event_log(Instant::now());
    let (status_tx, status_rx) = watch::channel(Arc::new(sup.snapshot(Instant::now())));

    // Backend. Every link (sim or serial) reaches the control loop through
    // `links_tx` from a port task that reopens it when it closes.
    let links = Vec::new();
    let mut sims = Vec::new();
    let mut port_tasks = Vec::new();
    // One simulated unit for axis `a` on `port` (sim backend, or a serial
    // backend's `sim_axes`). The model lives in the port task; `sims` keeps
    // a view of it for inspection / fault injection.
    let mut spawn_sim = |a: &crate::config::AxisConfig, sim: &crate::config::SimBackendConfig, port: String| {
        let boot_id = rand::random::<u32>().max(1);
        let selected_axis = if sim.report_selected_axis { a.axis_id.get() } else { 0 };
        // A unit always has a device_id; without a configured one the
        // simulator uses the documented placeholder pattern 5131…0N.
        let device_id = if a.device_id.is_unassigned() {
            DeviceId(0x5131_0000_0000_0000 + u64::from(a.axis_id.get()))
        } else {
            a.device_id
        };
        let handle = sim_detached(SimParams::from_backend(sim, device_id, boot_id, selected_axis));
        sims.push((a.axis_id, handle.view()));
        port_tasks.push(tokio::spawn(sim_port_task(
            port,
            handle,
            cfg.clone(),
            events_tx.clone(),
            links_tx.clone(),
            shutdown_rx.clone(),
        )));
    };
    match &cfg.backend {
        BackendConfig::Sim(sim) => {
            for (k, n) in SIM_PORT_AXES.iter().enumerate() {
                let Some(a) = AxisId::new(*n).and_then(|id| cfg.axis(id)) else { continue };
                spawn_sim(a, sim, format!("sim:{}", k + 1));
            }
            sims.sort_by_key(|(a, _)| *a);
        }
        BackendConfig::Serial(serial) => {
            // Simulated stand-ins first, so a real unit selecting the same
            // axis shows up as a conflict rather than taking it over.
            for id in &serial.sim_axes {
                let Some(a) = cfg.axis(*id) else { continue };
                spawn_sim(a, &serial.sim, format!("sim:{}", id.get()));
            }
            sims.sort_by_key(|(a, _)| *a);
            for port in &serial.ports {
                port_tasks.push(tokio::spawn(serial_port_task(
                    port.clone(),
                    serial.clone(),
                    cfg.clone(),
                    events_tx.clone(),
                    links_tx.clone(),
                    shutdown_rx.clone(),
                )));
            }
        }
    }
    drop(events_tx);
    drop(links_tx);

    let period = Duration::from_millis(cfg.timing.planner_period_ms.max(1));
    let control = ControlLoop {
        sup,
        links,
        links_rx,
        events_rx,
        admin_rx,
        pose_rx,
        session: session.clone(),
        limits: limits.clone(),
        status_tx,
        telemetry: Telemetry::with_event_log(event_log.clone()),
        telemetry_summary: None,
        period,
        shutting_down: false,
        last_state: RigState::Offline,
        last_pose_seq: None,
    };
    let control = {
        let done = DoneGuard(done_tx.clone());
        let sd = shutdown_rx.clone();
        tokio::spawn(async move {
            // Dropped on return and on panic unwinding alike.
            let _done = done;
            control.run(sd).await;
        })
    };

    let state = AppState::new(
        cfg.clone(),
        session,
        Arc::new(pose_tx),
        admin_tx,
        status_rx.clone(),
        admin_token.clone(),
        shutdown_rx.clone(),
    )
    .with_limits(limits)
    .with_event_log(event_log)
    .with_config_path(opts.config_path.clone());
    let app = api::router(state);
    let http = {
        let mut sd = shutdown_rx.clone();
        tokio::spawn(async move {
            let graceful = async move {
                let _ = sd.wait_for(|v| *v).await;
            };
            let app = app.into_make_service_with_connect_info::<SocketAddr>();
            if let Err(e) = axum::serve(listener, app).with_graceful_shutdown(graceful).await {
                tracing::error!("http server: {e}");
            }
            let _ = done_tx.send(true);
        })
    };
    tracing::info!(%local_addr, "listening");

    Ok(RunningServer {
        local_addr,
        admin_token,
        sims,
        status_rx,
        shutdown_tx,
        done_rx,
        control: Some(control),
        http: Some(http),
        port_tasks,
    })
}

/// Sends `true` on drop: "this task has ended", however it ended.
pub(crate) struct DoneGuard(pub(crate) Arc<watch::Sender<bool>>);

impl Drop for DoneGuard {
    fn drop(&mut self) {
        if std::thread::panicking() {
            tracing::error!("control loop panicked; the server is no longer controlling the rig");
        }
        let _ = self.0.send(true);
    }
}

/// The admin token, or an empty string when `server.require_admin_token` is
/// off and none was given (admin routes then need no Authorization header).
fn resolve_token(cfg: &Config, given: Option<String>) -> String {
    if let Some(t) = given.filter(|t| !t.is_empty()) {
        return t;
    }
    let var = &cfg.server.admin_token_env;
    if !cfg.server.require_admin_token {
        return match std::env::var(var) {
            Ok(t) if !t.trim().is_empty() => t.trim().to_string(),
            _ => {
                tracing::info!("admin token not required (server.require_admin_token = false, loopback only)");
                String::new()
            }
        };
    }
    match std::env::var(var) {
        Ok(t) if !t.trim().is_empty() => t.trim().to_string(),
        _ => {
            let t = format!("{:032x}", rand::random::<u128>());
            eprintln!("h3-control-server: {var} is not set; generated admin token for this run: {t}");
            t
        }
    }
}

/// The control-loop ticker: missed ticks are skipped, never burst.
pub(crate) fn make_interval(period: Duration) -> Interval {
    let mut iv = tokio::time::interval(period);
    iv.set_missed_tick_behavior(MissedTickBehavior::Skip);
    iv
}

// ------------------------------------------------------------------ control loop

struct ControlLoop {
    sup: Supervisor,
    links: Vec<LinkHandle>,
    links_rx: mpsc::Receiver<LinkHandle>,
    events_rx: mpsc::Receiver<(String, LinkEvent)>,
    admin_rx: mpsc::Receiver<AdminRequest>,
    pose_rx: watch::Receiver<Option<AcceptedPose>>,
    session: Arc<Mutex<PoseIntake>>,
    /// Requested motion limits (written by the API).
    limits: SharedLimits,
    status_tx: watch::Sender<Arc<StatusSnapshot>>,
    telemetry: Telemetry,
    telemetry_summary: Option<serde_json::Value>,
    period: Duration,
    shutting_down: bool,
    last_state: RigState,
    last_pose_seq: Option<u64>,
}

impl ControlLoop {
    async fn run(mut self, mut shutdown: watch::Receiver<bool>) {
        let mut iv = make_interval(self.period);
        let mut last: Option<Instant> = None;
        loop {
            tokio::select! {
                biased;
                r = shutdown.changed() => {
                    if r.is_err() || *shutdown.borrow() {
                        break;
                    }
                    continue;
                }
                _ = iv.tick() => {}
            }
            let now = Instant::now();
            let dt = last.map(|l| now.saturating_duration_since(l));
            last = Some(now);
            self.step(now, dt);
        }

        // Graceful: controlled DISARM (never center), then close the links.
        self.shutting_down = true;
        let start = Instant::now();
        if self.sup.is_armed() {
            tracing::info!("shutdown: disarming");
            if let Err(e) = self.sup.admin(AdminCmd::Disarm, start) {
                tracing::warn!("shutdown disarm refused: {e}");
            }
        }
        while self.sup.is_armed() && start.elapsed() < SHUTDOWN_DISARM_WAIT {
            iv.tick().await;
            let now = Instant::now();
            let dt = last.map(|l| now.saturating_duration_since(l));
            last = Some(now);
            self.step(now, dt);
        }
        if self.sup.is_armed() {
            tracing::warn!(state = ?self.sup.state(), "shutdown: axes not disarmed in time (MCU deadlines apply)");
        }
        self.step(Instant::now(), None);
        // Answer anything still queued so callers do not wait for the timeout.
        self.admin_rx.close();
        while let Ok((_, reply)) = self.admin_rx.try_recv() {
            let _ = reply.send(Err(shutting_down_error()));
        }
        for l in self.links.drain(..) {
            let _ = tokio::time::timeout(Duration::from_millis(500), l.shutdown()).await;
        }
        tracing::info!("control loop stopped");
    }

    /// One tick. `dt` is the measured time since the previous tick.
    fn step(&mut self, now: Instant, dt: Option<Duration>) {
        self.take_links();
        self.drain_events(now);
        self.drain_admin(now);

        let changed = self.pose_rx.has_changed().unwrap_or(false);
        let pose = if self.shutting_down { None } else { self.pose_rx.borrow_and_update().clone() };
        if changed {
            if let Some(p) = &pose {
                if self.last_pose_seq != Some(p.seq) {
                    self.last_pose_seq = Some(p.seq);
                    self.telemetry.pose_rx_to_tick_ms.record_duration(now.saturating_duration_since(p.received_at));
                }
            }
        }

        let dt_s = dt.map_or(self.period, |d| d.min(2 * self.period)).as_secs_f64();
        self.sup.set_limits_request(motion_limits::read(&self.limits));
        let out = self.sup.tick(now, dt_s, pose.as_ref());
        self.dispatch(out, now);
        self.record_telemetry(now, dt);
        self.publish(now);
    }

    fn take_links(&mut self) {
        while let Ok(link) = self.links_rx.try_recv() {
            self.links.retain(|l| l.port != link.port);
            self.links.push(link);
        }
    }

    fn drain_events(&mut self, now: Instant) {
        for _ in 0..MAX_EVENTS_PER_TICK {
            let Ok((port, ev)) = self.events_rx.try_recv() else { break };
            if let Some(ev) = route_event(&self.sup, port, ev) {
                self.sup.on_link_event(ev, now);
            }
        }
    }

    fn drain_admin(&mut self, now: Instant) {
        while let Ok((cmd, reply)) = self.admin_rx.try_recv() {
            let allowed = api::runs_when_abandoned(&cmd);
            // Claim first: once taken, the API waits for this answer.
            if !reply.claim() && !allowed {
                tracing::warn!(?cmd, "admin command dropped: requester already got 503");
                continue;
            }
            let r = if self.shutting_down && !allowed {
                Err(shutting_down_error())
            } else {
                self.sup.admin(cmd, now)
            };
            let _ = reply.send(r);
        }
    }

    /// Deliver admin messages, then SET_POSITION, per axis (order matters for START).
    fn dispatch(&mut self, out: TickOutput, now: Instant) {
        let TickOutput { admin_msgs, set_position } = out;
        let mut failure: Option<String> = None;
        for (axis, msgs) in AxisId::ALL.into_iter().zip(admin_msgs) {
            let link = self
                .sup
                .axis_port(axis)
                .and_then(|p| self.links.iter().find(|l| l.port == p));
            let Some(link) = link else {
                if !msgs.is_empty() {
                    tracing::debug!(%axis, n = msgs.len(), "no link; messages dropped");
                }
                continue;
            };
            for msg in msgs {
                let host_fault = matches!(msg, HostMsg::HostFault { .. });
                if let Err(e) = link.send_admin(msg) {
                    if !host_fault && self.send_error_is_fault(e) {
                        failure.get_or_insert_with(|| format!("link_send_failed: {axis} admin {e}"));
                    } else {
                        tracing::debug!(%axis, "admin send failed: {e}");
                    }
                }
            }
            if failure.is_some() {
                continue;
            }
            if let Some(sp) = set_position.as_ref().and_then(|s| s.get(axis.index())) {
                if let Err(e) = link.set_position(*sp) {
                    if self.send_error_is_fault(e) {
                        failure.get_or_insert_with(|| format!("link_send_failed: {axis} set_position {e}"));
                    }
                }
            }
        }
        if let Some(note) = failure {
            if self.sup.state() != RigState::FaultLatched {
                tracing::error!("{note}; latching fault");
                if let Err(e) = self.sup.admin(AdminCmd::Fault { note }, now) {
                    tracing::error!("fault latch refused: {e}");
                }
            }
        }
    }

    fn send_error_is_fault(&self, e: LinkSendError) -> bool {
        match e {
            LinkSendError::Full | LinkSendError::SetPositionOnAdminQueue => true,
            LinkSendError::Closed | LinkSendError::NotIdentified => self.sup.is_armed(),
        }
    }

    fn record_telemetry(&mut self, now: Instant, dt: Option<Duration>) {
        let t = &mut self.telemetry;
        t.tick_count = t.tick_count.saturating_add(1);
        let late = dt.is_some_and(|d| d > self.period * 3 / 2);
        let slow = now.elapsed() > self.period;
        if let Some(d) = dt {
            t.tick_dt_ms.record_duration(d);
        }
        if late || slow {
            t.tick_overruns = t.tick_overruns.saturating_add(1);
        }
        if self.sup.state() == RigState::Active {
            if let Some(skew) = self.sup.frame_skew() {
                t.frame_skew.record(f64::from(skew));
            }
        }
        let (events, dropped) = self.sup.drain_events();
        if dropped > 0 {
            tracing::warn!(dropped, "supervisor events dropped");
        }
        if dropped > 0 || !events.is_empty() {
            let mut log = lock_events(&t.events);
            log.add_dropped(dropped);
            for e in events {
                match e.kind {
                    "fault" => tracing::error!(detail = %e.detail, "rig fault"),
                    // Logged as "rig state" by note_state.
                    "rig_state" => tracing::debug!(detail = %e.detail, "supervisor event"),
                    kind => tracing::info!(kind, detail = %e.detail, "supervisor event"),
                }
                log.push(e.at, e.kind, e.detail);
            }
        }
        self.note_state();
    }

    fn note_state(&mut self) {
        let state = self.sup.state();
        if state != self.last_state {
            tracing::info!(from = ?self.last_state, to = ?state, "rig state");
            self.last_state = state;
        }
    }

    fn publish(&mut self, now: Instant) {
        let mut snap = self.sup.snapshot(now);
        if let Some(age) = snap.axes.iter().filter_map(|a| a.status_age_ms).reduce(f64::max) {
            self.telemetry.status_age_ms.record(age);
        }
        if self.telemetry_summary.is_none() || self.telemetry.tick_count.is_multiple_of(TELEMETRY_SUMMARY_EVERY) {
            let s: TelemetrySummary = self.telemetry.summary(None);
            self.telemetry_summary = serde_json::to_value(s).ok();
        }
        snap.session = serde_json::to_value(lock(&self.session).snapshot(now)).ok();
        snap.telemetry = self.telemetry_summary.clone();
        self.status_tx.send_replace(Arc::new(snap));
    }
}

fn shutting_down_error() -> AdminError {
    AdminError { error: "shutting_down".into(), blockers: Vec::new() }
}

/// Forward an event unless it comes from a port that is not the axis's link.
fn route_event(sup: &Supervisor, port: String, ev: LinkEvent) -> Option<LinkEvent> {
    let is_primary = |axis: AxisId| sup.axis_port(axis) == Some(port.as_str());
    match ev {
        LinkEvent::Status { axis, .. }
        | LinkEvent::Ack { axis, .. }
        | LinkEvent::FaultEvent { axis, .. }
        | LinkEvent::CalResult { axis, .. } => is_primary(axis).then_some(ev),
        LinkEvent::Disconnected { axis, .. } if !is_primary(axis) => Some(LinkEvent::PortClosed { port }),
        ev => Some(ev),
    }
}

// ------------------------------------------------------------------ sim

/// Run one simulated unit on `port` until shutdown, reopening its link after
/// it closes — the sim counterpart of [`serial_port_task`] (Issue #15: after a
/// host sleep every link ends with `status_timeout`; the real ports were
/// reopened, `sim:N` stayed OFFLINE). The [`SimMcu`](crate::sim_axis::SimMcu)
/// is kept: same boot_id, reference, position and clock, so the reopened link
/// identifies the same unit, as a real one would. A unit the server cannot
/// map (`UnknownDevice`) is not reopened: its identity never changes.
async fn sim_port_task(
    port: String,
    mut sim: SimHandle,
    cfg: Arc<Config>,
    events: mpsc::Sender<(String, LinkEvent)>,
    links: mpsc::Sender<LinkHandle>,
    shutdown: watch::Receiver<bool>,
) {
    let mut reopened = false;
    loop {
        if *shutdown.borrow() {
            return;
        }
        let host = sim.attach();
        if reopened {
            tracing::info!(%port, "sim port reopened");
        }
        reopened = true;
        let (tx, mut rx) = mpsc::channel(256);
        let handle = axis_link::spawn_link(host, port.clone(), cfg.clone(), tx);
        if links.send(handle).await.is_err() {
            return;
        }
        // Forward until the link task ends (it drops its sender).
        let mut unknown = false;
        while let Some((p, ev)) = rx.recv().await {
            unknown |= matches!(ev, LinkEvent::UnknownDevice { .. });
            if events.send((p, ev)).await.is_err() {
                return;
            }
        }
        if unknown || sleep_or_shutdown(SERIAL_RETRY, &shutdown).await {
            return;
        }
    }
}

// ------------------------------------------------------------------ serial

async fn sleep_or_shutdown(d: Duration, shutdown: &watch::Receiver<bool>) -> bool {
    // A dropped sender also means shutdown.
    let mut wait = shutdown.clone();
    tokio::select! {
        _ = tokio::time::sleep(d) => *shutdown.borrow(),
        _ = wait.wait_for(|v| *v) => true,
    }
}

async fn port_present(port: &str) -> bool {
    let port = port.to_string();
    tokio::task::spawn_blocking(move || axis_link::list_serial_ports().contains(&port))
        .await
        .unwrap_or(true)
}

/// Open / reopen one serial port forever (until shutdown).
async fn serial_port_task(
    port: String,
    serial: SerialBackendConfig,
    cfg: Arc<Config>,
    events: mpsc::Sender<(String, LinkEvent)>,
    links: mpsc::Sender<LinkHandle>,
    shutdown: watch::Receiver<bool>,
) {
    loop {
        if *shutdown.borrow() {
            return;
        }
        let stream = match axis_link::open_serial(&port, serial.baud).await {
            Ok(s) => s,
            Err(e) => {
                tracing::warn!(%port, "open failed: {e}; retrying in {}s", SERIAL_RETRY.as_secs());
                if sleep_or_shutdown(SERIAL_RETRY, &shutdown).await {
                    return;
                }
                continue;
            }
        };
        tracing::info!(%port, "port opened");
        let (tx, mut rx) = mpsc::channel(256);
        let handle = axis_link::spawn_link(stream, port.clone(), cfg.clone(), tx);
        if links.send(handle).await.is_err() {
            return;
        }
        // Forward until the link task ends (it drops its sender).
        let mut unknown = false;
        while let Some((p, ev)) = rx.recv().await {
            unknown |= matches!(ev, LinkEvent::UnknownDevice { .. });
            if events.send((p, ev)).await.is_err() {
                return;
            }
        }
        if unknown {
            // Do not keep re-probing a device that is not ours.
            loop {
                if sleep_or_shutdown(SERIAL_RETRY, &shutdown).await {
                    return;
                }
                if !port_present(&port).await {
                    break;
                }
            }
            if events.send((port.clone(), LinkEvent::PortClosed { port: port.clone() })).await.is_err() {
                return;
            }
        } else if sleep_or_shutdown(SERIAL_RETRY, &shutdown).await {
            return;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn control_interval_skips_missed_ticks() {
        let iv = make_interval(Duration::from_millis(10));
        assert_eq!(iv.missed_tick_behavior(), MissedTickBehavior::Skip);
        assert_eq!(iv.period(), Duration::from_millis(10));
    }

    #[tokio::test]
    async fn done_is_signalled_when_the_control_task_panics() {
        let (tx, mut rx) = watch::channel(false);
        let guard = DoneGuard(Arc::new(tx));
        let task = tokio::spawn(async move {
            let _done = guard;
            panic!("simulated control-loop bug");
        });
        assert!(task.await.is_err());
        let r = tokio::time::timeout(Duration::from_secs(1), rx.wait_for(|d| *d)).await;
        assert!(matches!(r, Ok(Ok(_))), "closed() would never resolve");
    }

    #[tokio::test(start_paused = true)]
    async fn control_interval_does_not_burst_after_a_stall() {
        let mut iv = make_interval(Duration::from_millis(10));
        iv.tick().await;
        tokio::time::advance(Duration::from_millis(55)).await;
        iv.tick().await; // the late tick fires once
        let t = Instant::now();
        iv.tick().await;
        assert!(t.elapsed() >= Duration::from_millis(4), "missed ticks must not burst");
    }
}
