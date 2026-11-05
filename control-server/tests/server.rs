//! server: end-to-end with the sim backend (DESIGN.md §9 "結合（sim）").
//! Real time on purpose: the WS client talks over loopback TCP, which a
//! paused clock would race. Each test stays well under 10 s.

mod common;

use std::time::Duration;

use common::*;
use futures_util::{SinkExt, StreamExt};
use h3cs::calibration;
use h3cs::compat;
use h3cs::config::Config;
use h3cs::protocol::msg_type;
use h3cs::server::{self, ServerError, ServerOptions};
use h3cs::sim_axis::SimHandle;
use h3cs::types::{AxisId, Counts, FaultCode, McuState, Pose, RigFault, RigState};
use serde_json::Value;
use tokio::sync::oneshot;
use tokio::task::JoinHandle;
use tokio_tungstenite::tungstenite::Message;

const POSE: Pose = Pose { pitch: 0.5, roll: 0.2, yaw: -0.3 };
const GAIN: f64 = 0.35;
const TOL: Counts = 5;
const S: Duration = Duration::from_secs(1);

fn fast_cfg() -> Config {
    let mut c = sim_cfg();
    c.timing.stop_hold_ms = 300;
    c
}

fn sim(srv: &h3cs::server::RunningServer, axis: u8) -> &SimHandle {
    let id = AxisId::new(axis).unwrap();
    &srv.sims.iter().find(|(a, _)| *a == id).expect("sim axis").1
}

/// Where each axis must end up for `POSE` (same path as the supervisor).
fn expected_counts(cfg: &Config) -> [Counts; 3] {
    let checked = compat::check_pose(POSE, GAIN, cfg.limits.max_gain, cfg.game.clamp_out_of_range).unwrap();
    let (virt, _) = compat::saturate(
        compat::mix(checked.pose, checked.effective_gain),
        cfg.game.virtual_min,
        cfg.game.virtual_max,
    );
    let cal = calibration::from_config(cfg);
    std::array::from_fn(|i| cal[i].clamp_soft(cal[i].virtual_to_count_f(virt[i])).0.round() as Counts)
}

/// Background 30 Hz pose sender with an advancing frame_seq; also drains the
/// socket and keeps rejected acks.
struct Streamer {
    stop: oneshot::Sender<()>,
    task: JoinHandle<Vec<Value>>,
}

impl Streamer {
    fn start(ws: Ws, session: String, first_seq: u64, pose: Pose, gain: f64) -> Self {
        let (stop, mut stop_rx) = oneshot::channel::<()>();
        let task = tokio::spawn(async move {
            let (mut tx, mut rx) = ws.split();
            let mut rejected = Vec::new();
            let mut iv = tokio::time::interval(Duration::from_millis(33));
            let mut seq = first_seq;
            loop {
                tokio::select! {
                    _ = &mut stop_rx => break,
                    _ = iv.tick() => {
                        let text = pose_json(&session, seq, seq, pose.pitch, pose.roll, pose.yaw, gain);
                        seq += 1;
                        if tx.send(Message::Text(text.into())).await.is_err() {
                            break;
                        }
                    }
                    m = rx.next() => match m {
                        Some(Ok(Message::Text(t))) => {
                            let v: Value = serde_json::from_str(t.as_str()).unwrap_or(Value::Null);
                            if v["type"] == "ack" && v["accepted"] != true {
                                rejected.push(v);
                            }
                        }
                        Some(Ok(_)) => {}
                        _ => break,
                    },
                }
            }
            let _ = tx.close().await;
            rejected
        });
        Self { stop, task }
    }

    async fn stop(self) -> Vec<Value> {
        let _ = self.stop.send(());
        self.task.await.unwrap()
    }
}

/// Server up, all axes READY_TO_ARM, poses streaming, armed and ACTIVE.
async fn active_rig(cfg: Config) -> (h3cs::server::RunningServer, Streamer) {
    let srv = start_sim(cfg).await;
    let streamer = activate(&srv).await;
    (srv, streamer)
}

/// READY_TO_ARM → session + 30 Hz poses → ARM → START → ACTIVE.
async fn activate(srv: &h3cs::server::RunningServer) -> Streamer {
    wait_state(srv, RigState::ReadyToArm, 3 * S).await;
    let id = new_session(srv.local_addr).await;
    let ws = ws_connect(srv.local_addr, &id).await;
    let streamer = Streamer::start(ws, id, 1, POSE, GAIN);
    wait_status(srv, S, "fresh session", |s| s.session.as_ref().is_some_and(|v| v["fresh"] == true)).await;

    let r = admin(srv.local_addr, "/v1/arm").await;
    assert_eq!(r.status, 200, "arm: {r:?}");
    wait_state(srv, RigState::Holding, 3 * S).await;
    let r = admin(srv.local_addr, "/v1/start").await;
    assert_eq!(r.status, 200, "start: {r:?}");
    wait_state(srv, RigState::Active, 2 * S).await;
    streamer
}

async fn wait_converged(srv: &h3cs::server::RunningServer, expected: [Counts; 3]) {
    let deadline = tokio::time::Instant::now() + 3 * S;
    loop {
        let actual: Vec<Counts> = (1..=3).map(|a| sim(srv, a).snapshot().actual_count).collect();
        if actual.iter().zip(expected).all(|(a, e)| (a - e).abs() <= TOL) {
            return;
        }
        assert!(
            tokio::time::Instant::now() < deadline,
            "no convergence: actual {actual:?} expected {expected:?}, rig {:?}",
            srv.status().rig_state
        );
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
}

#[tokio::test]
async fn happy_path_follows_game_then_stops_and_brakes_without_centering() {
    let cfg = fast_cfg();
    let expected = expected_counts(&cfg);
    assert!(expected[0] > 400 && expected[1] < -30 && expected[2] < -30, "test pose is off-neutral: {expected:?}");
    let (srv, streamer) = active_rig(cfg).await;

    wait_converged(&srv, expected).await;
    let st = srv.status();
    assert_eq!(st.rig_state, RigState::Active);
    assert_eq!(st.target_counts, expected, "planner target = calibration(mix(pose))");
    for a in &st.axes {
        assert_eq!(a.mcu_state, Some(McuState::Active), "{a:?}");
        assert_eq!(a.fault_code, Some(FaultCode::None));
    }
    let tel = st.telemetry.as_ref().unwrap();
    assert!(tel["pose_rx_to_tick_ms"]["count"].as_u64().unwrap() > 0, "{tel}");

    // Game stops sending: coordinated stop, hold, brake. No center.
    let rejected = streamer.stop().await;
    assert!(rejected.is_empty(), "every streamed pose accepted: {rejected:?}");
    wait_state(&srv, RigState::Stopping, S).await;
    let st = wait_state(&srv, RigState::BrakedDisarmed, 3 * S).await;
    assert!(!st.armed);
    for a in 1..=3u8 {
        let s = sim(&srv, a).snapshot();
        assert_eq!(s.state, McuState::Disarmed, "axis {a}");
        let e = expected[usize::from(a - 1)];
        assert!((s.actual_count - e).abs() <= TOL, "axis {a} stayed at {e}, is {}", s.actual_count);
    }
    // Still there a bit later: nothing moves it back to neutral.
    tokio::time::sleep(Duration::from_millis(300)).await;
    let s1 = sim(&srv, 1).snapshot();
    assert!((s1.actual_count - expected[0]).abs() <= TOL && s1.state == McuState::Disarmed);
    assert_eq!(srv.status().rig_state, RigState::BrakedDisarmed);
    srv.shutdown().await;
}

/// Issue #15: a simulated axis whose link closed with `status_timeout` (the
/// 2026-11-02 00:58 host sleep: the real ports were reopened, `sim:3` stayed
/// OFFLINE) is reopened like a serial port, on the same simulated unit.
#[tokio::test]
async fn sim_axis_link_is_reopened_after_a_status_timeout() {
    let srv = start_sim(sim_cfg()).await;
    let st = wait_state(&srv, RigState::ReadyToArm, 3 * S).await;
    assert_eq!(st.axes[1].port.as_deref(), Some("sim:3"));
    sim(&srv, 2).nudge_actual(123);
    let before = sim(&srv, 2).snapshot();
    // STATUS stops (as if the host slept past the watchdog): the link closes.
    sim(&srv, 2).set_freeze_status(true);
    let st = wait_status(&srv, 2 * S, "axis 2 disconnected", |s| !s.axes[1].connected).await;
    assert_eq!(st.rig_state, RigState::Offline, "{:?}", st.blockers);
    assert!(st.fault.is_none(), "not armed: a disconnect is no fault");
    sim(&srv, 2).set_freeze_status(false);
    // Reopened after SERIAL_RETRY on the same unit (boot_id, position kept),
    // re-identified and usable again.
    let st = wait_status(&srv, server::SERIAL_RETRY + 3 * S, "axis 2 reconnected", |s| {
        s.axes[1].connected && s.axes[1].mcu_state == Some(McuState::Disarmed)
    })
    .await;
    assert_eq!(st.axes[1].port.as_deref(), Some("sim:3"));
    assert_eq!(st.axes[1].boot_id, Some(before.boot_id), "same unit, not a reboot");
    let after = sim(&srv, 2).snapshot();
    assert_eq!((after.boot_id, after.actual_count, after.referenced), (before.boot_id, before.actual_count, before.referenced));
    wait_state(&srv, RigState::ReadyToArm, 3 * S).await;
    // The model ticks on the new link: STATUS keeps arriving and reflects a move.
    sim(&srv, 2).nudge_actual(50);
    wait_status(&srv, S, "moved position reported", |s| s.axes[1].actual_count == Some(before.actual_count + 50)).await;
    srv.shutdown().await;
}

#[tokio::test]
async fn frozen_status_on_one_axis_faults_the_whole_rig() {
    let (srv, streamer) = active_rig(fast_cfg()).await;
    tokio::time::sleep(Duration::from_millis(200)).await;
    sim(&srv, 2).set_freeze_status(true);

    let st = wait_state(&srv, RigState::FaultLatched, S).await;
    // Frame skew is measured per STATUS arrival, so a silent axis is reported
    // by the STATUS timeout, not as skew.
    assert!(
        matches!(st.fault, Some(RigFault::AxisDisconnected { axis }) if axis.get() == 2),
        "{:?}",
        st.fault
    );
    for a in [1u8, 3] {
        let deadline = tokio::time::Instant::now() + S;
        loop {
            let s = sim(&srv, a).snapshot();
            if s.state == McuState::Fault && s.fault_code == FaultCode::HostFault {
                break;
            }
            assert!(tokio::time::Instant::now() < deadline, "axis {a} not HOST_FAULT: {s:?}");
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    }
    let st = wait_status(&srv, S, "HOST_FAULT reported", |s| {
        [0, 2].iter().all(|&i| s.axes[i].fault_code == Some(FaultCode::HostFault))
    })
    .await;
    wait_status(&srv, S, "axis 2 disconnected", |s| !s.axes[1].connected).await;
    assert_eq!(st.rig_state, RigState::FaultLatched);
    // The game keeps streaming; nothing restarts.
    tokio::time::sleep(Duration::from_millis(300)).await;
    assert_eq!(srv.status().rig_state, RigState::FaultLatched);
    let r = admin(srv.local_addr, "/v1/start").await;
    assert_eq!(r.status, 409);
    streamer.stop().await;
    srv.shutdown().await;
}

#[tokio::test]
async fn server_start_neither_arms_nor_moves() {
    let srv = start_sim(sim_cfg()).await;
    let initial: Vec<_> = (1..=3).map(|a| sim(&srv, a).snapshot().actual_count).collect();
    let t0 = tokio::time::Instant::now();
    let mut ready = false;
    while t0.elapsed() < S {
        for a in 1..=3u8 {
            let s = sim(&srv, a).snapshot();
            assert_eq!(s.state, McuState::Disarmed, "axis {a} at {:?}", t0.elapsed());
            assert_eq!(s.actual_count, initial[usize::from(a - 1)], "axis {a} moved");
        }
        let st = srv.status();
        assert!(!st.armed, "{:?}", st.rig_state);
        ready |= st.rig_state == RigState::ReadyToArm;
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    assert!(ready, "connected and configured, but never armed");
    // Mapping is by device_id, not by port order (sim ports are scrambled).
    let st = srv.status();
    let ports: Vec<_> = st.axes.iter().map(|a| a.port.clone().unwrap_or_default()).collect();
    assert_eq!(ports, ["sim:2", "sim:3", "sim:1"]);
    let cfg = sim_cfg();
    for ax in &st.axes {
        assert_eq!(Some(ax.device_id), cfg.axis(ax.axis_id).map(|a| a.device_id));
        assert!(ax.configured && ax.referenced, "{ax:?}");
    }
    srv.shutdown().await;
}

#[tokio::test]
async fn reconnect_needs_new_session_and_old_seq_is_rejected() {
    let srv = start_sim(sim_cfg()).await;
    let addr = srv.local_addr;
    let a = new_session(addr).await;
    let mut ws = ws_connect(addr, &a).await;
    for seq in 1..=5 {
        ws_send(&mut ws, pose_json(&a, seq, seq, 0.1, 0.0, 0.0, GAIN)).await;
        assert_eq!(next_ack(&mut ws).await["accepted"], true);
    }
    ws.close(None).await.unwrap();

    // Reconnect within the same session: seq must keep increasing.
    let mut ws = ws_connect(addr, &a).await;
    ws_send(&mut ws, pose_json(&a, 3, 6, 0.1, 0.0, 0.0, GAIN)).await;
    assert_eq!(next_ack(&mut ws).await["reason"], "old_seq");
    ws.close(None).await.unwrap();

    // Client restart: new session; the old session's poses are stale.
    let b = new_session(addr).await;
    let mut ws = ws_connect(addr, &b).await;
    ws_send(&mut ws, pose_json(&a, 100, 100, 0.1, 0.0, 0.0, GAIN)).await;
    assert_eq!(next_ack(&mut ws).await["reason"], "stale_session");
    ws_send(&mut ws, pose_json(&b, 5, 1, 0.1, 0.0, 0.0, GAIN)).await;
    assert_eq!(next_ack(&mut ws).await["accepted"], true, "seq restarts per session");
    ws_send(&mut ws, pose_json(&b, 3, 2, 0.1, 0.0, 0.0, GAIN)).await;
    assert_eq!(next_ack(&mut ws).await["reason"], "old_seq");
    ws_send(&mut ws, pose_json(&b, 6, 0, 0.1, 0.0, 0.0, GAIN)).await;
    assert_eq!(next_ack(&mut ws).await["reason"], "old_frame");
    let st = wait_status(&srv, S, "session b", |s| {
        s.session.as_ref().is_some_and(|v| v["source_session_prefix"] == b[..8] && v["last_seq"] == 5)
    })
    .await;
    assert!(!st.armed, "a session alone never arms");
    srv.shutdown().await;
}

#[tokio::test]
async fn server_restart_uses_fresh_epoch_and_does_not_arm() {
    let s1 = start_sim(sim_cfg()).await;
    let st1 = wait_state(&s1, RigState::ReadyToArm, 3 * S).await;
    s1.shutdown().await;
    let s2 = start_sim(sim_cfg()).await;
    let st2 = wait_state(&s2, RigState::ReadyToArm, 3 * S).await;
    assert_ne!(st1.session_epoch, 0);
    assert_ne!(st2.session_epoch, 0);
    assert_ne!(st1.session_epoch, st2.session_epoch, "epoch is random per start");
    assert!(!st2.armed);
    s2.shutdown().await;
}

#[tokio::test]
async fn shutdown_while_active_disarms_in_place() {
    let cfg = fast_cfg();
    let expected = expected_counts(&cfg);
    let (mut srv, streamer) = active_rig(cfg).await;
    wait_converged(&srv, expected).await;
    let sims = std::mem::take(&mut srv.sims);
    srv.shutdown().await;
    for (axis, h) in &sims {
        let s = h.snapshot();
        assert_eq!(s.state, McuState::Disarmed, "{axis} after shutdown: {s:?}");
        let e = expected[axis.index()];
        assert!((s.actual_count - e).abs() <= TOL + 10, "{axis}: no center on shutdown (at {})", s.actual_count);
    }
    streamer.stop().await;
}

#[tokio::test]
async fn stop_without_token_brakes_and_late_poses_do_not_restart() {
    let cfg = fast_cfg();
    let (srv, streamer) = active_rig(cfg).await;
    let r = http(srv.local_addr, "POST", "/v1/stop", &[], None).await;
    assert_eq!(r.status, 200, "{r:?}");
    assert_eq!(r.json()["ok"], true);
    // Poses keep coming; the rig still goes to BRAKED_DISARMED.
    wait_state(&srv, RigState::BrakedDisarmed, 3 * S).await;
    tokio::time::sleep(Duration::from_millis(200)).await;
    assert_eq!(srv.status().rig_state, RigState::BrakedDisarmed);
    assert!((1..=3).all(|a| sim(&srv, a).state() == McuState::Disarmed));
    streamer.stop().await;
    srv.shutdown().await;
}

#[tokio::test]
async fn non_loopback_bind_is_refused() {
    let r = server::start(
        sim_cfg(),
        ServerOptions { admin_token: Some(TOKEN.into()), bind_override: Some("0.0.0.0:0".parse().unwrap()), config_path: None, serial_ports: None },
    )
    .await;
    assert!(matches!(r, Err(ServerError::NonLoopback(_))), "{:?}", r.err());
}

#[tokio::test]
async fn new_session_during_active_stops_and_never_drives_for_the_new_client() {
    let cfg = fast_cfg();
    let (srv, streamer) = active_rig(cfg).await;
    let mut states = srv.status_watch();
    // Reconnecting client: new session, first pose right away, then 30 Hz.
    let id = new_session(srv.local_addr).await;
    let ws = ws_connect(srv.local_addr, &id).await;
    let streamer2 = Streamer::start(ws, id, 1, POSE, GAIN);
    let deadline = tokio::time::Instant::now() + 3 * S;
    let mut seen = Vec::new();
    loop {
        let s = states.borrow_and_update().rig_state;
        if seen.last() != Some(&s) {
            seen.push(s);
        }
        if s == RigState::BrakedDisarmed {
            break;
        }
        assert!(tokio::time::Instant::now() < deadline, "rig kept going: {seen:?}");
        let _ = tokio::time::timeout(Duration::from_millis(50), states.changed()).await;
    }
    assert!(seen.contains(&RigState::Stopping), "{seen:?}");
    tokio::time::sleep(Duration::from_millis(300)).await;
    assert_eq!(srv.status().rig_state, RigState::BrakedDisarmed, "no drive without a new START");
    // Explicit ARM + START drive for the new session.
    assert_eq!(admin(srv.local_addr, "/v1/arm").await.status, 200);
    wait_state(&srv, RigState::Holding, 3 * S).await;
    assert_eq!(admin(srv.local_addr, "/v1/start").await.status, 200);
    wait_state(&srv, RigState::Active, 2 * S).await;
    let rejected = streamer2.stop().await;
    assert!(rejected.is_empty(), "{rejected:?}");
    let old = streamer.stop().await;
    assert!(old.iter().all(|a| a["reason"] == "stale_session"), "{old:?}");
    srv.shutdown().await;
}

#[tokio::test]
async fn events_before_and_after_a_fault_can_be_read_back() {
    let (srv, streamer) = active_rig(fast_cfg()).await;
    tokio::time::sleep(Duration::from_millis(200)).await;
    sim(&srv, 2).force_fault(FaultCode::FollowingError);
    wait_state(&srv, RigState::FaultLatched, S).await;
    // Let the HOST_FAULT round trip land in the log too.
    wait_status(&srv, S, "HOST_FAULT on the other axes", |s| {
        [0, 2].iter().all(|&i| s.axes[i].fault_code == Some(FaultCode::HostFault))
    })
    .await;
    tokio::time::sleep(Duration::from_millis(100)).await;

    let r = http(srv.local_addr, "GET", "/v1/events?n=500", &[], None).await;
    assert_eq!(r.status, 200, "{r:?}");
    let body = r.json();
    assert_eq!(body["ok"], true);
    assert_eq!(body["dropped"], 0);
    let events = body["events"].as_array().unwrap().clone();
    let pos = |pred: &dyn Fn(&Value) -> bool| events.iter().position(pred);
    let fault = pos(&|e| e["kind"] == "fault").expect("fault event");
    assert_eq!(events[fault]["detail"]["fault"]["axis"], 2, "{}", events[fault]);
    assert_eq!(events[fault]["detail"]["fault"]["code"], "FOLLOWING_ERROR");
    // Before the fault: how the rig got there.
    let arm = pos(&|e| e["kind"] == "admin" && e["detail"]["cmd"] == "Arm" && e["detail"]["ok"] == true);
    let start = pos(&|e| e["kind"] == "admin" && e["detail"]["cmd"] == "Start" && e["detail"]["ok"] == true);
    let active = pos(&|e| e["kind"] == "rig_state" && e["detail"]["to"] == "ACTIVE");
    assert!(arm < start && start < active && active < Some(fault), "{events:#?}");
    // After it: the latch.
    let latched = pos(&|e| e["kind"] == "rig_state" && e["detail"]["to"] == "FAULT_LATCHED");
    assert!(latched > Some(fault), "{events:#?}");
    let ids: Vec<u64> = events.iter().map(|e| e["id"].as_u64().unwrap()).collect();
    assert!(ids.windows(2).all(|w| w[1] == w[0] + 1), "oldest first, no gaps: {ids:?}");

    // Polling with `after`: only what happened since, e.g. the operator's reset.
    let fault_id = ids[fault];
    let r = admin(srv.local_addr, "/v1/fault/reset").await;
    assert_eq!(r.status, 200, "{r:?}");
    let later = http(srv.local_addr, "GET", &format!("/v1/events?after={fault_id}"), &[], None).await.json();
    let later = later["events"].as_array().unwrap().clone();
    assert!(later.iter().all(|e| e["id"].as_u64().unwrap() > fault_id), "{later:#?}");
    assert!(
        later.iter().any(|e| e["kind"] == "admin" && e["detail"]["cmd"] == "FaultReset"),
        "{later:#?}"
    );
    let last2 = http(srv.local_addr, "GET", "/v1/events?n=2", &[], None).await.json();
    assert_eq!(last2["events"].as_array().unwrap().len(), 2);
    streamer.stop().await;
    srv.shutdown().await;
}

// ------------------------------------------------- axis selection (DESIGN §10)

fn no_cross_check(mut cfg: Config, axes: &[u8]) -> Config {
    for a in &mut cfg.axes {
        if axes.contains(&a.axis_id.get()) {
            a.device_id = h3cs::types::DeviceId(0);
        }
    }
    cfg
}

fn has(blockers: &[String], needle: &str) -> bool {
    blockers.iter().any(|b| b.contains(needle))
}

async fn reference(srv: &h3cs::server::RunningServer, axis: u8) {
    let auth = format!("Bearer {TOKEN}");
    let body = format!(r#"{{"axis":{axis},"count":0}}"#);
    let r = http(srv.local_addr, "POST", "/v1/reference", &[("Authorization", &auth)], Some(&body)).await;
    assert_eq!(r.status, 200, "reference axis {axis}: {r:?}");
}

#[tokio::test]
async fn assignment_comes_from_selected_axis_even_on_scrambled_ports() {
    // (a) No device_id anywhere: only the units' selection can map them.
    let cfg = no_cross_check(sim_cfg(), &[1, 2, 3]);
    let srv = start_sim(cfg).await;
    let st = wait_state(&srv, RigState::ReadyToArm, 3 * S).await;
    let ports: Vec<_> = st.axes.iter().map(|a| a.port.clone().unwrap_or_default()).collect();
    assert_eq!(ports, ["sim:2", "sim:3", "sim:1"], "scrambled ports, mapped by selection");
    for (i, a) in st.axes.iter().enumerate() {
        assert_eq!(a.assignment, Some("selected"), "{a:?}");
        assert_eq!(a.device_id, h3cs::types::DeviceId(0x5131_0000_0000_0001 + i as u64), "connected unit's id");
        assert!(a.expected_device_id.is_unassigned());
        assert_eq!(a.claimants.len(), 1);
        assert!(!a.reference_stale);
    }
    assert!(st.axis_conflict.is_empty());
    assert!(st.blockers.is_empty(), "{:?}", st.blockers);
    srv.shutdown().await;

    // Cross-check on (sim.toml ids): units of axes 1 and 2 swap. Selection
    // still decides the assignment; the configured device_id blocks ARM in
    // both directions and the links stay up.
    let srv = start_sim(sim_cfg()).await;
    wait_state(&srv, RigState::ReadyToArm, 3 * S).await;
    sim(&srv, 1).select_axis(2);
    wait_status(&srv, S, "conflict", |s| !s.axis_conflict.is_empty()).await;
    sim(&srv, 2).select_axis(1);
    let st = wait_status(&srv, 2 * S, "mismatch blockers", |s| {
        s.axis_conflict.is_empty() && s.axes.iter().all(|a| a.connected) && has(&s.blockers, "axis_device_mismatch")
    })
    .await;
    assert_eq!(st.rig_state, RigState::CalibrationRequired);
    assert_eq!(st.axes[0].port.as_deref(), Some("sim:3"), "selection wins");
    assert_eq!(st.axes[0].device_id, h3cs::types::DeviceId(0x5131_0000_0000_0002));
    assert_eq!(st.axes[0].expected_device_id, h3cs::types::DeviceId(0x5131_0000_0000_0001));
    assert_eq!(st.axes[0].assignment, Some("selected"));
    for needle in [
        "axis_device_mismatch: axis1 device 5131000000000002, config expects 5131000000000001",
        "axis_device_mismatch: device 5131000000000002 is configured as axis2 but selected axis1",
        "axis_device_mismatch: axis2 device 5131000000000001, config expects 5131000000000002",
        "axis_device_mismatch: device 5131000000000001 is configured as axis1 but selected axis2",
    ] {
        assert!(has(&st.blockers, needle), "{needle} not in {:?}", st.blockers);
    }
    assert!(!has(&st.blockers, "unknown_device"), "{:?}", st.blockers);
    let r = admin(srv.local_addr, "/v1/arm").await;
    assert_eq!((r.status, r.json()["error"].as_str()), (409, Some("not_ready")), "{r:?}");
    srv.shutdown().await;
}

#[tokio::test]
async fn idle_unit_switching_axis_is_reassigned_and_needs_re_reference_before_arm() {
    // (b) Units of axes 1 and 2 swap. No cross-check on those two axes.
    let cfg = no_cross_check(sim_cfg(), &[1, 2]);
    let hashes: Vec<u32> = {
        let cal = calibration::from_config(&cfg);
        (0..3).map(|i| calibration::calibration_hash(&cfg.rig_id, &cal[i])).collect()
    };
    let srv = start_sim(cfg).await;
    wait_state(&srv, RigState::ReadyToArm, 3 * S).await;
    let unit_a = sim(&srv, 1); // started on axis 1
    let unit_b = sim(&srv, 2); // started on axis 2

    unit_a.select_axis(2);
    let st = wait_status(&srv, S, "conflict on axis 2", |s| s.axis_conflict.len() == 1).await;
    assert_eq!(st.axis_conflict[0].get(), 2);
    assert_eq!(st.rig_state, RigState::Offline);
    assert!(!st.axes[0].connected && !st.axes[1].connected);
    assert_eq!(st.axes[1].claimants.len(), 2);
    assert!(st.fault.is_none());

    unit_b.select_axis(1);
    let st = wait_status(&srv, S, "resolved", |s| {
        s.axis_conflict.is_empty() && s.axes.iter().all(|a| a.connected && a.configured)
    })
    .await;
    assert_eq!(st.axes[0].port.as_deref(), Some("sim:3"), "unit b now drives axis 1");
    assert_eq!(st.axes[1].port.as_deref(), Some("sim:2"), "unit a now drives axis 2");
    assert_eq!(st.axes[0].device_id, h3cs::types::DeviceId(0x5131_0000_0000_0002));
    assert_eq!(st.axes[1].device_id, h3cs::types::DeviceId(0x5131_0000_0000_0001));
    // Each unit holds the CONFIG of its new axis.
    assert_eq!(unit_a.with(|m| m.config().map(|c| c.calibration_hash)), Some(hashes[1]));
    assert_eq!(unit_b.with(|m| m.config().map(|c| c.calibration_hash)), Some(hashes[0]));
    // Stale reference blocks ARM (and the sim dropped POSITION_REFERENCED too).
    assert!(st.axes[0].reference_stale && st.axes[1].reference_stale && !st.axes[2].reference_stale);
    assert_eq!(st.rig_state, RigState::CalibrationRequired);
    assert!(has(&st.blockers, "reference_stale: axis1"), "{:?}", st.blockers);
    assert!(has(&st.blockers, "reference_stale: axis2"), "{:?}", st.blockers);
    let r = admin(srv.local_addr, "/v1/arm").await;
    assert_eq!(r.status, 409, "{r:?}");
    assert_eq!(r.json()["error"], "not_ready");
    assert!(r.json()["blockers"].as_array().unwrap().iter().any(|b| b.as_str().unwrap().starts_with("reference_stale: axis")));

    reference(&srv, 1).await;
    wait_status(&srv, S, "axis 1 re-referenced", |s| !s.axes[0].reference_stale && s.axes[0].referenced).await;
    assert!(srv.status().axes[1].reference_stale, "axis 2 still stale");
    reference(&srv, 2).await;
    wait_state(&srv, RigState::ReadyToArm, 2 * S).await;
    assert!(unit_a.snapshot().referenced && unit_b.snapshot().referenced);
    assert!((1..=3).all(|a| sim(&srv, a).state() == McuState::Disarmed), "re-assignment never armed anything");
    let r = admin(srv.local_addr, "/v1/arm").await;
    assert_eq!(r.status, 200, "{r:?}");
    wait_state(&srv, RigState::Holding, 3 * S).await;
    srv.shutdown().await;
}

#[tokio::test]
async fn two_units_selecting_the_same_axis_conflict_until_one_switches_back() {
    // (c)
    let srv = start_sim(no_cross_check(sim_cfg(), &[1, 2, 3])).await;
    wait_state(&srv, RigState::ReadyToArm, 3 * S).await;
    let id = new_session(srv.local_addr).await;
    let mut ws = ws_connect(srv.local_addr, &id).await;
    ws_send(&mut ws, pose_json(&id, 1, 1, 0.1, 0.0, 0.0, GAIN)).await;
    assert_eq!(next_ack(&mut ws).await["accepted"], true);

    sim(&srv, 1).select_axis(2);
    let st = wait_status(&srv, S, "conflict", |s| !s.axis_conflict.is_empty()).await;
    assert_eq!(st.axis_conflict.iter().map(|a| a.get()).collect::<Vec<_>>(), [2]);
    assert_eq!(st.rig_state, RigState::Offline);
    assert!(has(&st.blockers, "axis_conflict: axis2 claimed by"), "{:?}", st.blockers);
    assert!(has(&st.blockers, "not_connected: axis1"), "{:?}", st.blockers);
    assert!(has(&st.blockers, "not_connected: axis2"), "{:?}", st.blockers);
    assert!(st.axes[1].port.is_none() && st.axes[1].claimants.len() == 2);
    // Admin: arm 409 axis_conflict; stop still accepted.
    let r = admin(srv.local_addr, "/v1/arm").await;
    assert_eq!(r.status, 409, "{r:?}");
    assert_eq!(r.json()["error"], "axis_conflict");
    assert!(r.json()["blockers"].as_array().unwrap().iter().any(|b| b.as_str().unwrap().starts_with("axis_conflict: axis2")));
    let r = admin(srv.local_addr, "/v1/start").await;
    assert_eq!((r.status, r.json()["error"].as_str()), (409, Some("axis_conflict")));
    let r = http(srv.local_addr, "POST", "/v1/stop", &[], None).await;
    assert_eq!((r.status, r.json()["ok"].as_bool()), (200, Some(true)));
    // Game input: pose and gain refused, freshness not extended.
    ws_send(&mut ws, pose_json(&id, 2, 2, 0.1, 0.0, 0.0, GAIN)).await;
    let ack = next_ack(&mut ws).await;
    assert_eq!((ack["accepted"].as_bool(), ack["reason"].as_str()), (Some(false), Some("axis_conflict")), "{ack}");
    let gain = serde_json::json!({"type": "gain", "protocol": 1, "source_session": id, "seq": 3, "gain": 0.2}).to_string();
    ws_send(&mut ws, gain).await;
    assert_eq!(next_ack(&mut ws).await["reason"], "axis_conflict");
    tokio::time::sleep(Duration::from_millis(300)).await;
    let st = srv.status();
    assert_eq!(st.session.as_ref().unwrap()["fresh"], false);
    assert_eq!(st.session.as_ref().unwrap()["last_seq"], 1, "rejected messages do not advance seq");
    // Neither unit of axis 2 received CONFIG during the conflict (both idle, unconfigured
    // for the switched one; the original keeps its earlier CONFIG).
    assert!(!sim(&srv, 1).snapshot().configured);

    // Resolve: switch the unit back.
    sim(&srv, 1).select_axis(1);
    let st = wait_status(&srv, 2 * S, "conflict cleared", |s| {
        s.axis_conflict.is_empty() && s.axes.iter().all(|a| a.connected && a.configured)
    })
    .await;
    assert_eq!(st.axes[1].port.as_deref(), Some("sim:3"), "the remaining unit is primary again");
    assert_eq!(st.axes[0].port.as_deref(), Some("sim:2"));
    assert!(st.axes[0].reference_stale, "1→2→1 under one boot: re-reference");
    assert!(!st.axes[1].reference_stale);
    ws_send(&mut ws, pose_json(&id, 4, 3, 0.1, 0.0, 0.0, GAIN)).await;
    assert_eq!(next_ack(&mut ws).await["accepted"], true);
    assert!(!st.armed);
    ws.close(None).await.unwrap();
    srv.shutdown().await;
}

#[tokio::test]
async fn unit_switching_axis_while_active_latches_axis_changed() {
    // (d) a unit violating the DISARMED rule while the rig is ACTIVE.
    let (srv, streamer) = active_rig(no_cross_check(fast_cfg(), &[1, 2, 3])).await;
    tokio::time::sleep(Duration::from_millis(200)).await;
    sim(&srv, 1).select_axis_unchecked(2);
    let st = wait_state(&srv, RigState::FaultLatched, S).await;
    assert!(matches!(st.fault, Some(RigFault::AxisChanged { axis }) if axis.get() == 1), "{:?}", st.fault);
    let st = wait_status(&srv, S, "conflict on axis 2", |s| !s.axis_conflict.is_empty()).await;
    assert_eq!(st.axis_conflict[0].get(), 2);
    // HOST_FAULT reaches the untouched axis and the axis-2 unit: the latter is
    // demoted by the conflict only after it ACKed (not left to its own lease).
    // The switching unit gets nothing from the PC (claimant of a conflicted
    // axis): its own lease must stop it (protocol.md §1.1).
    let deadline = tokio::time::Instant::now() + 2 * S;
    loop {
        let s3 = sim(&srv, 3).snapshot();
        let s2 = sim(&srv, 2).snapshot();
        let s1 = sim(&srv, 1).snapshot();
        if s3.state == McuState::Fault
            && s3.fault_code == FaultCode::HostFault
            && s2.state == McuState::Fault
            && s2.fault_code == FaultCode::HostFault
            && s1.state == McuState::Fault
            && s1.fault_code == FaultCode::CommandTimeout
        {
            break;
        }
        assert!(tokio::time::Instant::now() < deadline, "axes not stopped: {s1:?} {s2:?} {s3:?}");
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    assert_eq!(srv.status().rig_state, RigState::FaultLatched);
    let r = admin(srv.local_addr, "/v1/start").await;
    assert_eq!(r.status, 409);
    streamer.stop().await;
    srv.shutdown().await;
}

#[tokio::test]
async fn report_selected_axis_false_keeps_the_device_id_path() {
    // (e)
    let mut cfg = sim_cfg();
    if let h3cs::config::BackendConfig::Sim(b) = &mut cfg.backend {
        b.report_selected_axis = false;
    }
    let srv = start_sim(cfg).await;
    let st = wait_state(&srv, RigState::ReadyToArm, 3 * S).await;
    let ports: Vec<_> = st.axes.iter().map(|a| a.port.clone().unwrap_or_default()).collect();
    assert_eq!(ports, ["sim:2", "sim:3", "sim:1"]);
    for a in &st.axes {
        assert_eq!(a.assignment, Some("config"), "{a:?}");
        assert_eq!(a.device_id, a.expected_device_id);
    }
    assert!((1..=3).all(|a| sim(&srv, a).snapshot().selected_axis == 0));
    srv.shutdown().await;

    // Without a selection and without a configured device_id the unit is unknown.
    let mut cfg = sim_cfg();
    if let h3cs::config::BackendConfig::Sim(b) = &mut cfg.backend {
        b.report_selected_axis = false;
    }
    let cfg = no_cross_check(cfg, &[3]);
    let srv = start_sim(cfg).await;
    let st = wait_status(&srv, 3 * S, "unknown device", |s| has(&s.blockers, "unknown_device")).await;
    assert_eq!(st.rig_state, RigState::Offline);
    assert!(!st.axes[2].connected);
    assert!(has(&st.blockers, "unknown_device: device 5131000000000003 on port sim:1"), "{:?}", st.blockers);
    srv.shutdown().await;
}

#[tokio::test]
async fn local_estop_from_an_axis_is_decoded_latched_and_named_in_status() {
    // protocol.md §3.2 code 12: the MCU's on-screen emergency stop. It is
    // decoded from the wire, latches the rig like any other axis fault, and
    // shows up by name in /v1/status (fault, axes[].fault_code, blockers).
    let srv = start_sim(fast_cfg()).await;
    wait_state(&srv, RigState::ReadyToArm, 3 * S).await;
    sim(&srv, 2).force_fault(FaultCode::LocalEstop);
    let st = wait_state(&srv, RigState::FaultLatched, S).await;
    assert_eq!(st.fault, Some(RigFault::AxisFault { axis: AxisId::new(2).unwrap(), code: FaultCode::LocalEstop }));
    wait_status(&srv, S, "axis 2 STATUS with LOCAL_ESTOP", |s| s.axes[1].fault_code == Some(FaultCode::LocalEstop)).await;

    let r = http(srv.local_addr, "GET", "/v1/status", &[], None).await;
    assert_eq!(r.status, 200, "{r:?}");
    let body = r.json();
    assert_eq!(body["fault"]["kind"], "axis_fault");
    assert_eq!(body["fault"]["axis"], 2);
    assert_eq!(body["fault"]["code"], "LOCAL_ESTOP");
    assert_eq!(body["axes"][1]["fault_code"], "LOCAL_ESTOP");
    let blockers: Vec<String> =
        body["blockers"].as_array().unwrap().iter().map(|b| b.as_str().unwrap().to_string()).collect();
    assert!(
        blockers.iter().any(|b| b.starts_with("axis_fault: axis2") && b.contains("LocalEstop")),
        "blockers name the local e-stop: {blockers:?}"
    );

    // The sim has no latched button, so the reset goes through once the
    // HOST_FAULT round trip has settled (on the real MCU the button latch
    // must be released first: FAULT_CAUSE_PRESENT).
    wait_status(&srv, S, "HOST_FAULT on the other axes", |s| {
        [0, 2].iter().all(|&i| s.axes[i].fault_code == Some(FaultCode::HostFault))
    })
    .await;
    let deadline = tokio::time::Instant::now() + 2 * S;
    loop {
        let r = admin(srv.local_addr, "/v1/fault/reset").await;
        if r.status == 200 {
            break;
        }
        assert!(tokio::time::Instant::now() < deadline, "fault reset never accepted: {r:?}");
        assert_eq!(r.json()["error"], "axis_not_settled", "{r:?}");
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
    wait_state(&srv, RigState::ReadyToArm, 3 * S).await;
    srv.shutdown().await;
}

// ------------------------------------------------------------------ calibration (DESIGN.md §11)

/// sim.toml with unreferenced sims whose end stops are ±500 (stroke 1000:
/// the calibrated neutral 500 is inside every soft window of sim.toml) and
/// `calibration_valid = false` on every axis, written to `dir/sim.toml`.
fn uncalibrated_cfg_file(dir: &std::path::Path) -> (Config, std::path::PathBuf) {
    let text = std::fs::read_to_string(SIM)
        .unwrap()
        .replace("start_referenced = true", "start_referenced = false")
        .replace("end_min_count = -3000", "end_min_count = -500")
        .replace("end_max_count = 3000", "end_max_count = 500")
        .replace("calibration_valid = true", "calibration_valid = false");
    let cfg = Config::from_toml_str(&text).unwrap();
    let path = dir.join("sim.toml");
    std::fs::write(&path, text).unwrap();
    (cfg, path)
}

fn blocker_codes(snap: &h3cs::supervisor::StatusSnapshot) -> Vec<String> {
    let mut v: Vec<String> = snap.blockers.iter().map(|b| b.split(':').next().unwrap_or(b).to_string()).collect();
    v.sort();
    v.dedup();
    v
}

#[tokio::test]
async fn calibrate_all_from_the_api_then_export_a_config_that_arms_but_for_calibration_valid() {
    let dir = std::env::temp_dir().join(format!("h3cs-server-cal-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    let (cfg, cfg_path) = uncalibrated_cfg_file(&dir);
    let srv = start_sim_with_path(cfg.clone(), Some(cfg_path.clone())).await;
    wait_status(&srv, 3 * S, "all connected, unreferenced", |s| {
        s.rig_state == RigState::CalibrationRequired && s.axes.iter().all(|a| a.connected && a.mcu_state == Some(McuState::Disarmed) && !a.referenced)
    })
    .await;
    let r = admin(srv.local_addr, "/v1/arm").await;
    assert_eq!(r.status, 409, "{r:?}");
    assert_eq!(r.json()["error"], "not_ready");

    let r = http(srv.local_addr, "POST", "/v1/calibrate", &[("Authorization", &format!("Bearer {TOKEN}"))], None).await;
    assert_eq!(r.status, 200, "{r:?}");
    let snap = wait_status(&srv, 2 * S, "all CALIBRATING", |s| s.axes.iter().all(|a| a.mcu_state == Some(McuState::Calibrating))).await;
    assert_eq!(snap.rig_state, RigState::CalibrationRequired);
    assert!(snap.blockers.iter().any(|b| b == "calibrating: axis1"), "{:?}", snap.blockers);
    let r = admin(srv.local_addr, "/v1/arm").await;
    assert_eq!(r.status, 409);
    assert_eq!(r.json()["error"], "calibrating");
    for (_, sim) in &srv.sims {
        assert_eq!(sim.state(), McuState::Calibrating);
    }

    let snap = wait_status(&srv, 6 * S, "all CAL_RESULT OK and referenced", |s| {
        s.axes.iter().all(|a| {
            a.calibration.as_ref().is_some_and(|c| c.result == h3cs::protocol::CalResultCode::Ok)
                && a.referenced
                && a.mcu_state == Some(McuState::Disarmed)
        })
    })
    .await;
    assert!(snap.fault.is_none());
    for a in &snap.axes {
        let c = a.calibration.as_ref().unwrap();
        assert_eq!((c.n_end_count, c.neutral_count, c.sign, c.reason.as_str(), c.step), (1000, 500, 1, "ok", 3));
        assert_eq!(c.device_id, a.device_id, "the result names the connected unit");
        assert_eq!(a.actual_count, Some(500));
    }
    for (_, sim) in &srv.sims {
        let s = sim.snapshot();
        assert!(s.referenced);
        assert_eq!((s.end_min_count, s.end_max_count), (0, 1000), "P end = 0");
    }
    // ARM is now blocked only by the operator's calibration_valid = false and
    // by the recalibration itself (Issue #15 H3: the geometry this server
    // started with is no longer approved; a restart with the export clears it).
    let snap = wait_status(&srv, S, "blockers settle", |s| blocker_codes(s) == ["calibration_invalid", "calibration_stale"]).await;
    assert_eq!(snap.blockers.len(), 6, "{:?}", snap.blockers);
    assert!(snap.axes.iter().all(|a| a.calibration_stale));
    let r = admin(srv.local_addr, "/v1/arm").await;
    assert_eq!(r.status, 409);
    assert_eq!(r.json()["error"], "not_ready");
    assert!(r.json()["blockers"].as_array().unwrap().iter().all(|b| {
        let b = b.as_str().unwrap();
        b.starts_with("calibration_invalid") || b.starts_with("calibration_stale")
    }));

    // Export: a config next to sim.toml with the measured numbers.
    let r = http(srv.local_addr, "POST", "/v1/calibration/export", &[("Authorization", &format!("Bearer {TOKEN}"))], None).await;
    assert_eq!(r.status, 200, "{r:?}");
    let body = r.json();
    let path = std::path::PathBuf::from(body["path"].as_str().unwrap());
    assert_eq!(path.parent(), Some(dir.as_path()));
    assert_eq!(body["axes"].as_array().unwrap().len(), 3);
    let out = Config::load(&path).unwrap();
    for (i, a) in out.axes.iter().enumerate() {
        assert_eq!((a.mechanical_min_count, a.mechanical_max_count, a.soft_min_count, a.soft_max_count), (0, 1000, 300, 700), "axis {}", i + 1);
        assert_eq!((a.count_at_virtual_0, a.count_at_virtual_512, a.count_at_virtual_1023), (300, 500, 700));
        assert!(!a.calibration_valid);
        assert_eq!(a.device_id, cfg.axis(a.axis_id).unwrap().device_id, "the connected unit's id (the sims use sim.toml's)");
    }
    srv.shutdown().await;

    // A server started from the exported file: the restart cleared the
    // calibration_stale mark; the unreferenced sims at 0 need a reference,
    // and a calibration later only calibration_valid plus the fresh
    // calibration_stale stand between it and ARM.
    let srv = start_sim_with_path(out, Some(path.clone())).await;
    let snap = wait_status(&srv, 3 * S, "connected", |s| s.axes.iter().all(|a| a.connected && a.mcu_state == Some(McuState::Disarmed))).await;
    assert!(snap.axes.iter().all(|a| !a.calibration_stale), "{:?}", snap.blockers);
    assert!(!blocker_codes(&snap).iter().any(|c| c == "calibration_stale"), "{:?}", snap.blockers);
    let r = http(srv.local_addr, "POST", "/v1/calibrate", &[("Authorization", &format!("Bearer {TOKEN}"))], Some(r#"{"axes":[1,2,3]}"#)).await;
    assert_eq!(r.status, 200, "{r:?}");
    let snap = wait_status(&srv, 8 * S, "calibrated again", |s| {
        s.axes.iter().all(|a| a.calibration.is_some() && a.referenced && a.mcu_state == Some(McuState::Disarmed))
            && blocker_codes(s) == ["calibration_invalid", "calibration_stale"]
    })
    .await;
    assert_eq!(snap.rig_state, RigState::CalibrationRequired);
    // Exporting again from a different second is a new file; the same name is never overwritten.
    let r = http(srv.local_addr, "POST", "/v1/calibration/export", &[("Authorization", &format!("Bearer {TOKEN}"))], None).await;
    assert!(r.status == 200 || (r.status == 409 && r.json()["error"] == "exists"), "{r:?}");
    srv.shutdown().await;
    let _ = std::fs::remove_dir_all(&dir);
}

#[tokio::test]
async fn calibrate_cancel_from_the_api_stops_the_sims_and_leaves_them_unreferenced() {
    let dir = std::env::temp_dir().join(format!("h3cs-server-calcancel-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    let (cfg, cfg_path) = uncalibrated_cfg_file(&dir);
    let srv = start_sim_with_path(cfg, Some(cfg_path)).await;
    wait_status(&srv, 3 * S, "connected", |s| s.axes.iter().all(|a| a.connected && a.mcu_state == Some(McuState::Disarmed))).await;
    // Without the power axis: refused, nothing moves.
    let r = http(srv.local_addr, "POST", "/v1/calibrate", &[("Authorization", &format!("Bearer {TOKEN}"))], Some(r#"{"axes":[2,3]}"#)).await;
    assert_eq!(r.status, 409, "{r:?}");
    assert_eq!(r.json()["error"], "power_axis_required");
    let r = http(srv.local_addr, "POST", "/v1/calibrate", &[("Authorization", &format!("Bearer {TOKEN}"))], None).await;
    assert_eq!(r.status, 200, "{r:?}");
    wait_status(&srv, 2 * S, "all CALIBRATING", |s| s.axes.iter().all(|a| a.mcu_state == Some(McuState::Calibrating))).await;
    // /v1/stop (no token) cancels a calibration too.
    let r = http(srv.local_addr, "POST", "/v1/stop", &[], None).await;
    assert_eq!(r.status, 200, "{r:?}");
    assert!(r.json()["note"].as_str().unwrap().starts_with("calibration cancelled"), "{r:?}");
    let snap = wait_status(&srv, 3 * S, "cancelled", |s| {
        s.axes.iter().all(|a| a.mcu_state == Some(McuState::Disarmed) && a.calibration.is_some())
    })
    .await;
    for a in &snap.axes {
        assert_eq!(a.calibration.as_ref().unwrap().result, h3cs::protocol::CalResultCode::Cancelled);
        assert!(!a.referenced);
    }
    assert!(snap.fault.is_none());
    // The export has nothing to write.
    let r = http(srv.local_addr, "POST", "/v1/calibration/export", &[("Authorization", &format!("Bearer {TOKEN}"))], None).await;
    assert_eq!(r.status, 409);
    assert_eq!(r.json()["error"], "no_calibration_result");
    srv.shutdown().await;
    let _ = std::fs::remove_dir_all(&dir);
}

#[tokio::test]
async fn admin_routes_need_no_token_by_default() {
    // PC-local default: server.require_admin_token = false and no token given.
    let cfg = fast_cfg();
    assert!(!cfg.server.require_admin_token);
    let srv = h3cs::server::start(
        cfg,
        h3cs::server::ServerOptions { admin_token: None, bind_override: Some("127.0.0.1:0".parse().unwrap()), ..Default::default() },
    )
    .await
    .unwrap();
    let _ = wait_state(&srv, RigState::ReadyToArm, 3 * S).await;
    let r = http(srv.local_addr, "POST", "/v1/fault", &[("content-type", "application/json")], Some("{\"note\":\"test\"}")).await;
    assert_eq!(r.status, 200, "fault without token: {}", r.body);
    let st = http(srv.local_addr, "GET", "/v1/status", &[], None).await;
    assert_eq!(st.json()["admin_token_required"], false, "{}", st.body);
    srv.shutdown().await;
}

/// CONTROLLED_STOP is enqueued on its own. A governed SET_POSITION may still
/// be the frame already on the wire, but the stop does not wait for a
/// full-limit SET_POSITION to be written first.
#[tokio::test]
async fn controlled_stop_is_enqueued_without_waiting_for_set_position() {
    let mut cfg = fast_cfg();
    cfg.governor.no_progress_ms = 10_000;
    // 軸 2 を仮想端（約 400 count）まで振る。既定の POSE は数十 count しか
    // 動かさず、止めても追従誤差が fe_hard に届かない。
    let pose = Pose { pitch: -1.0, roll: 1.0, yaw: 0.0 };
    let gain = 0.5;
    let srv = start_sim(cfg).await;
    wait_state(&srv, RigState::ReadyToArm, 3 * S).await;
    let id = new_session(srv.local_addr).await;
    let ws = ws_connect(srv.local_addr, &id).await;
    let streamer = Streamer::start(ws, id, 1, pose, gain);
    wait_status(&srv, S, "fresh session", |s| s.session.as_ref().is_some_and(|v| v["fresh"] == true)).await;
    let r = admin(srv.local_addr, "/v1/arm").await;
    assert_eq!(r.status, 200, "arm: {r:?}");
    wait_state(&srv, RigState::Holding, 3 * S).await;
    // 追跡が始まる前に止める。追いついてからでは誤差が開かない。
    sim(&srv, 2).set_stall(true);
    let r = admin(srv.local_addr, "/v1/start").await;
    assert_eq!(r.status, 200, "start: {r:?}");
    let st = wait_status(&srv, 2 * S, "governor engaged", |s| {
        s.rig_state == RigState::Active && s.governor.active && s.governor.scale < 0.9
    })
    .await;
    assert!(st.governor.scale < 0.9, "{}", st.governor.scale);

    let r = http(srv.local_addr, "POST", "/v1/stop", &[], None).await;
    assert_eq!(r.status, 200, "{r:?}");
    let st = wait_state(&srv, RigState::Stopping, S).await;
    assert!(st.fault.is_none(), "{:?}", st.fault);
    let full_vel = st.motion_limits.effective.max_vel_count_s;
    let full_acc = st.motion_limits.effective.max_acc_count_s2;
    assert!(full_acc > 1000 && full_vel > 100);
    // 状態が Stopping になっても、リンクの書き出しはまだ終わっていないことがある。
    // 模擬軸は減速が短いとすぐに HOLDING へ抜けるので、状態ではなく受信で待つ。
    let deadline = tokio::time::Instant::now() + S;
    loop {
        let seen = (1..=3u8).all(|axis| {
            sim(&srv, axis)
                .host_rx()
                .iter()
                .any(|n| n.msg_type == msg_type::CONTROLLED_STOP)
        });
        if seen {
            break;
        }
        assert!(
            tokio::time::Instant::now() < deadline,
            "stop frame did not reach every axis: {:?}",
            (1..=3u8)
                .map(|axis| {
                    let handle = sim(&srv, axis);
                    let rx = handle.host_rx();
                    (axis, handle.state(), rx.len(), rx.last().map(|n| n.msg_type))
                })
                .collect::<Vec<_>>()
        );
        tokio::time::sleep(Duration::from_millis(10)).await;
    }

    for axis in 1..=3u8 {
        let handle = sim(&srv, axis);
        let rx = handle.host_rx();
        let stop_at = rx
            .iter()
            .rposition(|n| n.msg_type == msg_type::CONTROLLED_STOP)
            .unwrap_or_else(|| panic!("axis {axis} never saw CONTROLLED_STOP: {rx:?}"));
        assert!(stop_at > 0, "axis {axis}: {rx:?}");
        let prev = rx[stop_at - 1];
        assert_eq!(prev.msg_type, msg_type::SET_POSITION, "axis {axis}: {rx:?}");
        // The stop went out against the governed frame already in force.
        // Nothing rewrote that frame up to the full limits first.
        assert!(
            prev.max_acc_count_s2 + 1 < full_acc,
            "axis {axis} waited for a full-limit SET_POSITION ({prev:?}, full acc {full_acc})"
        );
        assert!(
            rx[..stop_at].iter().any(|n| {
                n.msg_type == msg_type::SET_POSITION && n.max_acc_count_s2 + 1 < full_acc
            }),
            "axis {axis} had no governed frame before the stop"
        );
        let (_vel, acc) = handle.frame_limits();
        assert!(
            acc + 1.0 < f64::from(full_acc),
            "axis {axis} latched the full acc {acc} while still ACTIVE"
        );
    }
    let _ = streamer.stop().await;
    srv.shutdown().await;
}

#[tokio::test]
async fn serial_backend_sim_axes_stand_in_for_missing_units() {
    // Bring-up with fewer real units: axes listed in `sim_axes` are served by
    // in-process simulators (no serial port is opened for them).
    let mut cfg = fast_cfg();
    cfg.backend = h3cs::config::BackendConfig::Serial(h3cs::config::SerialBackendConfig {
        ports: vec![].into(),
        baud: 115_200,
        sim_axes: vec![AxisId::new(1).unwrap(), AxisId::new(2).unwrap(), AxisId::new(3).unwrap()],
        sim: h3cs::config::SimBackendConfig::default(),
    });
    let srv = h3cs::server::start(
        cfg,
        h3cs::server::ServerOptions { admin_token: None, bind_override: Some("127.0.0.1:0".parse().unwrap()), ..Default::default() },
    )
    .await
    .unwrap();
    let st = wait_state(&srv, RigState::ReadyToArm, 3 * S).await;
    let ports: Vec<_> = st.axes.iter().map(|a| a.port.clone().unwrap()).collect();
    assert_eq!(ports, ["sim:1", "sim:2", "sim:3"]);
    assert_eq!(srv.sims.len(), 3);
    assert!(st.axes.iter().all(|a| a.assignment == Some("selected")));
    srv.shutdown().await;
}

#[tokio::test]
async fn serial_backend_ports_auto_opens_nothing_that_does_not_match() {
    // `ports = "auto"` with device_ids no real unit has: the discovery task
    // scans the port list (on any machine) but never starts a port task, so
    // every axis stays unconnected, and shutdown stops the discovery task.
    let mut cfg = fast_cfg();
    for (k, a) in cfg.axes.iter_mut().enumerate() {
        a.device_id = h3cs::types::DeviceId(0x5131_0000_0000_0a01 + k as u64);
    }
    cfg.backend = h3cs::config::BackendConfig::Serial(h3cs::config::SerialBackendConfig {
        ports: h3cs::config::PortList::Auto,
        baud: 115_200,
        sim_axes: vec![],
        sim: h3cs::config::SimBackendConfig::default(),
    });
    let srv = h3cs::server::start(
        cfg,
        h3cs::server::ServerOptions { admin_token: None, bind_override: Some("127.0.0.1:0".parse().unwrap()), ..Default::default() },
    )
    .await
    .unwrap();
    // Two discovery periods.
    tokio::time::sleep(h3cs::server::DISCOVERY_PERIOD * 2 + Duration::from_millis(200)).await;
    let st = srv.status();
    assert_eq!(st.rig_state, RigState::Offline);
    assert!(st.axes.iter().all(|a| !a.connected && a.port.is_none()), "{:?}", st.axes);
    tokio::time::timeout(Duration::from_secs(10), srv.shutdown()).await.expect("shutdown hung");
}

// ------------------------------------------------- ports = "auto" (fake USB)
//
// `FakeUsb` lists simulated CoreS3 units as USB ports (VID:PID 303A:1001,
// serial = MAC; each also under its macOS `/dev/tty.*` alias) and opens
// them onto the simulator. Every open is recorded, so a test can assert what
// was never opened.

mod fake_usb {
    use h3cs::port_discovery::{ListedPort, UsbIds};
    use h3cs::server::{OpenFuture, PortStream, SerialPorts};
    use h3cs::sim_axis::{sim_detached, SimHandle, SimParams};
    use h3cs::types::DeviceId;
    use std::collections::BTreeMap;
    use std::sync::Mutex;

    pub const ID: [DeviceId; 3] =
        [DeviceId(0x5131_0000_0000_0a01), DeviceId(0x5131_0000_0000_0a02), DeviceId(0x5131_0000_0000_0a03)];
    /// Another project's ESP32-S3 (same VID:PID).
    pub const FOREIGN: DeviceId = DeviceId(0x5131_d0cf_130e_96f8);

    pub fn mac(id: DeviceId) -> String {
        let b = id.0.to_be_bytes();
        format!("{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}", b[2], b[3], b[4], b[5], b[6], b[7])
    }

    /// A unit: USB serial from `usb_id`; its HELLO says `hello_id` / `axis`.
    pub fn unit(usb_id: DeviceId, hello_id: DeviceId, axis: u8) -> (String, SimHandle) {
        let params = SimParams::from_backend(&Default::default(), hello_id, rand::random::<u32>().max(1), axis);
        (mac(usb_id), sim_detached(params))
    }

    #[derive(Debug, Default)]
    pub struct FakeUsb {
        state: Mutex<State>,
    }

    #[derive(Debug, Default)]
    struct State {
        plugged: BTreeMap<String, (String, SimHandle)>,
        opened: Vec<String>,
    }

    impl FakeUsb {
        pub fn plug(&self, name: &str, unit: (String, SimHandle)) {
            self.state.lock().unwrap().plugged.insert(name.into(), unit);
        }
        /// Pull the cable: the open link sees EOF. Returns the unit.
        pub fn unplug(&self, name: &str) -> (String, SimHandle) {
            let (ser, mut sim) = self.state.lock().unwrap().plugged.remove(name).expect("plugged");
            drop(sim.attach()); // stops the runner of the open link
            (ser, sim)
        }
        pub fn view(&self, name: &str) -> SimHandle {
            self.state.lock().unwrap().plugged[name].1.view()
        }
        pub fn opened(&self) -> Vec<String> {
            self.state.lock().unwrap().opened.clone()
        }
        pub fn opens_of(&self, name: &str) -> usize {
            self.opened().iter().filter(|n| *n == name).count()
        }
    }

    impl SerialPorts for FakeUsb {
        fn list(&self) -> Vec<ListedPort> {
            let st = self.state.lock().unwrap();
            let mut v = vec![ListedPort { name: "/dev/cu.Bluetooth-Incoming-Port".into(), usb: None }];
            for (name, (ser, _)) in &st.plugged {
                let usb = Some(UsbIds { vid: 0x303A, pid: 0x1001, serial: Some(ser.clone()) });
                v.push(ListedPort { name: name.clone(), usb: usb.clone() });
                v.push(ListedPort { name: name.replace("/dev/cu.", "/dev/tty."), usb });
            }
            v
        }
        fn open<'a>(&'a self, port: &'a str, _baud: u32) -> OpenFuture<'a> {
            let r = {
                let mut st = self.state.lock().unwrap();
                st.opened.push(port.to_string());
                match st.plugged.get_mut(port) {
                    Some((_, sim)) => Ok(Box::new(sim.attach()) as PortStream),
                    None => Err(std::io::Error::new(std::io::ErrorKind::NotFound, "no such port")),
                }
            };
            Box::pin(async move { r })
        }
    }
}

use fake_usb::{FakeUsb, FOREIGN, ID};

const P1: &str = "/dev/cu.usbmodem11101";
const P2: &str = "/dev/cu.usbmodem11301";
const P3: &str = "/dev/cu.usbmodem11501";

fn auto_cfg() -> Config {
    let mut cfg = fast_cfg();
    for a in cfg.axes.iter_mut() {
        a.device_id = ID[a.axis_id.index()];
    }
    cfg.backend = h3cs::config::BackendConfig::Serial(h3cs::config::SerialBackendConfig {
        ports: h3cs::config::PortList::Auto,
        baud: 115_200,
        sim_axes: vec![],
        sim: h3cs::config::SimBackendConfig::default(),
    });
    cfg
}

async fn start_auto(usb: &std::sync::Arc<FakeUsb>) -> h3cs::server::RunningServer {
    let ports: std::sync::Arc<dyn h3cs::server::SerialPorts> = usb.clone();
    server::start(
        auto_cfg(),
        ServerOptions {
            admin_token: Some(TOKEN.into()),
            bind_override: Some("127.0.0.1:0".parse().unwrap()),
            config_path: None,
            serial_ports: Some(ports),
        },
    )
    .await
    .expect("server start")
}

/// The three configured units on P1..P3 plus a foreign ESP32-S3.
fn rig_usb() -> std::sync::Arc<FakeUsb> {
    let usb = std::sync::Arc::new(FakeUsb::default());
    usb.plug(P1, fake_usb::unit(ID[0], ID[0], 1));
    usb.plug(P2, fake_usb::unit(ID[1], ID[1], 2));
    usb.plug(P3, fake_usb::unit(ID[2], ID[2], 3));
    usb.plug("/dev/cu.usbmodem2101", fake_usb::unit(FOREIGN, FOREIGN, 1));
    usb
}

fn axis_port(st: &h3cs::supervisor::StatusSnapshot, axis: u8) -> Option<String> {
    st.axes[usize::from(axis - 1)].port.clone()
}

#[tokio::test]
async fn ports_auto_opens_the_configured_units_and_nothing_else() {
    let usb = rig_usb();
    let srv = start_auto(&usb).await;
    let st = wait_state(&srv, RigState::ReadyToArm, 5 * S).await;
    assert_eq!([1, 2, 3].map(|a| axis_port(&st, a)), [Some(P1.into()), Some(P2.into()), Some(P3.into())]);
    let mut opened = usb.opened();
    opened.sort();
    assert_eq!(opened, [P1, P2, P3], "only the configured units, each once, never a tty alias or the foreign unit");
    srv.shutdown().await;
}

#[tokio::test]
async fn ports_auto_follows_a_unit_replugged_under_a_new_name() {
    let usb = rig_usb();
    let srv = start_auto(&usb).await;
    wait_state(&srv, RigState::ReadyToArm, 5 * S).await;
    // Hub socket change: unit 1 comes back as 14101, and the foreign unit
    // now sits on its old name.
    let unit1 = usb.unplug(P1);
    let foreign = usb.unplug("/dev/cu.usbmodem2101");
    usb.plug("/dev/cu.usbmodem14101", unit1);
    usb.plug(P1, foreign);
    let st = wait_status(&srv, 10 * S, "axis 1 on 14101", |s| {
        s.rig_state == RigState::ReadyToArm && axis_port(s, 1).as_deref() == Some("/dev/cu.usbmodem14101")
    })
    .await;
    assert!(st.blockers.is_empty(), "{:?}", st.blockers);
    // Give the old name's task time to look again: it must not reopen.
    tokio::time::sleep(h3cs::server::SERIAL_RETRY + S).await;
    assert_eq!(usb.opens_of(P1), 1, "old name reopened with the foreign unit on it: {:?}", usb.opened());
    assert_eq!(usb.opens_of("/dev/cu.usbmodem2101"), 0);
    srv.shutdown().await;
}

#[tokio::test]
async fn ports_auto_shutdown_while_active_still_disarms() {
    // The port tasks must keep forwarding the DISARM ACK / STATUS while the
    // control loop disarms (Codex review P1 on PR #59).
    let usb = rig_usb();
    let srv = start_auto(&usb).await;
    let streamer = activate(&srv).await;
    tokio::time::sleep(Duration::from_millis(300)).await;
    let views = [P1, P2, P3].map(|p| usb.view(p));
    let t0 = tokio::time::Instant::now();
    srv.shutdown().await;
    for (i, v) in views.iter().enumerate() {
        let s = v.snapshot();
        assert_eq!(s.state, McuState::Disarmed, "axis {} after shutdown: {s:?}", i + 1);
    }
    assert!(t0.elapsed() < server::SHUTDOWN_DISARM_WAIT, "DISARM was not confirmed (waited {:?})", t0.elapsed());
    streamer.stop().await;
}

#[tokio::test]
async fn ports_auto_rejected_unit_replaced_on_the_same_name() {
    // P1 carries unit 1's USB serial, but its HELLO names no axis and an
    // unknown device: the link is rejected (`unknown_device`). Then that
    // unit is swapped for unit 2 on the same name: the port task must notice
    // the unit left (not just wait for the name to vanish), clear the
    // blocker, and unit 2 must be opened on P1.
    let usb = std::sync::Arc::new(FakeUsb::default());
    usb.plug(P1, fake_usb::unit(ID[0], FOREIGN, 0));
    usb.plug(P3, fake_usb::unit(ID[2], ID[2], 3));
    let srv = start_auto(&usb).await;
    wait_status(&srv, 5 * S, "unknown_device", |s| s.blockers.iter().any(|b| b.starts_with("unknown_device"))).await;
    let _gone = usb.unplug(P1);
    usb.plug(P1, fake_usb::unit(ID[1], ID[1], 2));
    let st = wait_status(&srv, 10 * S, "axis 2 on P1", |s| axis_port(s, 2).as_deref() == Some(P1)).await;
    wait_status(&srv, 2 * S, "unknown_device cleared", |s| !s.blockers.iter().any(|b| b.starts_with("unknown_device")))
        .await;
    assert!(axis_port(&st, 1).is_none());
    srv.shutdown().await;
}

#[tokio::test]
async fn ports_auto_opens_a_unit_seen_under_two_names_once() {
    let usb = std::sync::Arc::new(FakeUsb::default());
    usb.plug(P1, fake_usb::unit(ID[0], ID[0], 1));
    usb.plug("/dev/cu.usbmodem99", fake_usb::unit(ID[0], ID[0], 1));
    let srv = start_auto(&usb).await;
    wait_status(&srv, 5 * S, "axis 1 connected", |s| axis_port(s, 1).is_some()).await;
    tokio::time::sleep(h3cs::server::DISCOVERY_PERIOD * 3).await;
    let st = srv.status();
    assert!(st.axis_conflict.is_empty(), "{:?}", st.axis_conflict);
    assert_eq!(usb.opened().len(), 1, "{:?}", usb.opened());
    srv.shutdown().await;
}

// ------------------------------------------------------------------ operator hold (console)

fn bearer() -> String {
    format!("Bearer {TOKEN}")
}

#[tokio::test]
async fn operator_hold_stops_and_refuses_the_games_arm_start_reset_until_released() {
    let (srv, streamer) = active_rig(fast_cfg()).await;
    // Hold: no token needed (a stop), and it performs the Stop.
    let r = http(srv.local_addr, "POST", "/v1/operator/hold", &[], None).await;
    assert_eq!(r.status, 200, "{r:?}");
    assert_eq!(r.json()["operator_hold"], true);
    let st = wait_state(&srv, RigState::BrakedDisarmed, 3 * S).await;
    assert!(st.operator_hold);
    assert!(st.blockers.iter().any(|b| b.starts_with("operator_hold")), "{:?}", st.blockers);
    // The game (no header) is refused on all three.
    for path in ["/v1/arm", "/v1/start", "/v1/fault/reset"] {
        let r = admin(srv.local_addr, path).await;
        assert_eq!(r.status, 409, "{path}: {r:?}");
        assert_eq!(r.json()["error"], "operator_hold", "{path}");
    }
    // A wrong header value is not the operator.
    let auth = bearer();
    let r = http(srv.local_addr, "POST", "/v1/arm", &[("Authorization", &auth), ("X-H3-Operator", "0")], None).await;
    assert_eq!(r.json()["error"], "operator_hold", "{r:?}");
    assert_eq!(srv.status().rig_state, RigState::BrakedDisarmed);
    // The console (header) works as before, also while held.
    let r = http(srv.local_addr, "POST", "/v1/arm", &[("Authorization", &auth), ("X-H3-Operator", "1")], None).await;
    assert_eq!(r.status, 200, "{r:?}");
    wait_state(&srv, RigState::Holding, 3 * S).await;
    let r = admin(srv.local_addr, "/v1/disarm").await;
    assert_eq!(r.status, 200, "{r:?}");
    wait_status(&srv, 3 * S, "disarmed", |s| !s.armed).await;
    // Release: never arms by itself; the game may arm again.
    let r = admin(srv.local_addr, "/v1/operator/release").await;
    assert_eq!(r.status, 200, "{r:?}");
    let st = wait_status(&srv, S, "hold cleared", |s| !s.operator_hold).await;
    assert!(!st.armed);
    assert!(!st.blockers.iter().any(|b| b.starts_with("operator_hold")));
    let r = admin(srv.local_addr, "/v1/arm").await;
    assert_eq!(r.status, 200, "{r:?}");
    // Both changes are in the event log.
    let ev = http(srv.local_addr, "GET", "/v1/events?n=500", &[], None).await.json();
    let holds: Vec<&Value> = ev["events"].as_array().unwrap().iter().filter(|e| e["kind"] == "operator_hold").collect();
    assert_eq!(holds.len(), 2, "{holds:?}");
    // Release needs the token when one is required (hold does not).
    let r = http(srv.local_addr, "POST", "/v1/operator/release", &[], None).await;
    assert_eq!(r.status, 401, "{r:?}");
    streamer.stop().await;
    srv.shutdown().await;
}

#[tokio::test]
async fn operator_header_passes_the_same_origin_check() {
    let srv = start_sim(fast_cfg()).await;
    wait_state(&srv, RigState::ReadyToArm, 3 * S).await;
    let r = http(srv.local_addr, "POST", "/v1/operator/hold", &[], None).await;
    assert_eq!(r.status, 200, "{r:?}");
    // The console page's own origin with the custom header.
    let origin = format!("http://{}", srv.local_addr);
    let auth = bearer();
    let r = http(
        srv.local_addr,
        "POST",
        "/v1/arm",
        &[("Origin", &origin), ("Authorization", &auth), ("X-H3-Operator", "1")],
        None,
    )
    .await;
    assert_eq!(r.status, 200, "{r:?}");
    srv.shutdown().await;
}

// ------------------------------------------------------------------ hot apply (console)

/// sim.toml's sections with three hand-made axes (unreferenced sims with end
/// stops ±500: stroke 1000, neutral 500 after calibration):
/// - axis 1: margin 350 inside the soft limits (like the live config),
///   decel 50, stroke 5556 → not within ±30 of 1000;
/// - axis 2: stroke 1020 (within ±30), calibration_valid = true, margins 100 / 120;
/// - axis 3: stroke 2000, calibration_valid = false, margin 100.
fn hot_cfg_text() -> String {
    let sim = std::fs::read_to_string(SIM).unwrap();
    let head = &sim[..sim.find("[[axis]]").unwrap()];
    let head = head
        .replace("start_referenced = true", "start_referenced = false")
        .replace("end_min_count = -3000", "end_min_count = -500")
        .replace("end_max_count = 3000", "end_max_count = 500");
    let axis = |id: u8, valid: bool, c: [i32; 3], mech: [i32; 2], soft: [i32; 2], decel: i32| {
        format!(
            "[[axis]]\naxis_id = {id}\ndevice_id = \"513100000000000{id}\"   # 照合用\nname = \"axis {id}\"\n\
             calibration_valid = {valid}   # 宣言\ncount_at_virtual_0 = {}        # soft_min + margin\n\
             count_at_virtual_512 = {}\ncount_at_virtual_1023 = {}\nmechanical_min_count = {}\n\
             mechanical_max_count = {}\nsoft_min_count = {}\nsoft_max_count = {}\ndecel_zone_count = {decel}\n\n",
            c[0], c[1], c[2], mech[0], mech[1], soft[0], soft[1]
        )
    };
    format!(
        "{head}{}{}{}",
        axis(1, true, [-1950, 0, 1950], [-2742, 2814], [-2300, 2300], 50),
        axis(2, true, [-100, 0, 100], [-500, 520], [-200, 220], 300),
        axis(3, false, [-500, 0, 500], [-1000, 1000], [-600, 600], 300),
    )
}

fn scratch(name: &str) -> std::path::PathBuf {
    let dir = std::env::temp_dir().join(format!("h3cs-server-{name}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

async fn post(srv: &h3cs::server::RunningServer, path: &str, body: &str) -> Resp {
    let auth = bearer();
    http(srv.local_addr, "POST", path, &[("Authorization", &auth), ("X-H3-Operator", "1")], Some(body)).await
}

async fn calibrate_all(srv: &h3cs::server::RunningServer) {
    let r = post(srv, "/v1/calibrate", "").await;
    assert_eq!(r.status, 200, "{r:?}");
    wait_status(srv, 8 * S, "all CAL_RESULT OK", |s| {
        s.axes.iter().all(|a| {
            a.calibration.as_ref().is_some_and(|c| c.result == h3cs::protocol::CalResultCode::Ok)
                && a.referenced
                && a.mcu_state == Some(McuState::Disarmed)
                && a.calibration_stale
        })
    })
    .await;
}

#[tokio::test]
async fn calibration_apply_keeps_margins_persists_with_backup_and_arms_without_restart() {
    let dir = scratch("apply");
    let path = dir.join("rig.toml");
    let text = hot_cfg_text();
    std::fs::write(&path, &text).unwrap();
    let cfg = Config::from_toml_str(&text).unwrap();
    let srv = start_sim_with_path(cfg, Some(path.clone())).await;
    let st = wait_status(&srv, 3 * S, "connected", |s| s.axes.iter().all(|a| a.connected && a.mcu_state == Some(McuState::Disarmed))).await;
    assert_eq!(st.config_path.as_deref(), Some(path.to_str().unwrap()));
    assert_eq!(st.axes.iter().map(|a| a.config_stroke).collect::<Vec<_>>(), [5556, 1020, 2000]);

    // Nothing measured yet.
    let r = post(&srv, "/v1/calibration/apply", r#"{"axis":1}"#).await;
    assert_eq!((r.status, r.json()["error"].as_str()), (409, Some("no_calibration_result")), "{r:?}");
    let r = post(&srv, "/v1/calibration/apply", r#"{"axis":4}"#).await;
    assert_eq!((r.status, r.json()["error"].as_str()), (400, Some("bad_axis")), "{r:?}");

    // While calibrating: refused.
    let r = post(&srv, "/v1/calibrate", "").await;
    assert_eq!(r.status, 200, "{r:?}");
    wait_status(&srv, 2 * S, "CALIBRATING", |s| s.axes.iter().any(|a| a.mcu_state == Some(McuState::Calibrating))).await;
    let r = post(&srv, "/v1/calibration/declare", r#"{"axis":3}"#).await;
    assert_eq!((r.status, r.json()["error"].as_str()), (409, Some("calibrating")), "{r:?}");
    wait_status(&srv, 8 * S, "all CAL_RESULT OK", |s| {
        s.axes.iter().all(|a| a.calibration.is_some() && a.calibration_stale && a.mcu_state == Some(McuState::Disarmed))
    })
    .await;
    assert_eq!(std::fs::read_to_string(&path).unwrap(), text, "nothing written so far");

    // Axis 1: margin 350 kept; stroke 1000 vs 5556 → declaration dropped.
    let r = post(&srv, "/v1/calibration/apply", r#"{"axis":1}"#).await;
    assert_eq!(r.status, 200, "{r:?}");
    let b = r.json();
    assert_eq!((b["ok"].as_bool(), b["axis"].as_u64(), b["stroke"].as_i64(), b["prev_stroke"].as_i64()), (Some(true), Some(1), Some(1000), Some(5556)));
    assert_eq!((b["within_tolerance"].as_bool(), b["calibration_valid"].as_bool()), (Some(false), Some(false)));
    assert_eq!(b["path"].as_str(), path.to_str());
    let backup = std::path::PathBuf::from(b["backup"].as_str().unwrap());
    assert_eq!(backup.parent(), Some(dir.as_path()));
    assert_eq!(std::fs::read_to_string(&backup).unwrap(), text, "backup is the file before the apply");
    let written = std::fs::read_to_string(&path).unwrap();
    assert!(written.contains("count_at_virtual_0 = 400        # soft_min + margin"), "value replaced, comment kept: {written}");
    let a1 = Config::load(&path).unwrap().axes[0].clone();
    assert_eq!((a1.mechanical_min_count, a1.mechanical_max_count, a1.soft_min_count, a1.soft_max_count), (0, 1000, 50, 950));
    assert_eq!((a1.count_at_virtual_0, a1.count_at_virtual_512, a1.count_at_virtual_1023), (400, 500, 600));
    assert!(!a1.calibration_valid);
    let st = wait_status(&srv, S, "axis 1 no longer stale", |s| !s.axes[0].calibration_stale).await;
    assert_eq!(st.axes[0].config_stroke, 1000);
    assert!(st.axes[1].calibration_stale && st.axes[2].calibration_stale);

    // Axis 2: same unit, 1000 vs 1020 → within ±30, declaration inherited.
    let r = post(&srv, "/v1/calibration/apply", r#"{"axis":2}"#).await;
    assert_eq!(r.status, 200, "{r:?}");
    assert_eq!((r.json()["within_tolerance"].as_bool(), r.json()["calibration_valid"].as_bool()), (Some(true), Some(true)));
    assert_ne!(r.json()["backup"], b["backup"], "a second backup in the same second gets its own name");
    // Axis 3: 1000 vs 2000 → not within; it was not declared anyway.
    let r = post(&srv, "/v1/calibration/apply", r#"{"axis":3}"#).await;
    assert_eq!(r.status, 200, "{r:?}");
    assert_eq!((r.json()["within_tolerance"].as_bool(), r.json()["calibration_valid"].as_bool()), (Some(false), Some(false)));
    let out = Config::load(&path).unwrap();
    assert_eq!((out.axes[1].count_at_virtual_0, out.axes[1].count_at_virtual_1023), (400, 580), "axis 2 margins 100 / 120");
    assert!(out.axes[1].calibration_valid);

    // Only the declarations of axes 1 and 3 stand between the rig and ARM.
    let st = wait_status(&srv, S, "stale cleared", |s| blocker_codes(s) == ["calibration_invalid"]).await;
    assert_eq!(st.blockers.len(), 2, "{:?}", st.blockers);
    for axis in [1, 3] {
        let r = post(&srv, "/v1/calibration/declare", &format!(r#"{{"axis":{axis}}}"#)).await;
        assert_eq!(r.status, 200, "{r:?}");
        assert_eq!(r.json()["calibration_valid"], true);
    }
    assert!(Config::load(&path).unwrap().axes.iter().all(|a| a.calibration_valid));
    let st = wait_state(&srv, RigState::ReadyToArm, 3 * S).await;
    assert!(st.calibration_valid);
    assert!(st.axes.iter().all(|a| a.configured), "CONFIG resent with the new geometry");
    let r = post(&srv, "/v1/arm", "").await;
    assert_eq!(r.status, 200, "ARM without a restart: {r:?}");
    wait_state(&srv, RigState::Holding, 3 * S).await;

    // Armed: every edit is refused and the file stays.
    let before = std::fs::read_to_string(&path).unwrap();
    for (route, body) in [
        ("/v1/calibration/apply", r#"{"axis":1}"#),
        ("/v1/calibration/declare", r#"{"axis":1}"#),
        ("/v1/units/assign", r#"{"axis":1,"device_id":"5131000000000009","confirm_suffix":"0009"}"#),
    ] {
        let r = post(&srv, route, body).await;
        assert_eq!((r.status, r.json()["error"].as_str()), (409, Some("not_disarmed")), "{route}: {r:?}");
    }
    assert_eq!(std::fs::read_to_string(&path).unwrap(), before);
    let ev = http(srv.local_addr, "GET", "/v1/events?n=1000", &[], None).await.json();
    let applied = ev["events"].as_array().unwrap().iter().filter(|e| e["kind"] == "config_applied").count();
    assert_eq!(applied, 5);
    srv.shutdown().await;
    let _ = std::fs::remove_dir_all(&dir);
}

#[cfg(unix)]
#[tokio::test]
async fn calibration_apply_that_cannot_write_leaves_memory_and_file_unchanged() {
    use std::os::unix::fs::PermissionsExt;
    let dir = scratch("applyfail");
    let path = dir.join("rig.toml");
    let text = hot_cfg_text();
    std::fs::write(&path, &text).unwrap();
    let srv = start_sim_with_path(Config::from_toml_str(&text).unwrap(), Some(path.clone())).await;
    wait_status(&srv, 3 * S, "connected", |s| s.axes.iter().all(|a| a.connected && a.mcu_state == Some(McuState::Disarmed))).await;
    calibrate_all(&srv).await;
    let before = srv.status();
    std::fs::set_permissions(&dir, std::fs::Permissions::from_mode(0o555)).unwrap();
    let r = post(&srv, "/v1/calibration/apply", r#"{"axis":1}"#).await;
    let r2 = post(&srv, "/v1/calibration/declare", r#"{"axis":3}"#).await;
    std::fs::set_permissions(&dir, std::fs::Permissions::from_mode(0o755)).unwrap();
    assert_eq!((r.status, r.json()["error"].as_str()), (500, Some("write_failed")), "{r:?}");
    assert_eq!((r2.status, r2.json()["error"].as_str()), (500, Some("write_failed")), "{r2:?}");
    tokio::time::sleep(Duration::from_millis(100)).await;
    let after = srv.status();
    for (b, a) in before.axes.iter().zip(&after.axes) {
        assert!(a.calibration_stale, "axis {}", a.axis_id);
        assert_eq!((a.config_stroke, a.expected_device_id), (b.config_stroke, b.expected_device_id));
    }
    assert_eq!(blocker_codes(&after), blocker_codes(&before));
    assert_eq!(std::fs::read_to_string(&path).unwrap(), text);
    assert_eq!(std::fs::read_dir(&dir).unwrap().count(), 1, "no backup or temp file left");
    // Without a config file: 500 no_config_path.
    srv.shutdown().await;
    let srv = start_sim(Config::from_toml_str(&text).unwrap()).await;
    let r = post(&srv, "/v1/calibration/declare", r#"{"axis":1}"#).await;
    assert_eq!((r.status, r.json()["error"].as_str()), (500, Some("no_config_path")), "{r:?}");
    srv.shutdown().await;
    let _ = std::fs::remove_dir_all(&dir);
}

// ------------------------------------------------------------------ unit swap (console)

const NEW_UNIT: h3cs::types::DeviceId = h3cs::types::DeviceId(0x5131_00aa_bbcc_8d58);

#[tokio::test]
async fn units_assign_swaps_the_device_id_and_ports_auto_follows_it() {
    let dir = scratch("assign");
    let path = dir.join("rig.toml");
    let cfg = auto_cfg();
    std::fs::write(&path, toml::to_string(&cfg).unwrap()).unwrap();
    let usb = rig_usb();
    let ports: std::sync::Arc<dyn h3cs::server::SerialPorts> = usb.clone();
    let srv = server::start(
        cfg,
        ServerOptions {
            admin_token: Some(TOKEN.into()),
            bind_override: Some("127.0.0.1:0".parse().unwrap()),
            config_path: Some(path.clone()),
            serial_ports: Some(ports),
        },
    )
    .await
    .expect("server start");
    wait_state(&srv, RigState::ReadyToArm, 5 * S).await;
    // The spare is plugged in: listed as unregistered, never opened.
    const P4: &str = "/dev/cu.usbmodem14701";
    usb.plug(P4, fake_usb::unit(NEW_UNIT, NEW_UNIT, 2));
    let st = wait_status(&srv, 3 * S, "spare listed", |s| s.unregistered_units.iter().any(|u| u.port == P4)).await;
    let spare = st.unregistered_units.iter().find(|u| u.port == P4).unwrap();
    assert_eq!((spare.usb_serial.as_str(), spare.device_id), ("00:AA:BB:CC:8D:58", Some(NEW_UNIT)));
    assert!(st.unregistered_units.iter().any(|u| u.port == "/dev/cu.usbmodem2101" && u.device_id == Some(FOREIGN)));
    assert!(!st.unregistered_units.iter().any(|u| u.port.starts_with("/dev/tty.") || u.port == P1));
    assert_eq!(usb.opens_of(P4), 0);

    // The suffix shown on the unit's screen must match.
    let body = |suffix: &str| format!(r#"{{"axis":2,"device_id":"{NEW_UNIT}","confirm_suffix":"{suffix}"}}"#);
    let r = post(&srv, "/v1/units/assign", &body("8D59")).await;
    assert_eq!((r.status, r.json()["error"].as_str()), (400, Some("confirm_mismatch")), "{r:?}");
    let r = post(&srv, "/v1/units/assign", &format!(r#"{{"axis":2,"device_id":"{}","confirm_suffix":"0a01"}}"#, ID[0])).await;
    assert_eq!((r.status, r.json()["error"].as_str()), (409, Some("device_in_use")), "{r:?}");
    assert_eq!(usb.opens_of(P4), 0);
    // The broken unit is pulled first (as the console's steps say).
    let old2 = usb.unplug(P2);
    let r = post(&srv, "/v1/units/assign", &body("8d58")).await;
    assert_eq!(r.status, 200, "{r:?}");
    let b = r.json();
    assert_eq!((b["device_id"].as_str(), b["previous_device_id"].as_str()), (Some(NEW_UNIT.to_string().as_str()), Some(ID[1].to_string().as_str())));
    assert_eq!(b["calibration_valid"], false);
    let saved = Config::load(&path).unwrap();
    assert_eq!(saved.axes[1].device_id, NEW_UNIT);
    assert!(!saved.axes[1].calibration_valid);

    // Discovery now opens the spare for axis 2; the declaration is gone.
    let st = wait_status(&srv, 5 * S, "axis 2 on the spare", |s| axis_port(s, 2).as_deref() == Some(P4)).await;
    assert_eq!(st.axes[1].expected_device_id, NEW_UNIT);
    assert_eq!(usb.opens_of(P4), 1);
    let st = wait_status(&srv, 2 * S, "spare no longer unregistered", |s| !s.unregistered_units.iter().any(|u| u.port == P4)).await;
    assert!(st.blockers.iter().any(|b| b == "calibration_invalid: axis2 calibration_valid = false"), "{:?}", st.blockers);
    assert!(!st.blockers.iter().any(|b| b.starts_with("axis_device_mismatch")), "{:?}", st.blockers);

    // The old unit plugged back in is no longer ours: never reopened.
    usb.plug(P2, old2);
    wait_status(&srv, 3 * S, "old unit unregistered", |s| s.unregistered_units.iter().any(|u| u.port == P2)).await;
    tokio::time::sleep(h3cs::server::SERIAL_RETRY + h3cs::server::DISCOVERY_PERIOD).await;
    assert_eq!(usb.opens_of(P2), 1, "{:?}", usb.opened());
    assert_eq!(axis_port(&srv.status(), 2).as_deref(), Some(P4));
    srv.shutdown().await;
    let _ = std::fs::remove_dir_all(&dir);
}

#[tokio::test]
async fn units_assign_with_named_ports_changes_only_the_cross_check() {
    let dir = scratch("assign-sim");
    let path = dir.join("rig.toml");
    let text = hot_cfg_text();
    std::fs::write(&path, &text).unwrap();
    let srv = start_sim_with_path(Config::from_toml_str(&text).unwrap(), Some(path.clone())).await;
    wait_status(&srv, 3 * S, "connected", |s| s.axes.iter().all(|a| a.connected)).await;
    let r = post(&srv, "/v1/units/assign", r#"{"axis":3,"device_id":"0","confirm_suffix":"0000"}"#).await;
    assert_eq!((r.status, r.json()["error"].as_str()), (400, Some("bad_device_id")), "{r:?}");
    let r = post(&srv, "/v1/units/assign", r#"{"axis":3,"device_id":"5131000000000abc","confirm_suffix":"0ABC"}"#).await;
    assert_eq!(r.status, 200, "{r:?}");
    let st = wait_status(&srv, S, "mismatch", |s| s.blockers.iter().any(|b| b.starts_with("axis_device_mismatch: axis3"))).await;
    assert_eq!(st.axes[2].expected_device_id, h3cs::types::DeviceId(0x5131_0000_0000_0abc));
    let written = std::fs::read_to_string(&path).unwrap();
    assert!(written.contains("device_id = \"5131000000000abc\"   # 照合用"), "{written}");
    srv.shutdown().await;
    let _ = std::fs::remove_dir_all(&dir);
}
