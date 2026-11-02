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
    fn start(ws: Ws, session: String, first_seq: u64) -> Self {
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
                        let text = pose_json(&session, seq, seq, POSE.pitch, POSE.roll, POSE.yaw, GAIN);
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
    wait_state(&srv, RigState::ReadyToArm, 3 * S).await;
    let id = new_session(srv.local_addr).await;
    let ws = ws_connect(srv.local_addr, &id).await;
    let streamer = Streamer::start(ws, id, 1);
    wait_status(&srv, S, "fresh session", |s| s.session.as_ref().is_some_and(|v| v["fresh"] == true)).await;

    let r = admin(srv.local_addr, "/v1/arm").await;
    assert_eq!(r.status, 200, "arm: {r:?}");
    wait_state(&srv, RigState::Holding, 3 * S).await;
    let r = admin(srv.local_addr, "/v1/start").await;
    assert_eq!(r.status, 200, "start: {r:?}");
    wait_state(&srv, RigState::Active, 2 * S).await;
    (srv, streamer)
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
        ServerOptions { admin_token: Some(TOKEN.into()), bind_override: Some("0.0.0.0:0".parse().unwrap()), config_path: None },
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
    let streamer2 = Streamer::start(ws, id, 1);
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
