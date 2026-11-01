//! sim_axis: protocol.md §3–§6 behavior of the MCU reference model.

use std::time::Duration;

use h3cs::protocol::cobs::FrameAccumulator;
use h3cs::protocol::{
    decode_mcu_frame, msg_type, role_flags, Ack, AckReason, AckResult, CalResult, CalResultCode,
    Config, HostMsg, McuMsg, SetPosition, Status, MAX_ENCODED_LEN,
};
use h3cs::sim_axis::{sim_pair, SimMcu, SimParams, DEDUPE_DEPTH};
use h3cs::types::{status_flags, FaultCode, McuState};
use tokio::io::{AsyncReadExt, AsyncWriteExt, DuplexStream};

const EPOCH: u32 = 3;
const MS: u64 = 1_000;

fn config(epoch: u32) -> Config {
    Config {
        request_id: 0, // filled by Harness::config
        session_epoch: epoch,
        calibration_hash: 0x1234_5678,
        soft_min_count: -2700,
        soft_max_count: 2700,
        max_vel_count_s: 4000,
        max_acc_count_s2: 20000,
        following_error_limit_count: 200,
        overspeed_count_s: 6000,
        command_timeout_ms: 100,
        stop_timeout_ms: 1500,
        role_flags: role_flags::POWER_CONTROLLER,
    }
}

/// Drives a pure SimMcu with injected time.
struct Harness {
    sim: SimMcu,
    now: u64,
    rid: u32,
    seq: u32,
    frame: u32,
    epoch: u32,
    out: Vec<McuMsg>,
}

impl Harness {
    fn new(params: SimParams) -> Self {
        let mut sim = SimMcu::new(params);
        let out = sim.tick(0);
        Harness {
            sim,
            now: 0,
            rid: 0,
            seq: 0,
            frame: 0,
            epoch: EPOCH,
            out,
        }
    }
    fn default() -> Self {
        Self::new(SimParams::default())
    }
    fn next_rid(&mut self) -> u32 {
        self.rid += 1;
        self.rid
    }
    fn send(&mut self, msg: HostMsg) -> Vec<McuMsg> {
        let out = self.sim.handle(&msg, self.now);
        self.out.extend(out.iter().copied());
        out
    }
    /// Send an ACKed command and return its ACK.
    fn cmd(&mut self, msg: HostMsg) -> Ack {
        let out = self.send(msg);
        acks(&out).into_iter().next().expect("ACK expected")
    }
    fn config_with(&mut self, mut c: Config) -> Ack {
        c.request_id = self.next_rid();
        self.cmd(HostMsg::Config(c))
    }
    fn config(&mut self) -> Ack {
        let c = config(self.epoch);
        self.config_with(c)
    }
    fn arm(&mut self) -> Ack {
        let (r, e) = (self.next_rid(), self.epoch);
        self.cmd(HostMsg::ArmPrepare {
            request_id: r,
            session_epoch: e,
        })
    }
    fn start(&mut self) -> Ack {
        let (r, e) = (self.next_rid(), self.epoch);
        self.cmd(HostMsg::Start {
            request_id: r,
            session_epoch: e,
        })
    }
    fn stop(&mut self) -> Ack {
        let (r, e) = (self.next_rid(), self.epoch);
        self.cmd(HostMsg::ControlledStop {
            request_id: r,
            session_epoch: e,
            reason: 1,
        })
    }
    fn disarm(&mut self) -> Ack {
        let r = self.next_rid();
        self.cmd(HostMsg::Disarm { request_id: r })
    }
    fn fault_reset(&mut self) -> Ack {
        let r = self.next_rid();
        self.cmd(HostMsg::FaultReset { request_id: r })
    }
    fn host_fault(&mut self) -> Ack {
        let r = self.next_rid();
        self.cmd(HostMsg::HostFault {
            request_id: r,
            fault_code: FaultCode::FollowingError,
        })
    }
    fn set_reference(&mut self, count: i32) -> Ack {
        let r = self.next_rid();
        self.cmd(HostMsg::SetReference {
            request_id: r,
            reference_count: count,
        })
    }
    fn set_position_msg(&mut self, target: i32) -> SetPosition {
        self.seq += 1;
        self.frame += 1;
        SetPosition {
            session_epoch: self.epoch,
            seq: self.seq,
            group_frame_id: self.frame,
            target_count: target,
            max_vel_count_s: 4000,
            max_acc_count_s2: 20000,
            valid_for_ms: 50,
            apply_at_us: 0,
        }
    }
    fn set_position(&mut self, target: i32) {
        let m = self.set_position_msg(target);
        self.send(HostMsg::SetPosition(m));
    }
    /// Advance `ms` milliseconds with 1 ms ticks, no SET_POSITION.
    fn idle(&mut self, ms: u64) {
        for _ in 0..ms {
            self.now += MS;
            let out = self.sim.tick(self.now);
            self.out.extend(out);
        }
    }
    /// Advance `ms` milliseconds sending SET_POSITION(target) every 10 ms.
    fn drive(&mut self, ms: u64, target: i32) {
        for i in 0..ms {
            if i % 10 == 0 {
                self.set_position(target);
            }
            self.now += MS;
            let out = self.sim.tick(self.now);
            self.out.extend(out);
        }
    }
    fn state(&self) -> McuState {
        self.sim.state()
    }
    /// CONFIG + ARM and wait for HOLDING (with lease).
    fn reach_holding(&mut self) {
        assert_eq!(self.config().result, AckResult::Accepted);
        assert_eq!(self.arm().result, AckResult::Accepted);
        assert_eq!(self.state(), McuState::Arming);
        let here = self.sim.actual_count();
        self.drive(60, here);
        assert_eq!(self.state(), McuState::Holding);
    }
    fn reach_active(&mut self) {
        self.reach_holding();
        let here = self.sim.actual_count();
        self.set_position(here);
        assert_eq!(self.start().result, AckResult::Accepted);
        assert_eq!(self.state(), McuState::Active);
    }
    fn statuses(&self) -> Vec<Status> {
        self.out
            .iter()
            .filter_map(|m| match m {
                McuMsg::Status(s) => Some(*s),
                _ => None,
            })
            .collect()
    }
    fn seen_states(&self) -> Vec<McuState> {
        let mut v: Vec<McuState> = Vec::new();
        for s in self.statuses() {
            let st = McuState::from_u8(s.mcu_state).unwrap();
            if v.last() != Some(&st) {
                v.push(st);
            }
        }
        v
    }
    fn fault_events(&self) -> Vec<h3cs::protocol::FaultEvent> {
        self.out
            .iter()
            .filter_map(|m| match m {
                McuMsg::FaultEvent(f) => Some(*f),
                _ => None,
            })
            .collect()
    }
}

fn acks(out: &[McuMsg]) -> Vec<Ack> {
    out.iter()
        .filter_map(|m| match m {
            McuMsg::Ack(a) => Some(*a),
            _ => None,
        })
        .collect()
}

fn rejected(a: Ack, reason: AckReason) {
    assert_eq!((a.result, a.reason), (AckResult::Rejected, reason), "{a:?}");
}

fn accepted(a: Ack) {
    assert_eq!(
        (a.result, a.reason),
        (AckResult::Accepted, AckReason::None),
        "{a:?}"
    );
}

// ---------------------------------------------------------------------------
// Boot, HELLO, CONFIG
// ---------------------------------------------------------------------------

#[test]
fn boots_disarmed_with_safe_outputs() {
    let h = Harness::default();
    assert_eq!(h.state(), McuState::Disarmed);
    let s = h.sim.status();
    assert_eq!(s.mcu_state, McuState::Disarmed as u8);
    assert_eq!(s.pwm_permille, 0);
    assert_eq!(
        s.flags & (status_flags::POWER_ON_CMD | status_flags::BRAKE_RELEASE_CMD),
        0
    );
    assert_eq!(s.flags & status_flags::CURRENT_VALID, 0);
    assert_eq!(s.current_ma, 0);
    assert_eq!(h.statuses().len(), 1, "first tick emits a STATUS");
}

#[test]
fn hello_answers_in_every_state() {
    let mut h = Harness::default();
    let check = |h: &mut Harness, state: McuState| {
        let out = h.send(HostMsg::HelloReq { request_id: 77 });
        let hello = out
            .iter()
            .find_map(|m| match m {
                McuMsg::Hello(x) => Some(*x),
                _ => None,
            })
            .expect("HELLO");
        assert_eq!(hello.request_id, 77);
        assert_eq!(hello.mcu_state, state as u8);
        assert_eq!(hello.device_id, SimParams::default().device_id);
        assert_eq!(hello.protocol_version, 1);
        assert!(acks(&out).is_empty(), "HELLO_REQ is not ACKed");
        hello
    };
    assert_eq!(check(&mut h, McuState::Disarmed).calibration_hash, 0);
    h.reach_active();
    assert_eq!(
        check(&mut h, McuState::Active).calibration_hash,
        0x1234_5678
    );
    h.host_fault();
    check(&mut h, McuState::Fault);
}

#[test]
fn clock_sync_replies() {
    let mut h = Harness::default();
    h.idle(5);
    let out = h.send(HostMsg::ClockSync {
        request_id: 9,
        pc_time_us: 1_000_000,
    });
    let r = out
        .iter()
        .find_map(|m| match m {
            McuMsg::ClockSyncReply(r) => Some(*r),
            _ => None,
        })
        .unwrap();
    assert_eq!(
        (r.request_id, r.pc_time_us, r.mcu_rx_us),
        (9, 1_000_000, 5_000)
    );
}

#[test]
fn config_sets_configured_flag_and_immediate_status() {
    let mut h = Harness::default();
    let before = h.statuses().len();
    accepted(h.config());
    assert!(h.sim.is_configured());
    assert_eq!(h.sim.session_epoch(), Some(EPOCH));
    let s = h.statuses();
    assert_eq!(s.len(), before + 1, "flag change → immediate STATUS");
    let last = s.last().unwrap();
    assert_ne!(last.flags & status_flags::CONFIGURED, 0);
    assert_ne!(last.flags & status_flags::POWER_CONTROLLER, 0);
    assert_eq!(
        last.flags & status_flags::POWER_ON_CMD,
        0,
        "DISARMED: power off"
    );
}

#[test]
fn config_invalid_params_rejected() {
    let base = config(EPOCH);
    let bad = [
        Config {
            soft_min_count: 100,
            soft_max_count: 100,
            ..base
        },
        Config {
            soft_min_count: 200,
            soft_max_count: -200,
            ..base
        },
        Config {
            max_vel_count_s: 0,
            ..base
        },
        Config {
            max_acc_count_s2: 0,
            ..base
        },
        Config {
            following_error_limit_count: 0,
            ..base
        },
        Config {
            overspeed_count_s: 0,
            ..base
        },
        Config {
            command_timeout_ms: 0,
            ..base
        },
        Config {
            stop_timeout_ms: 0,
            ..base
        },
    ];
    for c in bad {
        let mut h = Harness::default();
        rejected(h.config_with(c), AckReason::InvalidParam);
        assert!(!h.sim.is_configured());
    }
}

#[test]
fn config_only_in_disarmed() {
    let mut h = Harness::default();
    h.reach_holding();
    rejected(h.config(), AckReason::BadState);
    h.start();
    rejected(h.config(), AckReason::BadState);
}

#[test]
fn request_id_zero_rejected() {
    let mut h = Harness::default();
    let a = h.cmd(HostMsg::Disarm { request_id: 0 });
    rejected(a, AckReason::InvalidParam);
}

// ---------------------------------------------------------------------------
// ARM / START / STOP / DISARM transitions (§5)
// ---------------------------------------------------------------------------

#[test]
fn arm_prepare_rejections() {
    // Not configured.
    let mut h = Harness::default();
    rejected(h.arm(), AckReason::NotConfigured);
    // Not referenced.
    let mut h = Harness::new(SimParams {
        start_referenced: false,
        ..SimParams::default()
    });
    h.config();
    rejected(h.arm(), AckReason::NotReferenced);
    // Epoch mismatch.
    let mut h = Harness::default();
    h.config();
    let r = h.next_rid();
    rejected(
        h.cmd(HostMsg::ArmPrepare {
            request_id: r,
            session_epoch: EPOCH + 1,
        }),
        AckReason::EpochMismatch,
    );
    assert_eq!(h.state(), McuState::Disarmed);
    // Bad state (already armed).
    h.reach_holding();
    rejected(h.arm(), AckReason::BadState);
    // Fault active.
    h.host_fault();
    rejected(h.arm(), AckReason::FaultActive);
}

#[test]
fn arming_sequence_power_then_brake_then_holding() {
    let mut h = Harness::new(SimParams {
        initial_count: 321,
        ..SimParams::default()
    });
    h.config();
    accepted(h.arm());
    let s = h.sim.status();
    assert_eq!(s.mcu_state, McuState::Arming as u8);
    assert_ne!(s.flags & status_flags::POWER_ON_CMD, 0, "power on first");
    assert_eq!(
        s.flags & status_flags::BRAKE_RELEASE_CMD,
        0,
        "brake still applied"
    );
    assert_eq!(s.pwm_permille, 0);
    h.drive(26, 321);
    let s = h.sim.status();
    assert_eq!(s.mcu_state, McuState::Arming as u8);
    assert_ne!(
        s.flags & status_flags::BRAKE_RELEASE_CMD,
        0,
        "brake released after power settle"
    );
    h.drive(30, 321);
    assert_eq!(h.state(), McuState::Holding);
    assert_eq!(h.sim.target_count(), 321, "hold target = actual");
    assert_eq!(
        h.seen_states(),
        vec![McuState::Disarmed, McuState::Arming, McuState::Holding]
    );
}

#[test]
fn non_power_controller_never_commands_power() {
    let mut h = Harness::default();
    h.config_with(Config {
        role_flags: 0,
        ..config(EPOCH)
    });
    h.arm();
    h.drive(60, 0);
    assert_eq!(h.state(), McuState::Holding);
    for s in h.statuses() {
        assert_eq!(
            s.flags & (status_flags::POWER_ON_CMD | status_flags::POWER_CONTROLLER),
            0
        );
    }
}

#[test]
fn start_rejections_and_transition() {
    let mut h = Harness::default();
    h.config();
    rejected(h.start(), AckReason::BadState);
    h.reach_holding_from_configured();
    let r = h.next_rid();
    rejected(
        h.cmd(HostMsg::Start {
            request_id: r,
            session_epoch: EPOCH + 1,
        }),
        AckReason::EpochMismatch,
    );
    assert_eq!(h.state(), McuState::Holding);
    h.set_position(0);
    accepted(h.start());
    assert_eq!(h.state(), McuState::Active);
    rejected(h.start(), AckReason::BadState);
}

#[test]
fn start_rejected_without_unexpired_set_position() {
    let mut h = Harness::default();
    h.reach_holding();
    // The lease frames from reach_holding are 50 ms valid; let them expire
    // while a lease-only stream is absent but the lease itself is still alive.
    h.idle(60);
    assert_eq!(h.state(), McuState::Holding);
    rejected(h.start(), AckReason::BadState);
    assert_eq!(h.state(), McuState::Holding);
    assert!(h.seen_states().iter().all(|s| *s != McuState::Active));
    // A fresh frame makes START acceptable again.
    h.set_position(0);
    accepted(h.start());
    assert_eq!(h.state(), McuState::Active);
}

#[test]
fn start_ignores_frames_from_before_arm_prepare() {
    let mut h = Harness::default();
    h.config();
    // A long-valid frame received while DISARMED must not qualify START.
    let mut m = h.set_position_msg(0);
    m.valid_for_ms = 500;
    h.send(HostMsg::SetPosition(m));
    accepted(h.arm());
    h.idle(55);
    assert_eq!(h.state(), McuState::Holding, "lease still alive");
    rejected(h.start(), AckReason::BadState);
    h.set_position(0);
    accepted(h.start());
}

impl Harness {
    fn reach_holding_from_configured(&mut self) {
        accepted(self.arm());
        self.drive(60, self.sim.actual_count());
        assert_eq!(self.state(), McuState::Holding);
    }
}

#[test]
fn holding_ignores_target_but_refreshes_lease() {
    let mut h = Harness::default();
    h.reach_holding();
    h.drive(500, 1500);
    assert_eq!(
        h.state(),
        McuState::Holding,
        "lease kept alive by SET_POSITION"
    );
    assert_eq!(h.sim.actual_count(), 0);
    assert_eq!(h.sim.target_count(), 0);
    assert_eq!(h.sim.last_applied_seq(), 0, "not applied in HOLDING");
    assert_eq!(h.sim.last_rx_seq(), h.seq, "but recorded as received");
}

#[test]
fn active_tracks_target_with_limits() {
    let mut h = Harness::default();
    h.reach_active();
    h.drive(1500, 1000);
    assert_eq!(h.state(), McuState::Active);
    assert_eq!(h.sim.actual_count(), 1000);
    assert_eq!(h.sim.target_count(), 1000);
    let st = h.sim.status();
    assert_eq!(st.last_applied_seq, h.seq);
    assert_eq!(st.last_applied_frame, h.frame);
    // Velocity never exceeded the CONFIG max during the move.
    for s in h.statuses() {
        assert!(s.velocity_count_s.abs() <= 4000, "{s:?}");
        assert!(s.pwm_permille.abs() <= 1000);
    }
}

#[test]
fn frame_limits_are_capped_by_config_and_can_be_lower() {
    // Frame asks for a lower velocity: the move takes longer.
    let mut h = Harness::default();
    h.reach_active();
    for _ in 0..50 {
        let mut m = h.set_position_msg(1000);
        m.max_vel_count_s = 1000;
        h.send(HostMsg::SetPosition(m));
        h.idle(10);
    }
    // 500 ms at ≤ 1000 count/s → at most ~500 counts.
    let a = h.sim.actual_count();
    assert!(a > 300 && a <= 510, "actual {a}");
    // Frame asks for more than CONFIG: capped at CONFIG (4000).
    let mut h = Harness::default();
    h.reach_active();
    for _ in 0..30 {
        let mut m = h.set_position_msg(2600);
        m.max_vel_count_s = 100_000;
        m.max_acc_count_s2 = 10_000_000;
        h.send(HostMsg::SetPosition(m));
        h.idle(10);
    }
    for s in h.statuses() {
        assert!(s.velocity_count_s.abs() <= 4000, "{s:?}");
    }
}

#[test]
fn soft_limit_clamps_target() {
    let mut h = Harness::default();
    h.reach_active();
    h.drive(2000, 99_999);
    assert_eq!(h.state(), McuState::Active);
    assert_eq!(h.sim.control_target_count(), 2700);
    assert_eq!(h.sim.actual_count(), 2700);
    h.drive(2000, -99_999);
    assert_eq!(h.sim.actual_count(), -2700);
}

#[test]
fn controlled_stop_decelerates_to_holding() {
    let mut h = Harness::default();
    h.reach_active();
    h.drive(200, 2600);
    let moving_at = h.sim.actual_count();
    assert!(moving_at > 0);
    accepted(h.stop());
    assert_eq!(h.state(), McuState::Stopping);
    h.drive(500, 2600);
    assert_eq!(h.state(), McuState::Holding);
    let held = h.sim.actual_count();
    assert!(
        held > moving_at && held < 2600,
        "decelerated, not jumped to target: {held}"
    );
    // Late targets do not resume motion.
    h.drive(200, -2000);
    assert_eq!(h.state(), McuState::Holding);
    assert_eq!(h.sim.actual_count(), held);
    let seen = h.seen_states();
    assert_eq!(
        &seen[seen.len() - 3..],
        &[McuState::Active, McuState::Stopping, McuState::Holding]
    );
}

#[test]
fn controlled_stop_epoch_mismatch_and_noop_states() {
    let mut h = Harness::default();
    h.config();
    accepted(h.stop()); // DISARMED: no-op
    assert_eq!(h.state(), McuState::Disarmed);
    h.reach_active_from_configured();
    let r = h.next_rid();
    rejected(
        h.cmd(HostMsg::ControlledStop {
            request_id: r,
            session_epoch: EPOCH + 9,
            reason: 0,
        }),
        AckReason::EpochMismatch,
    );
    assert_eq!(h.state(), McuState::Active);
}

impl Harness {
    fn reach_active_from_configured(&mut self) {
        self.reach_holding_from_configured();
        self.set_position(self.sim.actual_count());
        accepted(self.start());
    }
}

#[test]
fn valid_for_expiry_stops_to_holding() {
    let mut h = Harness::default();
    h.reach_active();
    h.drive(100, 2000);
    let frames_before = h.sim.last_applied_frame();
    // One frame with a short validity and a longer command timeout window.
    let mut m = h.set_position_msg(2000);
    m.valid_for_ms = 20;
    h.send(HostMsg::SetPosition(m));
    h.idle(21);
    assert_eq!(h.state(), McuState::Stopping, "target expired");
    // Keep the lease alive; targets are not applied while stopping.
    h.drive(300, 2000);
    assert_eq!(h.state(), McuState::Holding);
    assert_eq!(h.sim.last_applied_frame(), frames_before + 1);
    assert_eq!(h.sim.fault_code(), FaultCode::None);
}

#[test]
fn disarm_from_holding() {
    let mut h = Harness::default();
    h.reach_holding();
    accepted(h.disarm());
    assert_eq!(h.state(), McuState::Disarming);
    let s = h.sim.status();
    assert_eq!(
        s.flags & status_flags::BRAKE_RELEASE_CMD,
        0,
        "brake applied first"
    );
    assert_ne!(
        s.flags & status_flags::POWER_ON_CMD,
        0,
        "power stays on during brake wait"
    );
    h.idle(31);
    assert_eq!(h.state(), McuState::Disarmed);
    let s = h.sim.status();
    assert_eq!(
        s.flags & (status_flags::POWER_ON_CMD | status_flags::BRAKE_RELEASE_CMD),
        0
    );
    assert_eq!(s.pwm_permille, 0);
    assert_eq!(
        h.sim.fault_code(),
        FaultCode::None,
        "no lease needed while disarming"
    );
}

#[test]
fn disarm_from_active_decelerates_first() {
    let mut h = Harness::default();
    h.reach_active();
    h.drive(200, 2600);
    let at = h.sim.actual_count();
    accepted(h.disarm());
    assert_eq!(h.state(), McuState::Stopping);
    h.idle(400); // no SET_POSITION: exempt from the lease
    assert_eq!(h.state(), McuState::Disarmed);
    assert!(h.sim.actual_count() > at);
    let seen = h.seen_states();
    assert_eq!(
        &seen[seen.len() - 4..],
        &[
            McuState::Active,
            McuState::Stopping,
            McuState::Disarming,
            McuState::Disarmed
        ]
    );
    assert!(h.fault_events().is_empty());
}

#[test]
fn disarm_from_arming_and_idempotent_in_disarmed() {
    let mut h = Harness::default();
    accepted(h.disarm());
    assert_eq!(h.state(), McuState::Disarmed);
    h.config();
    h.arm();
    accepted(h.disarm());
    assert_eq!(h.state(), McuState::Disarming);
    h.idle(40);
    assert_eq!(h.state(), McuState::Disarmed);
}

#[test]
fn rearm_after_disarm_works() {
    let mut h = Harness::default();
    h.reach_holding();
    h.disarm();
    h.idle(40);
    h.epoch += 1;
    h.seq = 0; // new epoch restarts seq
    h.reach_holding();
    h.set_position(0);
    accepted(h.start());
    h.drive(500, 500);
    assert_eq!(h.sim.actual_count(), 500);
}

// ---------------------------------------------------------------------------
// SET_POSITION staleness
// ---------------------------------------------------------------------------

#[test]
fn set_position_stale_seq_and_epoch_dropped() {
    let mut h = Harness::default();
    h.reach_active();
    h.drive(10, 0);
    let seq = h.seq;
    let drops = h.sim.stale_drop_count();
    // Same seq again.
    let mut m = h.set_position_msg(1000);
    m.seq = seq;
    h.send(HostMsg::SetPosition(m));
    // Older seq.
    m.seq = seq - 1;
    h.send(HostMsg::SetPosition(m));
    // Wrong epoch with a fresh seq.
    m.seq = seq + 100;
    m.session_epoch = EPOCH + 1;
    h.send(HostMsg::SetPosition(m));
    assert_eq!(h.sim.stale_drop_count(), drops + 3);
    assert_eq!(h.sim.last_rx_seq(), seq);
    assert_eq!(h.sim.control_target_count(), 0, "stale targets not applied");
    assert_eq!(h.sim.status().stale_drop_count, drops + 3);
}

#[test]
fn set_position_before_config_dropped() {
    let mut h = Harness::default();
    h.set_position(10);
    assert_eq!(h.sim.stale_drop_count(), 1);
    assert_eq!(h.sim.last_rx_seq(), 0);
}

#[test]
fn stale_frames_do_not_refresh_lease() {
    let mut h = Harness::default();
    h.reach_holding();
    // Wrong-epoch frames only → COMMAND_TIMEOUT.
    h.epoch += 1;
    h.drive(150, 0);
    assert_eq!(h.state(), McuState::Fault);
    assert_eq!(h.sim.fault_code(), FaultCode::CommandTimeout);
}

#[test]
fn apply_at_nonzero_dropped_as_rx_error() {
    let mut h = Harness::default();
    h.reach_active();
    let mut m = h.set_position_msg(500);
    m.apply_at_us = 123;
    h.send(HostMsg::SetPosition(m));
    assert_eq!(h.sim.rx_error_count(), 1);
    assert_ne!(h.sim.last_rx_seq(), m.seq);
}

#[test]
fn seq_resets_only_on_new_epoch() {
    let mut h = Harness::default();
    h.config();
    h.drive(30, 0);
    let last = h.sim.last_rx_seq();
    assert!(last > 0);
    h.config(); // same epoch: kept
    assert_eq!(h.sim.last_rx_seq(), last);
    h.epoch += 1;
    h.config(); // new epoch: reset
    assert_eq!(h.sim.last_rx_seq(), 0);
}

// ---------------------------------------------------------------------------
// Lease / command timeout
// ---------------------------------------------------------------------------

#[test]
fn command_timeout_in_holding_faults() {
    let mut h = Harness::default();
    h.reach_holding();
    h.set_position(0);
    h.idle(100);
    assert_eq!(
        h.state(),
        McuState::Holding,
        "exactly the timeout is still ok"
    );
    h.idle(2);
    assert_eq!(h.state(), McuState::Fault);
    assert_eq!(h.sim.fault_code(), FaultCode::CommandTimeout);
    let ev = h.fault_events();
    assert_eq!(ev.len(), 1);
    assert_eq!(ev[0].fault_code, FaultCode::CommandTimeout);
    assert_eq!(ev[0].mcu_state_before, McuState::Holding as u8);
    let s = h.sim.status();
    assert_eq!(s.pwm_permille, 0);
    assert_eq!(
        s.flags & (status_flags::POWER_ON_CMD | status_flags::BRAKE_RELEASE_CMD),
        0
    );
}

#[test]
fn command_timeout_in_arming_counts_from_arm_prepare() {
    let mut h = Harness::default();
    h.config();
    h.arm();
    h.idle(110);
    assert_eq!(h.state(), McuState::Fault);
    assert_eq!(h.sim.fault_code(), FaultCode::CommandTimeout);
}

#[test]
fn command_timeout_while_moving_decelerates_then_faults() {
    let mut h = Harness::default();
    // valid_for longer than the lease so the lease expires first.
    h.config_with(Config {
        command_timeout_ms: 30,
        ..config(EPOCH)
    });
    h.reach_holding_from_configured_lease(10);
    h.set_position(0);
    accepted(h.start());
    for _ in 0..30 {
        let mut m = h.set_position_msg(2600);
        m.valid_for_ms = 500;
        h.send(HostMsg::SetPosition(m));
        h.idle(10);
    }
    assert_eq!(h.state(), McuState::Active);
    h.idle(31);
    assert_eq!(h.state(), McuState::Stopping, "decelerating");
    assert_eq!(
        h.sim.fault_code(),
        FaultCode::CommandTimeout,
        "reported during decel"
    );
    assert!(h.fault_events().is_empty(), "FAULT_EVENT only on latch");
    rejected(h.disarm(), AckReason::FaultActive);
    h.idle(400);
    assert_eq!(h.state(), McuState::Fault);
    let ev = h.fault_events();
    assert_eq!(ev.len(), 1);
    assert_eq!(ev[0].fault_code, FaultCode::CommandTimeout);
    assert_eq!(ev[0].mcu_state_before, McuState::Stopping as u8);
}

impl Harness {
    fn reach_holding_from_configured_lease(&mut self, period_ms: u64) {
        accepted(self.arm());
        for _ in 0..(60 / period_ms) {
            self.set_position(0);
            self.idle(period_ms);
        }
        assert_eq!(self.state(), McuState::Holding);
    }
}

// ---------------------------------------------------------------------------
// Motion faults
// ---------------------------------------------------------------------------

#[test]
fn stall_causes_following_error() {
    let mut h = Harness::default();
    h.reach_active();
    h.sim.set_stall(true);
    h.drive(300, 2000);
    assert_eq!(h.state(), McuState::Fault);
    assert_eq!(h.sim.fault_code(), FaultCode::FollowingError);
    assert_eq!(h.fault_events()[0].mcu_state_before, McuState::Active as u8);
    assert_eq!(h.sim.actual_count(), 0);
}

#[test]
fn slow_plant_causes_following_error() {
    let mut h = Harness::new(SimParams {
        plant_max_vel_count_s: 500.0,
        ..SimParams::default()
    });
    h.reach_active();
    h.drive(500, 2000);
    assert_eq!(h.sim.fault_code(), FaultCode::FollowingError);
}

#[test]
fn overspeed_faults() {
    let mut h = Harness::default();
    h.config_with(Config {
        overspeed_count_s: 1000,
        ..config(EPOCH)
    });
    h.reach_active_from_configured();
    h.drive(300, 2000);
    assert_eq!(h.state(), McuState::Fault);
    assert_eq!(h.sim.fault_code(), FaultCode::Overspeed);
}

#[test]
fn actual_outside_soft_limits_faults() {
    let mut h = Harness::default();
    h.reach_holding();
    h.sim.nudge_actual(3000);
    h.drive(5, 0);
    assert_eq!(h.state(), McuState::Fault);
    // Following error is checked first; both are faults with brake applied.
    assert!(matches!(
        h.sim.fault_code(),
        FaultCode::FollowingError | FaultCode::SoftLimit
    ));
    // Pure soft-limit case: hold near the edge, pushed past it by less than
    // the following-error limit.
    let mut h = Harness::default();
    accepted(h.set_reference(2650));
    h.reach_holding();
    h.sim.nudge_actual(100); // FE 100 < 200, actual 2750 > 2700
    h.drive(5, 2650);
    assert_eq!(h.state(), McuState::Fault);
    assert_eq!(h.sim.fault_code(), FaultCode::SoftLimit);
}

#[test]
fn arm_prepare_rejected_outside_soft_window_without_brake_pulse() {
    let mut h = Harness::default();
    accepted(h.set_reference(2800));
    h.config();
    rejected(h.arm(), AckReason::BadState);
    h.drive(200, 2800);
    assert_eq!(h.state(), McuState::Disarmed);
    assert!(h.fault_events().is_empty());
    assert!(h
        .statuses()
        .iter()
        .all(|s| s.flags & status_flags::BRAKE_RELEASE_CMD == 0));
    // Bounds are inclusive.
    accepted(h.set_reference(2700));
    h.reach_holding_from_configured();
    h.disarm();
    h.idle(40);
    accepted(h.set_reference(-2700));
    h.reach_holding_from_configured();
}

#[test]
fn arming_never_releases_brake_outside_soft_window() {
    // Pushed out of the window after ARM_PREPARE, before power settle.
    let mut h = Harness::default();
    h.config();
    accepted(h.arm());
    h.drive(10, 0);
    h.sim.nudge_actual(3000);
    h.drive(60, 0);
    assert_eq!(h.state(), McuState::Fault);
    assert_eq!(h.sim.fault_code(), FaultCode::SoftLimit);
    assert_eq!(h.fault_events()[0].mcu_state_before, McuState::Arming as u8);
    assert!(
        h.statuses()
            .iter()
            .all(|s| s.flags & status_flags::BRAKE_RELEASE_CMD == 0),
        "brake release never commanded"
    );
}

#[test]
fn soft_limit_recovery_goes_through_disarmed_and_set_reference() {
    let mut h = Harness::default();
    accepted(h.set_reference(2650));
    h.reach_holding();
    h.sim.nudge_actual(100);
    h.drive(5, 2650);
    assert_eq!(h.sim.fault_code(), FaultCode::SoftLimit);
    // FAULT_RESET is allowed (DISARMED is as safe as FAULT, and SET_REFERENCE
    // needs DISARMED), but the axis cannot be armed while still outside.
    accepted(h.fault_reset());
    assert_eq!(h.state(), McuState::Disarmed);
    rejected(h.arm(), AckReason::BadState);
    assert_eq!(h.state(), McuState::Disarmed);
    accepted(h.set_reference(0));
    h.reach_holding_from_configured();
}

#[test]
fn stop_timeout_faults() {
    let mut h = Harness::default();
    h.config_with(Config {
        stop_timeout_ms: 20,
        max_acc_count_s2: 20000,
        ..config(EPOCH)
    });
    h.reach_active_from_configured();
    h.drive(200, 2600);
    // Lower the decel budget: CONFIG acc 20000 from ~4000 count/s needs 200 ms.
    h.stop();
    h.drive(100, 2600);
    assert_eq!(h.state(), McuState::Fault);
    assert_eq!(h.sim.fault_code(), FaultCode::StopTimeout);
}

#[test]
fn large_time_jump_stops_substepping_once_parked() {
    let mut h = Harness::default();
    h.reach_holding();
    let t0 = h.now;
    // ~11.6 days with no lease: 1e9 substeps if the model kept stepping.
    let jump = 1_000_000_000 * MS;
    let started = std::time::Instant::now();
    h.now += jump;
    let out = h.sim.tick(h.now);
    assert!(
        started.elapsed() < Duration::from_secs(2),
        "took {:?}",
        started.elapsed()
    );
    assert_eq!(h.state(), McuState::Fault);
    assert_eq!(h.sim.fault_code(), FaultCode::CommandTimeout);
    let ev: Vec<_> = out
        .iter()
        .filter_map(|m| match m {
            McuMsg::FaultEvent(f) => Some(*f),
            _ => None,
        })
        .collect();
    assert_eq!(ev.len(), 1);
    // Latched about command_timeout after the last lease frame, not at the end.
    assert!(ev[0].mcu_time_us < t0 + 200 * MS, "{}", ev[0].mcu_time_us);
    // Time itself still moved all the way.
    assert_eq!(h.sim.status().mcu_time_us, h.now);
    // One periodic STATUS for the jump, then the cadence continues.
    let periodic = out
        .iter()
        .filter(|m| matches!(m, McuMsg::Status(_)))
        .count();
    assert!(periodic >= 1);
    h.now += 20 * MS;
    assert_eq!(
        h.sim
            .tick(h.now)
            .iter()
            .filter(|m| matches!(m, McuMsg::Status(_)))
            .count(),
        1
    );
}

// ---------------------------------------------------------------------------
// HOST_FAULT / FAULT_RESET / force_fault
// ---------------------------------------------------------------------------

#[test]
fn host_fault_latches_immediately_from_active() {
    let mut h = Harness::default();
    h.reach_active();
    h.drive(100, 2000);
    let a = h.host_fault();
    accepted(a);
    assert_eq!(a.acked_type, msg_type::HOST_FAULT);
    assert_eq!(h.state(), McuState::Fault);
    assert_eq!(h.sim.fault_code(), FaultCode::HostFault);
    assert_eq!(
        h.sim.last_host_fault_code(),
        Some(FaultCode::FollowingError)
    );
    let s = h.sim.status();
    assert_eq!(s.pwm_permille, 0);
    assert_eq!(
        s.flags & (status_flags::POWER_ON_CMD | status_flags::BRAKE_RELEASE_CMD),
        0
    );
    // Brake applied: the position no longer changes.
    let at = h.sim.actual_count();
    h.drive(100, -2000);
    assert_eq!(h.sim.actual_count(), at);
}

#[test]
fn fault_gate_rejects_commands() {
    let mut h = Harness::default();
    h.config();
    h.host_fault();
    rejected(h.config(), AckReason::FaultActive);
    rejected(h.arm(), AckReason::FaultActive);
    rejected(h.start(), AckReason::FaultActive);
    rejected(h.stop(), AckReason::FaultActive);
    rejected(h.disarm(), AckReason::FaultActive);
    rejected(h.set_reference(0), AckReason::FaultActive);
    accepted(h.host_fault()); // still accepted, code kept
    assert_eq!(h.sim.fault_code(), FaultCode::HostFault);
}

#[test]
fn fault_reset_rules() {
    let mut h = Harness::default();
    rejected(h.fault_reset(), AckReason::BadState);
    h.reach_holding();
    h.sim.force_fault(FaultCode::PowerFault);
    h.idle(1);
    assert_eq!(h.state(), McuState::Fault);
    assert_eq!(
        h.fault_events().last().unwrap().fault_code,
        FaultCode::PowerFault
    );
    h.sim.set_fault_cause_present(true);
    rejected(h.fault_reset(), AckReason::FaultCausePresent);
    assert_eq!(h.state(), McuState::Fault);
    h.sim.set_fault_cause_present(false);
    accepted(h.fault_reset());
    assert_eq!(h.state(), McuState::Disarmed);
    assert_eq!(h.sim.fault_code(), FaultCode::None);
    // No auto re-arm.
    h.drive(200, 0);
    assert_eq!(h.state(), McuState::Disarmed);
    // CONFIG and reference survive; re-arm works with the same epoch.
    assert!(h.sim.is_configured() && h.sim.is_referenced());
    h.reach_holding_from_configured();
}

#[test]
fn encoder_fault_reset_clears_reference() {
    let mut h = Harness::default();
    h.sim.force_fault(FaultCode::EncoderFault);
    accepted(h.fault_reset());
    assert!(!h.sim.is_referenced());
    h.config();
    rejected(h.arm(), AckReason::NotReferenced);
}

// ---------------------------------------------------------------------------
// SET_REFERENCE
// ---------------------------------------------------------------------------

#[test]
fn set_reference_shifts_coordinate() {
    let mut h = Harness::new(SimParams {
        start_referenced: false,
        initial_count: 57,
        ..SimParams::default()
    });
    assert!(!h.sim.is_referenced());
    assert_eq!(h.sim.status().flags & status_flags::POSITION_REFERENCED, 0);
    accepted(h.set_reference(-2770));
    assert!(h.sim.is_referenced());
    assert_eq!(h.sim.actual_count(), -2770);
    assert_ne!(h.sim.status().flags & status_flags::POSITION_REFERENCED, 0);
}

#[test]
fn set_reference_only_in_disarmed() {
    let mut h = Harness::default();
    h.reach_holding();
    rejected(h.set_reference(0), AckReason::BadState);
}

// ---------------------------------------------------------------------------
// request_id dedupe
// ---------------------------------------------------------------------------

#[test]
fn duplicate_request_is_not_reexecuted() {
    let mut h = Harness::new(SimParams {
        start_referenced: false,
        ..SimParams::default()
    });
    let first = h.cmd(HostMsg::SetReference {
        request_id: 40,
        reference_count: 100,
    });
    accepted(first);
    assert_eq!(h.sim.actual_count(), 100);
    // Identical resend: DUPLICATE, nothing changes.
    h.sim.nudge_actual(7);
    let dup = h.cmd(HostMsg::SetReference {
        request_id: 40,
        reference_count: 100,
    });
    assert_eq!(dup.result, AckResult::Duplicate);
    assert_eq!(dup.reason, AckReason::None, "original was accepted");
    assert_eq!(dup.acked_type, msg_type::SET_REFERENCE);
    assert_eq!(h.sim.actual_count(), 107, "not re-executed");
    // Same request_id, different payload: not a resend → executed.
    let other = h.cmd(HostMsg::SetReference {
        request_id: 40,
        reference_count: -500,
    });
    accepted(other);
    assert_eq!(h.sim.actual_count(), -500);
    // ...and it replaced the cache entry: the first payload is new again.
    accepted(h.cmd(HostMsg::SetReference {
        request_id: 40,
        reference_count: 100,
    }));
    assert_eq!(h.sim.actual_count(), 100);
}

#[test]
fn host_fault_with_reused_request_id_still_latches() {
    // Cross-type collision: ARM_PREPARE rid=2 accepted, then HOST_FAULT rid=2.
    let mut h = Harness::default();
    accepted(h.cmd(HostMsg::Config(Config {
        request_id: 1,
        ..config(EPOCH)
    })));
    accepted(h.cmd(HostMsg::ArmPrepare {
        request_id: 2,
        session_epoch: EPOCH,
    }));
    h.drive(60, 0);
    assert_eq!(h.state(), McuState::Holding);
    let hf = HostMsg::HostFault {
        request_id: 2,
        fault_code: FaultCode::HostFault,
    };
    let a = h.cmd(hf);
    accepted(a);
    assert_eq!(a.acked_type, msg_type::HOST_FAULT);
    assert_eq!(h.state(), McuState::Fault);
    assert_eq!(h.sim.fault_code(), FaultCode::HostFault);
    // Identical HOST_FAULT resend: re-executed (idempotent), never DUPLICATE.
    accepted(h.cmd(hf));
    assert_eq!(h.fault_events().len(), 1, "latch is idempotent");
    // After reset, the same HOST_FAULT bytes latch again.
    accepted(h.fault_reset());
    accepted(h.cmd(hf));
    assert_eq!(h.state(), McuState::Fault);
}

#[test]
fn host_fault_with_request_id_zero_latches() {
    let mut h = Harness::default();
    h.reach_active();
    let a = h.cmd(HostMsg::HostFault {
        request_id: 0,
        fault_code: FaultCode::HostFault,
    });
    accepted(a);
    assert_eq!(a.request_id, 0);
    assert_eq!(h.state(), McuState::Fault);
}

#[test]
fn cross_type_collision_executes_the_new_command() {
    let mut h = Harness::default();
    accepted(h.cmd(HostMsg::Disarm { request_id: 9 }));
    // SET_REFERENCE with the DISARM's request_id is a different command.
    let a = h.cmd(HostMsg::SetReference {
        request_id: 9,
        reference_count: 250,
    });
    accepted(a);
    assert_eq!(a.acked_type, msg_type::SET_REFERENCE);
    assert_eq!(h.sim.actual_count(), 250);
}

#[test]
fn config_with_reused_request_id_and_new_content_is_applied() {
    // A restarted PC reuses rid=1 with a new epoch and narrower limits.
    let mut h = Harness::default();
    accepted(h.cmd(HostMsg::Config(Config {
        request_id: 1,
        ..config(1)
    })));
    let a = h.cmd(HostMsg::Config(Config {
        request_id: 1,
        soft_min_count: -100,
        soft_max_count: 100,
        ..config(7)
    }));
    accepted(a);
    assert_eq!(h.sim.session_epoch(), Some(7));
    let c = h.sim.config().unwrap();
    assert_eq!((c.soft_min_count, c.soft_max_count), (-100, 100));
}

#[test]
fn duplicate_disarm_does_not_disarm_again_after_rearm() {
    let mut h = Harness::default();
    h.reach_holding();
    let d = HostMsg::Disarm { request_id: 500 };
    accepted(h.cmd(d));
    h.idle(40);
    h.reach_holding_from_configured();
    let dup = h.cmd(d);
    assert_eq!(dup.result, AckResult::Duplicate);
    assert_eq!(h.state(), McuState::Holding);
}

#[test]
fn resent_rejected_command_is_reevaluated() {
    // DUPLICATE means "already accepted" (§3.3): rejections are not cached.
    let mut h = Harness::default();
    let a = HostMsg::ArmPrepare {
        request_id: 600,
        session_epoch: EPOCH,
    };
    rejected(h.cmd(a), AckReason::NotConfigured);
    rejected(h.cmd(a), AckReason::NotConfigured);
    h.config();
    accepted(h.cmd(a));
    assert_eq!(h.state(), McuState::Arming);
    // Now accepted → an identical resend is DUPLICATE.
    let dup = h.cmd(a);
    assert_eq!(
        (dup.result, dup.reason, dup.acked_type),
        (AckResult::Duplicate, AckReason::None, msg_type::ARM_PREPARE)
    );
    assert_eq!(h.state(), McuState::Arming);
}

#[test]
fn dedupe_window_is_bounded() {
    let mut h = Harness::default();
    accepted(h.cmd(HostMsg::Disarm { request_id: 1000 }));
    for i in 0..DEDUPE_DEPTH as u32 {
        accepted(h.cmd(HostMsg::Disarm {
            request_id: 2000 + i,
        }));
    }
    // Pushed out of the window → executed again.
    accepted(h.cmd(HostMsg::Disarm { request_id: 1000 }));
}

// ---------------------------------------------------------------------------
// Fault injection: reboot, freeze, drop_rx, rx errors
// ---------------------------------------------------------------------------

#[test]
fn reboot_clears_config_reference_epoch_seq() {
    let mut h = Harness::new(SimParams {
        start_referenced: false,
        ..SimParams::default()
    });
    h.set_reference(1000);
    h.reach_holding();
    let d = HostMsg::Disarm { request_id: 900 };
    accepted(h.cmd(d));
    h.sim.reboot(0xBEEF);
    assert_eq!(h.state(), McuState::Disarmed);
    assert_eq!(h.sim.boot_id(), 0xBEEF);
    assert!(!h.sim.is_configured());
    assert!(!h.sim.is_referenced());
    assert_eq!(h.sim.session_epoch(), None);
    assert_eq!(h.sim.last_rx_seq(), 0);
    assert_eq!(
        h.sim.actual_count(),
        0,
        "encoder restarts at 0 unreferenced"
    );
    h.idle(1);
    let s = h.statuses();
    let last = s.last().unwrap();
    assert_eq!(last.boot_id, 0xBEEF);
    assert_eq!(
        last.flags & (status_flags::CONFIGURED | status_flags::POSITION_REFERENCED),
        0
    );
    assert!(last.mcu_time_us < 5_000, "mcu clock restarts");
    // The request_id cache is cleared: a reused id executes again.
    accepted(h.cmd(d));
    let out = h.send(HostMsg::HelloReq { request_id: 1 });
    let hello = out
        .iter()
        .find_map(|m| {
            if let McuMsg::Hello(x) = m {
                Some(*x)
            } else {
                None
            }
        })
        .unwrap();
    assert_eq!(hello.boot_id, 0xBEEF);
    assert_eq!(hello.calibration_hash, 0);
    rejected(h.arm(), AckReason::NotConfigured);
}

#[test]
fn freeze_suppresses_all_output_but_model_runs() {
    let mut h = Harness::default();
    h.reach_holding();
    h.sim.set_freeze_status(true);
    let n = h.out.len();
    h.idle(200);
    assert_eq!(h.out.len(), n, "nothing transmitted");
    assert_eq!(h.state(), McuState::Fault, "lease still enforced");
    h.sim.set_freeze_status(false);
    h.idle(25);
    assert!(!h.statuses().is_empty() && h.out.len() > n);
}

#[test]
fn drop_rx_ignores_messages() {
    let mut h = Harness::default();
    h.sim.set_drop_rx(true);
    let out = h.send(HostMsg::HelloReq { request_id: 1 });
    assert!(out.iter().all(|m| !matches!(m, McuMsg::Hello(_))));
    let r = h.next_rid();
    let out = h.send(HostMsg::Config(Config {
        request_id: r,
        ..config(EPOCH)
    }));
    assert!(acks(&out).is_empty());
    assert!(!h.sim.is_configured());
}

#[test]
fn bad_frames_counted_and_unanswered() {
    let mut sim = SimMcu::new(SimParams::default());
    sim.tick(0);
    let mut wire = HostMsg::HelloReq { request_id: 1 }.to_wire();
    wire.pop(); // strip delimiter
                // Flip a CRC byte (non-zero so COBS framing stays valid).
    let n = wire.len();
    wire[n - 1] ^= 0x01;
    let out = sim.handle_frame(&wire, 1_000);
    assert!(out.iter().all(|m| !matches!(m, McuMsg::Hello(_))));
    assert_eq!(sim.rx_error_count(), 1);
    assert_eq!(sim.status().rx_error_count, 1);
    // A good frame still works.
    let mut good = HostMsg::HelloReq { request_id: 2 }.to_wire();
    good.pop();
    assert!(sim
        .handle_frame(&good, 2_000)
        .iter()
        .any(|m| matches!(m, McuMsg::Hello(_))));
}

// ---------------------------------------------------------------------------
// STATUS content
// ---------------------------------------------------------------------------

#[test]
fn periodic_status_every_period_in_pure_model() {
    let mut h = Harness::default();
    h.idle(200);
    let s = h.statuses();
    assert_eq!(s.len(), 11, "t=0 plus every 20 ms");
    for w in s.windows(2) {
        assert_eq!(w[1].mcu_time_us - w[0].mcu_time_us, 20_000);
    }
}

#[test]
fn status_reports_following_error_and_pwm_sign() {
    let mut h = Harness::new(SimParams {
        plant_max_vel_count_s: 3000.0,
        ..SimParams::default()
    });
    h.reach_active();
    h.drive(220, 2600);
    let s = h.sim.status();
    assert_eq!(s.mcu_state, McuState::Active as u8);
    assert_eq!(s.following_error_count, s.actual_count - s.target_count);
    assert!(
        s.following_error_count < 0,
        "lagging behind a positive move: {s:?}"
    );
    assert!(s.pwm_permille > 0, "driving positive: {s:?}");
    assert!(s.velocity_count_s > 0);
}

// ---------------------------------------------------------------------------
// Async runner over a duplex stream (paused tokio time)
// ---------------------------------------------------------------------------

struct HostEnd {
    io: DuplexStream,
    acc: FrameAccumulator,
    buf: Vec<McuMsg>,
    bad: usize,
}

impl HostEnd {
    fn new(io: DuplexStream) -> Self {
        HostEnd {
            io,
            acc: FrameAccumulator::new(MAX_ENCODED_LEN),
            buf: Vec::new(),
            bad: 0,
        }
    }
    async fn send(&mut self, m: HostMsg) {
        self.io.write_all(&m.to_wire()).await.unwrap();
    }
    /// Collect everything that arrives within `ms` (paused clock).
    async fn collect(&mut self, ms: u64) -> Vec<McuMsg> {
        let deadline = tokio::time::Instant::now() + Duration::from_millis(ms);
        let mut chunk = [0u8; 1024];
        loop {
            match tokio::time::timeout_at(deadline, self.io.read(&mut chunk)).await {
                Ok(Ok(0)) | Err(_) => break,
                Ok(Ok(n)) => {
                    for f in self.acc.push(&chunk[..n]) {
                        match decode_mcu_frame(&f) {
                            Ok(m) => self.buf.push(m),
                            Err(_) => self.bad += 1,
                        }
                    }
                }
                Ok(Err(e)) => panic!("read error {e}"),
            }
        }
        std::mem::take(&mut self.buf)
    }
}

#[tokio::test(start_paused = true)]
async fn runner_hello_config_and_status_cadence() {
    let (io, handle) = sim_pair(SimParams::default());
    let mut host = HostEnd::new(io);

    host.send(HostMsg::HelloReq { request_id: 1 }).await;
    let out = host.collect(10).await;
    let hello = out.iter().find_map(|m| {
        if let McuMsg::Hello(h) = m {
            Some(*h)
        } else {
            None
        }
    });
    let hello = hello.expect("HELLO over the wire");
    assert_eq!(hello.request_id, 1);
    assert_eq!(hello.device_id, SimParams::default().device_id);

    host.send(HostMsg::Config(Config {
        request_id: 2,
        ..config(EPOCH)
    }))
    .await;
    let out = host.collect(10).await;
    let ack = acks(&out).into_iter().next().expect("ACK over the wire");
    assert_eq!(
        (ack.request_id, ack.acked_type, ack.result),
        (2, msg_type::CONFIG, AckResult::Accepted)
    );
    assert!(handle.snapshot().configured);

    let out = host.collect(400).await;
    let stamps: Vec<u64> = out
        .iter()
        .filter_map(|m| {
            if let McuMsg::Status(s) = m {
                Some(s.mcu_time_us)
            } else {
                None
            }
        })
        .collect();
    assert!(
        stamps.len() >= 18 && stamps.len() <= 22,
        "{} STATUS in 400 ms",
        stamps.len()
    );
    for w in stamps.windows(2) {
        let d = w[1] - w[0];
        assert!((19_000..=21_000).contains(&d), "STATUS spacing {d} us");
    }
    assert_eq!(host.bad, 0);
    handle.shutdown().await;
}

#[tokio::test(start_paused = true)]
async fn runner_full_cycle_and_injection() {
    let (io, handle) = sim_pair(SimParams::default());
    let mut host = HostEnd::new(io);
    host.send(HostMsg::Config(Config {
        request_id: 1,
        ..config(EPOCH)
    }))
    .await;
    host.send(HostMsg::ArmPrepare {
        request_id: 2,
        session_epoch: EPOCH,
    })
    .await;
    let mut seq = 0;
    let mut sp = |target: i32| {
        seq += 1;
        HostMsg::SetPosition(SetPosition {
            session_epoch: EPOCH,
            seq,
            group_frame_id: seq,
            target_count: target,
            max_vel_count_s: 4000,
            max_acc_count_s2: 20000,
            valid_for_ms: 50,
            apply_at_us: 0,
        })
    };
    for _ in 0..8 {
        host.send(sp(0)).await;
        host.collect(10).await;
    }
    assert_eq!(handle.state(), McuState::Holding);
    host.send(HostMsg::Start {
        request_id: 3,
        session_epoch: EPOCH,
    })
    .await;
    for _ in 0..100 {
        host.send(sp(800)).await;
        host.collect(10).await;
    }
    let snap = handle.snapshot();
    assert_eq!(snap.state, McuState::Active);
    assert_eq!(snap.actual_count, 800);

    // A corrupted frame is discarded by the host decoder.
    handle.corrupt_next_tx();
    host.collect(25).await;
    assert_eq!(host.bad, 1);

    // Stall → FOLLOWING_ERROR → FAULT_EVENT over the wire.
    handle.set_stall(true);
    let mut events = Vec::new();
    for _ in 0..50 {
        host.send(sp(-800)).await;
        events.extend(host.collect(10).await.into_iter().filter_map(|m| {
            if let McuMsg::FaultEvent(f) = m {
                Some(f)
            } else {
                None
            }
        }));
    }
    assert_eq!(handle.state(), McuState::Fault);
    assert_eq!(events.len(), 1);
    assert_eq!(events[0].fault_code, FaultCode::FollowingError);

    // Frozen link: no bytes arrive.
    handle.set_freeze_status(true);
    let out = host.collect(100).await;
    assert!(out.is_empty());
    handle.shutdown().await;
}

#[tokio::test(start_paused = true)]
async fn runner_latency_and_eof() {
    let (io, handle) = sim_pair(SimParams {
        link_latency_ms: 5,
        ..SimParams::default()
    });
    let mut host = HostEnd::new(io);
    host.collect(30).await; // drain boot STATUS
    host.send(HostMsg::HelloReq { request_id: 1 }).await;
    let early = host.collect(8).await;
    assert!(
        !early.iter().any(|m| matches!(m, McuMsg::Hello(_))),
        "≥ 10 ms round trip"
    );
    let late = host.collect(10).await;
    assert!(late.iter().any(|m| matches!(m, McuMsg::Hello(_))));
    drop(host);
    tokio::time::sleep(Duration::from_millis(10)).await;
    assert!(
        handle.is_finished(),
        "task ends when the host closes the stream"
    );
}

// ---------------------------------------------------------------------------
// Rules the PC relies on (protocol.md §1, §4)
// ---------------------------------------------------------------------------

/// Every STATUS sent after an ACK shows the state the command left the MCU
/// in (the transition is committed before the ACK goes out).
fn assert_status_after_ack(out: &[McuMsg], what: &str, ok: impl Fn(&Status) -> bool) {
    let ack = out
        .iter()
        .position(|m| matches!(m, McuMsg::Ack(_)))
        .unwrap_or_else(|| panic!("{what}: no ACK in {out:?}"));
    for m in &out[ack..] {
        if let McuMsg::Status(s) = m {
            assert!(ok(s), "{what}: STATUS after the ACK shows the old state: {s:?}");
        }
    }
}

#[test]
fn status_after_an_ack_shows_the_post_command_state() {
    let mut h = Harness::default();
    let c = HostMsg::Config(Config { request_id: h.next_rid(), ..config(EPOCH) });
    let out = h.send(c);
    assert_status_after_ack(&out, "CONFIG", |s| s.flags & status_flags::CONFIGURED != 0);
    // A STATUS is due right after each of these: the model sends one on change.
    let r = h.next_rid();
    let out = h.send(HostMsg::ArmPrepare { request_id: r, session_epoch: EPOCH });
    assert!(out.iter().any(|m| matches!(m, McuMsg::Status(_))), "{out:?}");
    assert_status_after_ack(&out, "ARM_PREPARE", |s| s.mcu_state == McuState::Arming as u8);
    let here = h.sim.actual_count();
    h.drive(60, here);
    h.set_position(here);
    let r = h.next_rid();
    let out = h.send(HostMsg::Start { request_id: r, session_epoch: EPOCH });
    assert_status_after_ack(&out, "START", |s| s.mcu_state == McuState::Active as u8);
    h.drive(20, here + 300);
    let r = h.next_rid();
    let out = h.send(HostMsg::ControlledStop { request_id: r, session_epoch: EPOCH, reason: 1 });
    assert_status_after_ack(&out, "CONTROLLED_STOP", |s| s.mcu_state == McuState::Stopping as u8);
    let r = h.next_rid();
    let out = h.send(HostMsg::HostFault { request_id: r, fault_code: FaultCode::HostFault });
    assert!(out.iter().any(|m| matches!(m, McuMsg::Status(_))), "{out:?}");
    assert_status_after_ack(&out, "HOST_FAULT", |s| {
        s.mcu_state == McuState::Fault as u8 && s.fault_code != FaultCode::None
    });
    let r = h.next_rid();
    let out = h.send(HostMsg::FaultReset { request_id: r });
    assert_status_after_ack(&out, "FAULT_RESET", |s| {
        s.mcu_state == McuState::Disarmed as u8 && s.fault_code == FaultCode::None
    });
}

#[test]
fn stop_and_disarm_outside_moving_states_are_accepted() {
    let mut h = Harness::default();
    h.config();
    accepted(h.disarm()); // DISARMED: no-op
    assert_eq!(h.state(), McuState::Disarmed);
    accepted(h.arm());
    accepted(h.stop()); // ARMING: no-op, keeps arming
    assert_eq!(h.state(), McuState::Arming);
    let here = h.sim.actual_count();
    h.drive(60, here);
    assert_eq!(h.state(), McuState::Holding);
    accepted(h.stop()); // HOLDING: no-op (the PC sends it on a stop while holding)
    assert_eq!(h.state(), McuState::Holding);
    accepted(h.disarm()); // HOLDING: straight to DISARMING, no deceleration
    assert_eq!(h.state(), McuState::Disarming);
    accepted(h.stop()); // DISARMING: no-op
    accepted(h.disarm()); // DISARMING: no-op
    assert_eq!(h.state(), McuState::Disarming);
    h.idle(1000);
    assert_eq!(h.state(), McuState::Disarmed);
}

#[test]
fn status_target_is_the_profiled_setpoint_not_the_commanded_target() {
    let mut h = Harness::default();
    h.reach_active();
    let start = h.sim.actual_count();
    h.drive(30, start + 2000);
    let s = h.sim.status();
    assert_eq!(s.mcu_state, McuState::Active as u8);
    // 30 ms at 20000 count/s² gets nowhere near +2000: STATUS shows the
    // trajectory point the loop follows, not the SET_POSITION target.
    assert!(s.target_count > start && s.target_count < start + 100, "{s:?}");
    assert_eq!(s.following_error_count, s.actual_count - s.target_count);
}

// ---------------------------------------------------------------------------
// Axis selection (protocol.md §1.1, DESIGN.md §10.3)
// ---------------------------------------------------------------------------

fn axis_selects(out: &[McuMsg]) -> Vec<h3cs::protocol::AxisSelect> {
    out.iter()
        .filter_map(|m| match m {
            McuMsg::AxisSelect(a) => Some(*a),
            _ => None,
        })
        .collect()
}

#[test]
fn select_axis_in_disarmed_clears_config_and_reference_and_announces_once() {
    let mut h = Harness::new(SimParams {
        selected_axis: 1,
        ..SimParams::default()
    });
    accepted(h.config());
    let hello = h.send(HostMsg::HelloReq { request_id: 5 });
    assert!(
        matches!(hello[0], McuMsg::Hello(x) if x.selected_axis == 1 && x.calibration_hash == 0x1234_5678),
        "{hello:?}"
    );
    assert_eq!(h.sim.status().selected_axis, 1);

    let now = h.now;
    let out = h.sim.select_axis(2, now);
    // (1) flags cleared, (2) exactly one AXIS_SELECT, (3) STATUS after it
    // carries the new value (protocol §1.1 order).
    let sel = axis_selects(&out);
    assert_eq!(sel.len(), 1, "{out:?}");
    assert_eq!(
        (sel[0].selected_axis, sel[0].prev_selected_axis, sel[0].mcu_state, sel[0].boot_id),
        (2, 1, McuState::Disarmed as u8, h.sim.boot_id())
    );
    let pos = out.iter().position(|m| matches!(m, McuMsg::AxisSelect(_))).unwrap();
    let st = out[pos..]
        .iter()
        .find_map(|m| match m {
            McuMsg::Status(s) => Some(*s),
            _ => None,
        })
        .expect("immediate STATUS after AXIS_SELECT");
    assert_eq!(st.selected_axis, 2);
    assert_eq!(st.flags & (status_flags::CONFIGURED | status_flags::POSITION_REFERENCED), 0);
    assert_eq!(st.mcu_state, McuState::Disarmed as u8);
    assert!(!h.sim.is_configured() && !h.sim.is_referenced());
    assert_eq!(h.sim.selected_axis(), 2);
    let hello = h.send(HostMsg::HelloReq { request_id: 6 });
    assert!(
        matches!(hello.iter().find(|m| matches!(m, McuMsg::Hello(_))),
            Some(McuMsg::Hello(x)) if x.selected_axis == 2 && x.calibration_hash == 0),
        "{hello:?}"
    );
    // ARM needs CONFIG and SET_REFERENCE again.
    rejected(h.arm(), AckReason::NotConfigured);
    accepted(h.config());
    rejected(h.arm(), AckReason::NotReferenced);
    // Selecting the current value does nothing.
    let now = h.now;
    assert!(axis_selects(&h.sim.select_axis(2, now)).is_empty());
    assert!(h.sim.is_configured());
}

#[test]
fn select_axis_is_ignored_outside_disarmed_but_unchecked_switches_anyway() {
    let mut h = Harness::new(SimParams {
        selected_axis: 1,
        ..SimParams::default()
    });
    h.reach_holding();
    let now = h.now;
    assert!(axis_selects(&h.sim.select_axis(2, now)).is_empty());
    assert_eq!(h.sim.selected_axis(), 1);
    assert!(h.sim.is_configured(), "CONFIG kept: no switch happened");
    // FAULT: not accepted either (FAULT_RESET first).
    h.sim.force_fault(FaultCode::FollowingError);
    let now = h.now;
    assert!(axis_selects(&h.sim.select_axis(2, now)).is_empty());
    assert_eq!(h.sim.selected_axis(), 1);
    accepted(h.fault_reset());
    let now = h.now;
    let sel = axis_selects(&h.sim.select_axis(3, now));
    assert_eq!(sel.len(), 1);
    assert_eq!((sel[0].selected_axis, sel[0].prev_selected_axis), (3, 1));

    // Test hook: a unit that breaks the DISARMED rule while HOLDING.
    let mut h = Harness::new(SimParams {
        selected_axis: 1,
        ..SimParams::default()
    });
    h.reach_holding();
    let now = h.now;
    let sel = axis_selects(&h.sim.select_axis_unchecked(2, now));
    assert_eq!(sel.len(), 1);
    assert_eq!(sel[0].mcu_state, McuState::Holding as u8);
    assert_eq!(h.sim.selected_axis(), 2);
    assert_eq!(h.sim.status().selected_axis, 2);
}

#[test]
fn selection_never_moves_the_count_and_survives_reboot() {
    let mut h = Harness::new(SimParams {
        selected_axis: 1,
        initial_count: 123,
        ..SimParams::default()
    });
    let now = h.now;
    h.sim.select_axis(3, now);
    assert_eq!(h.sim.actual_count(), 123, "count is raw encoder + SET_REFERENCE only");
    assert_eq!(h.sim.target_count(), 123);
    h.sim.reboot(99);
    assert_eq!(h.sim.selected_axis(), 3, "selection is NVS: kept across reboot");
    assert_eq!(h.sim.status().selected_axis, 3);
    assert_eq!(h.sim.boot_id(), 99);
}

// ---------- end-stop calibration (protocol.md §1.2, DESIGN.md §11) ----------

fn cal_results(out: &[McuMsg]) -> Vec<CalResult> {
    out.iter()
        .filter_map(|m| match m {
            McuMsg::CalResult(c) => Some(*c),
            _ => None,
        })
        .collect()
}

impl Harness {
    fn calibrate(&mut self, mode: u8) -> Ack {
        let r = self.next_rid();
        self.cmd(HostMsg::Calibrate { request_id: r, mode })
    }
    /// Unreferenced unit with the default ±3000 end stops, CONFIG accepted
    /// (so the power role is known).
    fn uncalibrated() -> Self {
        let mut h = Harness::new(SimParams { start_referenced: false, ..SimParams::default() });
        assert_eq!(h.config().result, AckResult::Accepted);
        h.out.clear();
        h
    }
}

#[test]
fn calibration_finds_both_ends_then_neutral_and_references_p_end_at_zero() {
    let mut h = Harness::uncalibrated();
    assert!(!h.sim.is_referenced());
    assert_eq!(h.sim.end_counts(), (-3000, 3000));
    assert_eq!(h.calibrate(1).result, AckResult::Accepted);
    assert_eq!(h.state(), McuState::Calibrating);
    assert!(h.sim.is_calibrating());
    // The STATUS after the ACK already says CALIBRATING (protocol §1 ordering).
    let after_ack: Vec<u8> = h.statuses().iter().map(|s| s.mcu_state).collect();
    assert_eq!(after_ack.last(), Some(&(McuState::Calibrating as u8)));

    let mut min_seen = i32::MAX;
    let mut max_seen = i32::MIN;
    let mut powered = false;
    let mut released = false;
    for _ in 0..6000 {
        h.idle(1);
        let s = h.sim.status();
        min_seen = min_seen.min(s.actual_count);
        max_seen = max_seen.max(s.actual_count);
        powered |= s.flags & status_flags::POWER_ON_CMD != 0;
        released |= s.flags & status_flags::BRAKE_RELEASE_CMD != 0;
        if h.state() == McuState::Disarmed {
            break;
        }
    }
    assert_eq!(h.state(), McuState::Disarmed, "calibration must end in DISARMED");
    assert!(powered && released, "power on and brake release were commanded");
    // The plant touched both stops and never passed them (pre-shift coordinate).
    assert_eq!((min_seen, max_seen), (-3000, 3000));

    let results = cal_results(&h.out);
    assert_eq!(results.len(), 1, "exactly one CAL_RESULT: {results:?}");
    let r = results[0];
    assert_eq!(r.result, CalResultCode::Ok);
    assert_eq!(r.step, 3);
    assert_eq!(r.n_end_count, 6000, "N end − P end");
    assert_eq!(r.neutral_count, 3000);
    assert_eq!(r.sign, 1);
    assert_eq!(r.reason_str(), "ok");
    assert_eq!(r.boot_id, h.sim.boot_id());
    // CAL_RESULT precedes the STATUS that shows DISARMED.
    let i_res = h.out.iter().position(|m| matches!(m, McuMsg::CalResult(_))).unwrap();
    let i_dis = h
        .out
        .iter()
        .position(|m| matches!(m, McuMsg::Status(s) if s.mcu_state == McuState::Disarmed as u8))
        .unwrap();
    assert!(i_res < i_dis, "CAL_RESULT at {i_res}, DISARMED STATUS at {i_dis}");
    // P end = 0 now; the unit is at the neutral and referenced.
    assert!(h.sim.is_referenced());
    assert_eq!(h.sim.end_counts(), (0, 6000));
    assert_eq!(h.sim.actual_count(), 3000);
    let last = h.statuses().last().copied().unwrap();
    assert_eq!(last.mcu_state, McuState::Disarmed as u8);
    assert_ne!(last.flags & status_flags::POSITION_REFERENCED, 0);
    assert_eq!(last.flags & (status_flags::POWER_ON_CMD | status_flags::BRAKE_RELEASE_CMD), 0);
    assert_eq!(last.actual_count, 3000);
    // CONFIG survived the calibration (protocol §1.2 only touches the reference).
    assert!(h.sim.is_configured());
}

#[test]
fn calibrate_is_disarmed_only_and_mode_is_checked() {
    let mut h = Harness::uncalibrated();
    assert_eq!(h.calibrate(2).reason, AckReason::InvalidParam);
    assert_eq!(h.calibrate(255).reason, AckReason::InvalidParam);
    // Cancel while not calibrating: accepted no-op, no CAL_RESULT.
    assert_eq!(h.calibrate(0).result, AckResult::Accepted);
    assert_eq!(h.state(), McuState::Disarmed);
    assert!(cal_results(&h.out).is_empty());
    // Not DISARMED → BAD_STATE.
    assert_eq!(h.set_reference(0).result, AckResult::Accepted);
    h.reach_holding();
    assert_eq!(h.calibrate(1).reason, AckReason::BadState);
    assert_eq!(h.state(), McuState::Holding);
    assert_eq!(h.host_fault().result, AckResult::Accepted);
    assert_eq!(h.calibrate(1).reason, AckReason::FaultActive);
    assert_eq!(h.fault_reset().result, AckResult::Accepted);
    assert_eq!(h.calibrate(1).result, AckResult::Accepted);
    assert_eq!(h.state(), McuState::Calibrating);
    // Same message again is a resend: DUPLICATE, still calibrating.
    let rid = h.rid;
    assert_eq!(h.cmd(HostMsg::Calibrate { request_id: rid, mode: 1 }).result, AckResult::Duplicate);
    assert_eq!(h.state(), McuState::Calibrating);
}

#[test]
fn calibration_cancel_by_calibrate_0_and_by_disarm_clears_reference() {
    for by_disarm in [false, true] {
        let mut h = Harness::uncalibrated();
        assert_eq!(h.set_reference(0).result, AckResult::Accepted);
        assert!(h.sim.is_referenced());
        assert_eq!(h.calibrate(1).result, AckResult::Accepted);
        h.idle(200); // moving toward the P end
        assert_eq!(h.state(), McuState::Calibrating);
        let pos = h.sim.actual_count();
        assert!(pos < 0, "moving toward the P end: {pos}");
        let ack = if by_disarm { h.disarm() } else { h.calibrate(0) };
        assert_eq!(ack.result, AckResult::Accepted);
        assert_eq!(h.state(), McuState::Calibrating, "shutdown sequence first");
        h.idle(100);
        assert_eq!(h.state(), McuState::Disarmed);
        let results = cal_results(&h.out);
        assert_eq!(results.len(), 1);
        assert_eq!(results[0].result, CalResultCode::Cancelled);
        assert_eq!(results[0].step, 1);
        assert_eq!(results[0].reason_str(), "stop");
        assert_eq!((results[0].n_end_count, results[0].neutral_count), (0, 0));
        assert!(!h.sim.is_referenced(), "a cancelled calibration drops POSITION_REFERENCED");
        assert_eq!(h.sim.actual_count(), pos, "the mechanism stopped where it was");
        assert_eq!(h.sim.end_counts(), (-3000, 3000), "no re-reference on cancel");
    }
}

#[test]
fn host_fault_during_calibration_reports_failed_then_faults() {
    let mut h = Harness::uncalibrated();
    assert_eq!(h.calibrate(1).result, AckResult::Accepted);
    h.idle(300);
    assert_eq!(h.state(), McuState::Calibrating);
    h.out.clear();
    assert_eq!(h.host_fault().result, AckResult::Accepted);
    assert_eq!(h.state(), McuState::Fault);
    assert_eq!(h.sim.fault_code(), FaultCode::HostFault);
    let results = cal_results(&h.out);
    assert_eq!(results.len(), 1);
    assert_eq!(results[0].result, CalResultCode::Failed);
    assert_eq!(results[0].reason_str(), "fault");
    let ev = h
        .out
        .iter()
        .find_map(|m| match m {
            McuMsg::FaultEvent(e) => Some(*e),
            _ => None,
        })
        .expect("FAULT_EVENT");
    assert_eq!(ev.mcu_state_before, McuState::Calibrating as u8);
    assert_eq!(ev.fault_code, FaultCode::HostFault);
    assert!(!h.sim.is_referenced());
    // CAL_RESULT came before the FAULT STATUS.
    let i_res = h.out.iter().position(|m| matches!(m, McuMsg::CalResult(_))).unwrap();
    let i_fault = h
        .out
        .iter()
        .position(|m| matches!(m, McuMsg::Status(s) if s.mcu_state == McuState::Fault as u8))
        .unwrap();
    assert!(i_res < i_fault);
    assert_eq!(h.fault_reset().result, AckResult::Accepted);
    assert_eq!(h.state(), McuState::Disarmed);
    assert!(!h.sim.is_calibrating());
}

#[test]
fn commands_during_calibration_are_bad_state_and_set_position_is_dropped_uncounted() {
    let mut h = Harness::uncalibrated();
    assert_eq!(h.calibrate(1).result, AckResult::Accepted);
    h.idle(100);
    let before = (h.sim.stale_drop_count(), h.sim.rx_error_count(), h.sim.last_rx_seq());
    h.set_position(0);
    h.set_position(0);
    assert_eq!((h.sim.stale_drop_count(), h.sim.rx_error_count(), h.sim.last_rx_seq()), before, "dropped, not counted");
    assert_eq!(h.config().reason, AckReason::BadState);
    assert_eq!(h.arm().reason, AckReason::BadState);
    assert_eq!(h.start().reason, AckReason::BadState);
    assert_eq!(h.set_reference(0).reason, AckReason::BadState);
    assert_eq!(h.fault_reset().reason, AckReason::BadState);
    // CONTROLLED_STOP outside ACTIVE is the usual accepted no-op.
    assert_eq!(h.stop().result, AckResult::Accepted);
    assert_eq!(h.state(), McuState::Calibrating);
    // A lease was never started: nothing times out while calibrating.
    h.idle(1000);
    assert!(matches!(h.state(), McuState::Calibrating | McuState::Disarmed));
    assert_eq!(h.sim.fault_code(), FaultCode::None);
}

#[test]
fn injected_failure_and_stall_fail_the_calibration() {
    let mut h = Harness::uncalibrated();
    h.sim.set_calibration_failure(Some("slow"));
    assert_eq!(h.calibrate(1).result, AckResult::Accepted);
    h.idle(200);
    assert_eq!(h.state(), McuState::Disarmed);
    let r = cal_results(&h.out);
    assert_eq!(r.len(), 1);
    assert_eq!((r[0].result, r[0].step, r[0].reason_str().as_str()), (CalResultCode::Failed, 1, "slow"));
    assert!(!h.sim.is_referenced());
    // The injection is one-shot: the next run succeeds.
    h.out.clear();
    assert_eq!(h.calibrate(1).result, AckResult::Accepted);
    h.idle(4000);
    assert_eq!(cal_results(&h.out)[0].result, CalResultCode::Ok);

    let mut h = Harness::uncalibrated();
    h.sim.set_stall(true);
    assert_eq!(h.calibrate(1).result, AckResult::Accepted);
    h.idle(200);
    assert_eq!(h.state(), McuState::Disarmed);
    let r = cal_results(&h.out);
    assert_eq!((r[0].result, r[0].reason_str().as_str()), (CalResultCode::Failed, "stall"));
}

#[test]
fn plant_never_passes_the_end_stops_and_the_stops_follow_every_reference() {
    let mut h = Harness::new(SimParams { end_min_count: -100, end_max_count: 100, ..SimParams::default() });
    // A disturbance stops at the stop.
    h.sim.nudge_actual(10_000);
    assert_eq!(h.sim.actual_count(), 100);
    h.sim.nudge_actual(-10_000);
    assert_eq!(h.sim.actual_count(), -100);
    // SET_REFERENCE moves the coordinate, not the mechanism: the stops move with it.
    assert_eq!(h.set_reference(1000).result, AckResult::Accepted);
    assert_eq!(h.sim.actual_count(), 1000);
    assert_eq!(h.sim.end_counts(), (1000, 1200));
    // A reboot without a reference restarts the count at 0 at the same place
    // (the mechanism did not move: still 100 counts from the lower stop).
    let mut h2 = Harness::new(SimParams { start_referenced: false, end_min_count: -100, end_max_count: 100, ..SimParams::default() });
    assert_eq!(h2.set_reference(1000).result, AckResult::Accepted);
    assert_eq!(h2.sim.end_counts(), (900, 1100));
    h2.sim.reboot(78);
    assert_eq!(h2.sim.actual_count(), 0);
    assert_eq!(h2.sim.end_counts(), (-100, 100));
    // With `start_referenced` the coordinate survives the reboot, stops included.
    h.sim.reboot(77);
    assert_eq!(h.sim.actual_count(), 1000);
    assert_eq!(h.sim.end_counts(), (1000, 1200));
    // Position control cannot push past the stop either (CONFIG soft window wider than the stops).
    let mut h = Harness::new(SimParams { end_min_count: -100, end_max_count: 100, ..SimParams::default() });
    h.reach_holding();
    h.set_position(0);
    assert_eq!(h.start().result, AckResult::Accepted);
    h.drive(100, 2000);
    assert!(h.sim.actual_count() <= 100, "{}", h.sim.actual_count());
}
