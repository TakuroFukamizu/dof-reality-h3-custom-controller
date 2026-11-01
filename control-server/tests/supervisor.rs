//! supervisor: rig state machine driven against three SimMcu reference axes
//! on a fake 10 ms clock (DESIGN.md §5, §9).

// Per-axis loops index several parallel [_; 3] arrays at once.
#![allow(clippy::needless_range_loop)]

use std::path::Path;
use std::time::Duration;

use h3cs::calibration;
use h3cs::compat;
use h3cs::config::Config;
use h3cs::limits::MotionLimits;
use h3cs::protocol::{capabilities, msg_type, Ack, AckReason, AckResult, Hello, HostMsg, McuMsg, SetPosition};
use h3cs::session::{AcceptedPose, PoseIntake, PoseMsg};
use h3cs::sim_axis::{SimMcu, SimParams};
use h3cs::supervisor::{admin_error, ActiveSource, AdminCmd, LinkEvent, Supervisor, ACK_TIMEOUT, LIMITS_RETRY};
use h3cs::types::{status_flags, AxisId, DeviceId, FaultCode, LimitReason, McuState, Pose, RigFault, RigState};
use tokio::time::Instant;

const SIM: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/config/sim.toml");
const STEP: Duration = Duration::from_millis(10);

fn sim_cfg() -> Config {
    Config::load(Path::new(SIM)).unwrap()
}

fn ax(n: u8) -> AxisId {
    AxisId::new(n).unwrap()
}

// ------------------------------------------------------------------ harness

/// Supervisor + three simulated MCUs. Each step: MCUs tick → their output is
/// fed back as LinkEvents → game pose submitted → supervisor tick → admin
/// messages then SET_POSITION delivered to the MCUs (replies fed back).
struct Rig {
    cfg: Config,
    sup: Supervisor,
    mcus: Vec<SimMcu>,
    linked: [bool; 3],
    axis_of: [Option<AxisId>; 3],
    t0: Instant,
    now: Instant,
    /// Admin messages sent (sim index, time, message).
    sent: Vec<(usize, Instant, HostMsg)>,
    setpos: Vec<(Instant, RigState, [SetPosition; 3])>,
    states: Vec<(Instant, RigState)>,
    intake: PoseIntake,
    session: String,
    game: Option<(Pose, f64)>,
    pose_seq: u64,
    frame_seq: u64,
    pose: Option<AcceptedPose>,
    drop_setpos: [bool; 3],
    /// XORed into every STATUS `flags` of sim i (a firmware that misreports).
    flip_flags: [u16; 3],
    pre_tick: Vec<LinkEvent>,
}

impl Rig {
    fn new(cfg: Config) -> Self {
        Self::with(cfg, |_, _| {})
    }

    fn with(cfg: Config, tweak: impl Fn(usize, &mut SimParams)) -> Self {
        let mcus = (0..3)
            .map(|i| {
                let id = cfg
                    .axes
                    .iter()
                    .find(|a| a.axis_id.index() == i)
                    .map(|a| a.device_id)
                    .unwrap();
                let id = if id.is_unassigned() {
                    DeviceId(0x5131_0000_0000_0001 + i as u64)
                } else {
                    id
                };
                let mut p = SimParams {
                    device_id: id,
                    boot_id: 100 + i as u32,
                    // The new normal: the unit selects its axis (protocol §1.1).
                    selected_axis: i as u8 + 1,
                    ..SimParams::default()
                };
                tweak(i, &mut p);
                SimMcu::new(p)
            })
            .collect();
        let t0 = Instant::now();
        let mut intake = PoseIntake::new();
        let session = intake.new_session();
        Rig {
            sup: Supervisor::new(&cfg),
            cfg,
            mcus,
            linked: [false; 3],
            axis_of: [None; 3],
            t0,
            now: t0,
            sent: Vec::new(),
            setpos: Vec::new(),
            states: Vec::new(),
            intake,
            session,
            game: None,
            pose_seq: 0,
            frame_seq: 0,
            pose: None,
            drop_setpos: [false; 3],
            flip_flags: [0; 3],
            pre_tick: Vec::new(),
        }
    }

    fn us(&self) -> u64 {
        1_000_000 + (self.now - self.t0).as_micros() as u64
    }

    fn port(i: usize) -> String {
        format!("sim:{}", i + 1)
    }

    /// axis_link's rule (DESIGN.md §10.1): `selected_axis` 1..=3, else the
    /// `[[axis]]` configured with the device_id.
    fn resolve(&self, selected_axis: u8, device_id: DeviceId) -> Option<AxisId> {
        AxisId::new(selected_axis).or_else(|| {
            self.cfg
                .axes
                .iter()
                .find(|a| !a.device_id.is_unassigned() && a.device_id == device_id)
                .map(|a| a.axis_id)
        })
    }

    fn feed(&mut self, i: usize, msgs: Vec<McuMsg>) {
        for m in msgs {
            let ev = match m {
                McuMsg::Hello(h) => {
                    let axis = self.resolve(h.selected_axis, h.device_id);
                    if let (Some(old), Some(new)) = (self.axis_of[i], axis) {
                        if old != new {
                            // axis_link: Disconnected for the old axis first.
                            self.sup.on_link_event(
                                LinkEvent::Disconnected {
                                    axis: old,
                                    reason: h3cs::axis_link::axis_changed_reason(old.get(), new.get()),
                                },
                                self.now,
                            );
                        }
                    }
                    self.axis_of[i] = axis;
                    match axis {
                        Some(axis) => LinkEvent::Identified {
                            axis,
                            hello: h,
                            port: Self::port(i),
                        },
                        None => LinkEvent::UnknownDevice {
                            port: Self::port(i),
                            device_id: h.device_id,
                        },
                    }
                }
                McuMsg::AxisSelect(sel) => {
                    // axis_link: Disconnected{old, axis_changed} then Identified{new}
                    // (hello = last HELLO with the new selected_axis), or UnknownDevice.
                    let us = self.us();
                    let hello = self.mcus[i].handle(&HostMsg::HelloReq { request_id: 1 }, us);
                    let Some(McuMsg::Hello(h)) = hello.into_iter().find(|m| matches!(m, McuMsg::Hello(_)))
                    else {
                        continue;
                    };
                    let new = self.resolve(sel.selected_axis, h.device_id);
                    let old = self.axis_of[i];
                    if old == new {
                        continue;
                    }
                    if let Some(old) = old {
                        let reason = h3cs::axis_link::axis_changed_reason(
                            old.get(),
                            new.map_or(0, AxisId::get),
                        );
                        // The server turns a non-primary port's Disconnected into PortClosed.
                        let ev = if self.sup.axis_port(old) == Some(Self::port(i).as_str()) {
                            LinkEvent::Disconnected { axis: old, reason }
                        } else {
                            LinkEvent::PortClosed { port: Self::port(i) }
                        };
                        self.sup.on_link_event(ev, self.now);
                    }
                    self.axis_of[i] = new;
                    match new {
                        Some(axis) => LinkEvent::Identified {
                            axis,
                            hello: Hello { selected_axis: sel.selected_axis, ..h },
                            port: Self::port(i),
                        },
                        None => {
                            self.linked[i] = false;
                            LinkEvent::UnknownDevice {
                                port: Self::port(i),
                                device_id: h.device_id,
                            }
                        }
                    }
                }
                McuMsg::Ack(ack) => match self.axis_of[i] {
                    Some(axis) => LinkEvent::Ack { axis, ack },
                    None => continue,
                },
                McuMsg::Status(mut status) => match self.axis_of[i] {
                    Some(axis) => LinkEvent::Status {
                        axis,
                        status: {
                            status.flags ^= self.flip_flags[i];
                            status
                        },
                        received_at: self.now,
                    },
                    None => continue,
                },
                McuMsg::FaultEvent(ev) => match self.axis_of[i] {
                    Some(axis) => LinkEvent::FaultEvent { axis, ev },
                    None => continue,
                },
                McuMsg::CalResult(result) => match self.axis_of[i] {
                    Some(axis) => LinkEvent::CalResult { axis, result },
                    None => continue,
                },
                McuMsg::Limits(limits) => match self.axis_of[i] {
                    Some(axis) => LinkEvent::Limits { axis, limits },
                    None => continue,
                },
                McuMsg::ClockSyncReply(_) => continue,
            };
            self.sup.on_link_event(ev, self.now);
        }
    }

    fn connect(&mut self, i: usize) {
        self.linked[i] = true;
        let us = self.us();
        let out = self.mcus[i].handle(&HostMsg::HelloReq { request_id: 1 }, us);
        self.feed(i, out);
    }

    fn connect_all(&mut self) {
        (0..3).for_each(|i| self.connect(i));
    }

    fn disconnect(&mut self, i: usize) {
        self.linked[i] = false;
        if let Some(axis) = self.axis_of[i] {
            self.sup.on_link_event(
                LinkEvent::Disconnected {
                    axis,
                    reason: "test unplug".into(),
                },
                self.now,
            );
        }
    }

    fn set_game(&mut self, game: Option<(Pose, f64)>) {
        self.game = game;
    }

    fn submit_pose(&mut self) {
        let Some((pose, gain)) = self.game else {
            return;
        };
        self.pose_seq += 1;
        self.frame_seq += 1;
        let msg = PoseMsg {
            protocol: 1,
            source_session: self.session.clone(),
            seq: self.pose_seq,
            frame_seq: Some(self.frame_seq),
            source_age_ms: 0.0,
            pitch: pose.pitch,
            roll: pose.roll,
            yaw: pose.yaw,
            gain,
        };
        let p = self.intake.accept_pose(
            &msg,
            self.now,
            &self.cfg.game,
            self.cfg.limits.max_gain,
            self.cfg.timing.source_freshness_ms,
        );
        self.pose = Some(p.unwrap());
    }

    fn step(&mut self) {
        self.now += STEP;
        let us = self.us();
        for i in 0..3 {
            if self.linked[i] {
                let out = self.mcus[i].tick(us);
                self.feed(i, out);
            }
        }
        self.submit_pose();
        for ev in std::mem::take(&mut self.pre_tick) {
            self.sup.on_link_event(ev, self.now);
        }
        let out = self
            .sup
            .tick(self.now, STEP.as_secs_f64(), self.pose.as_ref());
        if let Some(sp) = out.set_position {
            // State after the tick: the one this SET_POSITION was built in.
            self.setpos.push((self.now, self.sup.state(), sp));
        }
        for i in 0..3 {
            // Route like server.rs `dispatch`: only to the axis's primary port
            // (a claimant of a conflicted axis gets nothing).
            let Some(axis) = self.axis_of[i] else {
                continue;
            };
            if self.sup.axis_port(axis) != Some(Self::port(i).as_str()) {
                continue;
            }
            let k = axis.index();
            for m in &out.admin_msgs[k] {
                self.sent.push((i, self.now, *m));
                if self.linked[i] {
                    let r = self.mcus[i].handle(m, us);
                    self.feed(i, r);
                }
            }
            if let Some(sp) = out.set_position {
                if self.linked[i] && !self.drop_setpos[i] {
                    let r = self.mcus[i].handle(&HostMsg::SetPosition(sp[k]), us);
                    self.feed(i, r);
                }
            }
        }
        let s = self.sup.state();
        if self.states.last().map(|(_, x)| *x) != Some(s) {
            self.states.push((self.now, s));
        }
    }

    fn run_ms(&mut self, ms: u64) {
        for _ in 0..ms / 10 {
            self.step();
        }
    }

    fn run_until(&mut self, max_ms: u64, pred: impl Fn(&Rig) -> bool) -> bool {
        for _ in 0..max_ms / 10 {
            if pred(self) {
                return true;
            }
            self.step();
        }
        pred(self)
    }

    fn wait_state(&mut self, s: RigState, max_ms: u64) {
        let ok = self.run_until(max_ms, |r| r.sup.state() == s);
        assert!(
            ok,
            "never reached {s:?}; now {:?}, fault {:?}, states {:?}",
            self.sup.state(),
            self.sup.fault(),
            self.states
        );
    }

    fn admin(&mut self, cmd: AdminCmd) -> Result<(), h3cs::supervisor::AdminError> {
        self.sup.admin(cmd, self.now).map(|_| ())
    }

    fn ready(&mut self) {
        self.connect_all();
        self.wait_state(RigState::ReadyToArm, 500);
    }

    fn holding(&mut self) {
        self.ready();
        self.admin(AdminCmd::Arm).unwrap();
        self.wait_state(RigState::Holding, 2000);
    }

    fn active(&mut self, pose: Pose, gain: f64) {
        self.holding();
        self.set_game(Some((pose, gain)));
        self.run_ms(30);
        self.admin(AdminCmd::Start).unwrap();
        self.wait_state(RigState::Active, 1000);
    }

    fn sent_since(&self, from: usize) -> &[(usize, Instant, HostMsg)] {
        &self.sent[from..]
    }

    fn count(&self, from: usize, t: u8) -> [usize; 3] {
        let mut c = [0; 3];
        for (i, _, m) in self.sent_since(from) {
            if m.msg_type() == t {
                c[*i] += 1;
            }
        }
        c
    }

    fn mcu_states(&self) -> Vec<McuState> {
        self.mcus.iter().map(|m| m.state()).collect()
    }

    fn blockers(&self) -> Vec<String> {
        self.sup.blockers(self.now)
    }

    /// Operator switches sim `i` to axis `new` (DISARMED only); the link's
    /// Disconnected / Identified are emulated by `feed`.
    fn select(&mut self, i: usize, new: u8) {
        let us = self.us();
        let out = self.mcus[i].select_axis(new, us);
        self.feed(i, out);
    }

    /// Same, breaking the DISARMED rule (a misbehaving unit).
    fn select_unchecked(&mut self, i: usize, new: u8) {
        let us = self.us();
        let out = self.mcus[i].select_axis_unchecked(new, us);
        self.feed(i, out);
    }
}

fn has(blockers: &[String], needle: &str) -> bool {
    blockers.iter().any(|b| b.contains(needle))
}

/// Count targets the supervisor should command for `pose` at `gain`, from the
/// library's own compat mix and calibration.
fn expected_counts(cfg: &Config, pose: Pose, gain: f64) -> [i32; 3] {
    let checked = compat::check_pose(pose, gain, cfg.limits.max_gain, true).unwrap();
    let (virt, _) = compat::saturate(
        compat::mix(checked.pose, checked.effective_gain),
        cfg.game.virtual_min,
        cfg.game.virtual_max,
    );
    let cal = calibration::from_config(cfg);
    std::array::from_fn(|i| {
        cal[i]
            .clamp_soft(cal[i].virtual_to_count_f(virt[i]))
            .0
            .round() as i32
    })
}

const POSE_A: Pose = Pose {
    pitch: 0.5,
    roll: -0.2,
    yaw: 0.3,
};

// ------------------------------------------------------------------ connect

#[test]
fn identify_sends_only_config_and_moves_nothing() {
    let mut r = Rig::new(sim_cfg());
    r.connect_all();
    r.run_ms(2000);
    assert_eq!(r.sup.state(), RigState::ReadyToArm);
    assert!(!r.sent.is_empty());
    // CONFIG and SET_LIMITS (the position-loop tuning) only: nothing moves.
    for (_, _, m) in &r.sent {
        assert!(matches!(m, HostMsg::Config(_) | HostMsg::SetLimits(_)), "unexpected {m:?}");
    }
    assert_eq!(r.count(0, msg_type::CONFIG), [1, 1, 1]);
    assert_eq!(r.count(0, msg_type::SET_LIMITS), [1, 1, 1]);
    assert!(r.setpos.is_empty(), "SET_POSITION sent without ARM");
    assert_eq!(r.mcu_states(), vec![McuState::Disarmed; 3]);
    assert!(r.mcus.iter().all(|m| m.actual_count() == 0));
    let snap = r.sup.snapshot(r.now);
    assert!(!snap.armed);
    assert!(snap.calibration_valid);
    assert!(snap.blockers.is_empty(), "{:?}", snap.blockers);
    assert!(snap
        .axes
        .iter()
        .all(|a| a.configured && a.referenced && a.connected));
}

#[test]
fn connecting_axes_one_by_one_stays_offline_until_all() {
    let mut r = Rig::new(sim_cfg());
    r.connect(0);
    r.connect(1);
    r.run_ms(100);
    assert_eq!(r.sup.state(), RigState::Offline);
    assert!(has(&r.blockers(), "not_connected: axis3"));
    assert_eq!(
        r.admin(AdminCmd::Arm).unwrap_err().error,
        admin_error::NOT_READY
    );
    r.connect(2);
    r.run_ms(100);
    assert_eq!(r.sup.state(), RigState::ReadyToArm);
}

#[test]
fn reconnect_after_session_does_not_arm() {
    let mut r = Rig::new(sim_cfg());
    r.active(POSE_A, 0.3);
    r.admin(AdminCmd::Disarm).unwrap();
    r.wait_state(RigState::BrakedDisarmed, 3000);
    let mark = r.sent.len();
    let sp_mark = r.setpos.len();
    r.disconnect(1);
    assert_eq!(r.sup.state(), RigState::Offline);
    assert!(r.sup.fault().is_none());
    r.run_ms(100);
    r.connect(1);
    r.run_ms(2000);
    assert_eq!(r.sup.state(), RigState::ReadyToArm);
    assert!(!r.sup.is_armed());
    for (_, _, m) in r.sent_since(mark) {
        assert!(matches!(m, HostMsg::Config(_) | HostMsg::SetLimits(_)), "unexpected {m:?}");
    }
    assert_eq!(r.setpos.len(), sp_mark);
    assert_eq!(r.mcu_states(), vec![McuState::Disarmed; 3]);
}

// ------------------------------------------------------------------ ARM refusals

#[test]
fn arm_refused_when_calibration_not_marked_valid() {
    let mut cfg = sim_cfg();
    cfg.axes[0].calibration_valid = false;
    let mut r = Rig::new(cfg);
    r.connect_all();
    r.run_ms(100);
    assert_eq!(r.sup.state(), RigState::CalibrationRequired);
    let e = r.admin(AdminCmd::Arm).unwrap_err();
    assert_eq!(e.error, admin_error::NOT_READY);
    assert!(
        has(&e.blockers, "calibration_invalid: axis1"),
        "{:?}",
        e.blockers
    );
    let snap = r.sup.snapshot(r.now);
    assert!(!snap.calibration_valid);
    assert!(!snap.axes[0].calibration_valid);
    assert!(r.setpos.is_empty());
    assert_eq!(r.count(0, msg_type::ARM_PREPARE), [0; 3]);
}

#[test]
fn unit_without_selection_and_without_configured_device_id_is_unknown() {
    // Config path (selected_axis = 0): device_id 0 means "no cross-check", so
    // a unit that selects nothing and is not configured anywhere is unknown.
    let mut cfg = sim_cfg();
    cfg.axes[1].device_id = DeviceId(0);
    let mut r = Rig::with(cfg, |_, p| p.selected_axis = 0);
    r.connect_all();
    r.run_ms(100);
    assert_eq!(r.sup.state(), RigState::Offline);
    let e = r.admin(AdminCmd::Arm).unwrap_err();
    // device_id 0 itself is not a blocker: the unit is unknown, nothing else.
    assert!(
        e.blockers.iter().all(|b| b.starts_with("unknown_device") || b.starts_with("not_connected")),
        "{:?}",
        e.blockers
    );
    assert!(
        has(&e.blockers, "unknown_device: device 5131000000000002"),
        "{:?}",
        e.blockers
    );
    // Nothing but CONFIG / SET_LIMITS to the known axes, nothing at all to the unknown port.
    assert!(r
        .sent
        .iter()
        .all(|(i, _, m)| *i != 1 && matches!(m, HostMsg::Config(_) | HostMsg::SetLimits(_))));
}

#[test]
fn second_unit_on_an_idle_axis_is_a_conflict_until_its_port_closes() {
    let mut r = Rig::new(sim_cfg());
    r.ready();
    let us = r.us();
    let Some(McuMsg::Hello(h)) = r.mcus[0]
        .handle(&HostMsg::HelloReq { request_id: 2 }, us)
        .into_iter()
        .next()
    else {
        panic!("no hello")
    };
    let sent = r.sent.len();
    r.sup.on_link_event(
        LinkEvent::Identified {
            axis: ax(1),
            hello: h,
            port: "sim:dup".into(),
        },
        r.now,
    );
    // DESIGN §10.2: two claimants while idle → no primary → OFFLINE.
    assert_eq!(r.sup.state(), RigState::Offline);
    assert_eq!(r.sup.axis_port(ax(1)), None);
    assert_eq!(r.sup.axis_conflict(), vec![ax(1)]);
    let e = r.admin(AdminCmd::Arm).unwrap_err();
    assert_eq!(e.error, admin_error::AXIS_CONFLICT);
    assert!(
        has(
            &e.blockers,
            "axis_conflict: axis1 claimed by sim:1(5131000000000001), sim:dup(5131000000000001)"
        ),
        "{:?}",
        e.blockers
    );
    assert!(has(&e.blockers, "not_connected: axis1"), "{:?}", e.blockers);
    let snap = r.sup.snapshot(r.now);
    assert_eq!(snap.axis_conflict, vec![ax(1)]);
    assert_eq!(snap.axes[0].claimants.len(), 2);
    assert!(!snap.axes[0].connected);
    // Nothing goes to either unit while in conflict; stop is still accepted.
    r.run_ms(100);
    assert_eq!(r.count(sent, msg_type::CONFIG), [0; 3]);
    assert!(r.admin(AdminCmd::Stop).is_ok());
    assert_eq!(r.admin(AdminCmd::Start).unwrap_err().error, admin_error::AXIS_CONFLICT);
    assert_eq!(r.admin(AdminCmd::Center).unwrap_err().error, admin_error::AXIS_CONFLICT);

    // The second port goes away: the remaining unit is promoted like a fresh
    // Identified (CONFIG again), then READY_TO_ARM.
    r.sup.on_link_event(
        LinkEvent::PortClosed {
            port: "sim:dup".into(),
        },
        r.now,
    );
    assert!(r.sup.axis_conflict().is_empty());
    assert_eq!(r.sup.axis_port(ax(1)), Some("sim:1"));
    r.wait_state(RigState::ReadyToArm, 500);
    assert_eq!(r.count(sent, msg_type::CONFIG), [1, 0, 0]);
    assert_eq!(r.count(sent, msg_type::ARM_PREPARE), [0; 3]);
}

#[test]
fn unknown_device_blocks_until_port_closes() {
    let mut r = Rig::new(sim_cfg());
    r.connect_all();
    r.run_ms(100);
    r.sup.on_link_event(
        LinkEvent::UnknownDevice {
            port: "/dev/x".into(),
            device_id: DeviceId(0xabc),
        },
        r.now,
    );
    assert_eq!(r.sup.state(), RigState::CalibrationRequired);
    assert!(has(
        &r.blockers(),
        "unknown_device: device 0000000000000abc on port /dev/x"
    ));
    r.sup.on_link_event(
        LinkEvent::PortClosed {
            port: "/dev/x".into(),
        },
        r.now,
    );
    assert_eq!(r.sup.state(), RigState::ReadyToArm);
}

#[test]
fn arm_refused_when_unreferenced_until_reference() {
    let mut r = Rig::with(sim_cfg(), |i, p| p.start_referenced = i != 2);
    r.connect_all();
    r.run_ms(100);
    assert_eq!(r.sup.state(), RigState::CalibrationRequired);
    let e = r.admin(AdminCmd::Arm).unwrap_err();
    assert!(
        has(&e.blockers, "not_referenced: axis3"),
        "{:?}",
        e.blockers
    );
    assert!(!r.sup.snapshot(r.now).axes[2].calibration_valid);

    r.admin(AdminCmd::Reference {
        axis: ax(3),
        count: 25,
    })
    .unwrap();
    r.run_ms(50);
    assert_eq!(r.count(0, msg_type::SET_REFERENCE), [0, 0, 1]);
    assert!(r.mcus[2].is_referenced());
    assert_eq!(r.mcus[2].actual_count(), 25);
    assert_eq!(r.sup.state(), RigState::ReadyToArm);
    r.admin(AdminCmd::Arm).unwrap();
}

#[test]
fn arm_refused_with_axis_in_fault() {
    let mut r = Rig::new(sim_cfg());
    r.connect_all();
    r.run_ms(100);
    r.mcus[1].force_fault(FaultCode::EncoderFault);
    r.run_ms(50);
    assert_eq!(r.sup.state(), RigState::FaultLatched);
    assert_eq!(
        r.sup.fault(),
        Some(&RigFault::AxisFault {
            axis: ax(2),
            code: FaultCode::EncoderFault
        })
    );
    let e = r.admin(AdminCmd::Arm).unwrap_err();
    assert_eq!(e.error, admin_error::FAULT_LATCHED);
    assert!(has(&e.blockers, "axis_fault: axis2"), "{:?}", e.blockers);
    assert_eq!(r.count(0, msg_type::ARM_PREPARE), [0; 3]);
}

#[test]
fn arm_refused_with_position_outside_soft_limits() {
    let mut r = Rig::with(sim_cfg(), |i, p| {
        if i == 0 {
            p.initial_count = 2500; // soft_max 2300
        }
    });
    r.connect_all();
    r.run_ms(100);
    assert_eq!(r.sup.state(), RigState::CalibrationRequired);
    let e = r.admin(AdminCmd::Arm).unwrap_err();
    assert!(
        has(
            &e.blockers,
            "position_outside_soft_limits: axis1 actual 2500"
        ),
        "{:?}",
        e.blockers
    );
}

#[test]
fn arm_prepare_rejected_by_mcu_latches_fault() {
    let mut r = Rig::new(sim_cfg());
    r.ready();
    r.admin(AdminCmd::Arm).unwrap();
    // Moved out of the soft window after the check: MCU rejects ARM_PREPARE.
    r.mcus[0].nudge_actual(5000);
    r.wait_state(RigState::FaultLatched, 500);
    r.run_ms(20);
    assert_eq!(r.sup.fault(), Some(&RigFault::ArmTimeout));
    let rec = r.sup.fault_history().next().unwrap();
    assert!(
        rec.detail
            .as_deref()
            .unwrap_or("")
            .contains("ARM_PREPARE rejected: BadState"),
        "{rec:?}"
    );
    assert_eq!(r.count(0, msg_type::HOST_FAULT), [1, 1, 1]);
}

#[test]
fn arm_times_out_without_acks() {
    let mut r = Rig::new(sim_cfg());
    r.ready();
    r.mcus[2].set_drop_rx(true);
    r.admin(AdminCmd::Arm).unwrap();
    r.run_ms(4900);
    assert_eq!(r.sup.state(), RigState::Arming);
    // Configuring: no ARM_PREPARE until every CONFIG is ACKed.
    assert_eq!(r.count(0, msg_type::ARM_PREPARE), [0; 3]);
    r.wait_state(RigState::FaultLatched, 300);
    assert_eq!(r.sup.fault(), Some(&RigFault::ArmTimeout));
}

// ------------------------------------------------------------------ counters

#[test]
fn counters_start_random_and_never_zero() {
    let cfg = sim_cfg();
    let a = Supervisor::new(&cfg);
    let b = Supervisor::new(&cfg);
    assert_ne!(
        (a.session_epoch(), a.next_request_id_peek()),
        (b.session_epoch(), b.next_request_id_peek())
    );
    assert_ne!(a.session_epoch(), 0);
    assert_ne!(a.next_request_id_peek(), 0);

    let mut r = Rig::new(cfg.clone());
    r.sup = Supervisor::with_initial_ids(&cfg, u32::MAX, u32::MAX);
    r.holding();
    // epoch MAX + 1 wraps past 0 to 1.
    assert_eq!(r.sup.session_epoch(), 1);
    let ids: Vec<u32> = r
        .sent
        .iter()
        .filter_map(|(_, _, m)| match *m {
            HostMsg::Config(c) => Some(c.request_id),
            HostMsg::ArmPrepare { request_id, .. } => Some(request_id),
            _ => None,
        })
        .collect();
    assert_eq!(ids[0], u32::MAX);
    assert!(ids.iter().all(|&id| id != 0), "{ids:?}");
    let mut sorted = ids.clone();
    sorted.sort_unstable();
    sorted.dedup();
    assert_eq!(sorted.len(), ids.len(), "request_id reused");
    // Identify CONFIGs used the start epoch; ARM CONFIGs the new one.
    let epochs: Vec<u32> = r
        .sent
        .iter()
        .filter_map(|(_, _, m)| {
            if let HostMsg::Config(c) = m {
                Some(c.session_epoch)
            } else {
                None
            }
        })
        .collect();
    assert_eq!(epochs, vec![u32::MAX, u32::MAX, u32::MAX, 1, 1, 1]);
}

// ------------------------------------------------------------------ happy path

#[test]
fn happy_path_arm_start_track_stale_stop_hold_disarm() {
    let cfg = sim_cfg();
    let mut r = Rig::new(cfg.clone());
    r.ready();
    let epoch0 = r.sup.session_epoch();
    r.admin(AdminCmd::Arm).unwrap();
    assert_eq!(r.sup.state(), RigState::Arming);
    assert!(r.sup.is_armed());
    r.wait_state(RigState::Holding, 2000);
    assert_eq!(r.sup.session_epoch(), epoch0.wrapping_add(1));
    assert_eq!(r.mcu_states(), vec![McuState::Holding; 3]);
    assert_eq!(r.count(0, msg_type::ARM_PREPARE), [1, 1, 1]);

    // SET_POSITION flows from ARMING on: per-axis seq strictly increasing,
    // one group_frame_id per tick, limits from the calibration.
    let arming: Vec<_> = r
        .setpos
        .iter()
        .filter(|(_, s, _)| *s == RigState::Arming)
        .collect();
    assert!(!arming.is_empty());
    for w in r.setpos.windows(2) {
        let (a, b) = (&w[0].2, &w[1].2);
        for k in 0..3 {
            assert!(b[k].seq > a[k].seq);
            assert_eq!(b[k].group_frame_id, b[0].group_frame_id);
        }
        assert!(b[0].group_frame_id > a[0].group_frame_id);
    }
    let first = &r.setpos[0].2;
    assert_eq!([first[0].seq, first[1].seq, first[2].seq], [1, 1, 1]);
    for k in 0..3 {
        assert_eq!(first[k].session_epoch, r.sup.session_epoch());
        assert_eq!(first[k].max_vel_count_s, cfg.limits.max_vel_count_s);
        assert_eq!(first[k].max_acc_count_s2, cfg.limits.max_acc_count_s2);
        assert_eq!(first[k].valid_for_ms, cfg.timing.set_position_valid_for_ms);
        assert_eq!(first[k].apply_at_us, 0);
    }

    // Start needs a fresh pose.
    assert_eq!(
        r.admin(AdminCmd::Start).unwrap_err().error,
        admin_error::NO_FRESH_POSE
    );
    r.set_game(Some((POSE_A, 0.3)));
    r.run_ms(30);
    r.admin(AdminCmd::Start).unwrap();
    r.wait_state(RigState::Active, 500);
    assert_eq!(r.sup.active_source(), Some(ActiveSource::Game));

    r.run_ms(3000);
    let want = expected_counts(&cfg, POSE_A, 0.3);
    // Sanity on the mix itself: m1 = 512 + 153.3·0.3 → axis 1 ≈ +180.
    assert!((175..=185).contains(&want[0]), "{want:?}");
    let snap = r.sup.snapshot(r.now);
    assert_eq!(snap.target_counts, want);
    assert_eq!(snap.requested_pose, Some(POSE_A));
    assert_eq!(snap.effective_gain, Some(0.3));
    for k in 0..3 {
        assert!(
            (r.mcus[k].actual_count() - want[k]).abs() <= 2,
            "axis{} {} vs {}",
            k + 1,
            r.mcus[k].actual_count(),
            want[k]
        );
    }
    assert!(snap
        .axes
        .iter()
        .all(|a| a.mcu_state == Some(McuState::Active)));

    // Game stops sending → stale after source_freshness_ms → coordinated stop.
    let stop_mark = r.sent.len();
    let sp_mark = r.setpos.len();
    let last_pose_at = r.now;
    r.set_game(None);
    r.wait_state(RigState::Stopping, 400);
    let stale_after = r.now - last_pose_at;
    assert!(
        stale_after >= Duration::from_millis(cfg.timing.source_freshness_ms - 10),
        "{stale_after:?}"
    );
    assert!(
        stale_after <= Duration::from_millis(cfg.timing.source_freshness_ms + 20),
        "{stale_after:?}"
    );
    let stopping_at = r.now;
    r.wait_state(RigState::BrakedDisarmed, 4000);

    assert_eq!(r.count(stop_mark, msg_type::CONTROLLED_STOP), [1, 1, 1]);
    assert_eq!(r.count(stop_mark, msg_type::DISARM), [1, 1, 1]);
    assert_eq!(r.count(stop_mark, msg_type::START), [0; 3]);
    let disarm_at = r
        .sent_since(stop_mark)
        .iter()
        .find(|(_, _, m)| m.msg_type() == msg_type::DISARM)
        .unwrap()
        .1;
    assert!(
        disarm_at - stopping_at >= Duration::from_millis(cfg.timing.stop_hold_ms),
        "{:?}",
        disarm_at - stopping_at
    );
    // SET_POSITION kept the lease through the hold, never aimed at neutral.
    let during: Vec<_> = r.setpos[sp_mark..].iter().collect();
    assert!(during
        .iter()
        .any(|(t, _, _)| *t > stopping_at + Duration::from_millis(900)));
    for (_, _, sp) in &during {
        for k in 0..3 {
            assert!(
                (sp[k].target_count - want[k]).abs() <= 1,
                "target moved: {sp:?}"
            );
        }
    }
    for k in 0..3 {
        assert!(
            (r.mcus[k].actual_count() - want[k]).abs() <= 2,
            "axis{} moved during stop",
            k + 1
        );
    }
    assert_eq!(r.mcu_states(), vec![McuState::Disarmed; 3]);
    let snap = r.sup.snapshot(r.now);
    assert!(!snap.armed);
    assert!(snap.fault.is_none());
    // No more SET_POSITION once disarmed.
    let n = r.setpos.len();
    r.run_ms(200);
    assert_eq!(r.setpos.len(), n);
    assert_eq!(r.sup.state(), RigState::BrakedDisarmed);
}

#[test]
fn late_pose_after_stop_does_not_restart() {
    let cfg = sim_cfg();
    let mut r = Rig::new(cfg.clone());
    r.active(POSE_A, 0.3);
    r.run_ms(2000);
    let want = expected_counts(&cfg, POSE_A, 0.3);
    r.admin(AdminCmd::Stop).unwrap();
    assert_eq!(r.sup.state(), RigState::Stopping);
    let mark = r.sent.len();
    // Game keeps (or resumes) sending fresh poses for another place.
    let other = Pose {
        pitch: -0.8,
        roll: 0.5,
        yaw: -0.5,
    };
    r.set_game(Some((other, 0.35)));
    r.wait_state(RigState::BrakedDisarmed, 4000);
    r.run_ms(1000);
    assert_eq!(r.sup.state(), RigState::BrakedDisarmed);
    assert_eq!(r.count(mark, msg_type::START), [0; 3]);
    assert_eq!(r.count(mark, msg_type::ARM_PREPARE), [0; 3]);
    for k in 0..3 {
        assert!((r.mcus[k].actual_count() - want[k]).abs() <= 2);
    }
    for (_, s, sp) in &r.setpos {
        if *s == RigState::Stopping {
            for k in 0..3 {
                assert!((sp[k].target_count - want[k]).abs() <= 1);
            }
        }
    }
    // Start is refused there too.
    assert_eq!(
        r.admin(AdminCmd::Start).unwrap_err().error,
        admin_error::NOT_HOLDING
    );
}

#[test]
fn stop_mid_motion_decelerates_without_reversal() {
    let cfg = sim_cfg();
    let mut r = Rig::new(cfg.clone());
    let a = Pose {
        pitch: -0.5,
        roll: 0.0,
        yaw: 0.0,
    };
    let b = Pose {
        pitch: 0.5,
        roll: 0.0,
        yaw: 0.0,
    };
    r.active(a, 0.35);
    r.run_ms(2000);
    r.set_game(Some((b, 0.35)));
    r.run_ms(120);
    let sp_mark = r.setpos.len();
    r.admin(AdminCmd::Stop).unwrap();
    let mut prev = r.mcus.iter().map(|m| m.actual_count()).collect::<Vec<_>>();
    let start = prev.clone();
    let mut pos_trace = Vec::new();
    while r.sup.state() != RigState::BrakedDisarmed {
        assert!(r.now - r.t0 < Duration::from_secs(30));
        r.step();
        let cur: Vec<_> = r.mcus.iter().map(|m| m.actual_count()).collect();
        // Axis 1 was moving up (+), axis 2 down (−): no reversal while stopping.
        assert!(cur[0] >= prev[0], "axis1 reversed {prev:?} → {cur:?}");
        assert!(cur[1] <= prev[1], "axis2 reversed {prev:?} → {cur:?}");
        pos_trace.push(cur.clone());
        prev = cur;
    }
    let target_b = expected_counts(&cfg, b, 0.35);
    assert!(
        prev[0] > start[0] && prev[0] < target_b[0],
        "{start:?} → {prev:?}"
    );
    // The planner decelerated too: no SET_POSITION step back toward neutral.
    for w in r.setpos[sp_mark..].windows(2) {
        assert!(w[1].2[0].target_count >= w[0].2[0].target_count);
    }
    assert!(r.sup.fault().is_none());
}

/// Issue #15 H1: "stopped" was judged on the commanded trajectory alone
/// (planner settled, MCUs HOLDING). The MCU declares HOLDING when its
/// SETPOINT velocity is 0, so a gravity-loaded axis still 200 counts off
/// would have been braked there. The rig must wait for the MEASURED rest
/// and fault, with the numbers, when it never comes.
#[test]
fn stop_waits_for_measured_rest_and_faults_on_a_stalled_axis() {
    let cfg = sim_cfg();
    let mut r = Rig::new(cfg.clone());
    r.holding();
    // Axis 2 is pushed 200 counts off its setpoint and cannot move back
    // (inside following_error_limit 800: the MCU itself stays quiet).
    r.mcus[1].set_stall(true);
    r.mcus[1].nudge_actual(200);
    r.run_ms(50);
    assert_eq!(r.mcu_states(), vec![McuState::Holding; 3]);
    r.admin(AdminCmd::Stop).unwrap();
    r.wait_state(RigState::Stopping, 100);
    // Trajectory stopped at once (nothing was moving), but axis 2 is not at rest.
    r.run_ms(cfg.timing.stop_settle_timeout_ms - 200);
    assert_eq!(r.sup.state(), RigState::Stopping, "{:?}", r.sup.fault());
    assert_eq!(r.mcu_states(), vec![McuState::Holding; 3], "no brake while an axis is off");
    r.wait_state(RigState::FaultLatched, 600);
    assert_eq!(r.sup.fault(), Some(&RigFault::StopTimeout));
    let rec = r.sup.fault_history().last().unwrap();
    let detail = rec.detail.clone().unwrap_or_default();
    assert!(detail.starts_with("deceleration did not settle: axis2 vel +0 count/s fe +200"), "{detail}");
    assert!(detail.contains("|fe| <= 50 for 200 ms"), "{detail}");
}

/// An axis that drifts off during the hold is not braked where it is: the
/// hold waits (up to stop_settle_timeout_ms more) and faults with the numbers.
#[test]
fn hold_does_not_brake_an_axis_that_drifted_and_faults_in_time() {
    let cfg = sim_cfg();
    let mut r = Rig::new(cfg.clone());
    r.active(POSE_A, 0.3);
    r.run_ms(1500);
    let mark = r.sent.len();
    r.admin(AdminCmd::Stop).unwrap();
    // Into the hold (all HOLDING, planner settled, at rest), then push axis 3.
    assert!(r.run_until(2000, |r| r.sup.state() == RigState::Stopping && r.mcu_states() == vec![McuState::Holding; 3]));
    r.run_ms(cfg.timing.rest_dwell_ms + 100);
    r.mcus[2].set_stall(true);
    r.mcus[2].nudge_actual(-120);
    r.run_ms(cfg.timing.stop_hold_ms + cfg.timing.stop_settle_timeout_ms - 300);
    assert_eq!(r.sup.state(), RigState::Stopping, "{:?}", r.sup.fault());
    assert_eq!(r.count(mark, msg_type::DISARM), [0; 3], "no DISARM (brake) with axis 3 off");
    r.wait_state(RigState::FaultLatched, 800);
    assert_eq!(r.sup.fault(), Some(&RigFault::StopTimeout));
    let detail = r.sup.fault_history().last().unwrap().detail.clone().unwrap_or_default();
    assert!(detail.starts_with("hold did not come to rest before the brake: axis3 vel +0 count/s fe -120"), "{detail}");
}

/// A healthy stop still brakes in the usual time: the rest dwell adds at most
/// `rest_dwell_ms` to the deceleration and nothing to the hold.
#[test]
fn healthy_stop_reaches_braked_within_hold_plus_dwell() {
    let cfg = sim_cfg();
    let mut r = Rig::new(cfg.clone());
    r.active(POSE_A, 0.3);
    r.run_ms(1500);
    r.set_game(Some((POSE_B, 0.3)));
    r.run_ms(200); // moving again
    let mark = r.sent.len();
    let t_stop = r.now;
    r.admin(AdminCmd::Stop).unwrap();
    r.wait_state(RigState::BrakedDisarmed, 4000);
    let took = r.now - t_stop;
    let disarm_at = r.sent_since(mark).iter().find(|(_, _, m)| m.msg_type() == msg_type::DISARM).unwrap().1;
    assert!(disarm_at - t_stop >= Duration::from_millis(cfg.timing.stop_hold_ms), "{:?}", disarm_at - t_stop);
    // Deceleration from 0.3 x max_vel plus the dwell, the hold and the sim's
    // disarming delay: well inside hold + dwell + 1 s.
    let budget = Duration::from_millis(cfg.timing.stop_hold_ms + cfg.timing.rest_dwell_ms + 1000);
    assert!(took <= budget, "{took:?} > {budget:?}");
    assert!(r.sup.fault().is_none());
    for k in 0..3 {
        assert_eq!(r.mcus[k].status().velocity_count_s, 0);
        assert_eq!(r.mcus[k].status().following_error_count, 0);
    }
}

#[test]
fn disarm_from_active_brakes_without_center() {
    let cfg = sim_cfg();
    let mut r = Rig::new(cfg.clone());
    r.active(POSE_A, 0.3);
    r.run_ms(2000);
    let mark = r.sent.len();
    r.admin(AdminCmd::Disarm).unwrap();
    r.wait_state(RigState::BrakedDisarmed, 3000);
    assert_eq!(r.count(mark, msg_type::DISARM), [1, 1, 1]);
    let want = expected_counts(&cfg, POSE_A, 0.3);
    for k in 0..3 {
        assert!((r.mcus[k].actual_count() - want[k]).abs() <= 2);
    }
    // Stop / Disarm are idempotent no-ops when not driving.
    assert!(r.sup.admin(AdminCmd::Stop, r.now).unwrap().note.is_some());
    assert!(r.sup.admin(AdminCmd::Disarm, r.now).unwrap().note.is_some());
    // Re-ARM from BRAKED_DISARMED works (explicitly).
    r.admin(AdminCmd::Arm).unwrap();
    r.wait_state(RigState::Holding, 2000);
}

#[test]
fn gain_over_max_reported_in_snapshot() {
    let cfg = sim_cfg();
    let mut r = Rig::new(cfg.clone());
    r.active(POSE_A, 0.6); // limits.max_gain 0.5
    r.run_ms(3000);
    let snap = r.sup.snapshot(r.now);
    assert_eq!(snap.requested_gain, Some(0.6));
    assert_eq!(snap.effective_gain, Some(cfg.limits.max_gain));
    assert!(
        snap.limits.contains(&LimitReason::GainLimited),
        "{:?}",
        snap.limits
    );
    assert_eq!(
        snap.target_counts,
        expected_counts(&cfg, POSE_A, cfg.limits.max_gain)
    );
}

#[test]
fn saturation_and_soft_limits_reported() {
    let mut cfg = sim_cfg();
    cfg.limits.max_gain = 1.0;
    cfg.game.virtual_max = 900.0;
    let mut r = Rig::new(cfg.clone());
    r.active(
        Pose {
            pitch: 1.0,
            roll: 1.0,
            yaw: 0.0,
        },
        1.0,
    );
    r.run_ms(200);
    let snap = r.sup.snapshot(r.now);
    assert!(
        snap.limits.contains(&LimitReason::VirtualSaturated),
        "{:?}",
        snap.limits
    );
    assert_eq!(snap.virtual_targets[0], 900.0);
}

// ------------------------------------------------------------------ faults

#[test]
fn one_axis_fault_sends_host_fault_to_all_once() {
    let mut r = Rig::new(sim_cfg());
    r.active(POSE_A, 0.3);
    r.run_ms(500);
    let mark = r.sent.len();
    let sp_mark = r.setpos.len();
    r.mcus[0].force_fault(FaultCode::FollowingError);
    r.wait_state(RigState::FaultLatched, 100);
    r.run_ms(500);
    assert_eq!(
        r.sup.fault(),
        Some(&RigFault::AxisFault {
            axis: ax(1),
            code: FaultCode::FollowingError
        })
    );
    assert_eq!(r.count(mark, msg_type::HOST_FAULT), [1, 1, 1]);
    assert_eq!(r.mcu_states(), vec![McuState::Fault; 3]);
    assert_eq!(r.mcus[1].fault_code(), FaultCode::HostFault);
    // No motion commands after the latch (the latch tick itself sends none).
    assert!(r.setpos.len() <= sp_mark + 1);
    let snap = r.sup.snapshot(r.now);
    assert_eq!(snap.rig_state, RigState::FaultLatched);
    assert!(!snap.armed);
    assert!(!snap.fault_history.is_empty());
    // Late poses do not move anything.
    assert_eq!(
        r.admin(AdminCmd::Start).unwrap_err().error,
        admin_error::FAULT_LATCHED
    );
}

#[test]
fn disconnect_while_active_faults() {
    let mut r = Rig::new(sim_cfg());
    r.active(POSE_A, 0.3);
    let mark = r.sent.len();
    r.disconnect(1);
    assert_eq!(r.sup.state(), RigState::FaultLatched);
    assert_eq!(
        r.sup.fault(),
        Some(&RigFault::AxisDisconnected { axis: ax(2) })
    );
    r.run_ms(50);
    assert_eq!(r.count(mark, msg_type::HOST_FAULT), [1, 0, 1]);
}

#[test]
fn disconnect_while_ready_goes_offline_without_fault() {
    let mut r = Rig::new(sim_cfg());
    r.ready();
    r.disconnect(2);
    assert_eq!(r.sup.state(), RigState::Offline);
    assert!(r.sup.fault().is_none());
    r.run_ms(500);
    assert_eq!(r.sup.state(), RigState::Offline);
    assert_eq!(r.count(0, msg_type::HOST_FAULT), [0; 3]);
    let snap = r.sup.snapshot(r.now);
    assert!(!snap.axes[2].connected);
    assert_eq!(snap.axes[2].mcu_state, None);
}

#[test]
fn status_silence_while_active_faults() {
    let cfg = sim_cfg();
    let mut r = Rig::new(cfg.clone());
    r.active(POSE_A, 0.3);
    r.mcus[2].set_freeze_status(true);
    let t = r.now;
    r.wait_state(RigState::FaultLatched, 500);
    assert_eq!(
        r.sup.fault(),
        Some(&RigFault::AxisDisconnected { axis: ax(3) })
    );
    assert!(r.now - t >= Duration::from_millis(cfg.timing.axis_status_timeout_ms));
}

#[test]
fn reboot_while_holding_faults() {
    let mut r = Rig::new(sim_cfg());
    r.holding();
    r.mcus[1].reboot(0xdead_beef);
    r.wait_state(RigState::FaultLatched, 100);
    assert_eq!(r.sup.fault(), Some(&RigFault::AxisRebooted { axis: ax(2) }));
    let snap = r.sup.snapshot(r.now);
    assert_eq!(snap.axes[1].boot_id, Some(0xdead_beef));
    assert!(!snap.axes[1].configured);
}

#[test]
fn operator_fault_latches_in_any_state() {
    let mut r = Rig::new(sim_cfg());
    r.ready();
    r.admin(AdminCmd::Fault {
        note: "test".into(),
    })
    .unwrap();
    assert_eq!(r.sup.state(), RigState::FaultLatched);
    assert_eq!(
        r.sup.fault(),
        Some(&RigFault::Operator {
            note: "test".into()
        })
    );
    r.run_ms(50);
    assert_eq!(r.count(0, msg_type::HOST_FAULT), [1, 1, 1]);
    r.run_ms(50);
    r.admin(AdminCmd::FaultReset).unwrap();
    r.wait_state(RigState::ReadyToArm, 500);
}

#[test]
fn fault_reset_waits_for_host_fault_ack() {
    let mut r = Rig::new(sim_cfg());
    r.ready();
    r.admin(AdminCmd::Fault { note: "now".into() }).unwrap();
    // Same instant: HOST_FAULT is still queued, the axes still read DISARMED.
    let e = r.admin(AdminCmd::FaultReset).unwrap_err();
    assert_eq!(e.error, admin_error::AXIS_NOT_SETTLED, "{e:?}");
    r.step();
    assert_eq!(r.mcu_states(), vec![McuState::Fault; 3]);
    r.admin(AdminCmd::FaultReset).unwrap();
    r.wait_state(RigState::ReadyToArm, 500);
    r.run_ms(200);
    assert_eq!(r.sup.state(), RigState::ReadyToArm);
    assert_eq!(r.count(0, msg_type::HOST_FAULT), [1, 1, 1]);
    assert_eq!(r.sup.fault_history().count(), 1);
}

#[test]
fn fault_reset_refused_while_cause_present_and_never_rearms() {
    let mut r = Rig::new(sim_cfg());
    r.holding();
    r.mcus[1].set_fault_cause_present(true);
    r.mcus[1].force_fault(FaultCode::Overspeed);
    // Only axis 2's report reaches the supervisor; HOST_FAULT is not delivered yet.
    let us = r.us() + 1_000;
    let out = r.mcus[1].tick(us);
    r.feed(1, out);
    assert_eq!(r.sup.state(), RigState::FaultLatched);
    assert_eq!(r.mcus[0].state(), McuState::Holding);
    let e = r.admin(AdminCmd::FaultReset).unwrap_err();
    assert_eq!(e.error, admin_error::AXIS_NOT_SETTLED, "{e:?}");
    r.run_ms(50);

    // Disconnected axis → refused.
    r.disconnect(0);
    let e = r.admin(AdminCmd::FaultReset).unwrap_err();
    assert_eq!(e.error, admin_error::AXIS_DISCONNECTED);
    assert!(has(&e.blockers, "not_connected: axis1"));
    r.connect(0);
    r.run_ms(50);
    assert_eq!(r.sup.state(), RigState::FaultLatched);

    // MCU-reported cause → FAULT_RESET rejected, rig stays latched.
    r.admin(AdminCmd::FaultReset).unwrap();
    r.run_ms(300);
    assert_eq!(r.sup.state(), RigState::FaultLatched);
    let b = r.sup.snapshot(r.now).blockers;
    assert!(
        has(&b, "fault_reset_rejected: axis2 FaultCausePresent"),
        "{b:?}"
    );
    assert_eq!(r.mcus[1].state(), McuState::Fault);

    // Cause cleared → reset → DISARMED → READY_TO_ARM, not armed.
    r.mcus[1].set_fault_cause_present(false);
    let mark = r.sent.len();
    r.admin(AdminCmd::FaultReset).unwrap();
    r.wait_state(RigState::ReadyToArm, 3000);
    assert!(r.sup.fault().is_none());
    assert!(!r.sup.is_armed());
    assert_eq!(r.mcu_states(), vec![McuState::Disarmed; 3]);
    r.run_ms(1000);
    assert_eq!(r.sup.state(), RigState::ReadyToArm);
    assert_eq!(r.count(mark, msg_type::ARM_PREPARE), [0; 3]);
    assert_eq!(r.count(mark, msg_type::START), [0; 3]);
    assert_eq!(r.count(mark, msg_type::SET_POSITION), [0; 3]);
    assert!(r.sup.fault_history().count() >= 1);
    // And it can be armed again explicitly.
    r.admin(AdminCmd::Arm).unwrap();
    r.wait_state(RigState::Holding, 2000);
}

#[test]
fn fault_reset_requires_a_fault() {
    let mut r = Rig::new(sim_cfg());
    r.ready();
    assert_eq!(
        r.admin(AdminCmd::FaultReset).unwrap_err().error,
        admin_error::NOT_FAULTED
    );
}

#[test]
fn frame_skew_faults_after_grace() {
    let cfg = sim_cfg();
    let mut r = Rig::new(cfg.clone());
    r.active(POSE_A, 0.3);
    let skewed = |r: &Rig| {
        let mut s = r.mcus[2].status();
        s.last_applied_frame = s
            .last_applied_frame
            .wrapping_sub(cfg.timing.max_frame_skew + 5);
        LinkEvent::Status {
            axis: ax(3),
            status: s,
            received_at: r.now,
        }
    };
    // Inside the 200 ms grace: tolerated.
    let ev = skewed(&r);
    r.pre_tick.push(ev);
    r.step();
    assert_eq!(r.sup.state(), RigState::Active);
    r.run_ms(300);
    assert_eq!(r.sup.state(), RigState::Active);
    assert!(r
        .sup
        .frame_skew()
        .is_some_and(|s| s <= cfg.timing.max_frame_skew));
    let ev = skewed(&r);
    r.pre_tick.push(ev);
    r.step();
    assert_eq!(r.sup.state(), RigState::FaultLatched);
    assert!(
        matches!(r.sup.fault(), Some(RigFault::FrameSkew { max_skew_frames }) if *max_skew_frames > cfg.timing.max_frame_skew)
    );
}

#[test]
fn axis_dropping_out_of_active_is_a_mismatch_fault() {
    let mut r = Rig::new(sim_cfg());
    r.active(POSE_A, 0.3);
    r.run_ms(200);
    // SET_POSITION to axis 1 stops: valid_for expires → MCU STOPPING while rig ACTIVE.
    r.drop_setpos[0] = true;
    r.wait_state(RigState::FaultLatched, 200);
    assert!(
        matches!(r.sup.fault(), Some(RigFault::AxisStateMismatch { axis, expected: McuState::Active, .. }) if *axis == ax(1)),
        "{:?}",
        r.sup.fault()
    );
}

// ------------------------------------------------------------------ center / reference

#[test]
fn center_moves_to_neutral_at_reduced_speed_then_holds() {
    let cfg = sim_cfg();
    let start = [300, -100, 50];
    let mut r = Rig::with(cfg.clone(), |i, p| p.initial_count = start[i]);
    r.holding();
    assert_eq!(r.mcus[0].actual_count(), 300);
    let sp_mark = r.setpos.len();
    r.admin(AdminCmd::Center).unwrap();
    r.wait_state(RigState::Active, 500);
    assert_eq!(r.sup.active_source(), Some(ActiveSource::Center));
    r.wait_state(RigState::Holding, 5000);
    // Back in HOLDING but still settling (Issue #15 H1: the measured rest
    // dwell must pass): START is refused as busy until then.
    r.run_ms(50);
    assert_eq!(r.sup.state(), RigState::Holding);
    assert_eq!(r.admin(AdminCmd::Start).unwrap_err().error, admin_error::BUSY);
    r.run_ms(cfg.timing.rest_dwell_ms + 100);
    assert_eq!(r.sup.state(), RigState::Holding);
    assert!(r.sup.is_armed());
    assert_eq!(r.mcu_states(), vec![McuState::Holding; 3]);
    let cal = calibration::from_config(&cfg);
    for k in 0..3 {
        assert!(
            (r.mcus[k].actual_count() - cal[k].neutral_count()).abs() <= 3,
            "axis{} at {}",
            k + 1,
            r.mcus[k].actual_count()
        );
    }
    let f = cfg.game.center_speed_fraction;
    let centering: Vec<_> = r.setpos[sp_mark..]
        .iter()
        .filter(|(_, s, _)| *s == RigState::Active)
        .collect();
    assert!(!centering.is_empty());
    for (_, _, sp) in &centering {
        for k in 0..3 {
            assert_eq!(
                sp[k].max_vel_count_s,
                (f64::from(cfg.limits.max_vel_count_s) * f).round() as u32
            );
        }
    }
    for w in r.setpos[sp_mark..].windows(2) {
        for k in 0..3 {
            let step = (w[1].2[k].target_count - w[0].2[k].target_count).abs();
            let max = f64::from(cfg.limits.max_vel_count_s) * f * STEP.as_secs_f64() + 1.0;
            assert!(f64::from(step) <= max, "axis{} step {step} > {max}", k + 1);
        }
    }
    // Moving 300 counts at 500 count/s takes well over 0.6 s.
    assert!(centering.len() >= 60, "{}", centering.len());
    // Pose is not used by center, and a game Start works afterwards.
    r.set_game(Some((POSE_A, 0.3)));
    r.run_ms(30);
    r.admin(AdminCmd::Start).unwrap();
    r.wait_state(RigState::Active, 500);
    assert_eq!(r.sup.active_source(), Some(ActiveSource::Game));
}

/// 2026-11-01 on hardware: CENTER reported `center_reached` with axis 2 still
/// 280 counts below neutral (gravity-loaded, position loop saturated at
/// duty_min). The setpoint had arrived; the mechanism had not. The move must
/// be judged on measured position and fault when an axis cannot get there.
#[test]
fn center_waits_for_measured_position_and_faults_on_a_stalled_axis() {
    let cfg = sim_cfg();
    let cal = calibration::from_config(&cfg);
    // 150 counts short: inside following_error_limit (800), so only the new
    // measured-position check can end the move.
    let mut r = Rig::with(cfg.clone(), |i, p| p.initial_count = cal[i].neutral_count() - 150);
    r.holding();
    r.mcus[1].set_stall(true); // axis 2's plant does not move at all
    r.sup.drain_events();
    r.admin(AdminCmd::Center).unwrap();
    r.wait_state(RigState::Active, 500);
    // 150 counts at max_vel × 0.25 takes about 1.2 s; the planner and the
    // setpoints are at neutral long before 3 s, but axis 2 is not.
    r.run_ms(3000);
    assert_eq!(r.sup.state(), RigState::Active, "centered reported with axis2 at {}", r.mcus[1].actual_count());
    assert_eq!(r.sup.active_source(), Some(ActiveSource::Center));
    let (ev, _) = r.sup.drain_events();
    assert!(ev.iter().all(|e| e.kind != "center_reached"), "{ev:?}");
    assert!((r.mcus[0].actual_count() - cal[0].neutral_count()).abs() <= 3);
    assert_eq!(r.mcus[1].actual_count(), cal[1].neutral_count() - 150);

    r.wait_state(RigState::FaultLatched, cfg.timing.center_reach_timeout_ms + 2000);
    assert_eq!(r.sup.fault(), Some(&RigFault::StopTimeout));
    let rec = r.sup.fault_history().last().unwrap();
    let detail = rec.detail.clone().unwrap_or_default();
    assert!(detail.starts_with("center: axis2"), "{detail}");
    let (ev, _) = r.sup.drain_events();
    assert!(ev.iter().all(|e| e.kind != "center_reached"), "{ev:?}");
}

/// The same move completes normally once the plant can follow: the measured
/// check must not reject a healthy centre (the end position sits inside the
/// firmware deadband, i.e. a few counts off neutral).
#[test]
fn center_completes_when_measured_position_catches_up() {
    let cfg = sim_cfg();
    let cal = calibration::from_config(&cfg);
    let mut r = Rig::with(cfg.clone(), |i, p| p.initial_count = cal[i].neutral_count() - 150);
    r.holding();
    r.mcus[1].set_stall(true);
    r.sup.drain_events();
    r.admin(AdminCmd::Center).unwrap();
    r.wait_state(RigState::Active, 500);
    r.run_ms(2000);
    assert_eq!(r.sup.state(), RigState::Active);
    r.mcus[1].set_stall(false); // the load lets go before the timeout
    r.wait_state(RigState::Holding, cfg.timing.center_reach_timeout_ms);
    assert!(r.sup.fault().is_none());
    let (ev, _) = r.sup.drain_events();
    assert_eq!(ev.iter().filter(|e| e.kind == "center_reached").count(), 1, "{ev:?}");
    for k in 0..3 {
        assert!((r.mcus[k].actual_count() - cal[k].neutral_count()).abs() <= 3, "axis{} at {}", k + 1, r.mcus[k].actual_count());
    }
}

#[test]
fn center_requires_holding() {
    let mut r = Rig::new(sim_cfg());
    r.ready();
    assert_eq!(
        r.admin(AdminCmd::Center).unwrap_err().error,
        admin_error::NOT_HOLDING
    );
    assert!(r.setpos.is_empty());
}

#[test]
fn reference_only_when_disarmed() {
    let mut r = Rig::new(sim_cfg());
    r.holding();
    let e = r
        .admin(AdminCmd::Reference {
            axis: ax(1),
            count: 0,
        })
        .unwrap_err();
    assert_eq!(e.error, admin_error::NOT_DISARMED);
    assert_eq!(r.count(0, msg_type::SET_REFERENCE), [0; 3]);
    r.admin(AdminCmd::Disarm).unwrap();
    r.wait_state(RigState::BrakedDisarmed, 2000);
    r.admin(AdminCmd::Reference {
        axis: ax(1),
        count: 10,
    })
    .unwrap();
    r.run_ms(50);
    assert_eq!(r.count(0, msg_type::SET_REFERENCE), [1, 0, 0]);
    assert_eq!(r.mcus[0].actual_count(), 10);
}

// ------------------------------------------------------------------ misc

#[test]
fn config_rejection_outside_arming_is_not_a_fault() {
    let cfg = sim_cfg();
    let mut sup = Supervisor::new(&cfg);
    let mut mcu = SimMcu::new(SimParams {
        device_id: cfg.axes[0].device_id,
        ..SimParams::default()
    });
    let now = Instant::now();
    let Some(McuMsg::Hello(hello)) = mcu
        .handle(&HostMsg::HelloReq { request_id: 1 }, 1)
        .into_iter()
        .next()
    else {
        panic!("no hello")
    };
    sup.on_link_event(
        LinkEvent::Identified {
            axis: ax(1),
            hello,
            port: "sim:1".into(),
        },
        now,
    );
    sup.on_link_event(
        LinkEvent::Status {
            axis: ax(1),
            status: mcu.status(),
            received_at: now,
        },
        now,
    );
    let out = sup.tick(now, 0.01, None);
    assert!(out.set_position.is_none());
    let [HostMsg::Config(c), HostMsg::SetLimits(_)] = out.admin_msgs[0].as_slice() else {
        panic!("{:?}", out.admin_msgs)
    };
    assert!(out.admin_msgs[1].is_empty() && out.admin_msgs[2].is_empty());
    let ack = Ack {
        request_id: c.request_id,
        acked_type: msg_type::CONFIG,
        result: AckResult::Rejected,
        reason: AckReason::BadState,
    };
    sup.on_link_event(LinkEvent::Ack { axis: ax(1), ack }, now);
    assert!(sup.fault().is_none());
    assert_eq!(sup.state(), RigState::Offline);
    assert!(
        has(
            &sup.blockers(now),
            "not_configured: axis1 CONFIG rejected (BadState)"
        ),
        "{:?}",
        sup.blockers(now)
    );
    // Retried later (at most once per CONFIG_RETRY), never anything else.
    let mut later = now;
    let mut configs = 0;
    for _ in 0..150 {
        later += STEP;
        sup.on_link_event(
            LinkEvent::Status {
                axis: ax(1),
                status: mcu.status(),
                received_at: later,
            },
            later,
        );
        let out = sup.tick(later, 0.01, None);
        for m in &out.admin_msgs[0] {
            assert!(matches!(m, HostMsg::Config(_)), "{m:?}");
            configs += 1;
        }
    }
    assert_eq!(configs, 1);
}

#[test]
fn snapshot_json_shape() {
    let mut r = Rig::new(sim_cfg());
    r.active(POSE_A, 0.3);
    r.run_ms(100);
    let v = serde_json::to_value(r.sup.snapshot(r.now)).unwrap();
    let keys: Vec<&str> = v.as_object().unwrap().keys().map(String::as_str).collect();
    let mut want = vec![
        "protocol",
        "rig_state",
        "armed",
        "active_source",
        "calibration_valid",
        "fault",
        "fault_history",
        "blockers",
        "session",
        "requested_pose",
        "requested_gain",
        "effective_gain",
        "limits",
        "virtual_targets",
        "target_counts",
        "session_epoch",
        "group_frame_id",
        "axes",
        "axis_conflict",
        "measured_angles",
        "physical_estop",
        "telemetry",
        "motion_limits",
    ];
    let mut got = keys.clone();
    got.sort_unstable();
    want.sort_unstable();
    assert_eq!(got, want);
    assert_eq!(v["rig_state"], "ACTIVE");
    assert_eq!(v["active_source"], "game");
    assert_eq!(v["armed"], true);
    assert_eq!(v["physical_estop"], "not_monitored");
    assert!(v["measured_angles"].is_null());
    let axis = &v["axes"][0];
    let mut akeys: Vec<&str> = axis
        .as_object()
        .unwrap()
        .keys()
        .map(String::as_str)
        .collect();
    let mut awant = vec![
        "axis_id",
        "name",
        "connected",
        "port",
        "device_id",
        "expected_device_id",
        "assignment",
        "claimants",
        "reference_stale",
        "calibration_stale",
        "boot_id",
        "mcu_state",
        "fault_code",
        "configured",
        "referenced",
        "calibration_valid",
        "actual_count",
        "target_count",
        "velocity_count_s",
        "following_error_count",
        "pwm_permille",
        "power_on_cmd",
        "brake_release_cmd",
        "power_controller",
        "current_a",
        "last_applied_frame",
        "status_age_ms",
        "rx_error_count",
        "stale_drop_count",
        "decode_errors",
        "calibration",
        "fw_limits",
    ];
    akeys.sort_unstable();
    awant.sort_unstable();
    assert_eq!(akeys, awant);
    assert_eq!(axis["axis_id"], 1);
    assert_eq!(axis["device_id"], "5131000000000001");
    assert_eq!(axis["expected_device_id"], "5131000000000001");
    assert_eq!(axis["assignment"], "selected");
    assert_eq!(axis["claimants"], serde_json::json!([{"port": "sim:1", "device_id": "5131000000000001"}]));
    assert_eq!(axis["reference_stale"], false);
    assert_eq!(axis["calibration_stale"], false);
    assert_eq!(v["axis_conflict"], serde_json::json!([]));
    assert_eq!(axis["mcu_state"], "ACTIVE");
    assert_eq!(axis["fault_code"], "NONE");
    assert!(axis["calibration"].is_null(), "no CAL_RESULT yet");
    assert!(
        axis["current_a"].is_null(),
        "current not valid → null, not 0"
    );
    assert_eq!(axis["power_on_cmd"], true);
    assert_eq!(axis["power_controller"], true, "axis 1 is power_controller_axis in sim.toml");
    assert_eq!(
        v["axes"][1]["power_on_cmd"], false,
        "only the power-controller axis drives G2"
    );
}

#[test]
fn events_are_drained() {
    let mut r = Rig::new(sim_cfg());
    r.ready();
    let (ev, dropped) = r.sup.drain_events();
    assert!(ev.iter().any(|e| e.kind == "identified"));
    assert!(ev.iter().any(|e| e.kind == "rig_state"));
    assert_eq!(dropped, 0);
    assert!(r.sup.drain_events().0.is_empty());
}

// ------------------------------------------------------------------ review fixes (stage B)

const POSE_B: Pose = Pose {
    pitch: -0.4,
    roll: 0.3,
    yaw: -0.2,
};

/// A second client takes over: new session, seq / frame_seq from 1.
fn swap_session(r: &mut Rig) {
    r.session = r.intake.new_session();
    r.pose_seq = 0;
    r.frame_seq = 0;
    r.set_game(Some((POSE_B, 0.3)));
}

#[test]
fn new_session_during_active_stops_the_rig() {
    let mut r = Rig::new(sim_cfg());
    r.active(POSE_A, 0.3);
    r.run_ms(2000); // settled at POSE_A
    let mark = r.sent.len();
    let t_mark = r.now;
    // The new client's first pose is in the same tick as the swap.
    swap_session(&mut r);
    r.step();
    assert_eq!(r.sup.state(), RigState::Stopping);
    r.wait_state(RigState::BrakedDisarmed, 3000);
    assert!(
        r.states.iter().filter(|(t, _)| *t > t_mark).all(|(_, s)| *s != RigState::Active),
        "{:?}",
        r.states
    );
    assert_eq!(r.count(mark, msg_type::START), [0; 3]);
    // It never followed the new client's pose.
    let b = expected_counts(&r.cfg, POSE_B, 0.3);
    let a = expected_counts(&r.cfg, POSE_A, 0.3);
    for i in 0..3 {
        let k = r.axis_of[i].unwrap().index();
        let act = r.mcus[i].actual_count();
        assert!((act - a[k]).abs() < (act - b[k]).abs(), "axis {k}: {act} moved toward B");
    }
    // An explicit ARM + START binds the drive to the new session.
    r.admin(AdminCmd::Arm).unwrap();
    r.wait_state(RigState::Holding, 2000);
    r.admin(AdminCmd::Start).unwrap();
    r.wait_state(RigState::Active, 1000);
    r.run_ms(300);
    assert_eq!(r.sup.state(), RigState::Active);
}

#[test]
fn new_session_while_start_in_flight_never_drives() {
    let mut r = Rig::new(sim_cfg());
    r.holding();
    r.set_game(Some((POSE_A, 0.3)));
    r.run_ms(30);
    r.admin(AdminCmd::Start).unwrap();
    let t_mark = r.now;
    swap_session(&mut r);
    assert!(r.run_until(1000, |r| r.sup.state() == RigState::Stopping));
    assert!(
        r.setpos.iter().filter(|(t, _, _)| *t > t_mark).all(|(_, s, _)| *s != RigState::Active),
        "a SET_POSITION was built in ACTIVE for the new session"
    );
    r.wait_state(RigState::BrakedDisarmed, 3000);
}

/// One supervisor tick by hand: admin messages reach the MCUs, but only
/// their ACKs come back; everything else (STATUS, FAULT_EVENT) is returned.
fn tick_feeding_only_acks(r: &mut Rig) -> Vec<Vec<McuMsg>> {
    r.now += STEP;
    let us = r.us();
    let out = r.sup.tick(r.now, STEP.as_secs_f64(), None);
    let mut held = vec![Vec::new(), Vec::new(), Vec::new()];
    for i in 0..3 {
        let k = r.axis_of[i].unwrap().index();
        for m in &out.admin_msgs[k] {
            r.sent.push((i, r.now, *m));
            let (acks, rest): (Vec<McuMsg>, Vec<McuMsg>) = r.mcus[i]
                .handle(m, us)
                .into_iter()
                .partition(|m| matches!(m, McuMsg::Ack(_)));
            r.feed(i, acks);
            held[i].extend(rest);
        }
    }
    held
}

#[test]
fn fault_reset_waits_for_status_after_host_fault_ack() {
    let mut r = Rig::new(sim_cfg());
    r.ready();
    r.admin(AdminCmd::Fault { note: "x".into() }).unwrap();
    let held = tick_feeding_only_acks(&mut r);
    assert_eq!(r.mcu_states(), vec![McuState::Fault; 3]);
    assert_eq!(r.count(0, msg_type::HOST_FAULT), [1, 1, 1]);
    // ACKed, but the cached STATUS still says DISARMED / NONE.
    let e = r.admin(AdminCmd::FaultReset).unwrap_err();
    assert_eq!(e.error, admin_error::AXIS_NOT_SETTLED, "{e:?}");
    assert!(has(&e.blockers, "no STATUS since HOST_FAULT ACK"), "{e:?}");
    assert_eq!(r.sup.state(), RigState::FaultLatched);
    assert_eq!(
        r.admin(AdminCmd::Arm).unwrap_err().error,
        admin_error::FAULT_LATCHED
    );
    for (i, h) in held.into_iter().enumerate() {
        r.feed(i, h);
    }
    let mark = r.sent.len();
    r.admin(AdminCmd::FaultReset).unwrap();
    r.wait_state(RigState::ReadyToArm, 500);
    assert_eq!(r.count(mark, msg_type::FAULT_RESET), [1, 1, 1]);
    assert_eq!(r.mcu_states(), vec![McuState::Disarmed; 3]);
    r.run_ms(200);
    assert_eq!(r.sup.state(), RigState::ReadyToArm);
    assert_eq!(r.sup.fault_history().count(), 1, "no second fault episode");
}

#[test]
fn fault_reset_clear_needs_fault_reset_acks() {
    let mut r = Rig::new(sim_cfg());
    r.ready();
    r.admin(AdminCmd::Fault { note: "x".into() }).unwrap();
    r.run_ms(50);
    r.admin(AdminCmd::FaultReset).unwrap();
    // FAULT_RESET ACKed, its STATUS held: still latched.
    let held = tick_feeding_only_acks(&mut r);
    assert_eq!(r.mcu_states(), vec![McuState::Disarmed; 3]);
    assert_eq!(r.sup.state(), RigState::FaultLatched);
    for (i, h) in held.into_iter().enumerate() {
        r.feed(i, h);
    }
    r.wait_state(RigState::ReadyToArm, 100);
}

#[test]
fn unacked_host_fault_is_resent_and_blocks_reset() {
    let mut r = Rig::new(sim_cfg());
    r.ready();
    r.mcus[1].set_drop_rx(true);
    r.admin(AdminCmd::Fault { note: "x".into() }).unwrap();
    r.run_ms(50);
    let e = r.admin(AdminCmd::FaultReset).unwrap_err();
    assert!(has(&e.blockers, "axis2 HOST_FAULT not acknowledged"), "{e:?}");
    r.run_ms(1100);
    assert!(r.count(0, msg_type::HOST_FAULT)[1] >= 2, "HOST_FAULT resent after ACK timeout");
    assert_eq!(r.mcus[1].state(), McuState::Disarmed);
    r.mcus[1].set_drop_rx(false);
    assert!(r.run_until(1500, |r| r.mcus[1].state() == McuState::Fault));
    r.run_ms(50);
    r.admin(AdminCmd::FaultReset).unwrap();
    r.wait_state(RigState::ReadyToArm, 500);
    assert_eq!(r.sup.fault_history().count(), 1);
}

#[test]
fn axis_reconnecting_while_latched_gets_host_fault_not_config() {
    let mut r = Rig::new(sim_cfg());
    r.holding();
    r.disconnect(0);
    assert_eq!(r.sup.state(), RigState::FaultLatched);
    r.run_ms(30);
    let mark = r.sent.len();
    // Same boot; the MCU never saw HOST_FAULT and is still HOLDING.
    r.connect(0);
    assert_eq!(r.mcus[0].state(), McuState::Holding);
    r.step();
    assert_eq!(r.count(mark, msg_type::HOST_FAULT), [1, 0, 0]);
    assert_eq!(r.count(mark, msg_type::CONFIG), [0, 0, 0]);
    assert_eq!(r.mcus[0].state(), McuState::Fault);
    assert_eq!(r.mcus[0].fault_code(), FaultCode::HostFault);
    r.run_ms(300);
    let timeout = RigFault::AxisFault {
        axis: ax(1),
        code: FaultCode::CommandTimeout,
    };
    assert!(r.sup.fault_history().all(|f| f.fault != timeout));
    r.admin(AdminCmd::FaultReset).unwrap();
    r.wait_state(RigState::ReadyToArm, 2000);
}

#[test]
fn late_status_on_one_axis_is_not_frame_skew() {
    let cfg = sim_cfg();
    let mut r = Rig::new(cfg.clone());
    r.active(POSE_A, 0.3);
    r.run_ms(400);
    // STATUS gaps below axis_status_timeout_ms (200): the MCU keeps applying.
    for d in [50, 100, 150] {
        r.mcus[2].set_freeze_status(true);
        r.run_ms(d);
        r.mcus[2].set_freeze_status(false);
        r.run_ms(200);
        assert_eq!(
            r.sup.state(),
            RigState::Active,
            "STATUS gap {d} ms: {:?}",
            r.sup.fault()
        );
        assert!(r.sup.frame_skew().is_some_and(|s| s <= cfg.timing.max_frame_skew));
    }
    // Past the STATUS timeout it is a disconnect, not frame skew.
    r.mcus[2].set_freeze_status(true);
    r.wait_state(RigState::FaultLatched, 400);
    let axis3 = r.axis_of[2].unwrap();
    assert!(
        matches!(r.sup.fault(), Some(RigFault::AxisDisconnected { axis }) if *axis == axis3),
        "{:?}",
        r.sup.fault()
    );
}

#[test]
fn admin_refusal_blockers_are_code_prefixed() {
    let mut r = Rig::new(sim_cfg());
    r.ready();
    let e = r.admin(AdminCmd::Center).unwrap_err();
    assert_eq!(e.blockers, vec!["rig_state: READY_TO_ARM".to_string()]);
    r.admin(AdminCmd::Arm).unwrap();
    r.wait_state(RigState::Holding, 2000);
    let e = r.admin(AdminCmd::Start).unwrap_err();
    assert_eq!(e.error, admin_error::NO_FRESH_POSE);
    assert!(e.blockers[0].starts_with("no_fresh_pose: "), "{e:?}");
    let e = r
        .admin(AdminCmd::Reference {
            axis: ax(1),
            count: 0,
        })
        .unwrap_err();
    assert_eq!(e.blockers, vec!["rig_state: HOLDING".to_string()]);
}

#[test]
fn status_age_counts_from_link_receive_time() {
    let mut r = Rig::new(sim_cfg());
    r.ready();
    let axis = r.axis_of[0].unwrap();
    let st = r.mcus[0].status();
    // Waited 60 ms in the event queue before the loop took it.
    r.sup.on_link_event(
        LinkEvent::Status {
            axis,
            status: st,
            received_at: r.now - Duration::from_millis(60),
        },
        r.now,
    );
    let age = r.sup.snapshot(r.now).axes[axis.index()].status_age_ms.unwrap();
    assert!((age - 60.0).abs() < 1e-6, "{age}");
    // A stamp after `now` is not trusted.
    r.sup.on_link_event(
        LinkEvent::Status {
            axis,
            status: st,
            received_at: r.now + Duration::from_secs(1),
        },
        r.now,
    );
    assert_eq!(r.sup.snapshot(r.now).axes[axis.index()].status_age_ms, Some(0.0));
}

// ------------------------------------------------------------------ power role (§10)

#[test]
fn power_role_must_match_config_before_arm() {
    let mut r = Rig::new(sim_cfg());
    let p = r.cfg.power_controller_axis.index();
    let other = (p + 1) % 3;
    // A second axis claims the main-power role (firmware ignored role_flags).
    r.flip_flags[other] = status_flags::POWER_CONTROLLER;
    r.connect_all();
    r.run_ms(500);
    assert_eq!(r.sup.state(), RigState::CalibrationRequired);
    let needle = format!("power_role_mismatch: axis{}", other + 1);
    assert!(has(&r.blockers(), &needle), "{:?}", r.blockers());
    assert_eq!(r.admin(AdminCmd::Arm).unwrap_err().error, admin_error::NOT_READY);
    let snap = r.sup.snapshot(r.now);
    assert_eq!(snap.axes[p].power_controller, Some(true));
    assert_eq!(snap.axes[other].power_controller, Some(true));
    assert!(r.count(0, msg_type::ARM_PREPARE).iter().all(|&c| c == 0));

    // The configured power axis does not report the role.
    r.flip_flags[other] = 0;
    r.flip_flags[p] = status_flags::POWER_CONTROLLER;
    r.run_ms(100);
    assert!(has(&r.blockers(), &format!("power_role_mismatch: axis{}", p + 1)), "{:?}", r.blockers());
    assert!(!has(&r.blockers(), &needle));

    // Correct roles: ready, and exactly one axis reports the role.
    r.flip_flags = [0; 3];
    r.wait_state(RigState::ReadyToArm, 200);
    let snap = r.sup.snapshot(r.now);
    let roles: Vec<_> = snap.axes.iter().map(|a| a.power_controller).collect();
    let mut want = vec![Some(false); 3];
    want[p] = Some(true);
    assert_eq!(roles, want);
    assert!(r.blockers().is_empty(), "{:?}", r.blockers());
}

#[test]
fn power_role_is_not_checked_before_config_is_accepted() {
    // Before CONFIG the flag is legitimately 0 on every axis: report the
    // missing CONFIG, not a role mismatch.
    let mut r = Rig::new(sim_cfg());
    r.connect_all();
    let b = r.blockers();
    assert!(has(&b, "not_configured"), "{b:?}");
    assert!(!has(&b, "power_role_mismatch"), "{b:?}");
}

// ------------------------------------------------------------------ reboot while disarmed (§6)

#[test]
fn reboot_while_ready_invalidates_reference_and_config_without_fault() {
    // start_referenced = false: after the reboot the encoder restarts at 0 and
    // the axis has no reference until SET_REFERENCE is redone.
    let mut r = Rig::with(sim_cfg(), |_, p| p.start_referenced = false);
    r.connect_all();
    r.run_ms(100);
    for axis in 1..=3u8 {
        r.admin(AdminCmd::Reference { axis: ax(axis), count: 0 }).unwrap();
    }
    r.wait_state(RigState::ReadyToArm, 500);
    let old_epoch = r.sup.session_epoch();
    let mark = r.sent.len();

    r.mcus[1].reboot(0xbeef_0001);
    r.run_ms(50);
    assert!(r.sup.fault().is_none(), "a reboot while disarmed is not a fault: {:?}", r.sup.fault());
    assert_ne!(r.sup.state(), RigState::ReadyToArm);
    let b = r.blockers();
    assert!(has(&b, "not_referenced: axis2"), "{b:?}");
    assert!(has(&b, "not_configured: axis2"), "{b:?}");
    let e = r.admin(AdminCmd::Arm).unwrap_err();
    assert_eq!(e.error, admin_error::NOT_READY, "{e:?}");
    assert!(has(&e.blockers, "not_referenced: axis2"), "{e:?}");
    let snap = r.sup.snapshot(r.now);
    assert_eq!(snap.axes[1].boot_id, Some(0xbeef_0001));
    assert!(!snap.axes[1].referenced);
    // The supervisor re-sends CONFIG to the rebooted axis by itself (next
    // CONFIG_RETRY slot, same epoch); the reference stays missing.
    assert!(r.run_until(1500, |r| r.count(mark, msg_type::CONFIG)[1] >= 1));
    r.run_ms(50);
    assert_eq!(r.sup.session_epoch(), old_epoch);
    let b = r.blockers();
    assert!(!has(&b, "not_configured: axis2"), "{b:?}");
    assert!(has(&b, "not_referenced: axis2"), "{b:?}");
    assert_ne!(r.sup.state(), RigState::ReadyToArm);
    assert!(r.count(mark, msg_type::ARM_PREPARE).iter().all(|&c| c == 0));
    assert_eq!(r.mcus[1].state(), McuState::Disarmed);

    // Re-referencing the axis makes the rig ready again; nothing armed meanwhile.
    r.admin(AdminCmd::Reference { axis: ax(2), count: 0 }).unwrap();
    r.wait_state(RigState::ReadyToArm, 500);
    assert!(r.mcu_states().iter().all(|s| *s == McuState::Disarmed));
}

// ------------------------------------------------- axis selection (DESIGN §10.2)

/// HELLO of a fourth unit (not one of the three sims) selecting `axis`.
fn intruder_hello(axis: u8, device_id: DeviceId) -> Hello {
    Hello {
        request_id: 1,
        protocol_version: 1,
        selected_axis: axis,
        mcu_state: McuState::Disarmed as u8,
        device_id,
        boot_id: 9,
        fw_version: [0, 1, 0],
        capabilities: h3cs::protocol::capabilities::POSITION_MODE,
        calibration_hash: 0,
    }
}

/// sim.toml without device_id cross-checks (units may move between axes).
fn no_cross_check_cfg() -> Config {
    let mut cfg = sim_cfg();
    for a in &mut cfg.axes {
        a.device_id = DeviceId(0);
    }
    cfg
}

#[test]
fn config_fallback_path_assigns_by_device_id_when_nothing_is_selected() {
    let mut r = Rig::with(sim_cfg(), |_, p| p.selected_axis = 0);
    r.ready();
    let snap = r.sup.snapshot(r.now);
    for (i, a) in snap.axes.iter().enumerate() {
        assert_eq!(a.assignment, Some("config"), "{a:?}");
        assert_eq!(a.device_id, r.cfg.axes[i].device_id);
        assert_eq!(a.expected_device_id, r.cfg.axes[i].device_id);
        assert_eq!(a.claimants.len(), 1);
        assert!(!a.reference_stale);
    }
    assert!(snap.axis_conflict.is_empty());
}

#[test]
fn selected_axis_wins_and_device_id_mismatch_blocks_arm_both_directions() {
    // Units 1 and 2 are swapped: the unit configured as axis 1 selects axis 2.
    let mut r = Rig::with(sim_cfg(), |i, p| {
        p.selected_axis = match i {
            0 => 2,
            1 => 1,
            _ => 3,
        }
    });
    r.connect_all();
    r.run_ms(200);
    // Assigned by selection, links kept (the operator can fix it by reselecting).
    assert_eq!(r.sup.axis_port(ax(2)), Some("sim:1"));
    assert_eq!(r.sup.axis_port(ax(1)), Some("sim:2"));
    assert_eq!(r.sup.state(), RigState::CalibrationRequired);
    let e = r.admin(AdminCmd::Arm).unwrap_err();
    assert_eq!(e.error, admin_error::NOT_READY);
    for needle in [
        "axis_device_mismatch: axis2 device 5131000000000001, config expects 5131000000000002",
        "axis_device_mismatch: device 5131000000000001 is configured as axis1 but selected axis2",
        "axis_device_mismatch: axis1 device 5131000000000002, config expects 5131000000000001",
        "axis_device_mismatch: device 5131000000000002 is configured as axis2 but selected axis1",
    ] {
        assert!(has(&e.blockers, needle), "{needle} not in {:?}", e.blockers);
    }
    assert!(!has(&e.blockers, "unknown_device"), "{:?}", e.blockers);
    assert!(!has(&e.blockers, "axis_conflict"), "{:?}", e.blockers);
    let snap = r.sup.snapshot(r.now);
    assert_eq!(snap.axes[1].device_id, DeviceId(0x5131_0000_0000_0001), "connected unit, not the config");
    assert_eq!(snap.axes[1].expected_device_id, DeviceId(0x5131_0000_0000_0002));
    assert_eq!(snap.axes[1].assignment, Some("selected"));
    // CONFIG went to both (identification is not blocked), ARM never.
    assert_eq!(r.count(0, msg_type::ARM_PREPARE), [0; 3]);
    assert!(r.count(0, msg_type::CONFIG).iter().all(|&n| n >= 1));

    // The same units with device_id = 0 (no cross-check) are fine.
    let mut r2 = Rig::with(no_cross_check_cfg(), |i, p| {
        p.selected_axis = match i {
            0 => 2,
            1 => 1,
            _ => 3,
        }
    });
    r2.ready();
    assert!(r2.blockers().is_empty());
}

#[test]
fn idle_unit_switching_axis_is_reassigned_configured_and_must_be_re_referenced() {
    let mut r = Rig::new(no_cross_check_cfg());
    r.ready();
    let cal = calibration::from_config(&r.cfg);
    let hashes: Vec<u32> = (0..3).map(|i| calibration::calibration_hash(&r.cfg.rig_id, &cal[i])).collect();
    let hash = |i: usize| hashes[i];
    let sent = r.sent.len();

    // Unit 0 (axis 1) selects axis 3: axis 1 empty, axis 3 in conflict.
    r.select(0, 3);
    r.run_ms(20);
    assert_eq!(r.sup.state(), RigState::Offline);
    assert_eq!(r.sup.axis_conflict(), vec![ax(3)]);
    assert_eq!(r.sup.axis_port(ax(1)), None);
    assert_eq!(r.sup.axis_port(ax(3)), None);
    assert!(r.sup.fault().is_none(), "idle switch is not a fault");
    // Unit 2 (axis 3) selects axis 1: both resolve.
    r.select(2, 1);
    r.run_ms(20);
    assert!(r.sup.axis_conflict().is_empty());
    assert_eq!(r.sup.axis_port(ax(3)), Some("sim:1"));
    assert_eq!(r.sup.axis_port(ax(1)), Some("sim:3"));
    // Each unit got the CONFIG of its new axis (its own calibration_hash).
    let cfg_hash = |i: usize| -> Vec<u32> {
        r.sent_since(sent)
            .iter()
            .filter_map(|(k, _, m)| match m {
                HostMsg::Config(c) if *k == i => Some(c.calibration_hash),
                _ => None,
            })
            .collect()
    };
    assert_eq!(cfg_hash(0), vec![hash(2)], "unit 0 now drives axis 3");
    assert_eq!(cfg_hash(2), vec![hash(0)], "unit 2 now drives axis 1");
    assert_eq!(r.count(sent, msg_type::ARM_PREPARE), [0; 3]);
    r.run_ms(100);
    assert!(r.mcus[0].is_configured() && r.mcus[2].is_configured());

    // Same boot, different axis: stale until this server re-references.
    assert_eq!(r.sup.state(), RigState::CalibrationRequired);
    let e = r.admin(AdminCmd::Arm).unwrap_err();
    assert_eq!(e.error, admin_error::NOT_READY);
    assert!(has(&e.blockers, "reference_stale: axis1 re-reference required after axis change"), "{:?}", e.blockers);
    assert!(has(&e.blockers, "reference_stale: axis3 re-reference required after axis change"), "{:?}", e.blockers);
    assert!(has(&e.blockers, "not_referenced: axis1"), "sim cleared the flag: {:?}", e.blockers);
    assert!(!has(&e.blockers, "reference_stale: axis2"), "{:?}", e.blockers);
    let snap = r.sup.snapshot(r.now);
    assert!(snap.axes[0].reference_stale && snap.axes[2].reference_stale && !snap.axes[1].reference_stale);
    assert_eq!(snap.axes[2].assignment, Some("selected"));

    r.admin(AdminCmd::Reference { axis: ax(1), count: 0 }).unwrap();
    r.run_ms(50);
    let b = r.blockers();
    assert!(!has(&b, "reference_stale: axis1"), "{b:?}");
    assert!(has(&b, "reference_stale: axis3"), "{b:?}");
    r.admin(AdminCmd::Reference { axis: ax(3), count: 0 }).unwrap();
    r.wait_state(RigState::ReadyToArm, 500);
    assert!(r.blockers().is_empty());
    // Switching back and forth never armed or moved anything.
    assert_eq!(r.count(0, msg_type::ARM_PREPARE), [0; 3]);
    assert!(r.setpos.is_empty());
}

#[test]
fn reboot_clears_reference_stale() {
    let mut r = Rig::new(no_cross_check_cfg());
    r.ready();
    r.select(0, 3);
    r.select(2, 1);
    r.run_ms(50);
    assert!(has(&r.blockers(), "reference_stale: axis3"));
    // A new boot has a fresh reference (established by the sim at boot here).
    r.mcus[0].reboot(555);
    r.connect(0);
    r.run_ms(50);
    let b = r.blockers();
    assert!(!has(&b, "reference_stale: axis3"), "{b:?}");
    assert!(has(&b, "reference_stale: axis1"), "the other unit is still stale: {b:?}");
}

#[test]
fn second_claimant_while_driving_stops_the_rig_then_conflicts() {
    let mut r = Rig::new(no_cross_check_cfg());
    r.active(POSE_A, 0.3);
    r.run_ms(100);
    let hello = intruder_hello(2, DeviceId(0xAAAA));
    let sent = r.sent.len();
    r.sup.on_link_event(
        LinkEvent::Identified {
            axis: ax(2),
            hello,
            port: "sim:4".into(),
        },
        r.now,
    );
    // Primary kept, rig stops (no fault), conflict visible already.
    assert_eq!(r.sup.state(), RigState::Stopping);
    assert_eq!(r.sup.axis_port(ax(2)), Some("sim:2"));
    assert_eq!(r.sup.axis_conflict(), vec![ax(2)]);
    assert!(r.sup.fault().is_none());
    assert_eq!(r.admin(AdminCmd::Start).unwrap_err().error, admin_error::AXIS_CONFLICT);
    assert_eq!(r.admin(AdminCmd::Arm).unwrap_err().error, admin_error::AXIS_CONFLICT);
    // Controlled stop → hold → DISARM; the moment the rig is idle the
    // conflict takes effect (no primary → OFFLINE, no BRAKED_DISARMED tick).
    r.wait_state(RigState::Offline, 3000);
    assert_eq!(r.mcu_states(), vec![McuState::Disarmed; 3]);
    let seen: Vec<RigState> = r.states.iter().map(|(_, s)| *s).collect();
    assert!(seen.contains(&RigState::Stopping), "{seen:?}");
    assert!(!seen.contains(&RigState::FaultLatched), "{seen:?}");
    assert_eq!(r.sup.state(), RigState::Offline);
    assert_eq!(r.sup.axis_port(ax(2)), None);
    assert!(has(&r.blockers(), "axis_conflict: axis2 claimed by sim:2(5131000000000002), sim:4(000000000000aaaa)"), "{:?}", r.blockers());
    assert!(r.sup.fault().is_none());
    let configs_during_conflict = r.count(sent, msg_type::CONFIG)[1];
    // The intruder leaves: the real unit is promoted (CONFIG), never armed.
    r.sup.on_link_event(LinkEvent::PortClosed { port: "sim:4".into() }, r.now);
    assert!(r.sup.axis_conflict().is_empty());
    assert_eq!(r.sup.axis_port(ax(2)), Some("sim:2"));
    r.wait_state(RigState::ReadyToArm, 1000);
    assert_eq!(r.count(sent, msg_type::CONFIG)[1], configs_during_conflict + 1);
    assert!(!r.sup.is_armed());
    assert_eq!(r.count(sent, msg_type::ARM_PREPARE), [0; 3]);
}

#[test]
fn second_claimant_while_arming_disarms() {
    let mut r = Rig::new(no_cross_check_cfg());
    r.ready();
    r.admin(AdminCmd::Arm).unwrap();
    assert_eq!(r.sup.state(), RigState::Arming);
    let h = intruder_hello(1, DeviceId(0xBBBB));
    r.sup.on_link_event(LinkEvent::Identified { axis: ax(1), hello: h, port: "sim:5".into() }, r.now);
    assert_eq!(r.sup.state(), RigState::Stopping);
    r.wait_state(RigState::Offline, 3000);
    assert!(r.sup.fault().is_none());
    assert_eq!(r.sup.axis_conflict(), vec![ax(1)]);
}

#[test]
fn primary_changing_axis_while_driving_latches_axis_changed() {
    let mut r = Rig::new(no_cross_check_cfg());
    r.active(POSE_A, 0.3);
    r.run_ms(100);
    let sent = r.sent.len();
    // Unit 0 (axis 1) switches to axis 2 while ACTIVE (rule violation).
    r.select_unchecked(0, 2);
    assert_eq!(r.sup.state(), RigState::FaultLatched);
    assert_eq!(r.sup.fault(), Some(&RigFault::AxisChanged { axis: ax(1) }));
    assert_eq!(r.sup.axis_conflict(), vec![ax(2)], "the unit now also claims axis 2");
    assert_eq!(r.sup.axis_port(ax(1)), None);
    // The axis-2 unit was being driven: it stays primary until its HOST_FAULT
    // is ACKed (a demotion in the same tick would clear the queued message).
    assert_eq!(r.sup.axis_port(ax(2)), Some("sim:2"), "demotion deferred to the HOST_FAULT ACK");
    r.run_ms(50);
    // HOST_FAULT reached the untouched axis and the demoted axis-2 unit
    // (which must stop by HOST_FAULT, not by its own lease).
    let hf = r.count(sent, msg_type::HOST_FAULT);
    assert!(hf[2] >= 1, "{hf:?}");
    assert_eq!(hf[1], 1, "{hf:?}");
    assert_eq!(hf[0], 0, "nothing is routed to the switching unit (claimant of a conflicted axis): {hf:?}");
    assert_eq!(r.mcus[2].state(), McuState::Fault);
    assert_eq!(r.mcus[1].state(), McuState::Fault);
    assert_eq!(r.mcus[1].fault_code(), FaultCode::HostFault);
    // ACKed: now the conflict demotes it.
    assert_eq!(r.sup.axis_port(ax(2)), None);
    assert_eq!(r.sup.axis_conflict(), vec![ax(2)]);
    // Reset is refused until the conflict is resolved and every axis is back.
    let e = r.admin(AdminCmd::FaultReset).unwrap_err();
    assert_eq!(e.error, admin_error::AXIS_DISCONNECTED);
    assert!(has(&e.blockers, "not_connected: axis1"), "{:?}", e.blockers);
    assert!(has(&e.blockers, "not_connected: axis2"), "{:?}", e.blockers);

    // Operator switches it back; the remaining unit of axis 2 is promoted and
    // gets HOST_FAULT (latched), so does the returning unit.
    r.select_unchecked(0, 1);
    r.run_ms(300);
    assert!(r.sup.axis_conflict().is_empty());
    assert_eq!(r.sup.axis_port(ax(1)), Some("sim:1"));
    assert_eq!(r.sup.axis_port(ax(2)), Some("sim:2"));
    assert!(r.mcu_states().iter().all(|s| *s == McuState::Fault), "{:?}", r.mcu_states());
    assert_eq!(r.sup.state(), RigState::FaultLatched);
    let mut ok = false;
    for _ in 0..200 {
        if r.admin(AdminCmd::FaultReset).is_ok() {
            ok = true;
            break;
        }
        r.step();
    }
    assert!(ok, "fault reset accepted once every axis settled: {:?}", r.blockers());
    r.run_until(2000, |r| r.sup.state() != RigState::FaultLatched);
    assert_ne!(r.sup.state(), RigState::FaultLatched);
    assert!(!r.sup.is_armed(), "no automatic re-ARM");
    assert!(has(&r.blockers(), "reference_stale: axis1"), "{:?}", r.blockers());
}

#[test]
fn conflicted_primary_that_never_acks_host_fault_is_demoted_after_ack_timeout() {
    // Same as above, but the axis-2 unit is dead (no STATUS, no ACK): the
    // deferral is bounded by one ACK_TIMEOUT and HOST_FAULT is not re-sent to
    // a conflicted axis.
    let mut r = Rig::new(no_cross_check_cfg());
    r.active(POSE_A, 0.3);
    r.run_ms(100);
    let sent = r.sent.len();
    r.linked[1] = false;
    r.select_unchecked(0, 2);
    assert_eq!(r.sup.state(), RigState::FaultLatched);
    assert_eq!(r.sup.fault(), Some(&RigFault::AxisChanged { axis: ax(1) }));
    r.run_ms(500);
    assert_eq!(r.sup.axis_port(ax(2)), Some("sim:2"), "still waiting for the ACK");
    assert_eq!(r.count(sent, msg_type::HOST_FAULT)[1], 1);
    r.run_ms(h3cs::supervisor::ACK_TIMEOUT.as_millis() as u64);
    assert_eq!(r.sup.axis_port(ax(2)), None, "demoted once the HOST_FAULT timed out");
    assert_eq!(r.count(sent, msg_type::HOST_FAULT)[1], 1, "no re-send to a conflicted axis");
    assert_eq!(r.sup.axis_conflict(), vec![ax(2)]);
    // The healthy axis got its HOST_FAULT as before.
    assert!(r.count(sent, msg_type::HOST_FAULT)[2] >= 1);
    assert_eq!(r.mcus[2].state(), McuState::Fault);
    assert_eq!(r.sup.state(), RigState::FaultLatched);
}

#[test]
fn selected_axis_change_to_none_closes_the_unit_as_unknown() {
    // All device_ids 0: a unit that deselects has nothing to fall back to.
    let mut r = Rig::new(no_cross_check_cfg());
    r.ready();
    r.select(1, 0);
    r.run_ms(20);
    assert_eq!(r.sup.state(), RigState::Offline);
    assert_eq!(r.sup.axis_port(ax(2)), None);
    assert!(has(&r.blockers(), "unknown_device: device 5131000000000002 on port sim:2"), "{:?}", r.blockers());
    assert!(r.sup.fault().is_none());
}

// ---------- end-stop calibration (DESIGN.md §11) ----------

/// Sims with ±500 end stops (stroke 1000: the calibrated rest position 500 lies
/// inside every soft window of sim.toml) and no reference at boot.
fn cal_rig(cfg: Config) -> Rig {
    Rig::with(cfg, |_, p| {
        p.start_referenced = false;
        p.end_min_count = -500;
        p.end_max_count = 500;
    })
}

fn all_calibrating(r: &Rig) -> bool {
    r.sup.calibrating_axes().len() == 3
}

fn all_disarmed(r: &Rig) -> bool {
    r.mcu_states().iter().all(|s| *s == McuState::Disarmed)
}

#[test]
fn calibrate_all_runs_to_cal_result_references_and_blocks_arm_meanwhile() {
    let mut r = cal_rig(sim_cfg());
    r.connect_all();
    r.run_ms(100);
    assert_eq!(r.sup.state(), RigState::CalibrationRequired);
    assert!(has(&r.blockers(), "not_referenced: axis1"));
    let sent = r.sent.len();
    r.admin(AdminCmd::Calibrate { axes: vec![] }).unwrap();
    r.step();
    assert_eq!(r.count(sent, msg_type::CALIBRATE), [1, 1, 1]);
    for (_, _, m) in r.sent_since(sent) {
        if let HostMsg::Calibrate { mode, .. } = m {
            assert_eq!(*mode, 1);
        }
    }
    assert!(r.run_until(200, all_calibrating), "{:?}", r.mcu_states());
    // Rig stays idle and refuses everything that would drive.
    assert_eq!(r.sup.state(), RigState::CalibrationRequired);
    let b = r.blockers();
    assert!(has(&b, "calibrating: axis1") && has(&b, "calibrating: axis2") && has(&b, "calibrating: axis3"), "{b:?}");
    assert!(!has(&b, "mcu_not_disarmed"), "{b:?}");
    let e = r.admin(AdminCmd::Arm).unwrap_err();
    assert_eq!(e.error, admin_error::CALIBRATING);
    assert!(has(&e.blockers, "calibrating: axis1"), "{:?}", e.blockers);
    assert_eq!(r.admin(AdminCmd::Start).unwrap_err().error, admin_error::CALIBRATING);
    assert_eq!(r.admin(AdminCmd::Center).unwrap_err().error, admin_error::CALIBRATING);
    assert_eq!(r.admin(AdminCmd::Reference { axis: ax(2), count: 0 }).unwrap_err().error, admin_error::CALIBRATING);
    assert_eq!(r.admin(AdminCmd::Calibrate { axes: vec![] }).unwrap_err().error, admin_error::CALIBRATING);
    let snap = r.sup.snapshot(r.now);
    assert_eq!(snap.axes[0].mcu_state, Some(McuState::Calibrating));
    assert!(snap.axes.iter().all(|a| a.calibration.is_none()));

    assert!(r.run_until(3000, all_disarmed), "{:?}", r.mcu_states());
    r.run_ms(50);
    // No SET_POSITION, no CONFIG, no ARM_PREPARE while calibrating.
    assert!(r.setpos.is_empty());
    assert_eq!(r.count(sent, msg_type::CONFIG), [0; 3]);
    assert_eq!(r.count(sent, msg_type::ARM_PREPARE), [0; 3]);
    assert!(r.sup.fault().is_none());
    let snap = r.sup.snapshot(r.now);
    for a in &snap.axes {
        let c = a.calibration.as_ref().expect("result recorded");
        assert_eq!(c.result, h3cs::protocol::CalResultCode::Ok);
        assert_eq!((c.step, c.n_end_count, c.neutral_count, c.sign), (3, 1000, 500, 1));
        assert_eq!(c.reason, "ok");
        assert_eq!(Some(c.boot_id), a.boot_id);
        assert!(a.referenced, "the MCU referenced itself");
        assert_eq!(a.actual_count, Some(500));
    }
    assert_eq!(r.count(sent, msg_type::SET_REFERENCE), [0; 3], "no SET_REFERENCE from the PC");
    // Referenced at the neutral and inside the soft window, but the geometry
    // this server was started with was approved for the OLD origin (Issue
    // #15 H3): ARM stays refused until a restart with the exported config.
    r.run_ms(500);
    assert_eq!(r.sup.state(), RigState::CalibrationRequired);
    let b = r.blockers();
    assert_eq!(b.len(), 3, "{b:?}");
    for n in 1..=3 {
        assert!(
            has(&b, &format!("calibration_stale: axis{n} recalibrated (stroke 1000, neutral 500); export and reload config before ARM")),
            "{b:?}"
        );
    }
    let e = r.admin(AdminCmd::Arm).unwrap_err();
    assert_eq!(e.error, admin_error::NOT_READY);
    assert!(has(&e.blockers, "calibration_stale: axis1"), "{:?}", e.blockers);
    let snap = r.sup.snapshot(r.now);
    assert!(snap.axes.iter().all(|a| a.calibration_stale && !a.calibration_valid));
    assert!(!snap.calibration_valid);
    let (events, _) = r.sup.drain_events();
    assert_eq!(events.iter().filter(|e| e.kind == "cal_result").count(), 3);
    assert_eq!(events.iter().filter(|e| e.kind == "calibrate").count(), 1);
    let stale: Vec<_> = events.iter().filter(|e| e.kind == "calibration_stale").collect();
    assert_eq!(stale.len(), 3, "{events:?}");
    assert_eq!(stale[0].detail["stroke"], 1000);
    assert_eq!(stale[0].detail["neutral"], 500);

    // The mark survives a reconnect and a reboot of the unit (the operator
    // must not be able to clear it by power-cycling and re-referencing).
    r.disconnect(1);
    r.run_ms(20);
    r.connect(1);
    r.run_ms(50);
    assert!(has(&r.blockers(), "calibration_stale: axis2"), "{:?}", r.blockers());
    r.mcus[1].reboot(4242);
    r.connect(1);
    r.run_ms(50);
    assert!(has(&r.blockers(), "calibration_stale: axis2"), "{:?}", r.blockers());
    assert!(r.sup.snapshot(r.now).axes[1].calibration.is_none(), "the CAL_RESULT itself is gone with the boot");

    // A new server (restart with the config): the mark is in memory only.
    // Sims 1 and 3 keep their reference; sim 2 rebooted and has none, so
    // only `not_referenced: axis2` is left (hardware-bringup.md §5.0 step 6).
    r.sup = Supervisor::new(&r.cfg);
    r.connect_all();
    r.run_ms(200);
    let b = r.blockers();
    assert!(!has(&b, "calibration_stale"), "{b:?}");
    assert!(b.iter().all(|x| x.starts_with("not_referenced: axis2")), "{b:?}");
    assert!(r.sup.snapshot(r.now).axes.iter().all(|a| !a.calibration_stale));
}

#[test]
fn calibrate_rules_power_axis_connected_disarmed_and_not_armed() {
    let mut r = cal_rig(sim_cfg());
    r.connect_all();
    r.run_ms(100);
    let sent = r.sent.len();
    // Without the power axis (axis 1 in sim.toml): refused before anything is sent.
    let e = r.admin(AdminCmd::Calibrate { axes: vec![ax(2), ax(3)] }).unwrap_err();
    assert_eq!(e.error, admin_error::POWER_AXIS_REQUIRED);
    assert!(has(&e.blockers, "power_axis_required: axis1"), "{:?}", e.blockers);
    // A disconnected target.
    r.disconnect(1);
    r.run_ms(20);
    let e = r.admin(AdminCmd::Calibrate { axes: vec![ax(1), ax(2)] }).unwrap_err();
    assert_eq!(e.error, admin_error::AXIS_DISCONNECTED);
    assert_eq!(e.blockers, vec!["not_connected: axis2".to_string()]);
    // Axis 1 alone is fine (it has the power).
    r.admin(AdminCmd::Calibrate { axes: vec![ax(1)] }).unwrap();
    r.step();
    assert_eq!(r.count(sent, msg_type::CALIBRATE), [1, 0, 0]);
    assert!(r.run_until(3000, |r| r.mcus[0].state() == McuState::Disarmed && r.sup.snapshot(r.now).axes[0].calibration.is_some()));
    r.connect(1);
    r.run_ms(50);
    // A target that is not DISARMED (driven behind the supervisor's back).
    let us = r.us();
    let _ = r.mcus[2].handle(&HostMsg::SetReference { request_id: 9000, reference_count: 0 }, us);
    let out = r.mcus[2].handle(&HostMsg::ArmPrepare { request_id: 9001, session_epoch: r.sup.session_epoch() }, us);
    r.feed(2, out);
    r.run_ms(20);
    let e = r.admin(AdminCmd::Calibrate { axes: vec![ax(1), ax(3)] }).unwrap_err();
    assert_eq!(e.error, admin_error::MCU_NOT_DISARMED);
    assert!(e.blockers.iter().any(|b| b.starts_with("mcu_not_disarmed: axis3")), "{:?}", e.blockers);
    assert!(r.sup.fault().is_none(), "an idle rig does not fault on a stray MCU state");

    // Armed rig: not_disarmed.
    let mut r = Rig::new(sim_cfg());
    r.holding();
    let e = r.admin(AdminCmd::Calibrate { axes: vec![] }).unwrap_err();
    assert_eq!(e.error, admin_error::NOT_DISARMED);
    assert_eq!(e.blockers, vec!["rig_state: HOLDING".to_string()]);
    // Latched rig: fault_latched (safer than DESIGN §11's not_disarmed wording).
    r.admin(AdminCmd::Fault { note: "test".into() }).unwrap();
    r.run_ms(50);
    assert_eq!(r.admin(AdminCmd::Calibrate { axes: vec![] }).unwrap_err().error, admin_error::FAULT_LATCHED);
    // No CALIBRATE ever left in those refusals.
    assert_eq!(r.count(0, msg_type::CALIBRATE), [0; 3]);
}

#[test]
fn calibrate_multi_axis_keeps_power_on_power_axis_then_releases_it_after_all_results() {
    let mut r = cal_rig(sim_cfg());
    r.connect_all();
    r.run_ms(100);
    let sent = r.sent.len();
    r.admin(AdminCmd::Calibrate { axes: vec![] }).unwrap();
    r.step();
    let flags: Vec<(usize, u8)> = r
        .sent_since(sent)
        .iter()
        .filter_map(|(i, _, m)| match m {
            HostMsg::Calibrate { mode: 1, flags, .. } => Some((*i, *flags)),
            _ => None,
        })
        .collect();
    assert_eq!(flags.len(), 3, "{flags:?}");
    for (i, f) in flags {
        assert_eq!(f, if i == 0 { h3cs::protocol::calibrate_flags::KEEP_POWER } else { 0 }, "axis index {i}");
    }
    // No release while any axis is still calibrating.
    assert!(r.run_until(300, all_calibrating));
    let mid = r.sent.len();
    r.run_ms(50);
    assert!(!r.sent_since(mid).iter().any(|(_, _, m)| matches!(m, HostMsg::Calibrate { mode: 0, .. })));
    // After every CAL_RESULT: exactly one CALIBRATE(0), to the power axis.
    assert!(r.run_until(120_000, all_disarmed));
    let rel: Vec<usize> = r
        .sent_since(mid)
        .iter()
        .filter(|(_, _, m)| matches!(m, HostMsg::Calibrate { mode: 0, .. }))
        .map(|(i, _, _)| *i)
        .collect();
    assert_eq!(rel, vec![0]);
}

#[test]
fn calibrate_single_axis_does_not_request_keep_power() {
    let mut r = cal_rig(sim_cfg());
    r.connect_all();
    r.run_ms(100);
    let sent = r.sent.len();
    r.admin(AdminCmd::Calibrate { axes: vec![ax(1)] }).unwrap();
    r.step();
    assert!(r
        .sent_since(sent)
        .iter()
        .all(|(_, _, m)| !matches!(m, HostMsg::Calibrate { flags, .. } if *flags != 0)));
}

#[test]
fn calibrate_cancel_and_stop_send_calibrate_0_and_result_is_cancelled() {
    let mut r = cal_rig(sim_cfg());
    r.connect_all();
    r.run_ms(100);
    r.admin(AdminCmd::Calibrate { axes: vec![] }).unwrap();
    assert!(r.run_until(300, all_calibrating));
    r.run_ms(100);
    let sent = r.sent.len();
    r.admin(AdminCmd::CalibrateCancel).unwrap();
    r.step();
    let cancels: Vec<usize> = r
        .sent_since(sent)
        .iter()
        .filter(|(_, _, m)| matches!(m, HostMsg::Calibrate { mode: 0, .. }))
        .map(|(i, _, _)| *i)
        .collect();
    assert_eq!(cancels.len(), 3, "{:?}", r.sent_since(sent));
    assert!(r.run_until(1000, all_disarmed));
    r.run_ms(50);
    let snap = r.sup.snapshot(r.now);
    for a in &snap.axes {
        let c = a.calibration.as_ref().unwrap();
        assert_eq!(c.result, h3cs::protocol::CalResultCode::Cancelled);
        assert_eq!(c.reason, "stop");
        assert!(!a.referenced, "a cancelled calibration leaves the unit unreferenced");
    }
    assert_eq!(r.sup.state(), RigState::CalibrationRequired);
    assert!(has(&r.blockers(), "not_referenced: axis2"));
    assert!(!has(&r.blockers(), "calibrating"));
    assert!(r.sup.fault().is_none());
    // Cancel with nothing running is a no-op.
    let ok = r.sup.admin(AdminCmd::CalibrateCancel, r.now).unwrap();
    assert_eq!(ok.note.as_deref(), Some("no axis calibrating"));

    // Stop also cancels.
    r.admin(AdminCmd::Calibrate { axes: vec![] }).unwrap();
    assert!(r.run_until(300, all_calibrating));
    let sent = r.sent.len();
    let ok = r.sup.admin(AdminCmd::Stop, r.now).unwrap();
    assert!(ok.note.as_deref().is_some_and(|n| n.starts_with("calibration cancelled")), "{ok:?}");
    r.step();
    assert_eq!(r.count(sent, msg_type::CALIBRATE), [1, 1, 1]);
    assert!(r.run_until(1000, all_disarmed));
    // Stop with nothing running keeps its old note.
    let ok = r.sup.admin(AdminCmd::Stop, r.now).unwrap();
    assert_eq!(ok.note.as_deref(), Some("not driving"));
}

#[test]
fn calibrate_rejected_by_the_power_axis_cancels_the_others_without_fault() {
    let mut r = cal_rig(sim_cfg());
    r.connect_all();
    r.run_ms(100);
    // Sim 0 (axis 1, the power axis) swallows the CALIBRATE; the test answers
    // REJECTED/BAD_STATE in its place.
    r.mcus[0].set_drop_rx(true);
    let sent = r.sent.len();
    r.admin(AdminCmd::Calibrate { axes: vec![] }).unwrap();
    r.step();
    let rid = r
        .sent_since(sent)
        .iter()
        .find_map(|(i, _, m)| match m {
            HostMsg::Calibrate { request_id, mode: 1, .. } if *i == 0 => Some(*request_id),
            _ => None,
        })
        .expect("CALIBRATE to axis 1");
    r.pre_tick.push(LinkEvent::Ack {
        axis: ax(1),
        ack: Ack { request_id: rid, acked_type: msg_type::CALIBRATE, result: AckResult::Rejected, reason: AckReason::BadState },
    });
    let sent = r.sent.len();
    r.step();
    r.step();
    let cancelled: Vec<usize> = r
        .sent_since(sent)
        .iter()
        .filter(|(_, _, m)| matches!(m, HostMsg::Calibrate { mode: 0, .. }))
        .map(|(i, _, _)| *i)
        .collect();
    assert_eq!(cancelled, vec![1, 2], "the other two targets are cancelled");
    assert!(r.sup.fault().is_none(), "a rejected CALIBRATE is not a rig fault");
    assert!(has(&r.blockers(), "calibrate_rejected: axis1 CALIBRATE rejected (BadState)"), "{:?}", r.blockers());
    assert_eq!(r.sup.state(), RigState::CalibrationRequired);
    r.mcus[0].set_drop_rx(false);
    assert!(r.run_until(1000, all_disarmed));
    r.run_ms(20);
    let snap = r.sup.snapshot(r.now);
    assert!(snap.axes[0].calibration.is_none());
    assert_eq!(snap.axes[1].calibration.as_ref().unwrap().result, h3cs::protocol::CalResultCode::Cancelled);
    let (events, _) = r.sup.drain_events();
    assert!(events.iter().any(|e| e.kind == "ack_rejected"));
    assert!(events.iter().any(|e| e.kind == "calibrate_cancelled"));
    // The blocker clears with the next calibrate command (a cancel will do).
    r.admin(AdminCmd::CalibrateCancel).unwrap();
    assert!(!has(&r.blockers(), "calibrate_rejected"), "{:?}", r.blockers());
}

#[test]
fn failed_cal_result_is_recorded_and_the_unit_stays_unreferenced() {
    let mut r = cal_rig(sim_cfg());
    r.connect_all();
    r.run_ms(100);
    r.mcus[1].set_calibration_failure(Some("slow"));
    r.admin(AdminCmd::Calibrate { axes: vec![] }).unwrap();
    assert!(r.run_until(300, all_calibrating));
    assert!(r.run_until(3000, all_disarmed));
    r.run_ms(50);
    let snap = r.sup.snapshot(r.now);
    let c = snap.axes[1].calibration.as_ref().unwrap();
    assert_eq!((c.result, c.step, c.reason.as_str()), (h3cs::protocol::CalResultCode::Failed, 1, "slow"));
    assert!(!snap.axes[1].referenced);
    assert!(snap.axes[0].referenced && snap.axes[2].referenced);
    assert_eq!(snap.axes[0].calibration.as_ref().unwrap().result, h3cs::protocol::CalResultCode::Ok);
    assert!(r.sup.fault().is_none());
    assert_eq!(r.sup.state(), RigState::CalibrationRequired);
    let b = r.blockers();
    assert!(has(&b, "not_referenced: axis2") && !has(&b, "not_referenced: axis1"), "{b:?}");
}

#[test]
fn calibration_result_survives_a_reconnect_but_not_a_reboot() {
    let mut r = cal_rig(sim_cfg());
    r.connect_all();
    r.run_ms(100);
    r.admin(AdminCmd::Calibrate { axes: vec![] }).unwrap();
    assert!(r.run_until(3000, |r| r.sup.snapshot(r.now).axes.iter().all(|a| a.calibration.is_some())));
    // Same boot, new link.
    r.disconnect(2);
    r.run_ms(20);
    r.connect(2);
    r.run_ms(50);
    assert!(r.sup.snapshot(r.now).axes[2].calibration.is_some(), "latest of this boot");
    // New boot: gone (the MCU lost it too).
    r.mcus[2].reboot(4242);
    r.connect(2);
    r.run_ms(50);
    assert!(r.sup.snapshot(r.now).axes[2].calibration.is_none());
    assert!(r.sup.snapshot(r.now).axes[0].calibration.is_some());
}

#[test]
fn status_silence_of_a_calibrating_axis_is_not_a_fault() {
    let mut r = cal_rig(sim_cfg());
    r.connect_all();
    r.run_ms(100);
    r.admin(AdminCmd::Calibrate { axes: vec![] }).unwrap();
    assert!(r.run_until(300, all_calibrating));
    // Sim 1's STATUS stops reaching the PC for a while (link keeps ticking the model).
    r.mcus[1].set_freeze_status(true);
    r.run_ms(400);
    assert!(r.sup.fault().is_none());
    assert_eq!(r.sup.state(), RigState::CalibrationRequired);
    assert!(has(&r.blockers(), "status_stale: axis2"), "{:?}", r.blockers());
    r.mcus[1].set_freeze_status(false);
    assert!(r.run_until(3000, all_disarmed));
    r.run_ms(50);
    assert!(r.sup.fault().is_none());
    // Axis 2's CAL_RESULT may have been lost in the frozen window; the others are recorded.
    assert!(r.sup.snapshot(r.now).axes[0].calibration.is_some());
}

#[test]
fn successful_calibration_clears_reference_stale() {
    let mut r = cal_rig(no_cross_check_cfg());
    r.connect_all();
    r.run_ms(100);
    // Swap the units of axes 1 and 3 under one boot: both stale.
    r.select(0, 3);
    r.select(2, 1);
    r.run_ms(50);
    assert!(has(&r.blockers(), "reference_stale: axis1") && has(&r.blockers(), "reference_stale: axis3"), "{:?}", r.blockers());
    r.admin(AdminCmd::Calibrate { axes: vec![] }).unwrap();
    assert!(r.run_until(300, all_calibrating));
    assert!(r.run_until(3000, all_disarmed));
    assert!(r.run_until(500, |r| !has(&r.blockers(), "reference_stale")), "{:?}", r.blockers());
    let b = r.blockers();
    assert!(!has(&b, "reference_stale"), "{b:?}");
    // Only the recalibration mark (Issue #15 H3) is left.
    assert!(!b.is_empty() && b.iter().all(|x| x.starts_with("calibration_stale:")), "{b:?}");
    let snap = r.sup.snapshot(r.now);
    assert!(snap.axes.iter().all(|a| !a.reference_stale && a.referenced));
    assert_eq!(r.count(0, msg_type::SET_REFERENCE), [0; 3]);
}

#[test]
fn unanswered_calibrate_is_forgotten_after_ack_timeout_and_the_power_axis_cancels_the_others() {
    // Axes 2 / 3 calibrate slowly (10 s) so they are still at it when the ACK
    // of axis 1 times out.
    let mut r = Rig::with(sim_cfg(), |i, p| {
        p.start_referenced = false;
        p.end_min_count = -500;
        p.end_max_count = 500;
        if i != 0 {
            p.plant_max_vel_count_s = 200.0;
        }
    });
    r.connect_all();
    r.run_ms(100);
    // Sim 0 (axis 1, the power axis) never answers (frame lost on the wire).
    r.mcus[0].set_drop_rx(true);
    let sent = r.sent.len();
    r.admin(AdminCmd::Calibrate { axes: vec![] }).unwrap();
    r.run_ms(1100); // > ACK_TIMEOUT
    let cancelled: Vec<usize> = r
        .sent_since(sent)
        .iter()
        .filter(|(_, _, m)| matches!(m, HostMsg::Calibrate { mode: 0, .. }))
        .map(|(i, _, _)| *i)
        .collect();
    assert_eq!(cancelled, vec![1, 2], "{:?}", r.sent_since(sent));
    assert!(r.sup.fault().is_none());
    assert!(!has(&r.blockers(), "calibrate_rejected"), "{:?}", r.blockers());
    r.mcus[0].set_drop_rx(false);
    assert!(r.run_until(2000, all_disarmed), "{:?}", r.mcu_states());
    r.run_ms(50);
    // Nothing is left pending on axis 1: Stop is a plain no-op and a new
    // calibration of axis 1 alone is accepted.
    let ok = r.sup.admin(AdminCmd::Stop, r.now).unwrap();
    assert_eq!(ok.note.as_deref(), Some("not driving"));
    assert_eq!(r.count(sent, msg_type::CALIBRATE), [1, 2, 2], "no extra CALIBRATE(0) to axis 1");
    r.admin(AdminCmd::Calibrate { axes: vec![ax(1)] }).unwrap();
    assert!(r.run_until(3000, |r| r.sup.snapshot(r.now).axes[0].calibration.is_some()));
    assert_eq!(r.sup.snapshot(r.now).axes[0].calibration.as_ref().unwrap().result, h3cs::protocol::CalResultCode::Ok);
    let (events, _) = r.sup.drain_events();
    assert!(events.iter().any(|e| e.kind == "ack_timeout"));
    assert!(events.iter().any(|e| e.kind == "calibrate_cancelled"));
}

// ------------------------------------------------------------------ motion limits (design 2026-11-01)

#[test]
fn limits_are_distributed_once_and_follow_runtime_changes() {
    let cfg = sim_cfg();
    let mut r = Rig::new(cfg.clone());
    r.ready();
    // One SET_LIMITS per axis with the [limits] values; CONFIG carried the ceilings.
    assert_eq!(r.count(0, msg_type::SET_LIMITS), [1, 1, 1]);
    let sl = r
        .sent
        .iter()
        .find_map(|(_, _, m)| match m {
            HostMsg::SetLimits(l) => Some(*l),
            _ => None,
        })
        .unwrap();
    assert_eq!((sl.duty_max_percent, sl.duty_min_percent, sl.kp_percent_per_100), (40, 3, 2));
    assert_eq!((sl.ff_count_s_per_percent, sl.deadband_count, sl.deadband_out_count), (100, 10, 20));
    let c = r
        .sent
        .iter()
        .find_map(|(_, _, m)| match m {
            HostMsg::Config(c) => Some(*c),
            _ => None,
        })
        .unwrap();
    assert_eq!((c.max_vel_count_s, c.max_acc_count_s2), (6000, 40000));
    assert_eq!((c.following_error_limit_count, c.overspeed_count_s), (800, 9000));
    // The units applied it and reported LIMITS.
    let snap = r.sup.snapshot(r.now);
    for a in &snap.axes {
        let l = a.fw_limits.expect("LIMITS received");
        assert_eq!((l.duty_max_percent, l.abs_duty_max_percent, l.abs_max_vel_count_s), (40, 60, 6000));
    }
    assert_eq!(snap.motion_limits.effective, MotionLimits::from_config(&cfg.limits));
    assert_eq!(snap.motion_limits.requested, snap.motion_limits.effective);
    assert_eq!(snap.motion_limits.defaults, snap.motion_limits.effective);
    assert_eq!(snap.motion_limits.ceiling.max_vel_count_s, 6000);
    assert_eq!(snap.motion_limits.ceiling.max_gain, 1.0);
    assert!(snap.motion_limits.limited.is_empty());
    assert!(!has(&r.blockers(), "firmware_limits_unsupported"));
    for m in r.mcus.iter() {
        assert_eq!((m.tuning().duty_max_percent, m.tuning().duty_min_percent), (40, 3));
    }

    // A request above every ceiling: clamped, one new SET_LIMITS per axis.
    let mark = r.sent.len();
    r.sup.set_requested_limits(MotionLimits {
        max_gain: 1.5,
        max_vel_count_s: 7000,
        max_acc_count_s2: 50000,
        duty_max_percent: 90,
    });
    r.run_ms(200);
    assert_eq!(r.count(mark, msg_type::SET_LIMITS), [1, 1, 1]);
    let snap = r.sup.snapshot(r.now);
    assert_eq!(
        snap.motion_limits.effective,
        MotionLimits { max_gain: 1.0, max_vel_count_s: 6000, max_acc_count_s2: 40000, duty_max_percent: 60 }
    );
    assert_eq!(snap.motion_limits.limited.len(), 4);
    for m in r.mcus.iter() {
        assert_eq!(m.tuning().duty_max_percent, 60);
    }
    // Nothing more while the request stays the same.
    let mark = r.sent.len();
    r.run_ms(500);
    assert_eq!(r.count(mark, msg_type::SET_LIMITS), [0, 0, 0]);
    // A change that does not touch the firmware tuning sends nothing to the MCUs.
    r.sup.set_requested_limits(MotionLimits { max_vel_count_s: 2000, ..r.sup.effective_limits() });
    r.run_ms(200);
    assert_eq!(r.count(mark, msg_type::SET_LIMITS), [0, 0, 0]);
    assert_eq!(r.sup.effective_limits().max_vel_count_s, 2000);
}

#[test]
fn runtime_velocity_limit_governs_set_position_while_active() {
    let mut r = Rig::new(sim_cfg());
    r.active(POSE_A, 0.3);
    r.sup.set_requested_limits(MotionLimits { max_vel_count_s: 1200, max_acc_count_s2: 5000, ..r.sup.effective_limits() });
    let mark = r.setpos.len();
    r.run_ms(200);
    assert!(r.setpos.len() > mark);
    for (_, _, sp) in &r.setpos[mark + 1..] {
        for k in 0..3 {
            assert_eq!(sp[k].max_vel_count_s, 1200);
            assert_eq!(sp[k].max_acc_count_s2, 5000);
        }
    }
    assert_eq!(r.sup.state(), RigState::Active);
    assert!(r.sup.fault().is_none());
}

#[test]
fn limits_are_resent_after_a_reconnect() {
    let mut r = Rig::new(sim_cfg());
    r.ready();
    let mark = r.sent.len();
    r.disconnect(1);
    r.run_ms(100);
    r.connect(1);
    r.run_ms(500);
    assert_eq!(r.count(mark, msg_type::SET_LIMITS), [0, 1, 0]);
    assert_eq!(r.sup.state(), RigState::ReadyToArm);
}

#[test]
fn firmware_without_limits_capability_blocks_arm_and_gets_no_set_limits() {
    let cfg = sim_cfg();
    let mut sup = Supervisor::new(&cfg);
    let mut mcu = SimMcu::new(SimParams { device_id: cfg.axes[0].device_id, ..SimParams::default() });
    let now = Instant::now();
    let Some(McuMsg::Hello(hello)) = mcu.handle(&HostMsg::HelloReq { request_id: 1 }, 1).into_iter().next() else {
        panic!("no hello")
    };
    // An older firmware: POSITION_MODE only.
    let hello = Hello { capabilities: capabilities::POSITION_MODE, ..hello };
    sup.on_link_event(LinkEvent::Identified { axis: ax(1), hello, port: "sim:1".into() }, now);
    sup.on_link_event(LinkEvent::Status { axis: ax(1), status: mcu.status(), received_at: now }, now);
    let out = sup.tick(now, 0.01, None);
    assert!(out.admin_msgs[0].iter().all(|m| matches!(m, HostMsg::Config(_))), "{:?}", out.admin_msgs);
    assert!(has(&sup.blockers(now), "firmware_limits_unsupported: axis1"), "{:?}", sup.blockers(now));
    // Its CONFIG still carries the fallback ceilings.
    let HostMsg::Config(c) = out.admin_msgs[0][0] else { panic!() };
    assert_eq!((c.max_vel_count_s, c.max_acc_count_s2), (6000, 40000));
}

#[test]
fn limits_are_resent_after_an_mcu_reboot() {
    let mut r = Rig::new(sim_cfg());
    r.ready();
    assert_eq!(r.mcus[1].tuning().duty_max_percent, 40);
    let mark = r.sent.len();
    // The unit reboots with the link up: firmware defaults are back (20 % / 7 %).
    r.mcus[1].reboot(0xbeef_0002);
    assert_eq!((r.mcus[1].tuning().duty_max_percent, r.mcus[1].tuning().duty_min_percent), (20, 7));
    // The next STATUS (new boot_id) clears the stale LIMITS and SET_LIMITS goes
    // out again; the sim answers within the same step, so only the end state
    // is observable: never the old boot's LIMITS.
    assert!(r.run_until(1500, |r| r.count(mark, msg_type::SET_LIMITS)[1] >= 1));
    let snap = r.sup.snapshot(r.now);
    assert!(snap.axes[1].fw_limits.is_none_or(|l| l.boot_id == 0xbeef_0002), "{:?}", snap.axes[1].fw_limits);
    r.run_ms(50);
    assert_eq!(r.count(mark, msg_type::SET_LIMITS), [0, 1, 0]);
    assert_eq!((r.mcus[1].tuning().duty_max_percent, r.mcus[1].tuning().duty_min_percent), (40, 3));
    let snap = r.sup.snapshot(r.now);
    assert_eq!(snap.axes[1].fw_limits.map(|l| l.boot_id), Some(0xbeef_0002));
    assert!(!has(&r.blockers(), "limits_rejected"), "{:?}", r.blockers());
}

#[test]
fn arm_is_blocked_until_set_limits_is_acked() {
    let cfg = sim_cfg();
    let mut sup = Supervisor::new(&cfg);
    let mut mcu = SimMcu::new(SimParams { device_id: cfg.axes[0].device_id, ..SimParams::default() });
    let now = Instant::now();
    let Some(McuMsg::Hello(hello)) = mcu.handle(&HostMsg::HelloReq { request_id: 1 }, 1).into_iter().next() else {
        panic!("no hello")
    };
    sup.on_link_event(LinkEvent::Identified { axis: ax(1), hello, port: "sim:1".into() }, now);
    sup.on_link_event(LinkEvent::Status { axis: ax(1), status: mcu.status(), received_at: now }, now);
    let out = sup.tick(now, 0.01, None);
    let [HostMsg::Config(c), HostMsg::SetLimits(l)] = out.admin_msgs[0].as_slice() else { panic!("{:?}", out.admin_msgs) };
    // CONFIG ACKed, SET_LIMITS unanswered (lost on the wire): the axis stays blocked.
    let ack = Ack { request_id: c.request_id, acked_type: msg_type::CONFIG, result: AckResult::Accepted, reason: AckReason::None };
    sup.on_link_event(LinkEvent::Ack { axis: ax(1), ack }, now);
    let configured = mcu.handle(&HostMsg::Config(*c), 2);
    let status = configured.iter().find_map(|m| match m { McuMsg::Status(s) => Some(*s), _ => None }).unwrap_or(mcu.status());
    sup.on_link_event(LinkEvent::Status { axis: ax(1), status, received_at: now }, now);
    assert!(has(&sup.blockers(now), "limits_unconfirmed: axis1"), "{:?}", sup.blockers(now));
    // The MCU answers (ACK then LIMITS): blocker gone.
    let replies = mcu.handle(&HostMsg::SetLimits(*l), 3);
    for m in replies {
        match m {
            McuMsg::Ack(ack) => sup.on_link_event(LinkEvent::Ack { axis: ax(1), ack }, now),
            McuMsg::Limits(limits) => sup.on_link_event(LinkEvent::Limits { axis: ax(1), limits }, now),
            _ => {}
        }
    }
    assert!(!has(&sup.blockers(now), "limits_unconfirmed"), "{:?}", sup.blockers(now));
    // An unanswered SET_LIMITS is re-sent after the ACK timeout; the blocker stays meanwhile.
    let later = now + ACK_TIMEOUT + LIMITS_RETRY + Duration::from_millis(50);
    sup.set_requested_limits(MotionLimits { duty_max_percent: 30, ..sup.effective_limits() });
    let out = sup.tick(now + Duration::from_millis(20), 0.01, None);
    assert!(out.admin_msgs[0].iter().any(|m| matches!(m, HostMsg::SetLimits(_))));
    assert!(has(&sup.blockers(now + Duration::from_millis(20)), "limits_unconfirmed: axis1"));
    // The ACK timeout forgets the request and schedules a retry one LIMITS_RETRY later.
    let out = sup.tick(later, 0.01, None);
    assert!(out.admin_msgs[0].is_empty(), "{:?}", out.admin_msgs);
    let out = sup.tick(later + LIMITS_RETRY + Duration::from_millis(10), 0.01, None);
    assert!(out.admin_msgs[0].iter().any(|m| matches!(m, HostMsg::SetLimits(_))), "re-sent after timeout: {:?}", out.admin_msgs);
}

#[test]
fn set_limits_acked_but_limits_report_lost_is_resent_with_a_new_request_id() {
    let cfg = sim_cfg();
    let mut sup = Supervisor::new(&cfg);
    let mut mcu = SimMcu::new(SimParams { device_id: cfg.axes[0].device_id, ..SimParams::default() });
    let now = Instant::now();
    let Some(McuMsg::Hello(hello)) = mcu.handle(&HostMsg::HelloReq { request_id: 1 }, 1).into_iter().next() else {
        panic!("no hello")
    };
    sup.on_link_event(LinkEvent::Identified { axis: ax(1), hello, port: "sim:1".into() }, now);
    sup.on_link_event(LinkEvent::Status { axis: ax(1), status: mcu.status(), received_at: now }, now);
    let out = sup.tick(now, 0.01, None);
    let [HostMsg::Config(_), HostMsg::SetLimits(l)] = out.admin_msgs[0].as_slice() else { panic!("{:?}", out.admin_msgs) };
    // The MCU ACKs, but its LIMITS frame is lost on the wire.
    let replies = mcu.handle(&HostMsg::SetLimits(*l), 2);
    let ack = replies.iter().find_map(|m| match m { McuMsg::Ack(a) => Some(*a), _ => None }).unwrap();
    sup.on_link_event(LinkEvent::Ack { axis: ax(1), ack }, now);
    let t1 = now + Duration::from_millis(20);
    assert!(has(&sup.blockers(t1), "limits_unconfirmed: axis1"), "{:?}", sup.blockers(t1));
    assert!(sup.tick(t1, 0.01, None).admin_msgs[0].is_empty(), "no resend before LIMITS_RETRY");
    // After LIMITS_RETRY: a new SET_LIMITS with a fresh request_id (a resend
    // of the old one would be DUPLICATE without LIMITS).
    let t2 = now + LIMITS_RETRY + Duration::from_millis(10);
    let out = sup.tick(t2, 0.01, None);
    let Some(HostMsg::SetLimits(l2)) = out.admin_msgs[0].iter().find(|m| matches!(m, HostMsg::SetLimits(_))) else {
        panic!("{:?}", out.admin_msgs)
    };
    assert_ne!(l2.request_id, l.request_id);
    for m in mcu.handle(&HostMsg::SetLimits(*l2), 3) {
        match m {
            McuMsg::Ack(ack) => sup.on_link_event(LinkEvent::Ack { axis: ax(1), ack }, t2),
            McuMsg::Limits(limits) => sup.on_link_event(LinkEvent::Limits { axis: ax(1), limits }, t2),
            _ => {}
        }
    }
    assert!(!has(&sup.blockers(t2), "limits_unconfirmed"), "{:?}", sup.blockers(t2));
    assert!(sup.snapshot(t2).axes[0].fw_limits.is_some());
}
