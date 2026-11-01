//! Rig supervisor: the whole-rig state machine (DESIGN.md §5, architecture §10).
//!
//! Pure logic: no tasks, no I/O, time is injected as `now`. Inputs are
//! [`Supervisor::on_link_event`], [`Supervisor::admin`] and [`Supervisor::tick`];
//! outputs are the per-axis messages in [`TickOutput`] and [`StatusSnapshot`].
//! The supervisor owns the planner and the calibrations.
//!
//! # Contract with the server / axis links
//!
//! - `on_link_event` and `admin` never return messages: they queue them, and
//!   the next `tick` drains the queue into [`TickOutput::admin_msgs`].
//! - Within one tick, deliver `admin_msgs[i]` to axis `i` **before**
//!   `set_position[i]`. START needs an unexpired SET_POSITION received after
//!   ARM_PREPARE (which resets that window), so reordering can make the MCU
//!   reject START.
//! - Route messages for an axis only to [`Supervisor::axis_port`] (the
//!   axis's *primary* port). Any other port claiming the axis gets nothing,
//!   and its Status/Ack/FaultEvent must not be forwarded (events carry no
//!   port). Report its closing with [`LinkEvent::PortClosed`].
//! - `DecodeErrors.count` is a delta (errors since the previous report).
//!
//! # Axis assignment and conflicts (DESIGN.md §10.2)
//!
//! Each axis keeps its **claimants**: the ports whose MCU selected it (or,
//! for `selected_axis = 0`, whose device_id is configured for it). The
//! primary — the one that gets CONFIG / SET_POSITION and whose STATUS counts —
//! is the sole claimant; two or more claimants while not driving (Idle /
//! Fault) is an **axis conflict**: no primary, no CONFIG, blocker
//! `axis_conflict`, rig OFFLINE, `Arm` / `Start` / `Center` refused with
//! `axis_conflict`, and the API refuses poses. A second claimant while
//! driving leaves the primary in place and starts a controlled stop; the
//! conflict takes effect once the rig is idle. When one claimant remains it is
//! promoted like a fresh `Identified` (CONFIG, or HOST_FAULT while latched).
//! The primary's own axis changing while driving (`Disconnected` with reason
//! `axis_changed:`) latches [`RigFault::AxisChanged`].
//!
//! The configured `device_id` is a cross-check only: a mismatch (either
//! direction) is blocker `axis_device_mismatch`, never an unknown device.
//! `reference_stale`: a unit that is assigned to a different axis under the
//! same boot_id must be re-referenced by this server (SET_REFERENCE accepted)
//! before ARM, whatever its `POSITION_REFERENCED` flag says.
//!
//! # Decisions not spelled out in DESIGN.md
//!
//! - An axis reporting `fault_code != NONE` (STATUS or FAULT_EVENT) latches
//!   FAULT_LATCHED in **any** rig state, including at connect (an MCU still in
//!   FAULT from a previous server run). FaultReset clears it.
//! - ACK REJECTED handling (no new `RigFault` variants; the ACK reason goes to
//!   [`FaultRecord::detail`]):
//!   CONFIG / ARM_PREPARE during ARMING → `ArmTimeout`;
//!   START → `AxisStateMismatch { expected: ACTIVE }`;
//!   CONTROLLED_STOP / DISARM while driving → `StopTimeout`;
//!   CONFIG outside ARMING → not a fault (blocker `not_configured`, retried);
//!   FAULT_RESET → stays latched, blocker `fault_reset_rejected`;
//!   SET_REFERENCE → event only. `DUPLICATE` is treated as `ACCEPTED`.
//! - FaultReset is refused synchronously when an axis is disconnected, has no
//!   STATUS, has no STATUS processed after its HOST_FAULT ACK (event order, not
//!   time), is still powered / decelerating with a fault, or a reset is in
//!   flight. An unACKed HOST_FAULT is resent after [`ACK_TIMEOUT`] while
//!   latched; an axis (re)identified while latched gets HOST_FAULT, not CONFIG. A cause the MCU reports (`FAULT_CAUSE_PRESENT`) arrives as an ACK
//!   later: the rig stays latched and the reason is listed in `blockers`; the
//!   operator may retry (the MCU re-evaluates every FAULT_RESET).
//! - Start checks freshness of the pose passed to the previous `tick` and binds
//!   a game drive to that pose's `session_gen`; in ACTIVE(game) a pose of
//!   another session is treated as stale (→ STOPPING).
//! - Frame skew is the spread of per-axis lag (`group_frame_id` when the STATUS
//!   was taken in − its `last_applied_frame`), so STATUS lateness is not skew.
//! - Stop in ARMING disarms (the axes are not holding yet). Stop / Disarm in
//!   non-driving states are accepted no-ops.
//! - The center move scales both velocity and acceleration limits by
//!   `game.center_speed_fraction`.
//! - `stop_settle_timeout_ms` bounds the deceleration, the post-center settle
//!   and the wait for DISARMED; `arm_timeout_ms` bounds ARMING and START
//!   confirmation.
//! - Center is complete when the planner is at neutral with zero velocity and
//!   every axis's STATUS.target_count (MCU setpoint) is within
//!   [`CENTER_TOLERANCE_COUNT`] of neutral.

use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::time::Duration;

use serde::Serialize;
use tokio::time::Instant;

use crate::calibration::{self, AxisCalibration};
use crate::compat;
use crate::config::{Config, GameConfig, TimingConfig};
use crate::planner::{AxisLimits, Planner};
use crate::protocol::{
    self, capabilities, role_flags, Ack, AckReason, AckResult, FaultEvent, Hello, HostMsg,
    SetPosition, Status,
};
use crate::session::AcceptedPose;
use crate::types::{
    status_flags, AxisId, BootId, Counts, DeviceId, FaultCode, LimitReason, McuState, Pose,
    RigFault, RigState, AXIS_COUNT, VIRTUAL_CENTER,
};

/// No frame-skew check for this long after all axes report ACTIVE.
pub const FRAME_SKEW_GRACE: Duration = Duration::from_millis(200);
/// Entries kept in `fault_history`.
pub const FAULT_HISTORY_LEN: usize = 32;
/// Supervisor events kept until drained.
pub const EVENT_QUEUE_LEN: usize = 256;
/// Outstanding ACKs tracked (oldest dropped beyond this).
pub const MAX_PENDING_ACKS: usize = 64;
/// An unanswered request is forgotten after this.
pub const ACK_TIMEOUT: Duration = Duration::from_millis(1000);
/// Re-send CONFIG to a DISARMED, unconfigured axis at most this often.
pub const CONFIG_RETRY: Duration = Duration::from_millis(1000);
/// A FAULT_RESET that leads nowhere is abandoned after this.
pub const FAULT_RESET_TIMEOUT: Duration = Duration::from_millis(2000);
/// Center move is complete within this many counts of neutral (at zero velocity).
pub const CENTER_TOLERANCE_COUNT: f64 = 1.0;
/// Units remembered for re-reference detection (`device_id → last axis`).
const MAX_TRACKED_DEVICES: usize = 64;

/// CONTROLLED_STOP `reason` byte (informational on the MCU).
pub mod stop_reason {
    pub const ADMIN: u8 = 1;
    pub const SOURCE_STALE: u8 = 2;
    pub const CENTER_REACHED: u8 = 3;
}

/// `AdminError::error` codes.
pub mod admin_error {
    pub const NOT_READY: &str = "not_ready";
    pub const ALREADY_ARMED: &str = "already_armed";
    pub const NOT_HOLDING: &str = "not_holding";
    pub const BUSY: &str = "busy";
    pub const NO_FRESH_POSE: &str = "no_fresh_pose";
    pub const FAULT_LATCHED: &str = "fault_latched";
    pub const NOT_FAULTED: &str = "not_faulted";
    pub const RESET_IN_PROGRESS: &str = "reset_in_progress";
    pub const AXIS_DISCONNECTED: &str = "axis_disconnected";
    pub const AXIS_NOT_SETTLED: &str = "axis_not_settled";
    pub const NOT_DISARMED: &str = "not_disarmed";
    /// Two units select the same axis (DESIGN.md §10.2).
    pub const AXIS_CONFLICT: &str = "axis_conflict";
}

// ------------------------------------------------------------------ inputs

/// Event from an axis link (one task per USB port).
#[derive(Debug, Clone, PartialEq)]
pub enum LinkEvent {
    /// The port's MCU claims `axis` (HELLO `selected_axis`, or the `[[axis]]`
    /// configured with its device_id when that is 0). Also sent again on a
    /// repeated HELLO and after an axis change (preceded by `Disconnected`).
    Identified {
        axis: AxisId,
        hello: Hello,
        port: String,
    },
    /// HELLO from a device not in the configuration (blocks ARM).
    UnknownDevice {
        port: String,
        device_id: DeviceId,
    },
    Status {
        axis: AxisId,
        status: Status,
        /// When the link decoded it (STATUS age counts from here). Additive
        /// to DESIGN.md §5.
        received_at: Instant,
    },
    Ack {
        axis: AxisId,
        ack: Ack,
    },
    FaultEvent {
        axis: AxisId,
        ev: FaultEvent,
    },
    /// Link lost (port closed, STATUS silent past the deadline, write error).
    Disconnected {
        axis: AxisId,
        reason: String,
    },
    /// Frames discarded by the PC-side decoder since the previous report.
    DecodeErrors {
        axis: Option<AxisId>,
        port: String,
        count: u64,
    },
    /// A port that is not an axis's primary (unknown device, or a second
    /// unit claiming an axis) closed; clears its blocker / claim. Additive
    /// to DESIGN.md §5.
    PortClosed {
        port: String,
    },
}

/// Admin command (DESIGN.md §7).
#[derive(Debug, Clone, PartialEq)]
pub enum AdminCmd {
    Arm,
    Start,
    Stop,
    Disarm,
    FaultReset,
    Center,
    Reference { axis: AxisId, count: Counts },
    Fault { note: String },
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct AdminOk {
    pub rig_state: RigState,
    /// Set when the command was a no-op or needs explanation.
    pub note: Option<String>,
}

/// Refused admin command: `error` is a code from [`admin_error`].
#[derive(Debug, Clone, PartialEq, Eq, Serialize, thiserror::Error)]
#[error("{error} {blockers:?}")]
pub struct AdminError {
    pub error: String,
    pub blockers: Vec<String>,
}

impl AdminError {
    fn new(error: &str, blockers: Vec<String>) -> Self {
        Self {
            error: error.to_string(),
            blockers,
        }
    }
}

// ------------------------------------------------------------------ outputs

/// Messages for each axis (index = `AxisId::index()`). Deliver `admin_msgs`
/// before `set_position` (see module docs).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct TickOutput {
    pub admin_msgs: [Vec<HostMsg>; AXIS_COUNT],
    /// Present while armed (ARMING..STOPPING); one common `group_frame_id`.
    pub set_position: Option<[SetPosition; AXIS_COUNT]>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ActiveSource {
    Game,
    Center,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct FaultRecord {
    /// ms since the supervisor first saw time.
    pub at_ms: f64,
    pub fault: RigFault,
    pub detail: Option<String>,
}

/// Supervisor-side log entry for telemetry (`EventLog::push(at, kind, detail)`).
#[derive(Debug, Clone, PartialEq)]
pub struct SupervisorEvent {
    pub at: Instant,
    pub kind: &'static str,
    pub detail: serde_json::Value,
}

/// One port claiming an axis (`AxisSnapshot::claimants`).
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct ClaimantSnapshot {
    pub port: String,
    pub device_id: DeviceId,
}

/// One entry of `StatusSnapshot::axes` (DESIGN.md §7.2). STATUS-derived
/// fields are `None` until the first STATUS of the current connection.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct AxisSnapshot {
    pub axis_id: AxisId,
    pub name: String,
    pub connected: bool,
    pub port: Option<String>,
    /// The connected unit's device_id; the configured value when none.
    pub device_id: DeviceId,
    /// Configured cross-check (0 = none).
    pub expected_device_id: DeviceId,
    /// How the primary was assigned: `"selected"` (MCU `selected_axis`),
    /// `"config"` (device_id fallback), null when not connected.
    pub assignment: Option<&'static str>,
    /// Ports claiming this axis: the primary alone normally, two or more in
    /// an axis conflict.
    pub claimants: Vec<ClaimantSnapshot>,
    /// The unit must be re-referenced by this server (DESIGN.md §10.2).
    pub reference_stale: bool,
    pub boot_id: Option<BootId>,
    pub mcu_state: Option<McuState>,
    pub fault_code: Option<FaultCode>,
    pub configured: bool,
    pub referenced: bool,
    pub calibration_valid: bool,
    pub actual_count: Option<Counts>,
    pub target_count: Option<Counts>,
    pub velocity_count_s: Option<i32>,
    pub following_error_count: Option<i32>,
    pub pwm_permille: Option<i16>,
    pub power_on_cmd: Option<bool>,
    pub brake_release_cmd: Option<bool>,
    /// STATUS flag POWER_CONTROLLER: the MCU's own idea of its main-power role.
    pub power_controller: Option<bool>,
    /// Amperes, only when the MCU sets CURRENT_VALID.
    pub current_a: Option<f64>,
    pub last_applied_frame: Option<u32>,
    pub status_age_ms: Option<f64>,
    pub rx_error_count: Option<u16>,
    pub stale_drop_count: Option<u16>,
    /// PC-side frames from this axis that failed to decode (COBS / CRC /
    /// length), cumulative since server start.
    pub decode_errors: u64,
}

/// GET /v1/status body (DESIGN.md §7.2). `session` and `telemetry` are filled
/// by the server.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct StatusSnapshot {
    pub protocol: u32,
    pub rig_state: RigState,
    pub armed: bool,
    pub active_source: Option<ActiveSource>,
    pub calibration_valid: bool,
    pub fault: Option<RigFault>,
    pub fault_history: Vec<FaultRecord>,
    pub blockers: Vec<String>,
    pub session: Option<serde_json::Value>,
    /// Commanded by the game (not measured).
    pub requested_pose: Option<Pose>,
    pub requested_gain: Option<f64>,
    pub effective_gain: Option<f64>,
    pub limits: Vec<LimitReason>,
    pub virtual_targets: [f64; AXIS_COUNT],
    pub target_counts: [Counts; AXIS_COUNT],
    pub session_epoch: u32,
    pub group_frame_id: u32,
    pub axes: Vec<AxisSnapshot>,
    /// Axes claimed by two or more units (empty = no conflict).
    pub axis_conflict: Vec<AxisId>,
    /// Always null: angles are not calibrated.
    pub measured_angles: Option<serde_json::Value>,
    pub physical_estop: &'static str,
    pub telemetry: Option<serde_json::Value>,
}

// ------------------------------------------------------------------ internals

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Purpose {
    /// CONFIG on identify / reconcile (current epoch).
    Sync,
    ArmConfig,
    ArmPrepare,
    Start,
    Stop,
    Disarm,
    FaultReset,
    Reference,
    HostFault,
}

#[derive(Debug, Clone)]
struct Pending {
    axis: usize,
    msg_type: u8,
    purpose: Purpose,
    sent_at: Instant,
    epoch: u32,
    gen: u64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum ArmSub {
    /// CONFIG (new epoch) sent; waiting for all ACKs.
    Configuring,
    /// ARM_PREPARE sent; waiting for ACKs and STATUS HOLDING.
    Preparing,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum StopSub {
    /// CONTROLLED_STOP sent; planner decelerating.
    Decel,
    /// Settled and holding before the brake.
    Hold { since: Instant },
    /// DISARM sent; waiting for DISARMED.
    Disarming { since: Instant },
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Phase {
    /// Offline / CalibrationRequired / ReadyToArm / BrakedDisarmed.
    Idle(RigState),
    Arming {
        sub: ArmSub,
        started: Instant,
    },
    Holding {
        /// START sent, waiting for all axes ACTIVE.
        start: Option<(ActiveSource, Instant)>,
        /// Back from center: waiting for all axes HOLDING.
        settling: Option<Instant>,
    },
    Active {
        source: ActiveSource,
        entered: Instant,
    },
    Stopping {
        sub: StopSub,
        started: Instant,
    },
    Fault {
        /// FAULT_RESET in flight since.
        reset: Option<Instant>,
    },
}

/// A port whose MCU claims an axis (DESIGN.md §10.2).
#[derive(Debug, Clone, PartialEq)]
struct Claimant {
    port: String,
    /// Last HELLO (its `selected_axis` is kept current by the link).
    hello: Hello,
}

#[derive(Debug, Clone, Default)]
struct AxisTrack {
    /// A primary port is assigned (`port`, `hello`, STATUS are its).
    connected: bool,
    port: Option<String>,
    hello: Option<Hello>,
    /// Last boot_id seen (HELLO or STATUS); survives disconnects.
    boot_id: Option<BootId>,
    identified_at: Option<Instant>,
    status: Option<Status>,
    status_at: Option<Instant>,
    /// Event ordinal ([`Supervisor::ev`]) of the last STATUS.
    status_ev: u64,
    /// `group_frame_id` when the last STATUS was taken in (frame-skew lag).
    status_frame: u32,
    /// request_id of the HOST_FAULT sent in the current fault episode.
    host_fault_req: Option<u32>,
    /// Event ordinal of that HOST_FAULT's ACK.
    host_fault_ack_ev: Option<u64>,
    /// Epoch of the last CONFIG this boot ACKed.
    configured_epoch: Option<u32>,
    config_rejected: Option<AckReason>,
    config_retry_at: Option<Instant>,
    /// Every port claiming this axis, the primary included.
    claimants: Vec<Claimant>,
    decode_errors: u64,
    reset_rejected: Option<AckReason>,
    /// ACKed the current group command.
    group_ok: bool,
}

impl AxisTrack {
    fn mcu_state(&self) -> Option<McuState> {
        self.status.and_then(|s| McuState::from_u8(s.mcu_state))
    }
    fn flag(&self, f: u16) -> bool {
        self.status.is_some_and(|s| s.flags & f != 0)
    }
    fn referenced(&self) -> bool {
        self.flag(status_flags::POSITION_REFERENCED)
    }
    fn configured(&self, epoch: u32) -> bool {
        self.configured_epoch == Some(epoch) && self.flag(status_flags::CONFIGURED)
    }
    /// No HOST_FAULT outstanding, or a STATUS arrived after its ACK (so the
    /// cached STATUS shows the state the HOST_FAULT left the MCU in).
    fn host_fault_confirmed(&self) -> bool {
        self.host_fault_req.is_none()
            || self.host_fault_ack_ev.is_some_and(|a| self.status_ev > a)
    }
    /// Frames this axis was behind the last sent frame when its STATUS came in.
    fn frame_lag(&self) -> Option<i64> {
        self.status
            .map(|s| i64::from(self.status_frame.wrapping_sub(s.last_applied_frame) as i32))
    }
    fn in_conflict(&self) -> bool {
        self.claimants.len() >= 2
    }
    fn status_fault(&self) -> Option<FaultCode> {
        let s = self.status?;
        if s.fault_code != FaultCode::None {
            Some(s.fault_code)
        } else if McuState::from_u8(s.mcu_state) == Some(McuState::Fault) {
            Some(FaultCode::Unknown)
        } else {
            None
        }
    }
}

#[derive(Debug, Clone, Default)]
struct RequestView {
    pose: Option<Pose>,
    gain: Option<f64>,
    effective_gain: Option<f64>,
    limits: Vec<LimitReason>,
}

#[derive(Debug, Clone)]
struct DriveView {
    virtual_targets: [f64; AXIS_COUNT],
    target_counts: [Counts; AXIS_COUNT],
    limits: Vec<LimitReason>,
}

/// The rig state machine. See the module docs.
#[derive(Debug, Clone)]
pub struct Supervisor {
    timing: TimingConfig,
    game: GameConfig,
    names: [String; AXIS_COUNT],
    device_ids: [DeviceId; AXIS_COUNT],
    operator_valid: [bool; AXIS_COUNT],
    cal: [AxisCalibration; AXIS_COUNT],
    cal_error: [Option<String>; AXIS_COUNT],
    hashes: [u32; AXIS_COUNT],
    power_axis: AxisId,

    axes: [AxisTrack; AXIS_COUNT],
    unknown_ports: BTreeMap<String, DeviceId>,
    host_decode_errors: u64,
    /// Last axis each unit was primary for, with its boot_id (re-reference
    /// detection, DESIGN.md §10.2).
    last_assignment: BTreeMap<DeviceId, (BootId, AxisId)>,
    /// Units (device_id, boot_id) that changed axis under one boot and were
    /// not re-referenced by this server since.
    stale_refs: BTreeSet<(DeviceId, BootId)>,

    phase: Phase,
    planner: Planner,
    planner_settled: bool,

    epoch: u32,
    next_request_id: u32,
    seq: [u32; AXIS_COUNT],
    group_frame_id: u32,
    group: Option<(Purpose, u64)>,
    group_gen: u64,
    pending: BTreeMap<u32, Pending>,
    outbox: [Vec<HostMsg>; AXIS_COUNT],

    fault: Option<RigFault>,
    fault_history: VecDeque<FaultRecord>,
    t0: Option<Instant>,
    /// Link event ordinal: orders STATUS against ACKs regardless of timestamps.
    ev: u64,
    /// `ev` when the current FaultReset was accepted.
    reset_ev: u64,
    /// Session generation the game drive is bound to (set by Start).
    drive_session: Option<u64>,

    last_pose: Option<AcceptedPose>,
    requested: RequestView,
    drive: DriveView,
    last_frame_skew: Option<u32>,

    events: VecDeque<SupervisorEvent>,
    events_dropped: u64,
    last_state: RigState,
}

impl Supervisor {
    /// `session_epoch` and `request_id` start from random non-zero values
    /// (protocol.md §4: never reuse them across server restarts).
    pub fn new(cfg: &Config) -> Self {
        Self::with_initial_ids(cfg, rand::random::<u32>(), rand::random::<u32>())
    }

    /// Explicit initial counters (tests). 0 is replaced by 1.
    pub fn with_initial_ids(cfg: &Config, session_epoch: u32, request_id: u32) -> Self {
        let cal = calibration::from_config(cfg);
        let axis_cfg = |i: usize| AxisId::from_index(i).and_then(|id| cfg.axis(id));
        let names = std::array::from_fn(|i| {
            axis_cfg(i).map_or_else(|| format!("axis{}", i + 1), |a| a.name.clone())
        });
        let device_ids = std::array::from_fn(|i| axis_cfg(i).map_or(DeviceId(0), |a| a.device_id));
        let operator_valid =
            std::array::from_fn(|i| axis_cfg(i).is_some_and(|a| a.calibration_valid));
        let cal_error = std::array::from_fn(|i| cal[i].validate().err().map(|e| e.to_string()));
        let hashes = std::array::from_fn(|i| calibration::calibration_hash(&cfg.rig_id, &cal[i]));
        let neutral: [Counts; AXIS_COUNT] = std::array::from_fn(|i| cal[i].neutral_count());
        Self {
            timing: cfg.timing.clone(),
            game: cfg.game.clone(),
            names,
            device_ids,
            operator_valid,
            cal,
            cal_error,
            hashes,
            power_axis: cfg.power_controller_axis,
            axes: Default::default(),
            unknown_ports: BTreeMap::new(),
            host_decode_errors: 0,
            last_assignment: BTreeMap::new(),
            stale_refs: BTreeSet::new(),
            phase: Phase::Idle(RigState::Offline),
            planner: Planner::new(),
            planner_settled: false,
            epoch: nonzero(session_epoch),
            next_request_id: nonzero(request_id),
            seq: [0; AXIS_COUNT],
            group_frame_id: 0,
            group: None,
            group_gen: 0,
            pending: BTreeMap::new(),
            outbox: Default::default(),
            fault: None,
            fault_history: VecDeque::new(),
            t0: None,
            ev: 0,
            reset_ev: 0,
            drive_session: None,
            last_pose: None,
            requested: RequestView::default(),
            drive: DriveView {
                virtual_targets: [VIRTUAL_CENTER; AXIS_COUNT],
                target_counts: neutral,
                limits: Vec::new(),
            },
            last_frame_skew: None,
            events: VecDeque::new(),
            events_dropped: 0,
            last_state: RigState::Offline,
        }
    }

    // ------------------------------------------------------------ accessors

    pub fn state(&self) -> RigState {
        match self.phase {
            Phase::Idle(s) => s,
            Phase::Arming { .. } => RigState::Arming,
            Phase::Holding { .. } => RigState::Holding,
            Phase::Active { .. } => RigState::Active,
            Phase::Stopping { .. } => RigState::Stopping,
            Phase::Fault { .. } => RigState::FaultLatched,
        }
    }

    /// ARMING / HOLDING / ACTIVE / STOPPING.
    pub fn is_armed(&self) -> bool {
        matches!(
            self.phase,
            Phase::Arming { .. }
                | Phase::Holding { .. }
                | Phase::Active { .. }
                | Phase::Stopping { .. }
        )
    }

    pub fn active_source(&self) -> Option<ActiveSource> {
        match self.phase {
            Phase::Active { source, .. } => Some(source),
            _ => None,
        }
    }

    pub fn session_epoch(&self) -> u32 {
        self.epoch
    }

    /// The request_id the next command will use.
    pub fn next_request_id_peek(&self) -> u32 {
        self.next_request_id
    }

    pub fn fault(&self) -> Option<&RigFault> {
        self.fault.as_ref()
    }

    pub fn fault_history(&self) -> impl Iterator<Item = &FaultRecord> {
        self.fault_history.iter()
    }

    /// The primary port of this axis (none while disconnected or in conflict).
    pub fn axis_port(&self, axis: AxisId) -> Option<&str> {
        self.axes
            .get(axis.index())
            .filter(|t| t.connected)
            .and_then(|t| t.port.as_deref())
    }

    /// Axes claimed by two or more units (DESIGN.md §10.2).
    pub fn axis_conflict(&self) -> Vec<AxisId> {
        (0..AXIS_COUNT)
            .filter(|&i| self.axes[i].in_conflict())
            .filter_map(AxisId::from_index)
            .collect()
    }

    /// The connected unit on axis `i` changed axis under its current boot and
    /// has not been re-referenced by this server since.
    fn reference_stale(&self, i: usize) -> bool {
        let t = &self.axes[i];
        match (t.connected, t.hello, t.boot_id) {
            (true, Some(h), Some(b)) => self.stale_refs.contains(&(h.device_id, b)),
            _ => false,
        }
    }

    /// Last measured spread of `last_applied_frame` across axes while ACTIVE.
    pub fn frame_skew(&self) -> Option<u32> {
        self.last_frame_skew
    }

    /// PC-side decode errors per axis, plus those on ports with no axis.
    pub fn decode_errors(&self) -> ([u64; AXIS_COUNT], u64) {
        (
            std::array::from_fn(|i| self.axes[i].decode_errors),
            self.host_decode_errors,
        )
    }

    /// Outstanding ACKs.
    pub fn pending_acks(&self) -> usize {
        self.pending.len()
    }

    /// Take queued supervisor events (oldest first) and the count dropped since
    /// the previous drain.
    pub fn drain_events(&mut self) -> (Vec<SupervisorEvent>, u64) {
        let dropped = std::mem::take(&mut self.events_dropped);
        (self.events.drain(..).collect(), dropped)
    }

    /// Why ARM is (or would be) refused right now. Empty = ready.
    pub fn blockers(&self, now: Instant) -> Vec<String> {
        let mut b = Vec::new();
        for (i, t) in self.axes.iter().enumerate() {
            let n = i + 1;
            if !t.connected {
                b.push(format!("not_connected: axis{n}"));
            }
            if t.in_conflict() {
                let who: Vec<String> = t
                    .claimants
                    .iter()
                    .map(|c| format!("{}({})", c.port, c.hello.device_id))
                    .collect();
                b.push(format!("axis_conflict: axis{n} claimed by {}", who.join(", ")));
            }
            // Cross-check against the configured device_id, both directions.
            let expected = self.device_ids[i];
            for c in &t.claimants {
                let d = c.hello.device_id;
                if !expected.is_unassigned() && d != expected {
                    b.push(format!(
                        "axis_device_mismatch: axis{n} device {d}, config expects {expected}"
                    ));
                }
                if let Some(m) = (0..AXIS_COUNT)
                    .find(|&m| m != i && !d.is_unassigned() && self.device_ids[m] == d)
                {
                    b.push(format!(
                        "axis_device_mismatch: device {d} is configured as axis{} but selected axis{n}",
                        m + 1
                    ));
                }
            }
            if self.reference_stale(i) {
                b.push(format!(
                    "reference_stale: axis{n} re-reference required after axis change"
                ));
            }
            if !self.operator_valid[i] {
                b.push(format!(
                    "calibration_invalid: axis{n} calibration_valid = false"
                ));
            }
            if let Some(e) = &self.cal_error[i] {
                b.push(format!("calibration_invalid: {e}"));
            }
            if let Some(r) = t.reset_rejected {
                b.push(format!("fault_reset_rejected: axis{n} {r:?}"));
            }
            if !t.connected {
                continue;
            }
            if let Some(h) = &t.hello {
                if h.protocol_version != protocol::PROTOCOL_VERSION {
                    b.push(format!(
                        "protocol_mismatch: axis{n} protocol_version {}",
                        h.protocol_version
                    ));
                }
                if h.capabilities & capabilities::POSITION_MODE == 0 {
                    b.push(format!("no_position_mode: axis{n}"));
                }
            }
            let Some(s) = t.status else {
                b.push(format!("no_status: axis{n}"));
                continue;
            };
            if let Some(code) = t.status_fault() {
                b.push(format!("axis_fault: axis{n} fault_code {code:?}"));
            } else if t.mcu_state() != Some(McuState::Disarmed) {
                b.push(format!(
                    "mcu_not_disarmed: axis{n} state {}",
                    state_name(s.mcu_state)
                ));
            }
            if !t.referenced() {
                b.push(format!(
                    "not_referenced: axis{n} POSITION_REFERENCED not set"
                ));
            }
            if !t.configured(self.epoch) {
                match t.config_rejected {
                    Some(r) => b.push(format!("not_configured: axis{n} CONFIG rejected ({r:?})")),
                    None => b.push(format!("not_configured: axis{n} CONFIG not accepted yet")),
                }
            } else {
                // Only the configured axis may drive the shared main-power relay
                // (architecture.md §10). Checked once the MCU took this CONFIG.
                let expected = AxisId::from_index(i) == Some(self.power_axis);
                let reported = t.flag(status_flags::POWER_CONTROLLER);
                if reported != expected {
                    b.push(format!(
                        "power_role_mismatch: axis{n} reports POWER_CONTROLLER={reported}, config expects {expected}"
                    ));
                }
            }
            let c = &self.cal[i];
            if s.actual_count < c.soft_min_count || s.actual_count > c.soft_max_count {
                b.push(format!(
                    "position_outside_soft_limits: axis{n} actual {} not in [{}, {}]",
                    s.actual_count, c.soft_min_count, c.soft_max_count
                ));
            }
            if self
                .status_age(i, now)
                .is_some_and(|a| a > self.status_timeout())
            {
                b.push(format!("status_stale: axis{n}"));
            }
        }
        for (port, id) in &self.unknown_ports {
            b.push(format!("unknown_device: device {id} on port {port}"));
        }
        b
    }

    // ------------------------------------------------------------ inputs

    pub fn on_link_event(&mut self, ev: LinkEvent, now: Instant) {
        self.touch(now);
        self.ev += 1;
        match ev {
            LinkEvent::Identified { axis, hello, port } => {
                self.on_identified(axis, hello, port, now)
            }
            LinkEvent::UnknownDevice { port, device_id } => {
                self.forget_port(&port, now);
                self.event(
                    now,
                    "unknown_device",
                    serde_json::json!({"port": port, "device_id": device_id}),
                );
                self.unknown_ports.insert(port, device_id);
            }
            LinkEvent::Status {
                axis,
                status,
                received_at,
            } => self.on_status(axis.index(), status, received_at.min(now), now),
            LinkEvent::Ack { axis, ack } => self.on_ack(axis.index(), ack, now),
            LinkEvent::FaultEvent { axis, ev } => self.on_fault_event(axis.index(), ev, now),
            LinkEvent::Disconnected { axis, reason } => {
                self.on_disconnected(axis.index(), reason, now)
            }
            LinkEvent::DecodeErrors { axis, count, .. } => match axis {
                Some(a) => {
                    if let Some(t) = self.axes.get_mut(a.index()) {
                        t.decode_errors = t.decode_errors.saturating_add(count);
                    }
                }
                None => self.host_decode_errors = self.host_decode_errors.saturating_add(count),
            },
            LinkEvent::PortClosed { port } => {
                self.unknown_ports.remove(&port);
                self.forget_port(&port, now);
            }
        }
        self.advance(now);
    }

    pub fn admin(&mut self, cmd: AdminCmd, now: Instant) -> Result<AdminOk, AdminError> {
        self.touch(now);
        self.advance(now);
        let label = format!("{cmd:?}");
        let r = self.admin_inner(cmd, now);
        self.advance(now);
        match &r {
            Ok(note) => self.event(
                now,
                "admin",
                serde_json::json!({"cmd": label, "ok": true, "note": note}),
            ),
            Err(e) => self.event(
                now,
                "admin",
                serde_json::json!({"cmd": label, "ok": false, "error": e}),
            ),
        }
        r.map(|note| AdminOk {
            rig_state: self.state(),
            note,
        })
    }

    /// One control-loop step. `dt_s` is the elapsed time since the previous
    /// tick (sanitized and capped at 2 × planner period here as well).
    pub fn tick(&mut self, now: Instant, dt_s: f64, pose: Option<&AcceptedPose>) -> TickOutput {
        self.touch(now);
        let period = self.timing.planner_period_ms as f64 / 1000.0;
        let dt = if dt_s.is_finite() && dt_s > 0.0 {
            dt_s.min(2.0 * period)
        } else {
            period
        };

        self.last_pose = pose.cloned();
        self.requested = match pose {
            Some(p) => RequestView {
                pose: Some(p.pose),
                gain: Some(p.requested_gain),
                effective_gain: Some(p.checked.effective_gain),
                limits: p.checked.limits.clone(),
            },
            None => RequestView::default(),
        };

        self.advance(now);
        self.check_status_age(now);
        self.check_frame_skew(now);

        let mut drive_limits = Vec::new();
        match self.phase {
            Phase::Active {
                source: ActiveSource::Game,
                ..
            } => {
                // Bound to the session that was current at START: a pose of a
                // newer session stops the rig like a stale one.
                let usable = pose.filter(|p| {
                    p.fresh_until > now && Some(p.session_gen) == self.drive_session
                });
                match usable {
                    Some(p) => drive_limits = self.track_game(p, dt),
                    None => {
                        if pose.is_some_and(|p| Some(p.session_gen) != self.drive_session) {
                            self.event(now, "session_changed", serde_json::json!({}));
                        }
                        self.begin_stop(stop_reason::SOURCE_STALE, now)
                    }
                }
            }
            Phase::Active {
                source: ActiveSource::Center,
                ..
            } => {
                let targets = self.cal.each_ref().map(|c| f64::from(c.neutral_count()));
                let out = self.planner.step_track(
                    targets,
                    self.axis_limits(self.game.center_speed_fraction),
                    dt,
                );
                drive_limits = out.limits.clone();
                // Planner at neutral and every MCU setpoint (STATUS.target_count)
                // caught up, so CONTROLLED_STOP does not cut the move short.
                let reached = (0..AXIS_COUNT).all(|i| {
                    (out.positions[i] - targets[i]).abs() <= CENTER_TOLERANCE_COUNT
                        && out.velocities[i] == 0.0
                        && self.axes[i].status.is_some_and(|s| {
                            (f64::from(s.target_count) - targets[i]).abs() <= CENTER_TOLERANCE_COUNT
                        })
                });
                if reached {
                    self.event(now, "center_reached", serde_json::json!({}));
                    self.send_group_stop(stop_reason::CENTER_REACHED, now);
                    self.phase = Phase::Holding {
                        start: None,
                        settling: Some(now),
                    };
                }
            }
            Phase::Holding { .. } => self.planner.hold(),
            _ => {}
        }
        if let Phase::Stopping { .. } = self.phase {
            let out = self.planner.step_stop(self.axis_limits(1.0), dt);
            self.planner_settled = out.settled;
            drive_limits = out.limits;
        }
        self.advance(now);

        let set_position = if self.is_armed() {
            Some(self.build_set_position(&drive_limits))
        } else {
            None
        };
        let admin_msgs = std::mem::take(&mut self.outbox);
        self.note_state(now);
        TickOutput {
            admin_msgs,
            set_position,
        }
    }

    pub fn snapshot(&self, now: Instant) -> StatusSnapshot {
        let calibration_valid = (0..AXIS_COUNT).all(|i| self.axis_calibration_valid(i));
        let blockers = if self.is_armed() {
            Vec::new()
        } else {
            self.blockers(now)
        };
        let mut limits = self.requested.limits.clone();
        for l in &self.drive.limits {
            if !limits.contains(l) {
                limits.push(*l);
            }
        }
        let axes = (0..AXIS_COUNT)
            .map(|i| {
                let t = &self.axes[i];
                let s = t.status;
                AxisSnapshot {
                    axis_id: AxisId::from_index(i).unwrap_or(AxisId::ALL[0]),
                    name: self.names[i].clone(),
                    connected: t.connected,
                    port: t.port.clone(),
                    device_id: t
                        .hello
                        .filter(|_| t.connected)
                        .map_or(self.device_ids[i], |h| h.device_id),
                    expected_device_id: self.device_ids[i],
                    assignment: t.hello.filter(|_| t.connected).map(|h| {
                        if h.selected_axis != 0 {
                            "selected"
                        } else {
                            "config"
                        }
                    }),
                    claimants: t
                        .claimants
                        .iter()
                        .map(|c| ClaimantSnapshot {
                            port: c.port.clone(),
                            device_id: c.hello.device_id,
                        })
                        .collect(),
                    reference_stale: self.reference_stale(i),
                    boot_id: t.boot_id,
                    mcu_state: t.mcu_state(),
                    fault_code: s.map(|s| s.fault_code),
                    configured: t.configured(self.epoch),
                    referenced: t.referenced(),
                    calibration_valid: self.axis_calibration_valid(i),
                    actual_count: s.map(|s| s.actual_count),
                    target_count: s.map(|s| s.target_count),
                    velocity_count_s: s.map(|s| s.velocity_count_s),
                    following_error_count: s.map(|s| s.following_error_count),
                    pwm_permille: s.map(|s| s.pwm_permille),
                    power_on_cmd: s.map(|s| s.flags & status_flags::POWER_ON_CMD != 0),
                    brake_release_cmd: s.map(|s| s.flags & status_flags::BRAKE_RELEASE_CMD != 0),
                    power_controller: s.map(|s| s.flags & status_flags::POWER_CONTROLLER != 0),
                    current_a: s
                        .filter(|s| s.flags & status_flags::CURRENT_VALID != 0)
                        .map(|s| f64::from(s.current_ma) / 1000.0),
                    last_applied_frame: s.map(|s| s.last_applied_frame),
                    status_age_ms: self
                        .status_age(i, now)
                        .filter(|_| s.is_some())
                        .map(|d| d.as_secs_f64() * 1000.0),
                    rx_error_count: s.map(|s| s.rx_error_count),
                    stale_drop_count: s.map(|s| s.stale_drop_count),
                    decode_errors: t.decode_errors,
                }
            })
            .collect();
        StatusSnapshot {
            protocol: 1,
            rig_state: self.state(),
            armed: self.is_armed(),
            active_source: self.active_source(),
            calibration_valid,
            fault: self.fault.clone(),
            fault_history: self.fault_history.iter().cloned().collect(),
            blockers,
            session: None,
            requested_pose: self.requested.pose,
            requested_gain: self.requested.gain,
            effective_gain: self.requested.effective_gain,
            limits,
            virtual_targets: self.drive.virtual_targets,
            target_counts: self.drive.target_counts,
            session_epoch: self.epoch,
            group_frame_id: self.group_frame_id,
            axes,
            axis_conflict: self.axis_conflict(),
            measured_angles: None,
            physical_estop: "not_monitored",
            telemetry: None,
        }
    }

    // ------------------------------------------------------------ link events

    fn on_identified(&mut self, axis: AxisId, hello: Hello, port: String, now: Instant) {
        let i = axis.index();
        self.unknown_ports.remove(&port);
        self.note_claim(axis, &hello, now);
        // A port claims one axis at a time.
        for (j, t) in self.axes.iter_mut().enumerate() {
            if j != i {
                t.claimants.retain(|c| c.port != port);
            }
        }
        let t = &mut self.axes[i];
        match t.claimants.iter_mut().find(|c| c.port == port) {
            Some(c) => c.hello = hello,
            None => t.claimants.push(Claimant {
                port: port.clone(),
                hello,
            }),
        }
        let is_primary = t.connected && t.port.as_deref() == Some(port.as_str());
        if is_primary {
            // Repeated HELLO of the primary (reboot detection below).
            self.promote(i, hello, port, now);
            return;
        }
        if t.connected || t.in_conflict() {
            // Another unit claims an axis that has a primary, or joins an
            // existing conflict. The primary is left alone here: while
            // driving the rig is brought to rest first; when idle / latched
            // `reconcile_claimants` demotes it (DESIGN.md §10.2).
            self.event(
                now,
                "axis_conflict",
                serde_json::json!({"axis": axis, "port": port, "device_id": hello.device_id,
                    "primary": self.axes[i].port}),
            );
            if self.is_armed() {
                match self.phase {
                    Phase::Arming { .. } => self.begin_disarm(now),
                    Phase::Holding { .. } | Phase::Active { .. } => {
                        self.begin_stop(stop_reason::ADMIN, now)
                    }
                    _ => {}
                }
            }
            return;
        }
        self.promote(i, hello, port, now);
    }

    /// Re-reference bookkeeping at claim time (DESIGN.md §10.2): the same unit
    /// under the same boot_id claiming another axis than last time is stale
    /// until this server's SET_REFERENCE is accepted or it reboots. The MCU
    /// should drop POSITION_REFERENCED itself; this does not rely on it.
    fn note_claim(&mut self, axis: AxisId, hello: &Hello, now: Instant) {
        if let Some(&(b, prev)) = self.last_assignment.get(&hello.device_id) {
            if b == hello.boot_id && prev != axis {
                self.stale_refs.insert((hello.device_id, b));
                self.event(
                    now,
                    "reference_stale",
                    serde_json::json!({"axis": axis, "device_id": hello.device_id, "previous_axis": prev}),
                );
            }
        }
        // Both maps are bounded, but never at the expense of a unit that is
        // still here: evicting a present unit would clear its `reference_stale`
        // blocker or hide its next switch (fail closed; present units are at
        // most one per port, so the maps stay bounded).
        let present = self.present_device_ids();
        if self.last_assignment.len() >= MAX_TRACKED_DEVICES
            && !self.last_assignment.contains_key(&hello.device_id)
        {
            if let Some(k) = self
                .last_assignment
                .keys()
                .find(|d| !present.contains(d))
                .copied()
            {
                self.last_assignment.remove(&k);
            }
        }
        self.last_assignment
            .insert(hello.device_id, (hello.boot_id, axis));
        if self.stale_refs.len() > MAX_TRACKED_DEVICES {
            if let Some(k) = self
                .stale_refs
                .iter()
                .find(|(d, _)| !present.contains(d))
                .copied()
            {
                self.stale_refs.remove(&k);
            }
        }
    }

    /// device_ids of every unit currently identified on some port (primary or
    /// claimant of any axis).
    fn present_device_ids(&self) -> BTreeSet<DeviceId> {
        self.axes
            .iter()
            .flat_map(|t| t.claimants.iter().map(|c| c.hello.device_id))
            .chain(self.axes.iter().filter_map(|t| t.hello.as_ref().map(|h| h.device_id)))
            .collect()
    }

    /// Make `port` the primary of axis `i` (a fresh connection, a repeated
    /// HELLO of the current primary, or the claimant left after a conflict):
    /// reboot / re-reference bookkeeping, then CONFIG — or HOST_FAULT while
    /// latched. Never moves, arms or starts anything.
    fn promote(&mut self, i: usize, hello: Hello, port: String, now: Instant) {
        let Some(axis) = AxisId::from_index(i) else {
            return;
        };
        let t = &mut self.axes[i];
        // A reboot is the *same* unit with a new boot_id; another unit taking
        // over the axis (after an axis change) is a reconnect, not a reboot.
        let same_unit = t.hello.is_some_and(|h| h.device_id == hello.device_id);
        let rebooted = same_unit && t.boot_id.is_some_and(|b| b != hello.boot_id);
        let reconnect = !t.connected || !same_unit;
        t.connected = true;
        t.port = Some(port.clone());
        t.hello = Some(hello);
        t.boot_id = Some(hello.boot_id);
        t.identified_at = Some(now);
        if rebooted {
            // A new boot has no reference to be stale about.
            self.stale_refs.retain(|(d, _)| *d != hello.device_id);
        }
        let t = &mut self.axes[i];
        if rebooted || reconnect {
            t.status = None;
            t.status_at = None;
            t.configured_epoch = None;
            t.config_rejected = None;
            // A new link / boot has not seen our HOST_FAULT.
            t.host_fault_req = None;
            t.host_fault_ack_ev = None;
        }
        if hello.calibration_hash != self.hashes[i] {
            // MCU holds no (or another) calibration: ours must be re-ACKed.
            t.configured_epoch = None;
        }
        self.event(
            now,
            "identified",
            serde_json::json!({"axis": axis, "port": port, "boot_id": hello.boot_id,
                "selected_axis": hello.selected_axis, "device_id": hello.device_id,
                "hash_match": hello.calibration_hash == self.hashes[i], "rebooted": rebooted}),
        );
        if rebooted && self.is_armed() {
            self.latch(
                RigFault::AxisRebooted { axis },
                Some("boot_id changed in HELLO".into()),
                now,
            );
        }
        if matches!(self.phase, Phase::Fault { .. }) {
            // Latched: the axis gets HOST_FAULT (not CONFIG) once reachable.
            // CONFIG follows after the reset (reconcile_config).
            if self.axes[i].host_fault_req.is_none() {
                self.send_host_fault(i, now);
            }
        } else if !self.is_armed() {
            // Only CONFIG: identifying must never move, center, arm or start anything.
            self.send_config(i, Purpose::Sync, now);
            self.axes[i].config_retry_at = Some(now + CONFIG_RETRY);
        }
    }

    fn on_status(&mut self, i: usize, status: Status, received_at: Instant, now: Instant) {
        let Some(axis) = AxisId::from_index(i) else {
            return;
        };
        if !self.axes[i].connected {
            return;
        }
        let t = &mut self.axes[i];
        let rebooted = t.boot_id.is_some_and(|b| b != status.boot_id);
        if rebooted {
            t.configured_epoch = None;
            t.config_rejected = None;
            if let Some(h) = t.hello {
                self.stale_refs.retain(|(d, _)| *d != h.device_id);
            }
        }
        let t = &mut self.axes[i];
        t.boot_id = Some(status.boot_id);
        if status.flags & status_flags::CONFIGURED == 0 {
            t.configured_epoch = None;
        }
        t.status = Some(status);
        t.status_at = Some(received_at);
        t.status_ev = self.ev;
        t.status_frame = self.group_frame_id;
        if rebooted {
            self.event(
                now,
                "axis_rebooted",
                serde_json::json!({"axis": axis, "boot_id": status.boot_id}),
            );
            if self.is_armed() {
                self.latch(
                    RigFault::AxisRebooted { axis },
                    Some("boot_id changed in STATUS".into()),
                    now,
                );
            }
        }
        if let Some(code) = self.axes[i].status_fault() {
            self.latch_axis_fault(axis, code, "STATUS", now);
        }
        self.check_axis_state(i, now);
    }

    fn on_fault_event(&mut self, i: usize, ev: FaultEvent, now: Instant) {
        let Some(axis) = AxisId::from_index(i) else {
            return;
        };
        if !self.axes[i].connected {
            return;
        }
        let code = if ev.fault_code == FaultCode::None {
            FaultCode::Unknown
        } else {
            ev.fault_code
        };
        self.latch_axis_fault(axis, code, "FAULT_EVENT", now);
    }

    fn on_disconnected(&mut self, i: usize, reason: String, now: Instant) {
        let Some(axis) = AxisId::from_index(i) else {
            return;
        };
        let t = &mut self.axes[i];
        if !t.connected {
            return;
        }
        if let Some(port) = t.port.clone() {
            t.claimants.retain(|c| c.port != port);
        }
        self.drop_primary(i);
        self.event(
            now,
            "disconnected",
            serde_json::json!({"axis": axis, "reason": reason}),
        );
        if self.is_armed() {
            let fault = if reason.starts_with(crate::axis_link::reason::AXIS_CHANGED) {
                RigFault::AxisChanged { axis }
            } else {
                RigFault::AxisDisconnected { axis }
            };
            self.latch(fault, Some(reason), now);
        } else if let Phase::Fault { reset: Some(_) } = self.phase {
            self.phase = Phase::Fault { reset: None };
        }
    }

    /// Forget the primary of axis `i` (link gone or demoted by a conflict):
    /// its STATUS, CONFIG state and outstanding requests are void.
    fn drop_primary(&mut self, i: usize) {
        let t = &mut self.axes[i];
        t.connected = false;
        t.port = None;
        t.status = None;
        t.status_at = None;
        t.configured_epoch = None;
        t.config_rejected = None;
        t.group_ok = false;
        self.pending.retain(|_, p| p.axis != i);
        if let Some(q) = self.outbox.get_mut(i) {
            q.clear();
        }
    }

    /// A port that is not an axis's primary went away (or became unknown):
    /// remove it from every claimant list. The remaining sole claimant of a
    /// conflicted axis is promoted by `reconcile_claimants`.
    fn forget_port(&mut self, port: &str, now: Instant) {
        for i in 0..AXIS_COUNT {
            if self.axes[i].connected && self.axes[i].port.as_deref() == Some(port) {
                // Should have come as Disconnected; treat it as one.
                self.on_disconnected(i, "port_closed".into(), now);
            }
            self.axes[i].claimants.retain(|c| c.port != port);
        }
    }

    /// Idle / Fault only (DESIGN.md §10.2): two or more claimants → no
    /// primary; exactly one and no primary → promote it.
    ///
    /// While latched, a primary that was being driven keeps its role until
    /// the HOST_FAULT of this episode is ACKed (or times out): `drop_primary`
    /// clears its outbox and the server routes nothing to a conflicted axis,
    /// so demoting it first would lose the HOST_FAULT (the unit would only
    /// stop on its own lease).
    fn reconcile_claimants(&mut self, now: Instant) {
        for i in 0..AXIS_COUNT {
            let t = &self.axes[i];
            if t.in_conflict() && t.connected {
                let Some(axis) = AxisId::from_index(i) else {
                    continue;
                };
                // Outstanding = sent this episode, not ACKed, not yet timed out
                // (`expire_pending` does not re-send to a conflicted axis, so
                // the deferral is bounded by one ACK_TIMEOUT).
                let host_fault_outstanding = matches!(self.phase, Phase::Fault { .. })
                    && t.host_fault_ack_ev.is_none()
                    && t.host_fault_req.is_some_and(|id| self.pending.contains_key(&id));
                if host_fault_outstanding {
                    continue;
                }
                self.event(
                    now,
                    "axis_conflict",
                    serde_json::json!({"axis": axis, "primary": t.port, "demoted": true}),
                );
                self.drop_primary(i);
                // Like a disconnect: a FAULT_RESET in flight cannot complete.
                if let Phase::Fault { reset: Some(_) } = self.phase {
                    self.phase = Phase::Fault { reset: None };
                }
            } else if t.claimants.len() == 1 && !t.connected {
                let c = t.claimants[0].clone();
                self.promote(i, c.hello, c.port, now);
            }
        }
    }

    fn on_ack(&mut self, i: usize, ack: Ack, now: Instant) {
        let Some(axis) = AxisId::from_index(i) else {
            return;
        };
        let Some(p) = self.pending.get(&ack.request_id).cloned() else {
            self.event(
                now,
                "unexpected_ack",
                serde_json::json!({"axis": axis, "ack": ack}),
            );
            return;
        };
        if p.axis != i || p.msg_type != ack.acked_type {
            self.event(
                now,
                "mismatched_ack",
                serde_json::json!({"axis": axis, "ack": ack}),
            );
            return;
        }
        self.pending.remove(&ack.request_id);
        if p.purpose == Purpose::HostFault && self.axes[i].host_fault_req == Some(ack.request_id) {
            // Any result: what matters is that later STATUS shows its effect.
            self.axes[i].host_fault_ack_ev = Some(self.ev);
        }
        let current_group = self.group == Some((p.purpose, p.gen));
        match ack.result {
            AckResult::Accepted | AckResult::Duplicate => {
                if current_group {
                    self.axes[i].group_ok = true;
                }
                match p.purpose {
                    Purpose::Sync | Purpose::ArmConfig => {
                        let t = &mut self.axes[i];
                        t.configured_epoch = Some(p.epoch);
                        t.config_rejected = None;
                    }
                    Purpose::FaultReset => self.axes[i].reset_rejected = None,
                    Purpose::Reference => {
                        // This server re-referenced the unit: no longer stale.
                        let t = &self.axes[i];
                        if let (Some(h), Some(b)) = (t.hello, t.boot_id) {
                            self.stale_refs.remove(&(h.device_id, b));
                        }
                    }
                    _ => {}
                }
            }
            AckResult::Rejected => self.on_rejected(axis, &p, ack.reason, current_group, now),
        }
    }

    fn on_rejected(
        &mut self,
        axis: AxisId,
        p: &Pending,
        reason: AckReason,
        current_group: bool,
        now: Instant,
    ) {
        let i = axis.index();
        let name = msg_name(p.msg_type);
        self.event(
            now,
            "ack_rejected",
            serde_json::json!({"axis": axis, "msg": name, "reason": reason}),
        );
        let detail = Some(format!("axis{} {name} rejected: {reason:?}", axis.get()));
        match p.purpose {
            Purpose::Sync => self.axes[i].config_rejected = Some(reason),
            Purpose::ArmConfig | Purpose::ArmPrepare => {
                if current_group && matches!(self.phase, Phase::Arming { .. }) {
                    self.latch(RigFault::ArmTimeout, detail, now);
                } else if p.purpose == Purpose::ArmConfig {
                    self.axes[i].config_rejected = Some(reason);
                }
            }
            Purpose::Start => {
                if current_group && self.is_armed() {
                    let actual = self.axes[i].mcu_state().unwrap_or(McuState::Holding);
                    self.latch(
                        RigFault::AxisStateMismatch {
                            axis,
                            expected: McuState::Active,
                            actual,
                        },
                        detail,
                        now,
                    );
                }
            }
            Purpose::Stop | Purpose::Disarm => {
                if current_group && self.is_armed() {
                    self.latch(RigFault::StopTimeout, detail, now);
                }
            }
            Purpose::FaultReset => {
                self.axes[i].reset_rejected = Some(reason);
                if let Phase::Fault { reset: Some(_) } = self.phase {
                    self.phase = Phase::Fault { reset: None };
                }
            }
            Purpose::Reference | Purpose::HostFault => {}
        }
    }

    // ------------------------------------------------------------ admin

    fn admin_inner(&mut self, cmd: AdminCmd, now: Instant) -> Result<Option<String>, AdminError> {
        match cmd {
            AdminCmd::Arm => self.cmd_arm(now),
            AdminCmd::Start => self.cmd_start(ActiveSource::Game, now),
            AdminCmd::Center => self.cmd_start(ActiveSource::Center, now),
            AdminCmd::Stop => Ok(self.cmd_stop(now)),
            AdminCmd::Disarm => self.cmd_disarm(now),
            AdminCmd::FaultReset => self.cmd_fault_reset(now),
            AdminCmd::Reference { axis, count } => self.cmd_reference(axis, count, now),
            AdminCmd::Fault { note } => {
                self.latch(RigFault::Operator { note }, None, now);
                Ok(None)
            }
        }
    }

    fn cmd_arm(&mut self, now: Instant) -> Result<Option<String>, AdminError> {
        if !self.axis_conflict().is_empty() {
            return Err(AdminError::new(
                admin_error::AXIS_CONFLICT,
                self.blockers(now),
            ));
        }
        match self.phase {
            Phase::Idle(RigState::ReadyToArm | RigState::BrakedDisarmed) => {}
            Phase::Idle(_) => {
                return Err(AdminError::new(admin_error::NOT_READY, self.blockers(now)))
            }
            Phase::Fault { .. } => {
                return Err(AdminError::new(
                    admin_error::FAULT_LATCHED,
                    self.blockers(now),
                ))
            }
            _ => return Err(AdminError::new(admin_error::ALREADY_ARMED, Vec::new())),
        }
        let blockers = self.blockers(now);
        if !blockers.is_empty() {
            return Err(AdminError::new(admin_error::NOT_READY, blockers));
        }
        self.epoch = nonzero(self.epoch.wrapping_add(1));
        self.seq = [0; AXIS_COUNT];
        self.planner.clear();
        self.planner_settled = false;
        self.begin_group(Purpose::ArmConfig, [true; AXIS_COUNT]);
        for i in 0..AXIS_COUNT {
            self.send_config(i, Purpose::ArmConfig, now);
        }
        self.phase = Phase::Arming {
            sub: ArmSub::Configuring,
            started: now,
        };
        Ok(None)
    }

    fn cmd_start(
        &mut self,
        source: ActiveSource,
        now: Instant,
    ) -> Result<Option<String>, AdminError> {
        if !self.axis_conflict().is_empty() {
            return Err(AdminError::new(
                admin_error::AXIS_CONFLICT,
                self.blockers(now),
            ));
        }
        match self.phase {
            Phase::Holding {
                start: None,
                settling: None,
            } => {}
            Phase::Holding { .. } => return Err(AdminError::new(admin_error::BUSY, Vec::new())),
            Phase::Fault { .. } => {
                return Err(AdminError::new(admin_error::FAULT_LATCHED, Vec::new()))
            }
            _ => {
                return Err(AdminError::new(
                    admin_error::NOT_HOLDING,
                    vec![format!("rig_state: {}", rig_state_name(self.state()))],
                ))
            }
        }
        let fresh = self.last_pose.as_ref().filter(|p| p.fresh_until > now);
        if source == ActiveSource::Game && fresh.is_none() {
            return Err(AdminError::new(
                admin_error::NO_FRESH_POSE,
                vec!["no_fresh_pose: no fresh game pose in the last tick".into()],
            ));
        }
        self.drive_session = match source {
            ActiveSource::Game => fresh.map(|p| p.session_gen),
            ActiveSource::Center => None,
        };
        self.begin_group(Purpose::Start, [true; AXIS_COUNT]);
        for i in 0..AXIS_COUNT {
            let id = self.take_request_id();
            self.push_msg(
                i,
                HostMsg::Start {
                    request_id: id,
                    session_epoch: self.epoch,
                },
                Purpose::Start,
                now,
            );
        }
        self.phase = Phase::Holding {
            start: Some((source, now)),
            settling: None,
        };
        Ok(None)
    }

    fn cmd_stop(&mut self, now: Instant) -> Option<String> {
        match self.phase {
            Phase::Arming { .. } => {
                self.begin_disarm(now);
                Some("arming aborted: disarming".into())
            }
            Phase::Holding { .. } | Phase::Active { .. } => {
                self.begin_stop(stop_reason::ADMIN, now);
                None
            }
            Phase::Stopping { .. } => Some("already stopping".into()),
            Phase::Idle(_) | Phase::Fault { .. } => Some("not driving".into()),
        }
    }

    fn cmd_disarm(&mut self, now: Instant) -> Result<Option<String>, AdminError> {
        match self.phase {
            Phase::Stopping {
                sub: StopSub::Disarming { .. },
                ..
            } => Ok(Some("already disarming".into())),
            Phase::Arming { .. }
            | Phase::Holding { .. }
            | Phase::Active { .. }
            | Phase::Stopping { .. } => {
                self.begin_disarm(now);
                Ok(None)
            }
            Phase::Fault { .. } => Err(AdminError::new(admin_error::FAULT_LATCHED, Vec::new())),
            Phase::Idle(_) => Ok(Some("not armed".into())),
        }
    }

    fn cmd_fault_reset(&mut self, now: Instant) -> Result<Option<String>, AdminError> {
        match self.phase {
            Phase::Fault { reset: None } => {}
            Phase::Fault { reset: Some(_) } => {
                return Err(AdminError::new(admin_error::RESET_IN_PROGRESS, Vec::new()))
            }
            _ => return Err(AdminError::new(admin_error::NOT_FAULTED, Vec::new())),
        }
        let disconnected: Vec<String> = (0..AXIS_COUNT)
            .filter(|&i| !self.axes[i].connected)
            .map(|i| format!("not_connected: axis{}", i + 1))
            .collect();
        if !disconnected.is_empty() {
            return Err(AdminError::new(
                admin_error::AXIS_DISCONNECTED,
                disconnected,
            ));
        }
        let mut unsettled = Vec::new();
        // The cached STATUS must postdate the HOST_FAULT ACK: an older one may
        // still say DISARMED while the MCU is in FAULT(HOST_FAULT), and the
        // reset would skip FAULT_RESET and clear the latch on stale data.
        for (i, t) in self.axes.iter().enumerate() {
            if t.host_fault_confirmed() {
                continue;
            }
            let why = if t.host_fault_ack_ev.is_none() {
                "HOST_FAULT not acknowledged yet"
            } else {
                "no STATUS since HOST_FAULT ACK yet"
            };
            unsettled.push(format!("axis_not_settled: axis{} {why}", i + 1));
        }
        for (i, t) in self.axes.iter().enumerate() {
            let n = i + 1;
            match (t.status, t.mcu_state()) {
                (None, _) => unsettled.push(format!("no_status: axis{n}")),
                (Some(s), Some(McuState::Fault | McuState::Disarmed)) => {
                    if s.fault_code != FaultCode::None && t.mcu_state() == Some(McuState::Disarmed)
                    {
                        unsettled.push(format!(
                            "axis_fault: axis{n} fault_code {:?} while DISARMED",
                            s.fault_code
                        ));
                    }
                }
                (Some(s), _) => unsettled.push(format!(
                    "axis_not_settled: axis{n} state {} fault_code {:?}",
                    state_name(s.mcu_state),
                    s.fault_code
                )),
            }
        }
        if !unsettled.is_empty() {
            return Err(AdminError::new(admin_error::AXIS_NOT_SETTLED, unsettled));
        }
        let targets: [bool; AXIS_COUNT] =
            std::array::from_fn(|i| self.axes[i].mcu_state() == Some(McuState::Fault));
        for t in &mut self.axes {
            t.reset_rejected = None;
        }
        self.begin_group(Purpose::FaultReset, targets);
        self.reset_ev = self.ev;
        for (i, _) in targets.iter().enumerate().filter(|(_, t)| **t) {
            let id = self.take_request_id();
            self.push_msg(
                i,
                HostMsg::FaultReset { request_id: id },
                Purpose::FaultReset,
                now,
            );
        }
        self.phase = Phase::Fault { reset: Some(now) };
        Ok(None)
    }

    fn cmd_reference(
        &mut self,
        axis: AxisId,
        count: Counts,
        now: Instant,
    ) -> Result<Option<String>, AdminError> {
        let i = axis.index();
        if !matches!(self.phase, Phase::Idle(_)) {
            return Err(AdminError::new(
                admin_error::NOT_DISARMED,
                vec![format!("rig_state: {}", rig_state_name(self.state()))],
            ));
        }
        if !self.axes.get(i).is_some_and(|t| t.connected) {
            return Err(AdminError::new(
                admin_error::AXIS_DISCONNECTED,
                vec![format!("not_connected: axis{}", i + 1)],
            ));
        }
        let powered: Vec<String> = (0..AXIS_COUNT)
            .filter(|&j| {
                self.axes[j].connected && self.axes[j].mcu_state() != Some(McuState::Disarmed)
            })
            .map(|j| format!("mcu_not_disarmed: axis{}", j + 1))
            .collect();
        if !powered.is_empty() {
            return Err(AdminError::new(admin_error::NOT_DISARMED, powered));
        }
        let id = self.take_request_id();
        self.push_msg(
            i,
            HostMsg::SetReference {
                request_id: id,
                reference_count: count,
            },
            Purpose::Reference,
            now,
        );
        Ok(None)
    }

    // ------------------------------------------------------------ transitions

    /// Event- and time-driven transitions that need no planner step.
    fn advance(&mut self, now: Instant) {
        self.expire_pending(now);
        let all_ok = self.axes.iter().all(|t| t.group_ok);
        let all_in = |s: McuState| {
            self.axes
                .iter()
                .all(|t| t.connected && t.mcu_state() == Some(s))
        };
        match self.phase {
            Phase::Idle(_) => {}
            Phase::Arming { sub, started } => {
                if elapsed(now, started) > ms(self.timing.arm_timeout_ms) {
                    self.latch(RigFault::ArmTimeout, Some(format!("{sub:?}")), now);
                } else if sub == ArmSub::Configuring && all_ok {
                    self.begin_group(Purpose::ArmPrepare, [true; AXIS_COUNT]);
                    for i in 0..AXIS_COUNT {
                        let id = self.take_request_id();
                        let m = HostMsg::ArmPrepare {
                            request_id: id,
                            session_epoch: self.epoch,
                        };
                        self.push_msg(i, m, Purpose::ArmPrepare, now);
                    }
                    self.phase = Phase::Arming {
                        sub: ArmSub::Preparing,
                        started,
                    };
                } else if sub == ArmSub::Preparing && all_ok && all_in(McuState::Holding) {
                    let actual = self.actual_counts();
                    self.planner.init(actual);
                    self.group = None;
                    self.phase = Phase::Holding {
                        start: None,
                        settling: None,
                    };
                }
            }
            Phase::Holding {
                start: Some((source, since)),
                ..
            } => {
                if all_ok && all_in(McuState::Active) {
                    self.group = None;
                    self.phase = Phase::Active {
                        source,
                        entered: now,
                    };
                } else if elapsed(now, since) > ms(self.timing.arm_timeout_ms) {
                    let (axis, actual) = self.first_axis_not_in(McuState::Active);
                    self.latch(
                        RigFault::AxisStateMismatch {
                            axis,
                            expected: McuState::Active,
                            actual,
                        },
                        Some("START not confirmed in time".into()),
                        now,
                    );
                }
            }
            Phase::Holding {
                start: None,
                settling: Some(since),
            } => {
                if all_ok && all_in(McuState::Holding) {
                    self.group = None;
                    self.phase = Phase::Holding {
                        start: None,
                        settling: None,
                    };
                } else if elapsed(now, since) > ms(self.timing.stop_settle_timeout_ms) {
                    self.latch(
                        RigFault::StopTimeout,
                        Some("center stop did not settle".into()),
                        now,
                    );
                }
            }
            Phase::Holding { .. } | Phase::Active { .. } => {}
            Phase::Stopping { sub, started } => match sub {
                StopSub::Decel => {
                    if self.planner_settled && all_ok && all_in(McuState::Holding) {
                        self.phase = Phase::Stopping {
                            sub: StopSub::Hold { since: now },
                            started,
                        };
                    } else if elapsed(now, started) > ms(self.timing.stop_settle_timeout_ms) {
                        self.latch(
                            RigFault::StopTimeout,
                            Some("deceleration did not settle".into()),
                            now,
                        );
                    }
                }
                StopSub::Hold { since } => {
                    if elapsed(now, since) >= ms(self.timing.stop_hold_ms) {
                        self.send_group_disarm(now);
                        self.phase = Phase::Stopping {
                            sub: StopSub::Disarming { since: now },
                            started,
                        };
                    }
                }
                StopSub::Disarming { since } => {
                    if all_ok && all_in(McuState::Disarmed) {
                        self.planner.clear();
                        self.planner_settled = false;
                        self.group = None;
                        self.drive.limits.clear();
                        self.phase = Phase::Idle(RigState::BrakedDisarmed);
                    } else if elapsed(now, since) > ms(self.timing.stop_settle_timeout_ms) {
                        self.latch(
                            RigFault::StopTimeout,
                            Some("DISARMED not reached in time".into()),
                            now,
                        );
                    }
                }
            },
            Phase::Fault { reset: Some(since) } => {
                // Every FAULT_RESET ACKed and a STATUS newer than the reset.
                let clear = all_ok
                    && self.axes.iter().all(|t| {
                        t.connected
                            && t.status_ev > self.reset_ev
                            && t.mcu_state() == Some(McuState::Disarmed)
                            && t.status.is_some_and(|s| s.fault_code == FaultCode::None)
                    });
                if clear {
                    self.event(
                        now,
                        "fault_cleared",
                        serde_json::json!({"fault": self.fault}),
                    );
                    self.fault = None;
                    self.group = None;
                    self.planner.clear();
                    for t in &mut self.axes {
                        t.host_fault_req = None;
                        t.host_fault_ack_ev = None;
                    }
                    self.phase = Phase::Idle(RigState::Offline);
                } else if elapsed(now, since) > FAULT_RESET_TIMEOUT {
                    self.event(now, "fault_reset_timeout", serde_json::json!({}));
                    self.phase = Phase::Fault { reset: None };
                }
            }
            Phase::Fault { reset: None } => {}
        }
        if matches!(self.phase, Phase::Idle(_) | Phase::Fault { .. }) {
            self.reconcile_claimants(now);
            self.reconcile_config(now);
        }
        self.classify(now);
        self.note_state(now);
    }

    /// Idle sub-state from the blockers. Never arms.
    fn classify(&mut self, now: Instant) {
        let Phase::Idle(cur) = self.phase else { return };
        let next = if self.axes.iter().any(|t| !t.connected) {
            RigState::Offline
        } else if !self.blockers(now).is_empty() {
            RigState::CalibrationRequired
        } else if cur == RigState::BrakedDisarmed {
            RigState::BrakedDisarmed
        } else {
            RigState::ReadyToArm
        };
        self.phase = Phase::Idle(next);
    }

    /// Re-send CONFIG to DISARMED axes that lack it (after reconnect, reboot,
    /// fault reset or an earlier rejection).
    fn reconcile_config(&mut self, now: Instant) {
        for i in 0..AXIS_COUNT {
            let t = &self.axes[i];
            let due = t.connected
                && t.mcu_state() == Some(McuState::Disarmed)
                && t.status_fault().is_none()
                && t.host_fault_confirmed()
                && !t.configured(self.epoch)
                && t.config_retry_at.is_none_or(|at| now >= at)
                && !self
                    .pending
                    .values()
                    .any(|p| p.axis == i && p.msg_type == protocol::msg_type::CONFIG);
            if due {
                self.send_config(i, Purpose::Sync, now);
                self.axes[i].config_retry_at = Some(now + CONFIG_RETRY);
            }
        }
    }

    fn begin_stop(&mut self, reason: u8, now: Instant) {
        self.event(now, "stopping", serde_json::json!({"reason": reason}));
        self.send_group_stop(reason, now);
        self.planner_settled = false;
        self.phase = Phase::Stopping {
            sub: StopSub::Decel,
            started: now,
        };
    }

    fn begin_disarm(&mut self, now: Instant) {
        self.send_group_disarm(now);
        self.phase = Phase::Stopping {
            sub: StopSub::Disarming { since: now },
            started: now,
        };
    }

    fn send_group_stop(&mut self, reason: u8, now: Instant) {
        self.begin_group(Purpose::Stop, [true; AXIS_COUNT]);
        for i in 0..AXIS_COUNT {
            let id = self.take_request_id();
            let m = HostMsg::ControlledStop {
                request_id: id,
                session_epoch: self.epoch,
                reason,
            };
            self.push_msg(i, m, Purpose::Stop, now);
        }
    }

    fn send_group_disarm(&mut self, now: Instant) {
        self.begin_group(Purpose::Disarm, [true; AXIS_COUNT]);
        for i in 0..AXIS_COUNT {
            let id = self.take_request_id();
            self.push_msg(i, HostMsg::Disarm { request_id: id }, Purpose::Disarm, now);
        }
    }

    // ------------------------------------------------------------ faults

    fn latch_axis_fault(&mut self, axis: AxisId, code: FaultCode, source: &str, now: Instant) {
        let latched = matches!(self.phase, Phase::Fault { .. });
        // HOST_FAULT on an axis is the echo of our own latch; already recorded.
        if latched && code == FaultCode::HostFault {
            return;
        }
        let fault = RigFault::AxisFault { axis, code };
        if latched && self.fault_history.iter().any(|r| r.fault == fault) {
            return;
        }
        self.latch(fault, Some(format!("reported in {source}")), now);
    }

    /// Latch FAULT_LATCHED. The first latch of an episode sends HOST_FAULT to
    /// every connected axis; later causes only go to the history.
    fn latch(&mut self, fault: RigFault, detail: Option<String>, now: Instant) {
        let at_ms = self.at_ms(now);
        if self.fault_history.len() >= FAULT_HISTORY_LEN {
            self.fault_history.pop_front();
        }
        self.fault_history.push_back(FaultRecord {
            at_ms,
            fault: fault.clone(),
            detail: detail.clone(),
        });
        self.event(
            now,
            "fault",
            serde_json::json!({"fault": fault, "detail": detail}),
        );
        if let Phase::Fault { .. } = self.phase {
            return;
        }
        self.fault = Some(fault);
        self.drive.limits.clear();
        self.phase = Phase::Fault { reset: None };
        self.group = None;
        self.planner.clear();
        self.planner_settled = false;
        for i in 0..AXIS_COUNT {
            self.axes[i].host_fault_req = None;
            self.axes[i].host_fault_ack_ev = None;
            if self.axes[i].connected {
                self.send_host_fault(i, now);
            }
        }
        self.note_state(now);
    }

    /// HOST_FAULT to one axis (idempotent on the MCU); FaultReset waits for a
    /// STATUS after its ACK.
    fn send_host_fault(&mut self, i: usize, now: Instant) {
        if !self.axes.get(i).is_some_and(|t| t.connected) {
            return;
        }
        let id = self.take_request_id();
        let m = HostMsg::HostFault {
            request_id: id,
            fault_code: FaultCode::HostFault,
        };
        self.push_msg(i, m, Purpose::HostFault, now);
        self.axes[i].host_fault_req = Some(id);
        self.axes[i].host_fault_ack_ev = None;
    }

    /// While driving: fault on an MCU state the current phase does not allow.
    fn check_axis_state(&mut self, i: usize, now: Instant) {
        use McuState::*;
        let Some(axis) = AxisId::from_index(i) else {
            return;
        };
        let t = &self.axes[i];
        let Some(raw) = t.status.map(|s| s.mcu_state) else {
            return;
        };
        let actual = McuState::from_u8(raw);
        let (expected, allowed): (McuState, &[McuState]) = match self.phase {
            Phase::Arming {
                sub: ArmSub::Configuring,
                ..
            } => (Holding, &[Disarmed]),
            Phase::Arming {
                sub: ArmSub::Preparing,
                ..
            } => {
                if t.group_ok {
                    (Holding, &[Arming, Holding])
                } else {
                    (Holding, &[Disarmed, Arming, Holding])
                }
            }
            Phase::Holding { start: Some(_), .. } => (Active, &[Holding, Active]),
            Phase::Holding {
                settling: Some(_), ..
            } => (Holding, &[Active, Stopping, Holding]),
            Phase::Holding { .. } => (Holding, &[Holding]),
            Phase::Active { .. } => (Active, &[Active]),
            Phase::Stopping {
                sub: StopSub::Decel,
                ..
            } => (Holding, &[Active, Stopping, Holding]),
            Phase::Stopping {
                sub: StopSub::Hold { .. },
                ..
            } => (Holding, &[Holding]),
            Phase::Stopping {
                sub: StopSub::Disarming { .. },
                ..
            } => (
                Disarmed,
                &[Arming, Holding, Active, Stopping, Disarming, Disarmed],
            ),
            Phase::Idle(_) | Phase::Fault { .. } => return,
        };
        if actual.is_some_and(|a| allowed.contains(&a)) {
            return;
        }
        let actual = actual.unwrap_or(Boot);
        self.latch(
            RigFault::AxisStateMismatch {
                axis,
                expected,
                actual,
            },
            Some(format!("raw mcu_state {raw} in {:?}", self.phase)),
            now,
        );
    }

    fn check_status_age(&mut self, now: Instant) {
        if !self.is_armed() {
            return;
        }
        for i in 0..AXIS_COUNT {
            let Some(axis) = AxisId::from_index(i) else {
                continue;
            };
            if let Some(age) = self
                .status_age(i, now)
                .filter(|a| *a > self.status_timeout())
            {
                let detail = format!("no STATUS for {} ms", age.as_millis());
                self.latch(RigFault::AxisDisconnected { axis }, Some(detail), now);
                return;
            }
        }
    }

    fn check_frame_skew(&mut self, now: Instant) {
        let Phase::Active { entered, .. } = self.phase else {
            return;
        };
        // Each axis's lag is taken when its STATUS came in, so a late or
        // missing STATUS does not read as skew (check_status_age covers that).
        let lags: Vec<i64> = self.axes.iter().filter_map(AxisTrack::frame_lag).collect();
        if lags.len() < AXIS_COUNT {
            return;
        }
        let spread = lags.iter().max().copied().unwrap_or(0) - lags.iter().min().copied().unwrap_or(0);
        let spread = u32::try_from(spread).unwrap_or(u32::MAX);
        if elapsed(now, entered) <= FRAME_SKEW_GRACE {
            return;
        }
        self.last_frame_skew = Some(spread);
        if spread > self.timing.max_frame_skew {
            let frames: Vec<Option<u32>> = self
                .axes
                .iter()
                .map(|t| t.status.map(|s| s.last_applied_frame))
                .collect();
            self.latch(
                RigFault::FrameSkew {
                    max_skew_frames: spread,
                },
                Some(format!("last_applied_frame {frames:?}, lag {lags:?}")),
                now,
            );
        }
    }

    // ------------------------------------------------------------ drive

    /// pose → mix → saturate → virtual_to_count → clamp_soft → step_track.
    fn track_game(&mut self, pose: &AcceptedPose, dt: f64) -> Vec<LimitReason> {
        let checked = &pose.checked;
        let mixed = compat::mix(checked.pose, checked.effective_gain);
        let (virt, saturated) =
            compat::saturate(mixed, self.game.virtual_min, self.game.virtual_max);
        let mut limits = Vec::new();
        if saturated {
            limits.push(LimitReason::VirtualSaturated);
        }
        let mut soft = false;
        let targets: [f64; AXIS_COUNT] = std::array::from_fn(|i| {
            let (c, clamped) = self.cal[i].clamp_soft(self.cal[i].virtual_to_count_f(virt[i]));
            soft |= clamped;
            c
        });
        if soft {
            limits.push(LimitReason::SoftLimited);
        }
        let out = self.planner.step_track(targets, self.axis_limits(1.0), dt);
        limits.extend(out.limits);
        self.drive.virtual_targets = virt.map(|v| v.0);
        limits
    }

    fn axis_limits(&self, fraction: f64) -> [AxisLimits; AXIS_COUNT] {
        self.cal.each_ref().map(|c| AxisLimits {
            max_vel_count_s: f64::from(c.max_vel_count_s) * fraction,
            max_acc_count_s2: f64::from(c.max_acc_count_s2) * fraction,
        })
    }

    fn build_set_position(&mut self, drive_limits: &[LimitReason]) -> [SetPosition; AXIS_COUNT] {
        self.group_frame_id = nonzero(self.group_frame_id.wrapping_add(1));
        let fraction = match self.phase {
            Phase::Active {
                source: ActiveSource::Center,
                ..
            } => self.game.center_speed_fraction,
            _ => 1.0,
        };
        let limits = self.axis_limits(fraction);
        let planned = self
            .planner
            .is_initialized()
            .then(|| self.planner.command_counts());
        let actual = self.actual_counts();
        let targets: [Counts; AXIS_COUNT] = std::array::from_fn(|i| {
            let c = &self.cal[i];
            planned
                .map_or(actual[i], |p| p[i])
                .clamp(c.soft_min_count.min(c.soft_max_count), c.soft_max_count)
        });
        let out = std::array::from_fn(|i| {
            self.seq[i] = nonzero(self.seq[i].wrapping_add(1));
            SetPosition {
                session_epoch: self.epoch,
                seq: self.seq[i],
                group_frame_id: self.group_frame_id,
                target_count: targets[i],
                max_vel_count_s: to_u32_limit(limits[i].max_vel_count_s),
                max_acc_count_s2: to_u32_limit(limits[i].max_acc_count_s2),
                valid_for_ms: self.timing.set_position_valid_for_ms,
                apply_at_us: 0,
            }
        });
        if !matches!(
            self.phase,
            Phase::Active {
                source: ActiveSource::Game,
                ..
            }
        ) {
            self.drive.virtual_targets =
                std::array::from_fn(|i| self.cal[i].count_to_virtual(f64::from(targets[i])).0);
        }
        self.drive.target_counts = targets;
        self.drive.limits = drive_limits.to_vec();
        out
    }

    // ------------------------------------------------------------ helpers

    fn send_config(&mut self, i: usize, purpose: Purpose, now: Instant) {
        let Some(axis) = AxisId::from_index(i) else {
            return;
        };
        let id = self.take_request_id();
        let c = &self.cal[i];
        let msg = HostMsg::Config(protocol::Config {
            request_id: id,
            session_epoch: self.epoch,
            calibration_hash: self.hashes[i],
            soft_min_count: c.soft_min_count,
            soft_max_count: c.soft_max_count,
            max_vel_count_s: c.max_vel_count_s,
            max_acc_count_s2: c.max_acc_count_s2,
            following_error_limit_count: c.following_error_limit_count,
            overspeed_count_s: c.overspeed_count_s,
            command_timeout_ms: self.timing.mcu_command_timeout_ms,
            stop_timeout_ms: self.timing.mcu_stop_timeout_ms,
            role_flags: if axis == self.power_axis {
                role_flags::POWER_CONTROLLER
            } else {
                0
            },
        });
        self.push_msg(i, msg, purpose, now);
    }

    /// Start a group command: `targets[i]` axes must ACK, others count as done.
    fn begin_group(&mut self, purpose: Purpose, targets: [bool; AXIS_COUNT]) {
        self.group_gen += 1;
        self.group = Some((purpose, self.group_gen));
        for (t, target) in self.axes.iter_mut().zip(targets) {
            t.group_ok = !target;
        }
    }

    fn push_msg(&mut self, i: usize, msg: HostMsg, purpose: Purpose, now: Instant) {
        if !self.axes.get(i).is_some_and(|t| t.connected) {
            return;
        }
        if let Some(id) = request_id_of(&msg) {
            if self.pending.len() >= MAX_PENDING_ACKS {
                if let Some(oldest) = self
                    .pending
                    .iter()
                    .min_by_key(|(_, p)| p.sent_at)
                    .map(|(k, _)| *k)
                {
                    self.pending.remove(&oldest);
                }
            }
            self.pending.insert(
                id,
                Pending {
                    axis: i,
                    msg_type: msg.msg_type(),
                    purpose,
                    sent_at: now,
                    epoch: self.epoch,
                    gen: self.group_gen,
                },
            );
        }
        if let Some(q) = self.outbox.get_mut(i) {
            q.push(msg);
        }
    }

    fn take_request_id(&mut self) -> u32 {
        let id = self.next_request_id;
        self.next_request_id = nonzero(id.wrapping_add(1));
        id
    }

    fn expire_pending(&mut self, now: Instant) {
        let expired: Vec<u32> = self
            .pending
            .iter()
            .filter(|(_, p)| elapsed(now, p.sent_at) > ACK_TIMEOUT)
            .map(|(k, _)| *k)
            .collect();
        for id in expired {
            if let Some(p) = self.pending.remove(&id) {
                let detail = serde_json::json!({"axis": p.axis + 1, "msg": msg_name(p.msg_type), "request_id": id});
                self.event(now, "ack_timeout", detail);
                // Re-send an unanswered HOST_FAULT while latched — except to a
                // conflicted axis, whose primary is demoted instead (one
                // attempt; see `reconcile_claimants`).
                let unconfirmed = self
                    .axes
                    .get(p.axis)
                    .is_some_and(|t| t.host_fault_req == Some(id) && !t.in_conflict());
                if unconfirmed && matches!(self.phase, Phase::Fault { .. }) {
                    self.send_host_fault(p.axis, now);
                }
            }
        }
    }

    fn actual_counts(&self) -> [Counts; AXIS_COUNT] {
        std::array::from_fn(|i| {
            self.axes[i]
                .status
                .map_or(self.cal[i].neutral_count(), |s| s.actual_count)
        })
    }

    fn first_axis_not_in(&self, s: McuState) -> (AxisId, McuState) {
        for (i, t) in self.axes.iter().enumerate() {
            let st = t.mcu_state();
            if st != Some(s) {
                return (
                    AxisId::from_index(i).unwrap_or(AxisId::ALL[0]),
                    st.unwrap_or(McuState::Boot),
                );
            }
        }
        (AxisId::ALL[0], s)
    }

    fn axis_calibration_valid(&self, i: usize) -> bool {
        self.operator_valid[i] && self.cal_error[i].is_none() && self.axes[i].referenced()
    }

    fn status_timeout(&self) -> Duration {
        ms(self.timing.axis_status_timeout_ms)
    }

    /// Time since the last STATUS (or since identification if none yet).
    fn status_age(&self, i: usize, now: Instant) -> Option<Duration> {
        let t = self.axes.get(i)?;
        if !t.connected {
            return None;
        }
        t.status_at
            .or(t.identified_at)
            .map(|at| now.saturating_duration_since(at))
    }

    fn touch(&mut self, now: Instant) {
        self.t0.get_or_insert(now);
    }

    fn at_ms(&self, now: Instant) -> f64 {
        self.t0.map_or(0.0, |t0| {
            now.saturating_duration_since(t0).as_secs_f64() * 1000.0
        })
    }

    fn event(&mut self, at: Instant, kind: &'static str, detail: serde_json::Value) {
        if self.events.len() >= EVENT_QUEUE_LEN {
            self.events.pop_front();
            self.events_dropped = self.events_dropped.saturating_add(1);
        }
        self.events.push_back(SupervisorEvent { at, kind, detail });
    }

    fn note_state(&mut self, now: Instant) {
        let s = self.state();
        if s != self.last_state {
            let from = self.last_state;
            self.last_state = s;
            self.event(now, "rig_state", serde_json::json!({"from": from, "to": s}));
        }
    }
}

fn nonzero(v: u32) -> u32 {
    if v == 0 {
        1
    } else {
        v
    }
}

fn ms(v: u64) -> Duration {
    Duration::from_millis(v)
}

fn elapsed(now: Instant, since: Instant) -> Duration {
    now.saturating_duration_since(since)
}

fn to_u32_limit(v: f64) -> u32 {
    if v.is_finite() && v >= 1.0 {
        v.round().min(f64::from(u32::MAX)) as u32
    } else {
        1
    }
}

fn request_id_of(m: &HostMsg) -> Option<u32> {
    match *m {
        HostMsg::HelloReq { request_id }
        | HostMsg::ArmPrepare { request_id, .. }
        | HostMsg::Start { request_id, .. }
        | HostMsg::ControlledStop { request_id, .. }
        | HostMsg::Disarm { request_id }
        | HostMsg::FaultReset { request_id }
        | HostMsg::ClockSync { request_id, .. }
        | HostMsg::HostFault { request_id, .. }
        | HostMsg::SetReference { request_id, .. } => Some(request_id),
        HostMsg::Config(c) => Some(c.request_id),
        HostMsg::SetPosition(_) => None,
    }
}

fn msg_name(t: u8) -> &'static str {
    use protocol::msg_type::*;
    match t {
        HELLO_REQ => "HELLO_REQ",
        CONFIG => "CONFIG",
        ARM_PREPARE => "ARM_PREPARE",
        START => "START",
        SET_POSITION => "SET_POSITION",
        CONTROLLED_STOP => "CONTROLLED_STOP",
        DISARM => "DISARM",
        FAULT_RESET => "FAULT_RESET",
        CLOCK_SYNC => "CLOCK_SYNC",
        HOST_FAULT => "HOST_FAULT",
        SET_REFERENCE => "SET_REFERENCE",
        _ => "UNKNOWN",
    }
}

/// Wire (serde) name of a rig state, e.g. `HOLDING`.
fn rig_state_name(s: RigState) -> String {
    match serde_json::to_value(s) {
        Ok(serde_json::Value::String(n)) => n,
        _ => format!("{s:?}"),
    }
}

fn state_name(raw: u8) -> String {
    McuState::from_u8(raw).map_or_else(|| format!("raw {raw}"), |s| format!("{s:?}").to_uppercase())
}
