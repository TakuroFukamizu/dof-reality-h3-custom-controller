//! MCU-side reference model of USB protocol v1 (DESIGN.md §6).
//!
//! [`SimMcu`] is a pure, time-injected model of one CoreS3 axis controller. The
//! CoreS3 firmware treats it as the behavioral reference for
//! `docs/control-server/protocol.md` (§3–§6). [`spawn_sim_axis`] runs it behind a
//! COBS byte stream so the server can use it in place of a USB port.
//!
//! # State machine (protocol.md §5)
//!
//! ```text
//! (boot) → DISARMED                       BOOT is not observable: new()/reboot() land in DISARMED
//! DISARMED --ARM_PREPARE--> ARMING        power on (power-controller axis only), brake still applied
//! ARMING   --arming_delay/2--> (position control on current position, brake release cmd)
//! ARMING   --arming_delay--> HOLDING      hold target = actual position at control start
//! HOLDING  --START--> ACTIVE              target stays = hold until the next SET_POSITION
//! ACTIVE   --CONTROLLED_STOP / valid_for expiry--> STOPPING --(setpoint velocity 0)--> HOLDING
//! HOLDING / ARMING --DISARM--> DISARMING --disarming_delay--> DISARMED
//! ACTIVE / STOPPING --DISARM--> STOPPING(disarm) --(velocity 0)--> DISARMING → DISARMED
//! any --HOST_FAULT / fault check / force_fault--> FAULT --FAULT_RESET(no cause)--> DISARMED
//! ```
//!
//! # Command gating and ACK reasons (protocol.md §3.3, §4)
//!
//! HOST_FAULT bypasses everything below: it always latches FAULT and is ACKed
//! ACCEPTED (any request_id, 0 included; never DUPLICATE, never cached).
//! Every other ACKed command goes through the same pipeline, in this order:
//! 1. `request_id == 0` → `REJECTED/INVALID_PARAM` (0 is reserved).
//! 2. Resend: the identical message (same type, request_id and payload) is
//!    among the last [`DEDUPE_DEPTH`] ACCEPTED commands → `DUPLICATE/NONE`, no
//!    re-execution. Only accepted commands are remembered, so DUPLICATE always
//!    means "already accepted"; a resent rejected command is evaluated again.
//!    The same request_id with different content is a new command: it is
//!    executed and replaces the old entry.
//! 3. Fault gate: while a fault is active (`STATUS.fault_code != NONE`, i.e.
//!    FAULT or a fault-stop deceleration) every command except FAULT_RESET is
//!    `REJECTED/FAULT_ACTIVE`. HELLO_REQ / CLOCK_SYNC always answer.
//! 4. Per-command checks:
//!
//! | command | accepted when | otherwise |
//! | --- | --- | --- |
//! | CONFIG | DISARMED and params valid | BAD_STATE; INVALID_PARAM |
//! | ARM_PREPARE | DISARMED, CONFIGURED, REFERENCED, epoch match, actual inside `[soft_min, soft_max]` | BAD_STATE, NOT_CONFIGURED, NOT_REFERENCED, EPOCH_MISMATCH, BAD_STATE (that order) |
//! | START | HOLDING, epoch match, an unexpired SET_POSITION (`valid_for_ms`) received since ARM_PREPARE | BAD_STATE, EPOCH_MISMATCH, BAD_STATE |
//! | CONTROLLED_STOP | epoch match; ACTIVE → STOPPING, any other state = no-op | EPOCH_MISMATCH |
//! | DISARM | any non-fault state (DISARMED / DISARMING = no-op) | — |
//! | FAULT_RESET | FAULT and no cause present | BAD_STATE, FAULT_CAUSE_PRESENT |
//! | SET_REFERENCE | DISARMED | BAD_STATE |
//!
//! CONFIG validation (INVALID_PARAM): `soft_min < soft_max`, and max_vel,
//! max_acc, following_error_limit, overspeed, command_timeout_ms and
//! stop_timeout_ms all > 0.
//!
//! # SET_POSITION (protocol.md §4 "SET_POSITION の規則")
//!
//! - No CONFIG yet, or `session_epoch` mismatch, or `seq <= last_rx_seq` →
//!   dropped, `stale_drop_count += 1`.
//! - `apply_at_us != 0` → dropped and counted in `rx_error_count`, lease not
//!   refreshed (v1 has no clock sync, so a timed apply cannot be honored).
//! - `seq` is monotonic within one epoch: an accepted CONFIG whose epoch differs
//!   from the stored one resets `last_rx_seq` to 0; a re-CONFIG with the same
//!   epoch keeps it so late frames cannot replay (hence the PC must never reuse
//!   an epoch across server restarts).
//! - Otherwise the frame is "valid": `last_rx_seq = seq`, the lease
//!   (`command_timeout_ms`) and the target validity (`valid_for_ms`) restart
//!   from the receive time.
//! - Only in ACTIVE is the frame applied: control target =
//!   `clamp(target_count, soft_min, soft_max)`, frame limits =
//!   `min(frame, CONFIG)`, `last_applied_frame/seq` updated. In every other
//!   state the target is ignored (HOLDING keeps the hold target).
//!
//! # Deadlines (protocol.md §3.2, §6)
//!
//! - Lease: in ARMING / HOLDING / ACTIVE / STOPPING, no valid SET_POSITION for
//!   more than `command_timeout_ms` (counted from ARM_PREPARE acceptance if none
//!   arrived since) → COMMAND_TIMEOUT. If the setpoint is moving the axis first
//!   decelerates: during that window STATUS reads `mcu_state = STOPPING`,
//!   `fault_code = COMMAND_TIMEOUT`; when the setpoint velocity reaches 0 (or
//!   `stop_timeout_ms` elapses) the axis latches FAULT and FAULT_EVENT is sent
//!   (with `mcu_state_before = STOPPING`). At rest it latches immediately.
//!   A DISARM-requested deceleration and a fault-stop deceleration are exempt
//!   from the lease (the host has asked for power-down / a fault is pending).
//! - `valid_for_ms` expiry in ACTIVE → STOPPING → HOLDING (a new START is needed
//!   to move again; late frames never resume motion by themselves).
//! - STOPPING longer than `stop_timeout_ms` → FAULT(STOP_TIMEOUT).
//!
//! # Fault checks (while position control is on)
//!
//! FOLLOWING_ERROR `|actual − setpoint| > following_error_limit_count`,
//! OVERSPEED `|actual velocity| > overspeed_count_s`, SOFT_LIMIT actual outside
//! `[soft_min, soft_max]`. These latch FAULT immediately. In FAULT: PWM 0, brake
//! applied (BRAKE_RELEASE_CMD = 0), main power command off. HOST_FAULT always
//! latches with `fault_code = HOST_FAULT` (the code in the message is kept in
//! [`SimMcu::last_host_fault_code`] only), keeping an already latched/pending code.
//! ARMING checks the soft window once more before releasing the brake (the
//! position could have moved since ARM_PREPARE) and latches SOFT_LIMIT with the
//! brake still applied, so the brake output never pulses.
//! FAULT_RESET keeps CONFIG and REFERENCE, except that ENCODER_FAULT clears
//! POSITION_REFERENCED. A position still outside the soft window is not a
//! FAULT_RESET "cause": reset is allowed (DISARMED is as safe as FAULT and is
//! the only state where SET_REFERENCE / CONFIG can fix it), and ARM_PREPARE is
//! rejected until it is fixed. CONFIG_INVALID / NOT_REFERENCED are reserved
//! codes, never latched: ARM_PREPARE is rejected with an ACK reason instead.
//!
//! # Axis selection (protocol.md §1.1)
//!
//! The operator selects the axis on the real unit (power button, NVS). Here it
//! is `SimParams::selected_axis`, changed at run time with
//! [`SimMcu::select_axis`]: accepted in DISARMED only; it clears CONFIG
//! (`CONFIGURED`, `calibration_hash`) and `POSITION_REFERENCED`, sends one
//! AXIS_SELECT and puts the new value in every later HELLO / STATUS. The
//! encoder count is untouched. [`SimMcu::select_axis_unchecked`] breaks the
//! DISARMED rule on purpose (tests of the PC side's reaction); outside DISARMED
//! the limits and the lease are kept, so the unit stops by COMMAND_TIMEOUT
//! (the PC never writes to a conflicted axis). `reboot` keeps
//! the selection (NVS).
//!
//! # End-stop calibration (protocol.md §1.2, DESIGN.md §11)
//!
//! The simulated mechanism has end stops at `SimParams::end_min_count` /
//! `end_max_count` (in the unit's current count coordinate; they move with
//! every reference change so the mechanism never moves by a SET_REFERENCE).
//! The plant can never pass them: the position clamps and the velocity reads 0
//! while pushing against one. CALIBRATE(1), DISARMED only, runs:
//! main power on (role bit) → settle (`arming_delay_ms / 2`) → brake release
//! → wait (`arming_delay_ms / 2`) → move to the **P end** → move to the N end
//! → move to the neutral (midpoint) → shutdown (brake applied,
//! `disarming_delay_ms`) → CAL_RESULT → STATUS(DISARMED). The P end of the
//! simulator is the end at the **minimal count** (`end_min_count`), i.e. the
//! unit has sign +1: `n_end_count = end_max − end_min > 0`, `neutral_count =
//! n_end_count / 2`, `sign = 1`, `reason = "ok"`, `step = 3`. On success the
//! unit re-references itself so that the P end reads 0 and sets
//! POSITION_REFERENCED. CALIBRATE(0) / DISARM cancel (`CANCELLED`, reason
//! `stop`), the injected failure (`set_calibration_failure`) and a stalled
//! plant (`set_stall`) fail (`FAILED`, reason `stall` / the injected one); both
//! clear POSITION_REFERENCED and end in DISARMED. HOST_FAULT (and
//! `force_fault`) during a calibration sends CAL_RESULT(`FAILED`, reason
//! `fault`) and then latches FAULT as usual. SET_POSITION is dropped without
//! being counted while CALIBRATING; CONFIG / ARM_PREPARE / START /
//! SET_REFERENCE / FAULT_RESET are `BAD_STATE`; CONTROLLED_STOP is the usual
//! no-op. No motion fault check runs during a calibration (the mechanism goes
//! past the soft window to the mechanical ends by design).
//!
//! # STATUS
//!
//! Sent every `status_period_ms` (from [`SimMcu::tick`]) and immediately whenever
//! `(mcu_state, fault_code, flags, selected_axis)` changes. `target_count` is the current
//! (profiled) control setpoint; `following_error_count = actual − setpoint`;
//! `pwm_permille` is a proportional + feed-forward estimate; `current_ma = 0`
//! with CURRENT_VALID unset. Counters saturate at `u16::MAX`.
//!
//! # Plant
//!
//! The setpoint moves toward the control target under the velocity/acceleration
//! limits (stopping-distance limited, no overshoot); the actual position follows
//! the setpoint at up to `plant_max_vel_count_s`, or not at all while stalled.
//! The model advances in 1 ms substeps, and jumps straight to `now` once it is
//! in DISARMED / FAULT (nothing moves with the brake applied).

use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;

use tokio::io::{AsyncReadExt, AsyncWriteExt, DuplexStream};
use tokio::sync::{mpsc, oneshot};
use tokio::task::JoinHandle;
use tokio::time::{Instant, MissedTickBehavior};

use crate::config::SimBackendConfig;
use crate::protocol::cobs::{self, FrameAccumulator};
use crate::protocol::{
    self, calibrate_mode, capabilities, frame, fw_absolute, msg_type, role_flags, Ack, AckReason,
    AckResult, CalResult, CalResultCode, ClockSyncReply, Config, FaultEvent, Hello, HostMsg,
    Limits, McuMsg, SetLimits, SetPosition, Status, CAL_REASON_LEN, MAX_ENCODED_LEN,
    PROTOCOL_VERSION,
};
use crate::types::{status_flags, Counts, DeviceId, FaultCode, McuState};

/// Number of recent request_ids remembered for DUPLICATE detection.
pub const DEDUPE_DEPTH: usize = 16;
/// Internal model step.
const SUBSTEP_US: u64 = 1_000;
/// PWM estimate: permille per count of following error.
const PWM_KP_PERMILLE_PER_COUNT: f64 = 5.0;

/// Parameters of one simulated axis MCU.
#[derive(Debug, Clone, PartialEq)]
pub struct SimParams {
    pub device_id: DeviceId,
    pub boot_id: u32,
    pub fw_version: [u8; 3],
    /// Axis selected on the MCU at boot (HELLO / STATUS `selected_axis`;
    /// 0 = none, the server then falls back to the configured `device_id`).
    pub selected_axis: u8,
    /// Establish POSITION_REFERENCED at boot (bench convenience; real firmware
    /// boots unreferenced). Re-applied by [`SimMcu::reboot`].
    pub start_referenced: bool,
    /// Max velocity the simulated motor can reach (count/s).
    pub plant_max_vel_count_s: f64,
    pub status_period_ms: u64,
    /// ARMING duration; brake release is commanded at half of it (power settle).
    pub arming_delay_ms: u64,
    /// DISARMING duration (brake apply wait) before PWM 0 / power off.
    pub disarming_delay_ms: u64,
    /// Encoder count at first boot.
    pub initial_count: Counts,
    /// One-way link latency applied by the async runner (both directions).
    pub link_latency_ms: u64,
    /// Mechanical end stops of the simulated mechanism in the boot coordinate
    /// (the plant cannot pass them). `end_min_count` is the calibration's P end.
    pub end_min_count: Counts,
    pub end_max_count: Counts,
}

impl Default for SimParams {
    fn default() -> Self {
        Self {
            device_id: DeviceId(0x5131_0000_0000_0001),
            boot_id: 1,
            fw_version: [0, 1, 0],
            selected_axis: 0,
            start_referenced: true,
            plant_max_vel_count_s: 4000.0,
            status_period_ms: 20,
            arming_delay_ms: 50,
            disarming_delay_ms: 30,
            initial_count: 0,
            link_latency_ms: 0,
            end_min_count: -3000,
            end_max_count: 3000,
        }
    }
}

impl SimParams {
    /// Parameters for one axis of the `[backend] kind = "sim"` configuration.
    pub fn from_backend(
        cfg: &SimBackendConfig,
        device_id: DeviceId,
        boot_id: u32,
        selected_axis: u8,
    ) -> Self {
        Self {
            device_id,
            boot_id,
            selected_axis,
            start_referenced: cfg.start_referenced,
            plant_max_vel_count_s: cfg.plant_max_vel_count_s,
            status_period_ms: cfg.status_period_ms,
            link_latency_ms: cfg.link_latency_ms,
            end_min_count: cfg.end_min_count,
            end_max_count: cfg.end_max_count,
            ..Self::default()
        }
    }
}

/// How a STOPPING deceleration ends.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum StopKind {
    /// CONTROLLED_STOP / valid_for expiry → HOLDING.
    Hold,
    /// DISARM from a moving state → DISARMING.
    Disarm,
    /// Fault that decelerates first (COMMAND_TIMEOUT) → FAULT(code).
    Fault(FaultCode),
}

/// Phase of a running end-stop calibration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CalPhase {
    /// Main power commanded on; waiting for it to settle.
    PowerSettle,
    /// Brake release commanded; waiting for it.
    BrakeRelease,
    ToP,
    ToN,
    ToNeutral,
    /// Brake applied; waiting before PWM 0 / power off → CAL_RESULT → DISARMED.
    Shutdown,
}

#[derive(Debug, Clone)]
struct CalRun {
    phase: CalPhase,
    phase_started_us: u64,
    /// CAL_RESULT `step` (1 = to P end, 2 = to N end, 3 = to neutral).
    step: u8,
    /// Move target of the current phase (beyond the stop for the end moves).
    target: f64,
    /// Positions (current coordinate) where the ends were found.
    at_p: Option<f64>,
    at_n: Option<f64>,
    /// Outcome, set when the shutdown starts.
    outcome: Option<(CalResultCode, [u8; CAL_REASON_LEN])>,
}

/// Position-loop tuning as the firmware holds it (SET_LIMITS content).
/// Defaults are the firmware's `config.h` values before any SET_LIMITS.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SimTuning {
    pub duty_max_percent: u8,
    pub duty_min_percent: u8,
    pub kp_percent_per_100: u8,
    pub ff_count_s_per_percent: u16,
    pub deadband_count: u16,
    pub deadband_out_count: u16,
}

impl Default for SimTuning {
    fn default() -> Self {
        Self {
            duty_max_percent: 20,
            duty_min_percent: 7,
            kp_percent_per_100: 2,
            ff_count_s_per_percent: 100,
            deadband_count: 10,
            deadband_out_count: 20,
        }
    }
}

/// Pure model of one axis MCU. Time is injected as `now_us` (any monotonic
/// microsecond clock); calls with an older `now_us` do not move time back.
#[derive(Debug, Clone)]
pub struct SimMcu {
    params: SimParams,
    boot_id: u32,
    /// Current selection (NVS on the real unit; survives `reboot`).
    selected_axis: u8,
    /// `now_us` at boot; `mcu_time_us = now_us − boot_origin_us`.
    boot_origin_us: Option<u64>,
    now_us: Option<u64>,

    state: McuState,
    /// Latched code, meaningful in FAULT only.
    fault_code: FaultCode,
    stop_kind: StopKind,
    last_host_fault_code: Option<FaultCode>,

    config: Option<Config>,
    referenced: bool,
    /// SET_LIMITS state (firmware defaults until the first one).
    tuning: SimTuning,
    /// A LIMITS reply is owed after the ACK of an accepted SET_LIMITS.
    limits_reply_due: bool,

    // Timers (absolute now_us).
    arming_started_us: u64,
    disarming_started_us: u64,
    stopping_started_us: u64,
    lease_us: u64,
    valid_until_us: u64,
    brake_release_cmd: bool,

    // SET_POSITION bookkeeping.
    last_rx_seq: u32,
    last_applied_seq: u32,
    last_applied_frame: u32,
    rx_error_count: u16,
    stale_drop_count: u16,

    // Motion (counts, count/s).
    actual: f64,
    actual_vel: f64,
    /// End stops in the current coordinate (shifted with every reference change).
    end_min: f64,
    end_max: f64,
    setpoint: f64,
    setpoint_vel: f64,
    control_target: f64,
    frame_max_vel: f64,
    frame_max_acc: f64,

    /// Recently ACCEPTED commands for resend detection (oldest first).
    recent_accepted: VecDeque<(u32, HostMsg)>,
    /// Recently decoded host frames (oldest first), including ones later dropped.
    host_rx: VecDeque<HostRx>,

    // Fault injection.
    stall: bool,
    drop_rx: bool,
    freeze_tx: bool,
    fault_cause_present: bool,
    /// The next calibration fails at its first move with this reason.
    cal_failure: Option<[u8; CAL_REASON_LEN]>,

    cal: Option<CalRun>,

    outbox: Vec<McuMsg>,
    next_status_us: Option<u64>,
    last_status_key: Option<(McuState, FaultCode, u16, u8)>,
}

impl SimMcu {
    pub fn new(params: SimParams) -> Self {
        let actual = (params.initial_count as f64)
            .clamp(params.end_min_count as f64, params.end_max_count as f64);
        Self {
            boot_id: params.boot_id,
            selected_axis: params.selected_axis,
            boot_origin_us: None,
            now_us: None,
            state: McuState::Disarmed,
            fault_code: FaultCode::None,
            stop_kind: StopKind::Hold,
            last_host_fault_code: None,
            config: None,
            referenced: params.start_referenced,
            tuning: SimTuning::default(),
            limits_reply_due: false,
            arming_started_us: 0,
            disarming_started_us: 0,
            stopping_started_us: 0,
            lease_us: 0,
            valid_until_us: 0,
            brake_release_cmd: false,
            last_rx_seq: 0,
            last_applied_seq: 0,
            last_applied_frame: 0,
            rx_error_count: 0,
            stale_drop_count: 0,
            actual,
            actual_vel: 0.0,
            end_min: params.end_min_count as f64,
            end_max: params.end_max_count as f64,
            setpoint: actual,
            setpoint_vel: 0.0,
            control_target: actual,
            frame_max_vel: 0.0,
            frame_max_acc: 0.0,
            recent_accepted: VecDeque::with_capacity(DEDUPE_DEPTH),
            host_rx: VecDeque::with_capacity(HOST_RX_KEEP),
            stall: false,
            drop_rx: false,
            freeze_tx: false,
            fault_cause_present: false,
            cal_failure: None,
            cal: None,
            outbox: Vec::new(),
            next_status_us: None,
            last_status_key: None,
            params,
        }
    }

    // ------------------------------------------------------------------
    // Public entry points
    // ------------------------------------------------------------------

    /// Handle one decoded PC → MCU message received at `now_us`. Time is first
    /// advanced to `now_us`, so the result may also carry STATUS / FAULT_EVENT
    /// produced by that advance; filter by variant rather than comparing whole.
    pub fn handle(&mut self, msg: &HostMsg, now_us: u64) -> Vec<McuMsg> {
        let now = self.advance_to(now_us);
        if !self.drop_rx {
            self.dispatch(msg, now);
            self.emit_status_if_changed();
        }
        self.drain()
    }

    /// Handle one COBS-encoded frame (delimiter stripped) as it came off the
    /// wire. Frames that fail to decode are discarded without reply and
    /// counted in `rx_error_count` (protocol.md §2).
    pub fn handle_frame(&mut self, encoded: &[u8], now_us: u64) -> Vec<McuMsg> {
        if self.drop_rx {
            self.advance_to(now_us);
            return self.drain();
        }
        match protocol::decode_host_frame(encoded) {
            Ok(msg) => self.handle(&msg, now_us),
            Err(_) => {
                self.advance_to(now_us);
                self.record_rx_error();
                self.drain()
            }
        }
    }

    /// Advance plant and timers to `now_us`; emits the periodic STATUS when due,
    /// immediate STATUS on state/flag changes and FAULT_EVENT on fault latch.
    pub fn tick(&mut self, now_us: u64) -> Vec<McuMsg> {
        let now = self.advance_to(now_us);
        self.emit_periodic_status(now);
        self.drain()
    }

    /// Count a frame discarded below the message layer (CRC, COBS, oversize).
    pub fn record_rx_error(&mut self) {
        if !self.drop_rx {
            self.rx_error_count = self.rx_error_count.saturating_add(1);
        }
    }

    // ------------------------------------------------------------------
    // Accessors
    // ------------------------------------------------------------------

    pub fn state(&self) -> McuState {
        self.state
    }
    /// Fault code as reported in STATUS (also non-NONE during a fault-stop).
    pub fn fault_code(&self) -> FaultCode {
        match (self.state, self.stop_kind) {
            (McuState::Fault, _) => self.fault_code,
            (McuState::Stopping, StopKind::Fault(code)) => code,
            _ => FaultCode::None,
        }
    }
    pub fn actual_count(&self) -> Counts {
        to_count(self.actual)
    }
    /// Current control setpoint (STATUS.target_count).
    pub fn target_count(&self) -> Counts {
        to_count(self.setpoint)
    }
    /// Target the setpoint is moving toward (clamped SET_POSITION target in ACTIVE).
    pub fn control_target_count(&self) -> Counts {
        to_count(self.control_target)
    }
    pub fn boot_id(&self) -> u32 {
        self.boot_id
    }
    /// Axis currently selected on this unit (0 = none).
    pub fn selected_axis(&self) -> u8 {
        self.selected_axis
    }
    /// Position-loop tuning in effect (SET_LIMITS, clamped).
    pub fn tuning(&self) -> SimTuning {
        self.tuning
    }

    pub fn config(&self) -> Option<&Config> {
        self.config.as_ref()
    }
    pub fn session_epoch(&self) -> Option<u32> {
        self.config.map(|c| c.session_epoch)
    }
    pub fn is_configured(&self) -> bool {
        self.config.is_some()
    }
    pub fn is_referenced(&self) -> bool {
        self.referenced
    }
    pub fn last_rx_seq(&self) -> u32 {
        self.last_rx_seq
    }
    pub fn last_applied_seq(&self) -> u32 {
        self.last_applied_seq
    }
    pub fn last_applied_frame(&self) -> u32 {
        self.last_applied_frame
    }
    pub fn stale_drop_count(&self) -> u16 {
        self.stale_drop_count
    }
    pub fn rx_error_count(&self) -> u16 {
        self.rx_error_count
    }
    /// fault_code carried by the last HOST_FAULT (informational only).
    pub fn last_host_fault_code(&self) -> Option<FaultCode> {
        self.last_host_fault_code
    }
    pub fn params(&self) -> &SimParams {
        &self.params
    }
    /// End stops in the current count coordinate (min, max).
    pub fn end_counts(&self) -> (Counts, Counts) {
        (to_count(self.end_min), to_count(self.end_max))
    }
    /// An end-stop calibration is running (state CALIBRATING).
    pub fn is_calibrating(&self) -> bool {
        self.cal.is_some()
    }

    /// STATUS as it would be sent now.
    pub fn status(&self) -> Status {
        let now = self.now_us.unwrap_or(0);
        Status {
            boot_id: self.boot_id,
            mcu_time_us: self.mcu_time(now),
            mcu_state: self.state as u8,
            selected_axis: self.selected_axis,
            fault_code: self.fault_code(),
            flags: self.flags(),
            pwm_permille: self.pwm_permille(),
            actual_count: self.actual_count(),
            target_count: self.target_count(),
            velocity_count_s: to_count(self.actual_vel),
            // Same integer definition as the FOLLOWING_ERROR check.
            following_error_count: self.actual_count().saturating_sub(self.target_count()),
            last_applied_frame: self.last_applied_frame,
            last_applied_seq: self.last_applied_seq,
            last_rx_seq: self.last_rx_seq,
            current_ma: 0,
            rx_error_count: self.rx_error_count,
            stale_drop_count: self.stale_drop_count,
        }
    }

    // ------------------------------------------------------------------
    // Fault injection (tests / bench)
    // ------------------------------------------------------------------

    /// Actual position stops following the setpoint (→ FOLLOWING_ERROR when moving).
    pub fn set_stall(&mut self, stall: bool) {
        self.stall = stall;
    }
    /// External disturbance: move the actual position by `delta` counts
    /// (never past the end stops).
    pub fn nudge_actual(&mut self, delta: Counts) {
        self.actual = (self.actual + delta as f64).clamp(self.end_min, self.end_max);
    }
    /// The next calibration fails at its first move with `reason` (ASCII,
    /// truncated to 8 bytes), as a stalled / too slow unit would.
    pub fn set_calibration_failure(&mut self, reason: Option<&str>) {
        self.cal_failure = reason.map(CalResult::reason_bytes);
    }
    /// Latch `code` now (FAULT_EVENT goes out with the next handle/tick).
    pub fn force_fault(&mut self, code: FaultCode) {
        let code = if code == FaultCode::None {
            FaultCode::Unknown
        } else {
            code
        };
        let now = self.now_us.unwrap_or(0);
        self.latch_fault(code, now);
        self.emit_status_if_changed();
    }
    /// Ignore every received frame (not even counted).
    pub fn set_drop_rx(&mut self, drop: bool) {
        self.drop_rx = drop;
    }
    /// Stop transmitting anything (USB disconnect seen from the PC). The model
    /// keeps running; output produced meanwhile is discarded.
    pub fn set_freeze_status(&mut self, freeze: bool) {
        self.freeze_tx = freeze;
    }
    /// While set, FAULT_RESET is rejected with FAULT_CAUSE_PRESENT.
    pub fn set_fault_cause_present(&mut self, present: bool) {
        self.fault_cause_present = present;
    }
    /// MCU reset: new boot_id, DISARMED, CONFIG / epoch / seq / counters /
    /// request_id cache cleared, reference re-initialized from
    /// `params.start_referenced`. The encoder count restarts at 0 unless the
    /// simulated homing (`start_referenced`) re-establishes the same coordinate.
    /// Fault-injection switches and the axis selection (NVS) are kept.
    pub fn reboot(&mut self, new_boot_id: u32) {
        let mut fresh = SimMcu::new(self.params.clone());
        fresh.boot_id = new_boot_id;
        fresh.selected_axis = self.selected_axis;
        let pos = if self.params.start_referenced {
            self.actual
        } else {
            0.0
        };
        // The mechanism does not move on a reset: the end stops follow the
        // new coordinate.
        fresh.end_min = self.end_min + (pos - self.actual);
        fresh.end_max = self.end_max + (pos - self.actual);
        fresh.actual = pos;
        fresh.setpoint = pos;
        fresh.control_target = pos;
        fresh.now_us = self.now_us;
        fresh.boot_origin_us = self.now_us;
        fresh.stall = self.stall;
        fresh.drop_rx = self.drop_rx;
        fresh.freeze_tx = self.freeze_tx;
        fresh.fault_cause_present = self.fault_cause_present;
        *self = fresh;
    }

    /// Operator selects axis `new` (protocol.md §1.1): accepted in DISARMED
    /// only, otherwise ignored (the real unit shows a message). On a change:
    /// CONFIG / `CONFIGURED` / `calibration_hash` / `POSITION_REFERENCED`
    /// cleared, one AXIS_SELECT, and every later HELLO / STATUS carries `new`.
    /// The encoder count is not touched. Like `handle`, time is advanced first
    /// and the result may also carry STATUS.
    pub fn select_axis(&mut self, new: u8, now_us: u64) -> Vec<McuMsg> {
        let now = self.advance_to(now_us);
        if self.state == McuState::Disarmed {
            self.apply_axis_select(new, now);
        }
        self.drain()
    }

    /// Test hook: switch in any state (a unit that violates the DISARMED rule,
    /// so the PC side's fault handling can be exercised). Outside DISARMED the
    /// CONFIG limits and the lease stay in force (only `POSITION_REFERENCED`
    /// is cleared): the unit ends in FAULT(COMMAND_TIMEOUT) once the PC stops
    /// writing to it.
    pub fn select_axis_unchecked(&mut self, new: u8, now_us: u64) -> Vec<McuMsg> {
        let now = self.advance_to(now_us);
        self.apply_axis_select(new, now);
        self.drain()
    }

    /// The switch itself; output stays in the outbox (the async runner sends it).
    fn apply_axis_select(&mut self, new: u8, _now: u64) {
        if new == self.selected_axis {
            return;
        }
        let prev = self.selected_axis;
        self.selected_axis = new;
        // Axis-specific values must not carry over to the other axis.
        // Outside DISARMED (rule violation, `select_axis_unchecked`) the limits
        // and the lease are kept: the PC sends nothing to a unit on a conflicted
        // axis, so COMMAND_TIMEOUT is the only thing that stops it (protocol.md
        // §1.1). Dropping `config` here would silence the lease check.
        if self.state == McuState::Disarmed {
            self.config = None;
        }
        self.referenced = false;
        self.push(McuMsg::AxisSelect(protocol::AxisSelect {
            boot_id: self.boot_id,
            selected_axis: new,
            prev_selected_axis: prev,
            mcu_state: self.state as u8,
        }));
        // selected_axis is part of the STATUS key: a STATUS follows at once.
        self.emit_status_if_changed();
    }

    // ------------------------------------------------------------------
    // Message dispatch: one handler per message type (protocol.md §4)
    // ------------------------------------------------------------------

    fn note_host_rx(&mut self, msg: &HostMsg) {
        let (max_vel_count_s, max_acc_count_s2) = match msg {
            HostMsg::SetPosition(sp) => (sp.max_vel_count_s, sp.max_acc_count_s2),
            _ => (0, 0),
        };
        if self.host_rx.len() == HOST_RX_KEEP {
            self.host_rx.pop_front();
        }
        self.host_rx.push_back(HostRx {
            msg_type: msg.msg_type(),
            max_vel_count_s,
            max_acc_count_s2,
        });
    }

    fn dispatch(&mut self, msg: &HostMsg, now: u64) {
        self.note_host_rx(msg);
        match *msg {
            HostMsg::HelloReq { request_id } => self.on_hello_req(request_id),
            HostMsg::ClockSync {
                request_id,
                pc_time_us,
            } => self.on_clock_sync(request_id, pc_time_us, now),
            HostMsg::SetPosition(sp) => self.on_set_position(&sp, now),
            HostMsg::HostFault {
                request_id,
                fault_code,
            } => self.on_host_fault(request_id, fault_code, now),
            HostMsg::Config(c) => self.command(msg, c.request_id, msg_type::CONFIG, now, |s, _| {
                s.on_config(&c)
            }),
            HostMsg::ArmPrepare {
                request_id,
                session_epoch,
            } => self.command(msg, request_id, msg_type::ARM_PREPARE, now, |s, now| {
                s.on_arm_prepare(session_epoch, now)
            }),
            HostMsg::Start {
                request_id,
                session_epoch,
            } => self.command(msg, request_id, msg_type::START, now, |s, now| {
                s.on_start(session_epoch, now)
            }),
            HostMsg::ControlledStop {
                request_id,
                session_epoch,
                ..
            } => self.command(msg, request_id, msg_type::CONTROLLED_STOP, now, |s, now| {
                s.on_controlled_stop(session_epoch, now)
            }),
            HostMsg::Disarm { request_id } => {
                self.command(msg, request_id, msg_type::DISARM, now, |s, now| {
                    s.on_disarm(now)
                })
            }
            HostMsg::FaultReset { request_id } => {
                self.command(msg, request_id, msg_type::FAULT_RESET, now, |s, _| {
                    s.on_fault_reset()
                })
            }
            HostMsg::SetReference {
                request_id,
                reference_count,
            } => self.command(msg, request_id, msg_type::SET_REFERENCE, now, |s, _| {
                s.on_set_reference(reference_count)
            }),
            HostMsg::SetLimits(l) => {
                self.command(msg, l.request_id, msg_type::SET_LIMITS, now, |s, _| s.on_set_limits(&l));
                // After the ACK (protocol: ACK first, then one LIMITS).
                if std::mem::take(&mut self.limits_reply_due) {
                    self.push_limits();
                }
            }
            HostMsg::Calibrate { request_id, mode, .. } => {
                self.command(msg, request_id, msg_type::CALIBRATE, now, |s, now| {
                    s.on_calibrate(mode, now)
                })
            }
        }
    }

    /// Shared pipeline for ACKed commands (not HOST_FAULT): request_id 0 →
    /// dedupe → fault gate → handler; then ACK, and remember it if accepted.
    fn command(
        &mut self,
        msg: &HostMsg,
        request_id: u32,
        acked_type: u8,
        now: u64,
        handler: impl FnOnce(&mut Self, u64) -> Result<(), AckReason>,
    ) {
        let ack = |result, reason| Ack {
            request_id,
            acked_type,
            result,
            reason,
        };
        if request_id == 0 {
            self.push(McuMsg::Ack(ack(
                AckResult::Rejected,
                AckReason::InvalidParam,
            )));
            return;
        }
        // A resend is the identical message (type, request_id and payload).
        if self.recent_accepted.iter().any(|(_, m)| m == msg) {
            self.push(McuMsg::Ack(ack(AckResult::Duplicate, AckReason::None)));
            return;
        }
        // Same request_id with different content is a new command (e.g. the PC
        // restarted its counter): execute it; on accept it replaces the old entry.
        let result = if acked_type != msg_type::FAULT_RESET && self.fault_code() != FaultCode::None
        {
            Err(AckReason::FaultActive)
        } else {
            handler(self, now)
        };
        match result {
            Ok(()) => {
                self.recent_accepted.retain(|(id, _)| *id != request_id);
                if self.recent_accepted.len() >= DEDUPE_DEPTH {
                    self.recent_accepted.pop_front();
                }
                self.recent_accepted.push_back((request_id, *msg));
                self.push(McuMsg::Ack(ack(AckResult::Accepted, AckReason::None)));
            }
            // Rejections had no effect and are not cached: a resend is re-evaluated.
            Err(reason) => self.push(McuMsg::Ack(ack(AckResult::Rejected, reason))),
        }
    }

    /// HELLO_REQ → HELLO in any state (no ACK, no dedupe).
    fn on_hello_req(&mut self, request_id: u32) {
        self.push(McuMsg::Hello(Hello {
            request_id,
            protocol_version: PROTOCOL_VERSION,
            selected_axis: self.selected_axis,
            mcu_state: self.state as u8,
            device_id: self.params.device_id,
            boot_id: self.boot_id,
            fw_version: self.params.fw_version,
            capabilities: capabilities::POSITION_MODE | capabilities::LIMITS,
            calibration_hash: self.config.map_or(0, |c| c.calibration_hash),
        }));
    }

    /// CLOCK_SYNC → CLOCK_SYNC_REPLY (rx and tx stamped at the same instant).
    fn on_clock_sync(&mut self, request_id: u32, pc_time_us: u64, now: u64) {
        let t = self.mcu_time(now);
        self.push(McuMsg::ClockSyncReply(ClockSyncReply {
            request_id,
            pc_time_us,
            mcu_rx_us: t,
            mcu_tx_us: t,
        }));
    }

    /// CONFIG: DISARMED only; validates and stores limits + epoch.
    fn on_config(&mut self, c: &Config) -> Result<(), AckReason> {
        if self.state != McuState::Disarmed {
            return Err(AckReason::BadState);
        }
        let valid = c.soft_min_count < c.soft_max_count
            && c.max_vel_count_s > 0
            && c.max_acc_count_s2 > 0
            && c.following_error_limit_count > 0
            && c.overspeed_count_s > 0
            && c.command_timeout_ms > 0
            && c.stop_timeout_ms > 0;
        if !valid {
            return Err(AckReason::InvalidParam);
        }
        if self.session_epoch() != Some(c.session_epoch) {
            // seq is monotonic per epoch: a new epoch restarts it.
            self.last_rx_seq = 0;
        }
        // Firmware absolute ceilings (design 2026-11-01 §4.4): clamp, never reject.
        self.config = Some(Config {
            max_vel_count_s: c.max_vel_count_s.min(fw_absolute::MAX_VEL_COUNT_S),
            max_acc_count_s2: c.max_acc_count_s2.min(fw_absolute::MAX_ACC_COUNT_S2),
            ..*c
        });
        Ok(())
    }

    /// SET_LIMITS (design §4.2): any state; invalid combinations are
    /// INVALID_PARAM; duty values above the absolute ceiling are clamped.
    fn on_set_limits(&mut self, l: &SetLimits) -> Result<(), AckReason> {
        if l.duty_max_percent == 0
            || l.duty_min_percent > l.duty_max_percent
            || l.ff_count_s_per_percent == 0
            || l.deadband_out_count < l.deadband_count
        {
            return Err(AckReason::InvalidParam);
        }
        let duty_max = l.duty_max_percent.min(fw_absolute::DUTY_MAX_PERCENT);
        self.tuning = SimTuning {
            duty_max_percent: duty_max,
            duty_min_percent: l.duty_min_percent.min(duty_max),
            kp_percent_per_100: l.kp_percent_per_100,
            ff_count_s_per_percent: l.ff_count_s_per_percent,
            deadband_count: l.deadband_count,
            deadband_out_count: l.deadband_out_count,
        };
        self.limits_reply_due = true;
        Ok(())
    }

    fn push_limits(&mut self) {
        let t = self.tuning;
        self.push(McuMsg::Limits(Limits {
            boot_id: self.boot_id,
            duty_max_percent: t.duty_max_percent,
            duty_min_percent: t.duty_min_percent,
            kp_percent_per_100: t.kp_percent_per_100,
            abs_duty_max_percent: fw_absolute::DUTY_MAX_PERCENT,
            ff_count_s_per_percent: t.ff_count_s_per_percent,
            deadband_count: t.deadband_count,
            deadband_out_count: t.deadband_out_count,
            abs_max_vel_count_s: fw_absolute::MAX_VEL_COUNT_S,
            abs_max_acc_count_s2: fw_absolute::MAX_ACC_COUNT_S2,
        }));
    }

    /// ARM_PREPARE: DISARMED + CONFIGURED + REFERENCED + epoch → ARMING.
    fn on_arm_prepare(&mut self, epoch: u32, now: u64) -> Result<(), AckReason> {
        if self.state != McuState::Disarmed {
            return Err(AckReason::BadState);
        }
        let Some(cfg) = self.config else {
            return Err(AckReason::NotConfigured);
        };
        if !self.referenced {
            return Err(AckReason::NotReferenced);
        }
        if cfg.session_epoch != epoch {
            return Err(AckReason::EpochMismatch);
        }
        if !self.inside_soft_window(&cfg) {
            // Arming would release the brake only to latch SOFT_LIMIT.
            return Err(AckReason::BadState);
        }
        // §5: PWM 0, power on (power-controller axis only), brake still applied.
        self.state = McuState::Arming;
        self.arming_started_us = now;
        self.lease_us = now;
        self.valid_until_us = 0; // only frames received from now on count for START
        self.brake_release_cmd = false;
        self.hold_here();
        Ok(())
    }

    /// START: HOLDING + epoch + unexpired SET_POSITION → ACTIVE. Target stays at
    /// hold until the next SET_POSITION.
    fn on_start(&mut self, epoch: u32, now: u64) -> Result<(), AckReason> {
        if self.state != McuState::Holding {
            return Err(AckReason::BadState);
        }
        let cfg = self.config.ok_or(AckReason::NotConfigured)?;
        if cfg.session_epoch != epoch {
            return Err(AckReason::EpochMismatch);
        }
        if now > self.valid_until_us {
            // ACTIVE would expire on the next step (ACTIVE → STOPPING → HOLDING).
            return Err(AckReason::BadState);
        }
        self.state = McuState::Active;
        self.control_target = self.setpoint;
        self.frame_max_vel = cfg.max_vel_count_s as f64;
        self.frame_max_acc = cfg.max_acc_count_s2 as f64;
        Ok(())
    }

    /// SET_POSITION: no ACK. Staleness → drop; valid → lease; ACTIVE → apply.
    fn on_set_position(&mut self, m: &SetPosition, now: u64) {
        if self.state == McuState::Calibrating {
            // Dropped without being counted (protocol.md §1.2).
            return;
        }
        let Some(cfg) = self.config else {
            self.stale_drop_count = self.stale_drop_count.saturating_add(1);
            return;
        };
        if m.session_epoch != cfg.session_epoch || m.seq <= self.last_rx_seq {
            self.stale_drop_count = self.stale_drop_count.saturating_add(1);
            return;
        }
        if m.apply_at_us != 0 {
            // No clock sync in this model: a timed apply cannot be honored.
            self.rx_error_count = self.rx_error_count.saturating_add(1);
            return;
        }
        self.last_rx_seq = m.seq;
        self.lease_us = now;
        self.valid_until_us = now + u64::from(m.valid_for_ms) * 1_000;
        if self.state == McuState::Active {
            let target = m.target_count.clamp(cfg.soft_min_count, cfg.soft_max_count);
            self.control_target = target as f64;
            self.frame_max_vel = m.max_vel_count_s.min(cfg.max_vel_count_s) as f64;
            self.frame_max_acc = m.max_acc_count_s2.min(cfg.max_acc_count_s2) as f64;
            self.last_applied_seq = m.seq;
            self.last_applied_frame = m.group_frame_id;
        }
    }

    /// CONTROLLED_STOP: epoch must match; ACTIVE → STOPPING, otherwise no-op.
    fn on_controlled_stop(&mut self, epoch: u32, now: u64) -> Result<(), AckReason> {
        if self.session_epoch() != Some(epoch) {
            return Err(AckReason::EpochMismatch);
        }
        if self.state == McuState::Active {
            self.begin_stop(StopKind::Hold, now);
        }
        Ok(())
    }

    /// DISARM: decelerate if moving, then DISARMING → DISARMED.
    fn on_disarm(&mut self, now: u64) -> Result<(), AckReason> {
        match self.state {
            McuState::Disarmed | McuState::Disarming | McuState::Boot => {}
            McuState::Arming | McuState::Holding => self.enter_disarming(now),
            McuState::Active => self.begin_stop(StopKind::Disarm, now),
            McuState::Stopping => self.stop_kind = StopKind::Disarm,
            // DISARM is the cancel of a running calibration (protocol.md §1.2).
            McuState::Calibrating => self.cancel_calibration(now),
            // Unreachable through `command` (fault gate), kept for completeness.
            McuState::Fault => return Err(AckReason::FaultActive),
        }
        Ok(())
    }

    /// FAULT_RESET: FAULT only; rejected while the cause is present; → DISARMED.
    fn on_fault_reset(&mut self) -> Result<(), AckReason> {
        if self.state != McuState::Fault {
            return Err(AckReason::BadState);
        }
        if self.fault_cause_present {
            return Err(AckReason::FaultCausePresent);
        }
        if self.fault_code == FaultCode::EncoderFault {
            self.referenced = false;
        }
        self.state = McuState::Disarmed;
        self.fault_code = FaultCode::None;
        self.stop_kind = StopKind::Hold;
        self.hold_here();
        Ok(())
    }

    /// HOST_FAULT: latch FAULT immediately in any state and ACK ACCEPTED. No
    /// request_id check, no dedupe, not cached: latching is idempotent, so
    /// executing every copy is the safe direction.
    fn on_host_fault(&mut self, request_id: u32, code: FaultCode, now: u64) {
        self.last_host_fault_code = Some(code);
        let latched = match (self.state, self.stop_kind) {
            (McuState::Stopping, StopKind::Fault(pending)) => pending,
            _ => FaultCode::HostFault,
        };
        self.latch_fault(latched, now);
        self.push(McuMsg::Ack(Ack {
            request_id,
            acked_type: msg_type::HOST_FAULT,
            result: AckResult::Accepted,
            reason: AckReason::None,
        }));
    }

    /// SET_REFERENCE: DISARMED only; the current position becomes `reference_count`.
    fn on_set_reference(&mut self, reference_count: Counts) -> Result<(), AckReason> {
        if self.state != McuState::Disarmed {
            return Err(AckReason::BadState);
        }
        self.shift_coordinate(reference_count as f64 - self.actual);
        self.hold_here();
        self.referenced = true;
        Ok(())
    }

    /// Re-define the count coordinate: everything that is a position moves by
    /// `delta`, the mechanism itself does not.
    fn shift_coordinate(&mut self, delta: f64) {
        self.actual += delta;
        self.setpoint += delta;
        self.control_target += delta;
        self.end_min += delta;
        self.end_max += delta;
    }

    /// CALIBRATE: `mode` 1 starts (DISARMED only), 0 cancels (no-op unless
    /// CALIBRATING), anything else is INVALID_PARAM (protocol.md §4).
    fn on_calibrate(&mut self, mode: u8, now: u64) -> Result<(), AckReason> {
        match mode {
            calibrate_mode::START => {
                if self.state != McuState::Disarmed {
                    return Err(AckReason::BadState);
                }
                // Power on command (role bit, see `flags`), brake still applied.
                self.state = McuState::Calibrating;
                self.brake_release_cmd = false;
                self.hold_here();
                self.cal = Some(CalRun {
                    phase: CalPhase::PowerSettle,
                    phase_started_us: now,
                    step: 0,
                    target: self.actual,
                    at_p: None,
                    at_n: None,
                    outcome: None,
                });
                Ok(())
            }
            calibrate_mode::CANCEL => {
                if self.state == McuState::Calibrating {
                    self.cancel_calibration(now);
                }
                Ok(())
            }
            _ => Err(AckReason::InvalidParam),
        }
    }

    fn cancel_calibration(&mut self, now: u64) {
        self.end_calibration(CalResultCode::Cancelled, "stop", now);
    }

    /// Stop moving and begin the shutdown sequence with this outcome (unless
    /// the calibration is already shutting down).
    fn end_calibration(&mut self, result: CalResultCode, reason: &str, now: u64) {
        let Some(run) = self.cal.as_mut() else { return };
        if run.phase == CalPhase::Shutdown {
            return;
        }
        run.phase = CalPhase::Shutdown;
        run.phase_started_us = now;
        run.outcome = Some((result, CalResult::reason_bytes(reason)));
        self.brake_release_cmd = false;
        self.actual_vel = 0.0;
        self.setpoint = self.actual;
        self.control_target = self.actual;
    }

    /// One substep of a running calibration (state CALIBRATING).
    fn cal_step(&mut self, now: u64, dt: f64) {
        let settle = self.params.arming_delay_ms * 1_000 / 2;
        let Some(run) = self.cal.as_mut() else { return };
        let elapsed = now.saturating_sub(run.phase_started_us);
        match run.phase {
            CalPhase::PowerSettle => {
                if elapsed >= settle {
                    run.phase = CalPhase::BrakeRelease;
                    run.phase_started_us = now;
                    self.brake_release_cmd = true;
                }
            }
            CalPhase::BrakeRelease => {
                if elapsed >= settle {
                    run.phase = CalPhase::ToP;
                    run.phase_started_us = now;
                    run.step = 1;
                    // Push toward the stop: the plant clamps there.
                    run.target = self.end_min - 1.0e6;
                }
            }
            CalPhase::ToP | CalPhase::ToN | CalPhase::ToNeutral => {
                if let Some(reason) = self.cal_failure.take() {
                    let reason = reason_text(&reason);
                    self.end_calibration(CalResultCode::Failed, &reason, now);
                    return;
                }
                if self.stall || dt <= 0.0 {
                    // No pulses while driving: auto_move's `stall`.
                    self.end_calibration(CalResultCode::Failed, "stall", now);
                    return;
                }
                let max_step = self.params.plant_max_vel_count_s.max(0.0) * dt;
                let before = self.actual;
                let delta = (run.target - before).clamp(-max_step, max_step);
                // Land exactly on a reachable target (no float residue that
                // would keep the neutral move going forever).
                let after = if (run.target - before).abs() <= max_step {
                    run.target
                } else {
                    before + delta
                }
                .clamp(self.end_min, self.end_max);
                self.actual = after;
                self.actual_vel = (after - before) / dt;
                self.setpoint = after;
                self.control_target = after;
                let moved = after != before;
                match run.phase {
                    CalPhase::ToP if !moved => {
                        run.at_p = Some(after);
                        run.phase = CalPhase::ToN;
                        run.phase_started_us = now;
                        run.step = 2;
                        run.target = self.end_max + 1.0e6;
                    }
                    CalPhase::ToN if !moved => {
                        run.at_n = Some(after);
                        run.phase = CalPhase::ToNeutral;
                        run.phase_started_us = now;
                        run.step = 3;
                        let p = run.at_p.unwrap_or(after);
                        run.target = (p + after) / 2.0;
                    }
                    CalPhase::ToNeutral if after == run.target => {
                        self.end_calibration(CalResultCode::Ok, "ok", now);
                    }
                    _ => {}
                }
            }
            CalPhase::Shutdown => {
                if elapsed >= self.params.disarming_delay_ms * 1_000 {
                    self.finish_calibration();
                }
            }
        }
    }

    /// Shutdown done: CAL_RESULT, then (on success) the P end becomes 0 and
    /// POSITION_REFERENCED is set; then DISARMED (its STATUS follows the
    /// CAL_RESULT because the state key changes here).
    fn finish_calibration(&mut self) {
        let Some(run) = self.cal.take() else { return };
        let (result, reason) = run
            .outcome
            .unwrap_or((CalResultCode::Failed, CalResult::reason_bytes("timeout")));
        let (n_end, neutral) = match (result, run.at_p, run.at_n) {
            (CalResultCode::Ok, Some(p), Some(n)) => (n - p, (n - p) / 2.0),
            _ => (0.0, 0.0),
        };
        self.push(McuMsg::CalResult(CalResult {
            boot_id: self.boot_id,
            result,
            step: run.step,
            n_end_count: to_count(n_end),
            neutral_count: to_count(neutral),
            sign: 1,
            reason,
        }));
        if result == CalResultCode::Ok {
            if let Some(p) = run.at_p {
                self.shift_coordinate(-p);
            }
            self.referenced = true;
        } else {
            self.referenced = false;
        }
        self.state = McuState::Disarmed;
        self.brake_release_cmd = false;
        self.hold_here();
    }

    /// CAL_RESULT(FAILED, `fault`) when a fault latch interrupts a calibration.
    fn abort_calibration_for_fault(&mut self) {
        let Some(run) = self.cal.take() else { return };
        self.push(McuMsg::CalResult(CalResult {
            boot_id: self.boot_id,
            result: CalResultCode::Failed,
            step: run.step,
            n_end_count: 0,
            neutral_count: 0,
            sign: 1,
            reason: CalResult::reason_bytes("fault"),
        }));
        self.referenced = false;
    }

    // ------------------------------------------------------------------
    // Transitions
    // ------------------------------------------------------------------

    fn begin_stop(&mut self, kind: StopKind, now: u64) {
        self.state = McuState::Stopping;
        self.stop_kind = kind;
        self.stopping_started_us = now;
    }

    /// Fault that decelerates before latching (protocol.md §3.2 COMMAND_TIMEOUT).
    fn begin_fault_stop(&mut self, code: FaultCode, now: u64) {
        if self.setpoint_vel == 0.0 {
            self.latch_fault(code, now);
        } else if self.state == McuState::Stopping {
            // Keep the original stop start so stop_timeout still bounds it.
            self.stop_kind = StopKind::Fault(code);
        } else {
            self.begin_stop(StopKind::Fault(code), now);
        }
    }

    /// Deceleration finished (setpoint velocity 0).
    fn finish_stop(&mut self, now: u64) {
        match self.stop_kind {
            StopKind::Hold => {
                self.state = McuState::Holding;
                self.control_target = self.setpoint;
            }
            StopKind::Disarm => self.enter_disarming(now),
            StopKind::Fault(code) => self.latch_fault(code, now),
        }
    }

    /// Brake release off, then wait `disarming_delay_ms` → DISARMED.
    fn enter_disarming(&mut self, now: u64) {
        self.state = McuState::Disarming;
        self.disarming_started_us = now;
        self.brake_release_cmd = false;
        self.stop_kind = StopKind::Hold;
    }

    /// Latch FAULT: PWM 0, brake applied, power off. First code wins.
    fn latch_fault(&mut self, code: FaultCode, now: u64) {
        if self.state == McuState::Fault {
            return;
        }
        self.abort_calibration_for_fault();
        let before = self.state;
        self.state = McuState::Fault;
        self.fault_code = code;
        self.stop_kind = StopKind::Hold;
        self.brake_release_cmd = false;
        self.hold_here();
        let ev = FaultEvent {
            boot_id: self.boot_id,
            mcu_time_us: self.mcu_time(now),
            fault_code: code,
            mcu_state_before: before as u8,
            actual_count: self.actual_count(),
        };
        self.push(McuMsg::FaultEvent(ev));
    }

    /// Freeze motion at the current actual position.
    fn hold_here(&mut self) {
        self.actual_vel = 0.0;
        self.setpoint = self.actual;
        self.setpoint_vel = 0.0;
        self.control_target = self.actual;
    }

    // ------------------------------------------------------------------
    // Time advance: timers → setpoint → plant → checks → STATUS
    // ------------------------------------------------------------------

    /// Advance to `now_us` (never backwards) in 1 ms substeps. Returns the
    /// effective current time.
    fn advance_to(&mut self, now_us: u64) -> u64 {
        let Some(mut t) = self.now_us else {
            self.now_us = Some(now_us);
            self.boot_origin_us.get_or_insert(now_us);
            return now_us;
        };
        if now_us <= t {
            return t;
        }
        while t < now_us {
            // Brake applied and nothing pending: nothing changes, jump.
            if self.is_parked() {
                self.now_us = Some(now_us);
                self.actual_vel = 0.0;
                return now_us;
            }
            let step = SUBSTEP_US.min(now_us - t);
            t += step;
            self.now_us = Some(t);
            self.step(t, step as f64 / 1e6);
        }
        t
    }

    /// States with no timers and no motion (DISARMED / FAULT / BOOT).
    fn is_parked(&self) -> bool {
        matches!(
            self.state,
            McuState::Disarmed | McuState::Fault | McuState::Boot
        )
    }

    fn inside_soft_window(&self, cfg: &Config) -> bool {
        (cfg.soft_min_count..=cfg.soft_max_count).contains(&self.actual_count())
    }

    fn step(&mut self, now: u64, dt: f64) {
        self.check_timers(now);
        if self.state == McuState::Calibrating {
            self.cal_step(now, dt);
        } else if self.position_control_on(now) {
            self.update_setpoint(dt);
            self.update_plant(dt);
            self.check_motion_faults(now);
        } else {
            self.actual_vel = 0.0;
        }
        if self.state == McuState::Stopping && self.setpoint_vel == 0.0 {
            self.finish_stop(now);
        }
        self.emit_status_if_changed();
    }

    fn check_timers(&mut self, now: u64) {
        let ms = |v: u64| v * 1_000;
        match self.state {
            McuState::Arming => {
                let t = now.saturating_sub(self.arming_started_us);
                if !self.brake_release_cmd && t >= ms(self.params.arming_delay_ms) / 2 {
                    // Power settled. Never release the brake outside the soft window
                    // (the motion check would latch SOFT_LIMIT right after: a pulse).
                    if self.config.is_some_and(|c| !self.inside_soft_window(&c)) {
                        self.latch_fault(FaultCode::SoftLimit, now);
                        return;
                    }
                    // Position control on current position, release brake.
                    self.brake_release_cmd = true;
                    self.hold_here();
                }
                if t >= ms(self.params.arming_delay_ms) {
                    self.state = McuState::Holding;
                }
            }
            McuState::Disarming
                if now.saturating_sub(self.disarming_started_us)
                    >= ms(self.params.disarming_delay_ms) =>
            {
                // Brake applied: PWM 0, power off.
                self.state = McuState::Disarmed;
                self.hold_here();
            }
            _ => {}
        }
        let Some(cfg) = self.config else { return };

        // Lease: command_timeout_ms without a valid SET_POSITION.
        let leased = match self.state {
            McuState::Arming | McuState::Holding | McuState::Active => true,
            McuState::Stopping => self.stop_kind == StopKind::Hold,
            _ => false,
        };
        if leased && now.saturating_sub(self.lease_us) > ms(u64::from(cfg.command_timeout_ms)) {
            self.begin_fault_stop(FaultCode::CommandTimeout, now);
        }
        // Target validity: ACTIVE → STOPPING → HOLDING.
        if self.state == McuState::Active && now > self.valid_until_us {
            self.begin_stop(StopKind::Hold, now);
        }
        // Deceleration must finish within stop_timeout_ms.
        if self.state == McuState::Stopping
            && now.saturating_sub(self.stopping_started_us) > ms(u64::from(cfg.stop_timeout_ms))
        {
            let code = match self.stop_kind {
                StopKind::Fault(code) => code,
                _ => FaultCode::StopTimeout,
            };
            self.latch_fault(code, now);
        }
    }

    /// Position loop running (brake released or being applied).
    fn position_control_on(&self, now: u64) -> bool {
        match self.state {
            McuState::Arming => {
                now.saturating_sub(self.arming_started_us)
                    >= self.params.arming_delay_ms * 1_000 / 2
            }
            McuState::Holding | McuState::Active | McuState::Stopping | McuState::Disarming => true,
            _ => false,
        }
    }

    fn update_setpoint(&mut self, dt: f64) {
        let Some(cfg) = self.config else { return };
        match self.state {
            McuState::Active => {
                let (p, v) = profile_step(
                    self.setpoint,
                    self.setpoint_vel,
                    self.control_target,
                    self.frame_max_vel,
                    self.frame_max_acc,
                    dt,
                );
                // The target is already clamped; also keep any overshoot inside.
                self.setpoint = p.clamp(cfg.soft_min_count as f64, cfg.soft_max_count as f64);
                self.setpoint_vel = v;
            }
            McuState::Stopping => {
                // Decelerate at the last applied frame's acceleration limit
                // (design 2026-11-01 §4.4: CONFIG carries the absolute ceiling).
                let amax = if self.frame_max_acc > 0.0 { self.frame_max_acc } else { cfg.max_acc_count_s2 as f64 };
                let dv = amax * dt;
                let v = self.setpoint_vel;
                let v = if v.abs() <= dv {
                    0.0
                } else {
                    v - dv * v.signum()
                };
                self.setpoint_vel = v;
                self.setpoint = (self.setpoint + v * dt)
                    .clamp(cfg.soft_min_count as f64, cfg.soft_max_count as f64);
            }
            // ARMING / HOLDING / DISARMING hold the setpoint.
            _ => self.setpoint_vel = 0.0,
        }
    }

    fn update_plant(&mut self, dt: f64) {
        if self.stall || dt <= 0.0 {
            self.actual_vel = 0.0;
            return;
        }
        let max_step = self.params.plant_max_vel_count_s.max(0.0) * dt;
        let delta = (self.setpoint - self.actual).clamp(-max_step, max_step);
        let after = (self.actual + delta).clamp(self.end_min, self.end_max);
        self.actual_vel = (after - self.actual) / dt;
        self.actual = after;
    }

    fn check_motion_faults(&mut self, now: u64) {
        let Some(cfg) = self.config else { return };
        let actual = self.actual_count();
        let fe = i64::from(actual) - i64::from(self.target_count());
        if fe.unsigned_abs() > u64::from(cfg.following_error_limit_count) {
            self.latch_fault(FaultCode::FollowingError, now);
        } else if self.actual_vel.abs() > cfg.overspeed_count_s as f64 {
            self.latch_fault(FaultCode::Overspeed, now);
        } else if actual < cfg.soft_min_count || actual > cfg.soft_max_count {
            self.latch_fault(FaultCode::SoftLimit, now);
        }
    }

    // ------------------------------------------------------------------
    // Outputs
    // ------------------------------------------------------------------

    fn flags(&self) -> u16 {
        let mut f = 0;
        let power_controller = self
            .config
            .is_some_and(|c| c.role_flags & role_flags::POWER_CONTROLLER != 0);
        let powered = matches!(
            self.state,
            McuState::Arming
                | McuState::Holding
                | McuState::Active
                | McuState::Stopping
                | McuState::Disarming
                | McuState::Calibrating
        );
        if power_controller {
            f |= status_flags::POWER_CONTROLLER;
            if powered {
                f |= status_flags::POWER_ON_CMD;
            }
        }
        if self.brake_release_cmd {
            f |= status_flags::BRAKE_RELEASE_CMD;
        }
        if self.referenced {
            f |= status_flags::POSITION_REFERENCED;
        }
        if self.config.is_some() {
            f |= status_flags::CONFIGURED;
        }
        f
    }

    fn pwm_permille(&self) -> i16 {
        let now = self.now_us.unwrap_or(0);
        if self.state == McuState::Calibrating {
            // Open-loop drive toward the stops: feed-forward only.
            let ff = if self.params.plant_max_vel_count_s > 0.0 {
                1000.0 * self.actual_vel / self.params.plant_max_vel_count_s
            } else {
                0.0
            };
            return ff.clamp(-1000.0, 1000.0).round() as i16;
        }
        if !self.position_control_on(now) {
            return 0;
        }
        let ff = if self.params.plant_max_vel_count_s > 0.0 {
            1000.0 * self.setpoint_vel / self.params.plant_max_vel_count_s
        } else {
            0.0
        };
        let p = PWM_KP_PERMILLE_PER_COUNT * (self.setpoint - self.actual);
        (p + ff).clamp(-1000.0, 1000.0).round() as i16
    }

    fn mcu_time(&self, now: u64) -> u64 {
        now.saturating_sub(self.boot_origin_us.unwrap_or(now))
    }

    fn push(&mut self, m: McuMsg) {
        self.outbox.push(m);
    }

    fn drain(&mut self) -> Vec<McuMsg> {
        let out = std::mem::take(&mut self.outbox);
        if self.freeze_tx {
            Vec::new()
        } else {
            out
        }
    }

    fn status_key(&self) -> (McuState, FaultCode, u16, u8) {
        (self.state, self.fault_code(), self.flags(), self.selected_axis)
    }

    /// Immediate STATUS when state, fault code, flags or selected_axis changed.
    fn emit_status_if_changed(&mut self) {
        let key = self.status_key();
        if self.last_status_key != Some(key) {
            self.last_status_key = Some(key);
            let s = self.status();
            self.push(McuMsg::Status(s));
        }
    }

    /// Periodic STATUS every `status_period_ms` (fixed cadence, at most one per call).
    fn emit_periodic_status(&mut self, now: u64) {
        let period = self.params.status_period_ms.max(1) * 1_000;
        let due = *self.next_status_us.get_or_insert(now);
        if now >= due {
            // Next slot strictly after `now` on the fixed cadence.
            let next = due.saturating_add(((now - due) / period + 1).saturating_mul(period));
            self.next_status_us = Some(next);
            self.last_status_key = Some(self.status_key());
            let s = self.status();
            self.push(McuMsg::Status(s));
        }
    }
}

/// `reason` bytes as text (up to the first NUL).
fn reason_text(r: &[u8; CAL_REASON_LEN]) -> String {
    CalResult { reason: *r, ..CalResult::EMPTY }.reason_str()
}

/// Encoder count from an f64 position (saturating, NaN → 0).
fn to_count(v: f64) -> Counts {
    v.round() as Counts
}

/// One step of a velocity/acceleration-limited move toward `target` that
/// decelerates in time to stop on it (no overshoot). Returns (pos, vel).
fn profile_step(pos: f64, vel: f64, target: f64, vmax: f64, amax: f64, dt: f64) -> (f64, f64) {
    let err = target - pos;
    let vmax = vmax.max(0.0);
    let amax = amax.max(0.0);
    let v_des = if err == 0.0 {
        0.0
    } else {
        err.signum() * vmax.min((2.0 * amax * err.abs()).sqrt())
    };
    let dv = (v_des - vel).clamp(-amax * dt, amax * dt);
    let v = vel + dv;
    let p = pos + v * dt;
    // Reached or crossed the target this step: settle on it.
    let settled = if err == 0.0 {
        v == 0.0
    } else {
        (target - p) * err <= 0.0
    };
    if settled {
        (target, 0.0)
    } else {
        (p, v)
    }
}

// ----------------------------------------------------------------------
// Async runner
// ----------------------------------------------------------------------

/// Model tick of the async runner.
const RUNNER_TICK: Duration = Duration::from_millis(1);
/// Outgoing frames buffered toward the host before new ones are dropped
/// (a real MCU drops TX when the host stops reading).
const TX_QUEUE_FRAMES: usize = 256;
/// Duplex buffer used by [`sim_pair`].
const DUPLEX_BUF: usize = 64 * 1024;
/// Host frames kept per axis so a test can see what arrived just before a stop.
const HOST_RX_KEEP: usize = 128;

/// One host frame the simulator decoded, in arrival order.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HostRx {
    pub msg_type: u8,
    /// SET_POSITION limits. Zero for every other message.
    pub max_vel_count_s: u32,
    pub max_acc_count_s2: u32,
}

#[derive(Debug)]
struct Shared {
    mcu: Mutex<SimMcu>,
    corrupt_next_tx: AtomicBool,
    tx_dropped: AtomicU64,
    /// Zero of the model's `now_us`. Shared by every runner attached to this
    /// model, so a re-attached runner never feeds time backwards
    /// (`SimMcu::advance_to` would ignore it).
    origin: Instant,
}

impl Shared {
    fn lock(&self) -> MutexGuard<'_, SimMcu> {
        self.mcu.lock().unwrap_or_else(|e| e.into_inner())
    }
}

/// Point-in-time view of a running simulated axis.
#[derive(Debug, Clone, PartialEq)]
pub struct SimSnapshot {
    pub state: McuState,
    pub fault_code: FaultCode,
    pub actual_count: Counts,
    pub target_count: Counts,
    pub boot_id: u32,
    pub selected_axis: u8,
    pub session_epoch: Option<u32>,
    pub configured: bool,
    pub referenced: bool,
    pub last_rx_seq: u32,
    pub last_applied_frame: u32,
    pub stale_drop_count: u16,
    pub rx_error_count: u16,
    /// End stops in the current count coordinate.
    pub end_min_count: Counts,
    pub end_max_count: Counts,
}

/// Control handle of a simulated axis task. Dropping it stops the task.
#[derive(Debug)]
pub struct SimHandle {
    shared: Arc<Shared>,
    shutdown: Option<oneshot::Sender<()>>,
    task: Option<JoinHandle<()>>,
}

impl SimHandle {
    pub fn state(&self) -> McuState {
        self.shared.lock().state()
    }
    pub fn snapshot(&self) -> SimSnapshot {
        let m = self.shared.lock();
        SimSnapshot {
            state: m.state(),
            fault_code: m.fault_code(),
            actual_count: m.actual_count(),
            target_count: m.target_count(),
            boot_id: m.boot_id(),
            selected_axis: m.selected_axis(),
            session_epoch: m.session_epoch(),
            configured: m.is_configured(),
            referenced: m.is_referenced(),
            last_rx_seq: m.last_rx_seq(),
            last_applied_frame: m.last_applied_frame(),
            stale_drop_count: m.stale_drop_count(),
            rx_error_count: m.rx_error_count(),
            end_min_count: m.end_counts().0,
            end_max_count: m.end_counts().1,
        }
    }
    /// Run `f` with the model locked (do not block inside).
    pub fn with<R>(&self, f: impl FnOnce(&mut SimMcu) -> R) -> R {
        f(&mut self.shared.lock())
    }
    pub fn set_stall(&self, stall: bool) {
        self.shared.lock().set_stall(stall);
    }
    /// Frame limits latched while ACTIVE. STOPPING does not update them.
    pub fn frame_limits(&self) -> (f64, f64) {
        let m = self.shared.lock();
        (m.frame_max_vel, m.frame_max_acc)
    }
    /// Host frames decoded on this axis, oldest first. Capped.
    pub fn host_rx(&self) -> Vec<HostRx> {
        self.shared.lock().host_rx.iter().copied().collect()
    }
    pub fn nudge_actual(&self, delta: Counts) {
        self.shared.lock().nudge_actual(delta);
    }
    pub fn force_fault(&self, code: FaultCode) {
        self.shared.lock().force_fault(code);
    }
    pub fn set_drop_rx(&self, drop: bool) {
        self.shared.lock().set_drop_rx(drop);
    }
    pub fn set_freeze_status(&self, freeze: bool) {
        self.shared.lock().set_freeze_status(freeze);
    }
    pub fn set_fault_cause_present(&self, present: bool) {
        self.shared.lock().set_fault_cause_present(present);
    }
    /// The next calibration fails at its first move with `reason`.
    pub fn set_calibration_failure(&self, reason: Option<&str>) {
        self.shared.lock().set_calibration_failure(reason);
    }
    pub fn reboot(&self, new_boot_id: u32) {
        self.shared.lock().reboot(new_boot_id);
    }
    /// Operator switches the unit to axis `new` (DISARMED only, protocol.md
    /// §1.1). AXIS_SELECT and the new STATUS go out with the runner's next tick.
    pub fn select_axis(&self, new: u8) {
        let mut m = self.shared.lock();
        if m.state() == McuState::Disarmed {
            let now = m.now_us.unwrap_or(0);
            m.apply_axis_select(new, now);
        }
    }
    /// Switch in any state (rule violation, for tests of the PC side).
    pub fn select_axis_unchecked(&self, new: u8) {
        let mut m = self.shared.lock();
        let now = m.now_us.unwrap_or(0);
        m.apply_axis_select(new, now);
    }
    /// Corrupt the CRC of the next transmitted frame (the host must discard it).
    pub fn corrupt_next_tx(&self) {
        self.shared.corrupt_next_tx.store(true, Ordering::SeqCst);
    }
    /// Frames dropped because the host was not reading.
    pub fn tx_dropped(&self) -> u64 {
        self.shared.tx_dropped.load(Ordering::SeqCst)
    }
    /// True when no runner is attached or the attached one has ended (its
    /// host end was dropped, or it was stopped).
    pub fn is_finished(&self) -> bool {
        self.task.as_ref().is_none_or(|t| t.is_finished())
    }
    /// Stop the task and wait for it.
    pub async fn shutdown(mut self) {
        if let Some(tx) = self.shutdown.take() {
            let _ = tx.send(());
        }
        if let Some(task) = self.task.take() {
            let _ = task.await;
        }
    }
    /// A second handle on the same model without a runner of its own:
    /// inspection / fault injection (`snapshot`, `set_*`, `with`) work;
    /// dropping it stops nothing. For code that only watches the simulator
    /// while another owner (the server's sim port task) runs it.
    pub fn view(&self) -> SimHandle {
        SimHandle { shared: self.shared.clone(), shutdown: None, task: None }
    }
    /// (Re)attach the model to a fresh byte stream and return the host end:
    /// the previous runner, if any, is stopped and a new one started on the
    /// same [`SimMcu`] (same state, boot_id, reference, position, clock).
    /// How the server reopens a simulated port after its link closed
    /// (`status_timeout` after a host sleep, Issue #15).
    pub fn attach(&mut self) -> DuplexStream {
        if let Some(tx) = self.shutdown.take() {
            let _ = tx.send(());
        }
        if let Some(task) = self.task.take() {
            task.abort();
        }
        let (host, mcu) = tokio::io::duplex(DUPLEX_BUF);
        let (stop_tx, stop_rx) = oneshot::channel();
        let latency = Duration::from_millis(self.shared.lock().params().link_latency_ms);
        self.task = Some(tokio::spawn(run_sim_axis(mcu, self.shared.clone(), latency, stop_rx)));
        self.shutdown = Some(stop_tx);
        host
    }
}

impl Drop for SimHandle {
    fn drop(&mut self) {
        if let Some(task) = self.task.take() {
            task.abort();
        }
    }
}

/// A [`SimMcu`] with no byte stream yet: [`SimHandle::attach`] starts a
/// runner on it (and again after the host end closed).
pub fn sim_detached(params: SimParams) -> SimHandle {
    let shared = Arc::new(Shared {
        mcu: Mutex::new(SimMcu::new(params)),
        corrupt_next_tx: AtomicBool::new(false),
        tx_dropped: AtomicU64::new(0),
        origin: Instant::now(),
    });
    SimHandle { shared, shutdown: None, task: None }
}

/// Run a [`SimMcu`] on one side of a duplex stream: COBS frames in, model
/// ticked every 1 ms with `tokio::time`, COBS frames out.
pub fn spawn_sim_axis(io: DuplexStream, params: SimParams) -> SimHandle {
    let mut h = sim_detached(params);
    let latency = Duration::from_millis(h.shared.lock().params().link_latency_ms);
    let (stop_tx, stop_rx) = oneshot::channel();
    h.task = Some(tokio::spawn(run_sim_axis(io, h.shared.clone(), latency, stop_rx)));
    h.shutdown = Some(stop_tx);
    h
}

/// A simulated axis plus the host end of its byte stream.
pub fn sim_pair(params: SimParams) -> (DuplexStream, SimHandle) {
    let (host, mcu) = tokio::io::duplex(DUPLEX_BUF);
    (host, spawn_sim_axis(mcu, params))
}

async fn run_sim_axis(
    io: DuplexStream,
    shared: Arc<Shared>,
    latency: Duration,
    mut stop_rx: oneshot::Receiver<()>,
) {
    let (mut rd, wr) = tokio::io::split(io);
    let (tx, rx) = mpsc::channel::<(Instant, Vec<u8>)>(TX_QUEUE_FRAMES);
    let writer = tokio::spawn(write_frames(wr, rx));

    let origin = shared.origin;
    let now_us = || origin.elapsed().as_micros().min(u128::from(u64::MAX)) as u64;
    let mut acc = FrameAccumulator::new(MAX_ENCODED_LEN);
    let mut oversize_seen = 0u64;
    let mut rx_queue: VecDeque<(Instant, Vec<u8>)> = VecDeque::new();
    let mut buf = [0u8; 512];
    let mut ticker = tokio::time::interval(RUNNER_TICK);
    ticker.set_missed_tick_behavior(MissedTickBehavior::Skip);

    loop {
        tokio::select! {
            _ = &mut stop_rx => break,
            r = rd.read(&mut buf) => match r {
                Ok(0) | Err(_) => break,
                Ok(n) => {
                    let due = Instant::now() + latency;
                    for f in acc.push(buf.get(..n).unwrap_or_default()) {
                        rx_queue.push_back((due, f));
                    }
                }
            },
            _ = ticker.tick() => {}
        }

        // Lock, compute, unlock: never hold the guard across an await.
        let out = {
            let mut mcu = shared.lock();
            let mut out = Vec::new();
            for _ in oversize_seen..acc.dropped() {
                mcu.record_rx_error();
            }
            oversize_seen = acc.dropped();
            let now = Instant::now();
            while rx_queue.front().is_some_and(|(due, _)| *due <= now) {
                if let Some((_, f)) = rx_queue.pop_front() {
                    out.extend(mcu.handle_frame(&f, now_us()));
                }
            }
            out.extend(mcu.tick(now_us()));
            out
        };
        let due = Instant::now() + latency;
        for m in out {
            let bytes = if shared.corrupt_next_tx.swap(false, Ordering::SeqCst) {
                corrupted_wire(&m)
            } else {
                m.to_wire()
            };
            if tx.try_send((due, bytes)).is_err() {
                shared.tx_dropped.fetch_add(1, Ordering::SeqCst);
            }
        }
    }
    drop(tx);
    let _ = writer.await;
}

async fn write_frames(
    mut wr: tokio::io::WriteHalf<DuplexStream>,
    mut rx: mpsc::Receiver<(Instant, Vec<u8>)>,
) {
    while let Some((due, bytes)) = rx.recv().await {
        tokio::time::sleep_until(due).await;
        if wr.write_all(&bytes).await.is_err() {
            break;
        }
    }
}

/// Wire bytes for `m` with a wrong CRC (still correctly COBS-framed).
fn corrupted_wire(m: &McuMsg) -> Vec<u8> {
    let mut f = frame::build(m.msg_type(), &m.encode_payload());
    if let Some(last) = f.last_mut() {
        *last ^= 0x5A;
    }
    let mut out = cobs::encode(&f);
    out.push(0);
    out
}
