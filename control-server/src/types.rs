//! Shared domain types. Units are part of the names on purpose:
//! a game pose is dimensionless, a virtual position is the legacy 0..1023
//! target coordinate, and a count is an encoder count on one axis.
//! None of them is a PWM duty.

use serde::{Deserialize, Serialize};
use std::fmt;

/// Number of motion axes on the H3.
pub const AXIS_COUNT: usize = 3;

/// Legacy h3game neutral virtual position.
pub const VIRTUAL_CENTER: f64 = 512.0;
/// Legacy h3game virtual range.
pub const VIRTUAL_MIN: f64 = 0.0;
pub const VIRTUAL_MAX: f64 = 1023.0;

/// Physical axis identifier, 1..=3. Index 0 of `[_; AXIS_COUNT]` arrays is axis 1.
/// Axis 1 = right-front lift (motor #1), axis 2 = left lift (motor #2),
/// axis 3 = rear yaw (motor #3). The ID comes from configuration that maps a
/// MCU `device_id` to a physical axis, never from USB enumeration order.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(try_from = "u8", into = "u8")]
pub struct AxisId(u8);

impl AxisId {
    pub const ALL: [AxisId; AXIS_COUNT] = [AxisId(1), AxisId(2), AxisId(3)];

    pub fn new(id: u8) -> Option<Self> {
        (1..=AXIS_COUNT as u8).contains(&id).then_some(AxisId(id))
    }
    pub fn get(self) -> u8 {
        self.0
    }
    /// 0-based index into per-axis arrays.
    pub fn index(self) -> usize {
        (self.0 - 1) as usize
    }
    /// Inverse of [`AxisId::index`]; `None` for any index ≥ 3 (no truncation).
    pub fn from_index(i: usize) -> Option<Self> {
        u8::try_from(i).ok().and_then(|v| v.checked_add(1)).and_then(Self::new)
    }
}

impl TryFrom<u8> for AxisId {
    type Error = String;
    fn try_from(v: u8) -> Result<Self, Self::Error> {
        AxisId::new(v).ok_or_else(|| format!("axis_id must be 1..=3, got {v}"))
    }
}
impl From<AxisId> for u8 {
    fn from(a: AxisId) -> u8 {
        a.0
    }
}
impl fmt::Display for AxisId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "axis{}", self.0)
    }
}

/// MCU unique identifier reported in HELLO (derived from the eFuse MAC).
/// Serialized in config/JSON as a 16-digit lowercase hex string.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(try_from = "String", into = "String")]
pub struct DeviceId(pub u64);

impl DeviceId {
    /// All-zero placeholder ("not yet read from the real MCU's HELLO").
    /// Config validation accepts it (any number of times), but an axis with an
    /// unassigned device_id can never be matched or ARMed.
    pub fn is_unassigned(&self) -> bool {
        self.0 == 0
    }
}

impl TryFrom<String> for DeviceId {
    type Error = String;
    fn try_from(s: String) -> Result<Self, Self::Error> {
        let t = s.trim().trim_start_matches("0x");
        u64::from_str_radix(t, 16)
            .map(DeviceId)
            .map_err(|e| format!("device_id must be hex: {e}"))
    }
}
impl From<DeviceId> for String {
    fn from(d: DeviceId) -> String {
        format!("{:016x}", d.0)
    }
}
impl fmt::Display for DeviceId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:016x}", self.0)
    }
}

/// Random per-boot identifier reported by the MCU. A change invalidates the
/// runtime calibration (incremental encoders lose their reference on reset).
pub type BootId = u32;

/// Dimensionless game pose, h3game contract: each of pitch/roll/yaw in -1..=1,
/// already inverted by the browser if the user ticked an invert box.
/// Not degrees, not radians.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct Pose {
    pub pitch: f64,
    pub roll: f64,
    pub yaw: f64,
}

impl Pose {
    pub const ZERO: Pose = Pose { pitch: 0.0, roll: 0.0, yaw: 0.0 };
}

/// Legacy virtual target position on one axis, 0..=1023 with 512 neutral.
/// This is the h3game/H3 target coordinate, NOT an MD13S PWM duty, even though
/// both happen to use 10 bits.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default, Serialize, Deserialize)]
pub struct VirtualPos(pub f64);

/// Encoder count on one axis, in the axis's referenced coordinate
/// (4x quadrature counts; motor #1 ≈ 400 counts / motor revolution).
pub type Counts = i32;

/// Why a requested value was changed before use. Reported back to the game and
/// in status so a limit is never silent.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LimitReason {
    /// A pose component was outside -1..=1 and was clamped (compat mode).
    PoseClamped,
    /// Requested gain exceeded the machine profile's max gain.
    GainLimited,
    /// Requested gain was negative and was raised to 0.
    GainNegative,
    /// Mixed virtual position fell outside 0..=1023 (or the profile's virtual
    /// limits) and was saturated.
    VirtualSaturated,
    /// Planner velocity limit reduced the step this tick (all axes scaled together).
    VelocityLimited,
    /// Planner acceleration limit reduced the step this tick.
    AccelerationLimited,
    /// Target count was clamped to the axis operational soft limits.
    SoftLimited,
}

/// Whole-rig state (server side), architecture §10.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum RigState {
    /// Not all configured axes are connected and identified.
    Offline,
    /// All three axes identified; nothing armed. Transitional: the supervisor
    /// immediately classifies into CalibrationRequired or ReadyToArm.
    ConnectedDisarmed,
    /// Connected but some axis lacks a valid calibration/reference for its
    /// current boot_id, or configuration is incomplete. ARM refused.
    CalibrationRequired,
    /// Every precondition for ARM holds.
    ReadyToArm,
    /// ARM_PREPARE sent; waiting for all axes to report HOLDING.
    Arming,
    /// All axes holding position under position control (brakes released).
    Holding,
    /// Following game poses.
    Active,
    /// Coordinated deceleration to hold, then brake (game timeout, admin stop).
    Stopping,
    /// Brakes applied, drive disabled after a stop. Re-ARM required.
    BrakedDisarmed,
    /// Latched fault. Only an explicit fault reset (with causes cleared) leaves it.
    FaultLatched,
}

/// MCU state as reported in STATUS (protocol §3.1).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
#[repr(u8)]
pub enum McuState {
    Boot = 0,
    Disarmed = 1,
    Arming = 2,
    Holding = 3,
    Active = 4,
    Stopping = 5,
    Disarming = 6,
    Fault = 7,
}

impl McuState {
    pub fn from_u8(v: u8) -> Option<Self> {
        use McuState::*;
        Some(match v {
            0 => Boot,
            1 => Disarmed,
            2 => Arming,
            3 => Holding,
            4 => Active,
            5 => Stopping,
            6 => Disarming,
            7 => Fault,
            _ => return None,
        })
    }
}

/// MCU fault code (protocol §3.2). Unknown wire values map to `Unknown`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum FaultCode {
    None,
    CommandTimeout,
    FollowingError,
    Overspeed,
    SoftLimit,
    EncoderFault,
    ControlOverrun,
    ConfigInvalid,
    StopTimeout,
    PowerFault,
    HostFault,
    NotReferenced,
    Unknown,
}

impl FaultCode {
    pub fn to_u16(self) -> u16 {
        use FaultCode::*;
        match self {
            None => 0,
            CommandTimeout => 1,
            FollowingError => 2,
            Overspeed => 3,
            SoftLimit => 4,
            EncoderFault => 5,
            ControlOverrun => 6,
            ConfigInvalid => 7,
            StopTimeout => 8,
            PowerFault => 9,
            HostFault => 10,
            NotReferenced => 11,
            Unknown => 0xFFFF,
        }
    }
    pub fn from_u16(v: u16) -> Self {
        use FaultCode::*;
        match v {
            0 => None,
            1 => CommandTimeout,
            2 => FollowingError,
            3 => Overspeed,
            4 => SoftLimit,
            5 => EncoderFault,
            6 => ControlOverrun,
            7 => ConfigInvalid,
            8 => StopTimeout,
            9 => PowerFault,
            10 => HostFault,
            11 => NotReferenced,
            _ => Unknown,
        }
    }
}

/// Server-side reasons for latching a rig fault. Serialized in status.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum RigFault {
    /// An axis reported a fault.
    AxisFault { axis: AxisId, code: FaultCode },
    /// An axis link dropped (USB unplugged, STATUS silent past the deadline).
    AxisDisconnected { axis: AxisId },
    /// An axis rebooted (boot_id changed) while armed.
    AxisRebooted { axis: AxisId },
    /// An axis left the state the supervisor commanded (e.g. dropped to DISARMED while ACTIVE).
    AxisStateMismatch { axis: AxisId, expected: McuState, actual: McuState },
    /// ARMING did not reach HOLDING on all axes in time.
    ArmTimeout,
    /// STOPPING did not settle in time.
    StopTimeout,
    /// The three axes' last applied group_frame_id diverged beyond the limit.
    FrameSkew { max_skew_frames: u32 },
    /// The operator asked for a fault (admin API).
    Operator { note: String },
}

/// Status flags bitfield (protocol §3.4).
pub mod status_flags {
    pub const POWER_ON_CMD: u16 = 1 << 0;
    pub const BRAKE_RELEASE_CMD: u16 = 1 << 1;
    pub const POSITION_REFERENCED: u16 = 1 << 2;
    pub const CONFIGURED: u16 = 1 << 3;
    pub const POWER_CONTROLLER: u16 = 1 << 4;
    pub const CURRENT_VALID: u16 = 1 << 5;
    pub const CLOCK_SYNCED: u16 = 1 << 6;
}
