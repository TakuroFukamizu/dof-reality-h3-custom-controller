//! h3game-v1 compatibility (architecture §2.2).
//!
//! The mixer is the existing application contract and MUST NOT be "corrected":
//! ```text
//! g  = gain * 511
//! m1 = 512 + g * (roll + pitch)
//! m2 = 512 + g * (roll - pitch)
//! m3 = 512 + g * yaw
//! ```
//! (docs/cores3-controller/software.md writes a different sign arrangement for
//! the kinematics; that is a sketch, not the contract. Keep h3_direct.py's.)
//! Direction differences of the new count coordinates are absorbed by the
//! per-axis calibration, never here.

use crate::types::{LimitReason, Pose, VirtualPos, AXIS_COUNT, VIRTUAL_CENTER};

/// Result of validating a game pose + gain for use.
#[derive(Debug, Clone, PartialEq)]
pub struct CheckedPose {
    pub pose: Pose,
    pub effective_gain: f64,
    pub limits: Vec<LimitReason>,
}

#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum PoseError {
    #[error("non-finite value in {0}")]
    NonFinite(&'static str),
    #[error("{0} outside -1..=1 and clamping is disabled")]
    OutOfRange(&'static str),
}

/// Validate a pose: reject non-finite; clamp (or reject if `clamp_out_of_range`
/// is false) components outside -1..=1; gain clamped to 0..=max_gain with
/// GainNegative / GainLimited reported. Gain is applied once, later, by `mix`.
pub fn check_pose(pose: Pose, requested_gain: f64, max_gain: f64, clamp_out_of_range: bool)
    -> Result<CheckedPose, PoseError> {
    let fields = [("pitch", pose.pitch), ("roll", pose.roll), ("yaw", pose.yaw), ("gain", requested_gain)];
    if let Some((name, _)) = fields.iter().find(|(_, v)| !v.is_finite()) {
        return Err(PoseError::NonFinite(name));
    }
    let mut limits = Vec::new();
    let mut clamped = false;
    let mut fix = |name: &'static str, v: f64| -> Result<f64, PoseError> {
        if (-1.0..=1.0).contains(&v) {
            Ok(v)
        } else if clamp_out_of_range {
            clamped = true;
            Ok(v.signum())
        } else {
            Err(PoseError::OutOfRange(name))
        }
    };
    let pose = Pose {
        pitch: fix("pitch", pose.pitch)?,
        roll: fix("roll", pose.roll)?,
        yaw: fix("yaw", pose.yaw)?,
    };
    if clamped {
        limits.push(LimitReason::PoseClamped);
    }
    // A bad max_gain (negative / NaN) degrades to 0 instead of panicking.
    let max_gain = if max_gain.is_finite() && max_gain > 0.0 { max_gain } else { 0.0 };
    let effective_gain = if requested_gain < 0.0 {
        limits.push(LimitReason::GainNegative);
        0.0
    } else if requested_gain > max_gain {
        limits.push(LimitReason::GainLimited);
        max_gain
    } else {
        requested_gain
    };
    Ok(CheckedPose { pose, effective_gain, limits })
}

/// The h3game-v1 mixer, unclamped float output (may exceed 0..=1023 when gain
/// is large; saturation is a separate, reported step).
pub fn mix(pose: Pose, gain: f64) -> [VirtualPos; AXIS_COUNT] {
    // Same operation order as h3_direct.py so results are bit-identical.
    let g = gain * 511.0;
    [
        VirtualPos(VIRTUAL_CENTER + g * (pose.roll + pose.pitch)),
        VirtualPos(VIRTUAL_CENTER + g * (pose.roll - pose.pitch)),
        VirtualPos(VIRTUAL_CENTER + g * pose.yaw),
    ]
}

/// Saturate virtual positions into [virtual_min, virtual_max]; returns whether any saturated.
/// NaN inputs are mapped to `virtual_min` and reported as saturated.
pub fn saturate(v: [VirtualPos; AXIS_COUNT], virtual_min: f64, virtual_max: f64)
    -> ([VirtualPos; AXIS_COUNT], bool) {
    let mut any = false;
    let out = v.map(|VirtualPos(x)| {
        // Manual min/max: f64::clamp panics on NaN or inverted bounds.
        let y = if x.is_nan() || x < virtual_min {
            virtual_min
        } else if x > virtual_max {
            virtual_max
        } else {
            x
        };
        if y != x || x.is_nan() {
            any = true;
        }
        VirtualPos(y)
    });
    (out, any)
}

/// Bit-faithful model of legacy `h3_direct.H3` send behaviour, used ONLY as a
/// reference in tests and for documenting intentional differences. Semantics:
/// `cur` starts at [512.0;3] and accumulates as float with per-call step limited
/// to ±max_step toward the (unclipped) mixed target; `cur` keeps unclipped
/// values; each send emits `clamp(0, 1023, int(cur))` where Python `int()`
/// truncates toward zero.
///
/// `new` does not model the 30 centre sends `H3.__init__` performs; they leave
/// `cur` at 512 and so do not change later output.
#[derive(Debug, Clone)]
pub struct LegacyH3Model {
    pub gain: f64,
    pub max_step: f64,
    pub cur: [f64; AXIS_COUNT],
}

impl LegacyH3Model {
    pub fn new(gain: f64, max_step: f64) -> Self {
        Self { gain, max_step, cur: [VIRTUAL_CENTER; AXIS_COUNT] }
    }
    /// `H3.move(pitch, roll, yaw)` → the three values written to the wire.
    pub fn move_(&mut self, pose: Pose) -> [u16; AXIS_COUNT] {
        let [m1, m2, m3] = mix(pose, self.gain);
        self.set_motors([m1.0, m2.0, m3.0])
    }
    /// `H3.set_motors(m1, m2, m3)`.
    pub fn set_motors(&mut self, target: [f64; AXIS_COUNT]) -> [u16; AXIS_COUNT] {
        let ms = self.max_step;
        for (cur, tgt) in self.cur.iter_mut().zip(target) {
            // Python: d = max(-ms, min(ms, tgt - cur)), including its NaN behaviour.
            let x = tgt - *cur;
            let t = if x < ms { x } else { ms };
            let d = if t > -ms { t } else { -ms };
            *cur += d;
        }
        self.cur.map(Self::send_value)
    }
    /// `H3.center()`: `steps` calls of `set_motors(512, 512, 512)`; returns each send.
    pub fn center(&mut self, steps: usize) -> Vec<[u16; AXIS_COUNT]> {
        (0..steps).map(|_| self.set_motors([VIRTUAL_CENTER; AXIS_COUNT])).collect()
    }
    /// `H3._send` value: `max(0, min(1023, int(v)))`. `as i64` truncates toward
    /// zero like Python `int()` (and saturates instead of raising on inf/NaN).
    pub fn send_value(v: f64) -> u16 {
        (v as i64).clamp(0, 1023) as u16
    }
    /// Legacy wire bytes for one send: for each of A,B,C `[` ch hi lo `]`.
    pub fn wire_bytes(values: [u16; AXIS_COUNT]) -> Vec<u8> {
        let mut out = Vec::with_capacity(5 * AXIS_COUNT);
        for (ch, v) in [b'A', b'B', b'C'].into_iter().zip(values) {
            let [hi, lo] = v.to_be_bytes();
            out.extend_from_slice(&[b'[', ch, hi, lo, b']']);
        }
        out
    }
}
