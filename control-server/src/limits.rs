//! Motion limits in two tiers (design 2026-11-01).
//!
//! 1. **Firmware absolute ceilings**: compile-time constants in the MCU
//!    (`protocol::fw_absolute` mirrors them; the real values arrive in
//!    LIMITS). The MCU clamps whatever it receives to them.
//! 2. **One current set shared by the three axes** ([`MotionLimits`]):
//!    initialised from `[limits]`, changed at runtime by the game (WS
//!    `limits`) or the operator (`POST /v1/limits`), reset to the config on
//!    every new game session, and clamped to the ceilings by [`clamp`] —
//!    the one function both the WS ack and the supervisor use, so the two
//!    never disagree.
//!
//! Everything else in `[limits]` (duty_min, Kp, FF, deadbands, the
//! FOLLOWING_ERROR / OVERSPEED thresholds) is an initial value or a protection
//! threshold: distributed to the MCUs, not changeable at runtime.

use std::sync::{Arc, Mutex};

use serde::{Deserialize, Serialize};

use crate::config::LimitsConfig;
use crate::protocol::{fw_absolute, Limits as FwLimits, SetLimits};

/// The four runtime-changeable limits.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MotionLimits {
    pub max_gain: f64,
    pub max_vel_count_s: u32,
    pub max_acc_count_s2: u32,
    pub duty_max_percent: u8,
}

impl MotionLimits {
    pub fn from_config(c: &LimitsConfig) -> Self {
        Self {
            max_gain: c.max_gain,
            max_vel_count_s: c.max_vel_count_s,
            max_acc_count_s2: c.max_acc_count_s2,
            duty_max_percent: c.duty_max_percent,
        }
    }
}

/// Ceilings the current set is clamped to. `max_gain` is always 1.0 (the
/// mix saturates in virtual 0..=1023 anyway); the others come from the
/// firmware's LIMITS (minimum over the connected axes, else
/// [`fw_absolute`]) and, for velocity, from `overspeed_count_s / 1.5`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct Ceilings {
    pub max_gain: f64,
    pub max_vel_count_s: u32,
    pub max_acc_count_s2: u32,
    pub duty_max_percent: u8,
}

/// The gain ceiling: pitch and roll both at ±1 do not saturate below 0.5,
/// and the virtual range saturates anything beyond.
pub const GAIN_CEILING: f64 = 1.0;

/// Margin between the velocity limit and the OVERSPEED threshold (the
/// bring-up record: 2× tripped on transients, 3.3× did not).
pub const OVERSPEED_MARGIN: f64 = 1.5;

impl Ceilings {
    /// Ceilings from the firmware fallback constants and the configured
    /// OVERSPEED threshold (before any axis reported LIMITS).
    pub fn fallback(overspeed_count_s: u32) -> Self {
        Self::from_absolute(
            FwAbsolute {
                duty_max_percent: fw_absolute::DUTY_MAX_PERCENT,
                max_vel_count_s: fw_absolute::MAX_VEL_COUNT_S,
                max_acc_count_s2: fw_absolute::MAX_ACC_COUNT_S2,
            },
            overspeed_count_s,
        )
    }

    pub fn from_absolute(a: FwAbsolute, overspeed_count_s: u32) -> Self {
        Self {
            max_gain: GAIN_CEILING,
            max_vel_count_s: a.max_vel_count_s.min(vel_ceiling_for_overspeed(overspeed_count_s)),
            max_acc_count_s2: a.max_acc_count_s2,
            duty_max_percent: a.duty_max_percent,
        }
    }
}

/// `overspeed / OVERSPEED_MARGIN`, floored.
pub fn vel_ceiling_for_overspeed(overspeed_count_s: u32) -> u32 {
    (f64::from(overspeed_count_s) / OVERSPEED_MARGIN).floor() as u32
}

/// Absolute ceilings one MCU reported (LIMITS).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub struct FwAbsolute {
    pub duty_max_percent: u8,
    pub max_vel_count_s: u32,
    pub max_acc_count_s2: u32,
}

impl FwAbsolute {
    pub fn from_limits(l: &FwLimits) -> Self {
        Self {
            duty_max_percent: l.abs_duty_max_percent,
            max_vel_count_s: l.abs_max_vel_count_s,
            max_acc_count_s2: l.abs_max_acc_count_s2,
        }
    }

    /// Component-wise minimum.
    pub fn min(self, o: FwAbsolute) -> FwAbsolute {
        FwAbsolute {
            duty_max_percent: self.duty_max_percent.min(o.duty_max_percent),
            max_vel_count_s: self.max_vel_count_s.min(o.max_vel_count_s),
            max_acc_count_s2: self.max_acc_count_s2.min(o.max_acc_count_s2),
        }
    }
}

/// Which field [`clamp`] reduced.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LimitedField {
    MaxGain,
    MaxVelCountS,
    MaxAccCountS2,
    DutyMaxPercent,
}

/// Clamp a requested set to the ceilings. Non-finite / negative gain
/// degrades to 0 (never panics); `duty_max_percent` 0 stays 0.
pub fn clamp(requested: &MotionLimits, c: &Ceilings) -> (MotionLimits, Vec<LimitedField>) {
    let mut limited = Vec::new();
    let gain = if requested.max_gain.is_finite() && requested.max_gain > 0.0 { requested.max_gain } else { 0.0 };
    let max_gain = if gain > c.max_gain {
        limited.push(LimitedField::MaxGain);
        c.max_gain
    } else {
        gain
    };
    let max_vel_count_s = if requested.max_vel_count_s > c.max_vel_count_s {
        limited.push(LimitedField::MaxVelCountS);
        c.max_vel_count_s
    } else {
        requested.max_vel_count_s
    };
    let max_acc_count_s2 = if requested.max_acc_count_s2 > c.max_acc_count_s2 {
        limited.push(LimitedField::MaxAccCountS2);
        c.max_acc_count_s2
    } else {
        requested.max_acc_count_s2
    };
    let duty_max_percent = if requested.duty_max_percent > c.duty_max_percent {
        limited.push(LimitedField::DutyMaxPercent);
        c.duty_max_percent
    } else {
        requested.duty_max_percent
    };
    (MotionLimits { max_gain, max_vel_count_s, max_acc_count_s2, duty_max_percent }, limited)
}

/// A partial update (WS `limits` message body / `POST /v1/limits`): only the
/// present fields change. Numbers are accepted as JSON numbers (f64) and
/// validated by [`LimitsPatch::apply`].
#[derive(Debug, Clone, Copy, PartialEq, Default, Deserialize, Serialize)]
#[serde(deny_unknown_fields, default)]
pub struct LimitsPatch {
    pub max_gain: Option<f64>,
    pub max_vel_count_s: Option<f64>,
    pub max_acc_count_s2: Option<f64>,
    pub duty_max_percent: Option<f64>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum PatchError {
    #[error("non-finite value")]
    NonFinite,
    #[error("value out of range")]
    OutOfRange,
}

impl LimitsPatch {
    pub fn is_empty(&self) -> bool {
        *self == Self::default()
    }

    /// `base` with the present fields replaced. Rules: every value finite;
    /// `max_gain >= 0`; `max_vel_count_s`, `max_acc_count_s2` > 0 and within
    /// u32; `duty_max_percent` in 1..=100. Rounded to the field's integer type.
    pub fn apply(&self, base: MotionLimits) -> Result<MotionLimits, PatchError> {
        let mut out = base;
        if let Some(g) = self.max_gain {
            if !g.is_finite() {
                return Err(PatchError::NonFinite);
            }
            if g < 0.0 {
                return Err(PatchError::OutOfRange);
            }
            out.max_gain = g;
        }
        if let Some(v) = self.max_vel_count_s {
            out.max_vel_count_s = positive_u32(v)?;
        }
        if let Some(a) = self.max_acc_count_s2 {
            out.max_acc_count_s2 = positive_u32(a)?;
        }
        if let Some(d) = self.duty_max_percent {
            if !d.is_finite() {
                return Err(PatchError::NonFinite);
            }
            let r = d.round();
            if !(1.0..=100.0).contains(&r) {
                return Err(PatchError::OutOfRange);
            }
            out.duty_max_percent = r as u8;
        }
        Ok(out)
    }
}

fn positive_u32(v: f64) -> Result<u32, PatchError> {
    if !v.is_finite() {
        return Err(PatchError::NonFinite);
    }
    let r = v.round();
    if r < 1.0 || r > f64::from(u32::MAX) {
        return Err(PatchError::OutOfRange);
    }
    Ok(r as u32)
}

/// The SET_LIMITS content for the current effective set (without request_id).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub struct FwTuning {
    pub duty_max_percent: u8,
    pub duty_min_percent: u8,
    pub kp_percent_per_100: u8,
    pub ff_count_s_per_percent: u16,
    pub deadband_count: u16,
    pub deadband_out_count: u16,
}

impl FwTuning {
    /// `effective.duty_max_percent` with the operator-only values from the
    /// config. `duty_min` never exceeds `duty_max` (the MCU would reject it).
    pub fn new(cfg: &LimitsConfig, effective: &MotionLimits) -> Self {
        Self {
            duty_max_percent: effective.duty_max_percent,
            duty_min_percent: cfg.duty_min_percent.min(effective.duty_max_percent),
            kp_percent_per_100: cfg.kp_percent_per_100,
            ff_count_s_per_percent: cfg.ff_count_s_per_percent,
            deadband_count: cfg.deadband_count,
            deadband_out_count: cfg.deadband_out_count,
        }
    }

    pub fn to_msg(self, request_id: u32) -> SetLimits {
        SetLimits {
            request_id,
            duty_max_percent: self.duty_max_percent,
            duty_min_percent: self.duty_min_percent,
            kp_percent_per_100: self.kp_percent_per_100,
            ff_count_s_per_percent: self.ff_count_s_per_percent,
            deadband_count: self.deadband_count,
            deadband_out_count: self.deadband_out_count,
        }
    }
}

/// `motion_limits` of the status snapshot.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct MotionLimitsSnapshot {
    /// In effect now (requested, clamped to the ceilings).
    pub effective: MotionLimits,
    pub requested: MotionLimits,
    pub ceiling: Ceilings,
    /// The `[limits]` values (what a new session resets to).
    pub defaults: MotionLimits,
    /// Fields of `requested` the ceilings reduced.
    pub limited: Vec<LimitedField>,
}

/// The requested set shared by the API (writer) and the control loop (reader).
pub type SharedLimits = Arc<Mutex<MotionLimits>>;

pub fn shared(initial: MotionLimits) -> SharedLimits {
    Arc::new(Mutex::new(initial))
}

/// Lock, tolerating poisoning (the value is plain data).
pub fn read(s: &SharedLimits) -> MotionLimits {
    *s.lock().unwrap_or_else(|e| e.into_inner())
}

pub fn write(s: &SharedLimits, v: MotionLimits) {
    *s.lock().unwrap_or_else(|e| e.into_inner()) = v;
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ceilings() -> Ceilings {
        Ceilings { max_gain: 1.0, max_vel_count_s: 6000, max_acc_count_s2: 40000, duty_max_percent: 60 }
    }

    #[test]
    fn clamp_reports_each_field() {
        let req = MotionLimits { max_gain: 1.5, max_vel_count_s: 7000, max_acc_count_s2: 50000, duty_max_percent: 90 };
        let (e, l) = clamp(&req, &ceilings());
        assert_eq!(e, MotionLimits { max_gain: 1.0, max_vel_count_s: 6000, max_acc_count_s2: 40000, duty_max_percent: 60 });
        assert_eq!(
            l,
            vec![LimitedField::MaxGain, LimitedField::MaxVelCountS, LimitedField::MaxAccCountS2, LimitedField::DutyMaxPercent]
        );
        let ok = MotionLimits { max_gain: 0.5, max_vel_count_s: 3000, max_acc_count_s2: 15000, duty_max_percent: 40 };
        assert_eq!(clamp(&ok, &ceilings()), (ok, vec![]));
    }

    #[test]
    fn clamp_degrades_bad_gain_to_zero() {
        let req = MotionLimits { max_gain: f64::NAN, max_vel_count_s: 1, max_acc_count_s2: 1, duty_max_percent: 1 };
        assert_eq!(clamp(&req, &ceilings()).0.max_gain, 0.0);
        let req = MotionLimits { max_gain: -0.1, ..req };
        assert_eq!(clamp(&req, &ceilings()).0.max_gain, 0.0);
    }

    #[test]
    fn vel_ceiling_from_overspeed() {
        assert_eq!(vel_ceiling_for_overspeed(9000), 6000);
        assert_eq!(vel_ceiling_for_overspeed(2000), 1333);
        assert_eq!(Ceilings::fallback(2000).max_vel_count_s, 1333);
        assert_eq!(Ceilings::fallback(9000).max_vel_count_s, 6000);
    }

    #[test]
    fn patch_rules() {
        let base = MotionLimits { max_gain: 0.5, max_vel_count_s: 3000, max_acc_count_s2: 15000, duty_max_percent: 40 };
        let p = LimitsPatch { max_vel_count_s: Some(1234.6), ..Default::default() };
        assert_eq!(p.apply(base).unwrap(), MotionLimits { max_vel_count_s: 1235, ..base });
        assert_eq!(LimitsPatch { max_gain: Some(f64::NAN), ..Default::default() }.apply(base), Err(PatchError::NonFinite));
        assert_eq!(LimitsPatch { max_gain: Some(-0.1), ..Default::default() }.apply(base), Err(PatchError::OutOfRange));
        assert_eq!(LimitsPatch { max_vel_count_s: Some(0.0), ..Default::default() }.apply(base), Err(PatchError::OutOfRange));
        assert_eq!(LimitsPatch { max_acc_count_s2: Some(-5.0), ..Default::default() }.apply(base), Err(PatchError::OutOfRange));
        assert_eq!(LimitsPatch { duty_max_percent: Some(0.4), ..Default::default() }.apply(base), Err(PatchError::OutOfRange));
        assert_eq!(LimitsPatch { duty_max_percent: Some(101.0), ..Default::default() }.apply(base), Err(PatchError::OutOfRange));
        assert_eq!(LimitsPatch { duty_max_percent: Some(100.0), ..Default::default() }.apply(base).unwrap().duty_max_percent, 100);
        assert!(LimitsPatch::default().is_empty());
        assert_eq!(LimitsPatch::default().apply(base).unwrap(), base);
    }

    #[test]
    fn tuning_keeps_duty_min_below_duty_max() {
        let cfg = LimitsConfig { duty_min_percent: 7, ..LimitsConfig::default() };
        let e = MotionLimits { max_gain: 0.5, max_vel_count_s: 3000, max_acc_count_s2: 15000, duty_max_percent: 5 };
        let t = FwTuning::new(&cfg, &e);
        assert_eq!((t.duty_max_percent, t.duty_min_percent), (5, 5));
        assert_eq!(t.to_msg(9).request_id, 9);
    }
}
