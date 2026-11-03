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
//! **Operator cap on the game channel (Issue #15).** The game may only
//! *lower* the operator's limits: a field written over the WS game channel
//! (no token) is held to the `[limits]` value ([`OperatorCap`], reported as
//! `operator_limited`); a field written by `POST /v1/limits` (admin, Bearer)
//! may exceed it up to the firmware ceilings. The shared request
//! ([`LimitsRequest`]) therefore remembers, per field, whether the game wrote
//! it last; a new session clears that together with the values.
//!
//! Everything else in `[limits]` (duty_min, Kp, FF, deadbands, the
//! FOLLOWING_ERROR / OVERSPEED thresholds) is an initial value or a protection
//! threshold: distributed to the MCUs, not changeable at runtime.
//!
//! **Stop-time rule (Issue #15 H2).** A STOP decelerates the planner at the
//! effective `max_acc_count_s2`; the MCU gives up after `mcu_stop_timeout_ms`
//! and the server after `stop_settle_timeout_ms`, and both end in a FAULT that
//! brakes at whatever speed is left. So the worst-case stop time
//! `max_vel / max_acc` must fit inside [`stop_deadline`] (a fraction of the
//! shorter timeout). [`clamp`] enforces it on every requested set — raising
//! `max_acc` to the bound, or, when the acceleration ceiling cannot deliver
//! that, lowering `max_vel` instead — and reports `StopTimeLimited`;
//! `Config::validate` enforces it on `[limits]` and on the firmware ceilings.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use serde::{Deserialize, Serialize};

use crate::config::{LimitsConfig, TimingConfig};
use crate::protocol::{fw_absolute, Limits as FwLimits, SetLimits};
use crate::types::LimitReason;

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

/// Fraction of the shorter stop timeout a worst-case deceleration may use
/// (the rest covers link latency, the MCU's own ramp and the settle check).
pub const STOP_TIME_FRACTION: f64 = 0.8;

/// The deadline a controlled stop from `max_vel` at `max_acc` must meet:
/// `STOP_TIME_FRACTION × min(mcu_stop_timeout_ms, stop_settle_timeout_ms)`.
/// The one definition the config check and the runtime clamp share.
pub fn stop_deadline(t: &TimingConfig) -> Duration {
    let ms = u64::from(t.mcu_stop_timeout_ms).min(t.stop_settle_timeout_ms);
    Duration::from_secs_f64(ms as f64 / 1000.0 * STOP_TIME_FRACTION)
}

/// Worst-case time (s) to decelerate from `max_vel` at `max_acc`
/// (`max_acc` 0 → infinite).
pub fn stop_time_s(max_vel_count_s: u32, max_acc_count_s2: u32) -> f64 {
    if max_acc_count_s2 == 0 {
        f64::INFINITY
    } else {
        f64::from(max_vel_count_s) / f64::from(max_acc_count_s2)
    }
}

/// Smallest `max_acc_count_s2` that stops `max_vel` within `deadline`
/// (ceil of `vel / deadline`; a zero deadline → `u32::MAX`).
pub fn min_acc_for_stop(max_vel_count_s: u32, deadline: Duration) -> u32 {
    let d = deadline.as_secs_f64();
    if d <= 0.0 {
        return u32::MAX;
    }
    let a = (f64::from(max_vel_count_s) / d).ceil();
    if a >= f64::from(u32::MAX) {
        u32::MAX
    } else {
        a as u32
    }
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

/// Which field [`clamp`] changed (reduced to a ceiling, or moved by the
/// stop-time rule).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LimitedField {
    MaxGain,
    MaxVelCountS,
    MaxAccCountS2,
    DutyMaxPercent,
}

impl LimitedField {
    pub const ALL: [LimitedField; 4] =
        [LimitedField::MaxGain, LimitedField::MaxVelCountS, LimitedField::MaxAccCountS2, LimitedField::DutyMaxPercent];

    fn bit(self) -> u8 {
        match self {
            LimitedField::MaxGain => 1,
            LimitedField::MaxVelCountS => 2,
            LimitedField::MaxAccCountS2 => 4,
            LimitedField::DutyMaxPercent => 8,
        }
    }
}

/// A set of [`LimitedField`]s (which fields the game channel wrote last).
/// Serializes as the member list in field order.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct FieldSet(u8);

impl FieldSet {
    pub const NONE: FieldSet = FieldSet(0);
    pub const ALL: FieldSet = FieldSet(15);

    pub fn contains(self, f: LimitedField) -> bool {
        self.0 & f.bit() != 0
    }
    pub fn insert(&mut self, f: LimitedField) {
        self.0 |= f.bit();
    }
    pub fn remove(&mut self, f: LimitedField) {
        self.0 &= !f.bit();
    }
    pub fn is_empty(self) -> bool {
        self.0 == 0
    }
    /// The members in field order.
    pub fn to_vec(self) -> Vec<LimitedField> {
        LimitedField::ALL.into_iter().filter(|f| self.contains(*f)).collect()
    }
}

impl Serialize for FieldSet {
    fn serialize<S: serde::Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        self.to_vec().serialize(s)
    }
}

/// The operator's cap on the game channel: `max` (the `[limits]` values)
/// applies to the fields in `fields` (those the game wrote last). A field the
/// admin wrote, or nobody since the session reset, is capped by the firmware
/// ceilings only.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OperatorCap {
    pub max: MotionLimits,
    pub fields: FieldSet,
}

impl OperatorCap {
    /// No field under the cap (admin-only / config values).
    pub fn none(max: MotionLimits) -> Self {
        Self { max, fields: FieldSet::NONE }
    }
}

/// Who wrote a limits patch.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Source {
    /// WS `/v1/motion` `{"type":"limits"}` (no token): may only lower.
    Game,
    /// `POST /v1/limits` (Bearer): up to the firmware ceilings.
    Admin,
}

/// The shared requested set with its provenance: which fields the game
/// channel wrote last (held to the operator's values by [`clamp`]).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LimitsRequest {
    pub requested: MotionLimits,
    pub game_fields: FieldSet,
}

impl LimitsRequest {
    /// Fresh from the config / a session reset: nothing game-written.
    pub fn new(requested: MotionLimits) -> Self {
        Self { requested, game_fields: FieldSet::NONE }
    }

    /// Apply `patch` written by `source`: the present fields take the value
    /// and the provenance (`Game` → under the operator cap; `Admin` →
    /// released). Nothing changes on an error.
    pub fn apply(&mut self, patch: &LimitsPatch, source: Source) -> Result<(), PatchError> {
        self.requested = patch.apply(self.requested)?;
        for f in patch.fields() {
            match source {
                Source::Game => self.game_fields.insert(f),
                Source::Admin => self.game_fields.remove(f),
            }
        }
        Ok(())
    }

    /// The cap [`clamp`] applies to this request given the operator's values.
    pub fn operator_cap(&self, max: MotionLimits) -> OperatorCap {
        OperatorCap { max, fields: self.game_fields }
    }
}

/// Result of [`clamp`].
#[derive(Debug, Clone, PartialEq)]
pub struct Clamped {
    /// The set to use.
    pub effective: MotionLimits,
    /// Fields that differ from the request (each at most once, in field order).
    pub limited: Vec<LimitedField>,
    /// Why, in this order: `CeilingClamped` (a field exceeded a firmware
    /// ceiling), `OperatorLimited` (a game-written field exceeded the
    /// operator's value) and/or `StopTimeLimited` (the stop-time rule moved
    /// `max_acc` up or `max_vel` down). Empty when `limited` is.
    pub reasons: Vec<LimitReason>,
}

/// Clamp a requested set to the ceilings — the firmware ceilings `c`, and for
/// the fields in `op.fields` (game-written) also the operator's values
/// `op.max`; the tighter bound wins and names the reason — then enforce the
/// stop-time rule (module docs): `max_acc ≥ ceil(max_vel / deadline)`,
/// evaluated on the *effective* `max_vel` (so a patch that changes both is
/// judged jointly). When that bound exceeds the acceleration ceiling,
/// `max_acc` takes the ceiling and `max_vel` is lowered to
/// `floor(ceiling × deadline)` instead. Non-finite / negative gain degrades
/// to 0 (never panics); `duty_max_percent` 0 stays 0.
pub fn clamp(requested: &MotionLimits, c: &Ceilings, op: &OperatorCap, stop_deadline: Duration) -> Clamped {
    let mut ceiling = Vec::new();
    let mut operator = Vec::new();
    // `v` capped at the firmware ceiling `fw` and, for a game-written field,
    // at the operator's `opmax`; records which bound was the binding one.
    let mut cap = |field: LimitedField, v: f64, fw: f64, opmax: f64| -> f64 {
        let by_operator = op.fields.contains(field) && opmax < fw;
        let bound = if by_operator { opmax } else { fw };
        if v > bound {
            if by_operator {
                operator.push(field);
            } else {
                ceiling.push(field);
            }
            bound
        } else {
            v
        }
    };
    let gain = if requested.max_gain.is_finite() && requested.max_gain > 0.0 { requested.max_gain } else { 0.0 };
    let max_gain = cap(LimitedField::MaxGain, gain, c.max_gain, op.max.max_gain);
    // Integer fields: the bound is one of the integer inputs, so `as` is exact.
    let mut max_vel_count_s = cap(
        LimitedField::MaxVelCountS,
        f64::from(requested.max_vel_count_s),
        f64::from(c.max_vel_count_s),
        f64::from(op.max.max_vel_count_s),
    ) as u32;
    let mut max_acc_count_s2 = cap(
        LimitedField::MaxAccCountS2,
        f64::from(requested.max_acc_count_s2),
        f64::from(c.max_acc_count_s2),
        f64::from(op.max.max_acc_count_s2),
    ) as u32;
    let duty_max_percent = cap(
        LimitedField::DutyMaxPercent,
        f64::from(requested.duty_max_percent),
        f64::from(c.duty_max_percent),
        f64::from(op.max.duty_max_percent),
    ) as u8;

    // Stop-time rule, judged on the capped velocity.
    let mut stop = Vec::new();
    let min_acc = min_acc_for_stop(max_vel_count_s, stop_deadline);
    if max_acc_count_s2 < min_acc {
        if min_acc <= c.max_acc_count_s2 {
            max_acc_count_s2 = min_acc;
            stop.push(LimitedField::MaxAccCountS2);
        } else {
            // Even the ceiling cannot stop this speed in time: cap the speed.
            if max_acc_count_s2 != c.max_acc_count_s2 {
                max_acc_count_s2 = c.max_acc_count_s2;
                stop.push(LimitedField::MaxAccCountS2);
            }
            let v = (f64::from(c.max_acc_count_s2) * stop_deadline.as_secs_f64()).floor();
            let v = if v.is_finite() && v >= 0.0 { v.min(f64::from(u32::MAX)) as u32 } else { 0 };
            if v < max_vel_count_s {
                max_vel_count_s = v;
                stop.push(LimitedField::MaxVelCountS);
            }
        }
    }

    let mut reasons = Vec::new();
    if !ceiling.is_empty() {
        reasons.push(LimitReason::CeilingClamped);
    }
    if !operator.is_empty() {
        reasons.push(LimitReason::OperatorLimited);
    }
    if !stop.is_empty() {
        reasons.push(LimitReason::StopTimeLimited);
    }
    let limited = LimitedField::ALL
        .into_iter()
        .filter(|f| ceiling.contains(f) || operator.contains(f) || stop.contains(f))
        .collect();
    Clamped {
        effective: MotionLimits { max_gain, max_vel_count_s, max_acc_count_s2, duty_max_percent },
        limited,
        reasons,
    }
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

    /// The fields present in the patch, in field order.
    pub fn fields(&self) -> Vec<LimitedField> {
        let present = [
            (self.max_gain.is_some(), LimitedField::MaxGain),
            (self.max_vel_count_s.is_some(), LimitedField::MaxVelCountS),
            (self.max_acc_count_s2.is_some(), LimitedField::MaxAccCountS2),
            (self.duty_max_percent.is_some(), LimitedField::DutyMaxPercent),
        ];
        present.into_iter().filter(|(p, _)| *p).map(|(_, f)| f).collect()
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
    /// Controlled-stop deceleration: the operator `max_acc` of the set this
    /// tuning was built from, clamped to the firmware ceiling. Not a duty
    /// field — see [`FwTuning::same_duty`].
    pub stop_acc_count_s2: u16,
}

/// Operator `max_acc` for the SET_LIMITS `stop_acc` field. The firmware
/// ceiling fits in the u16 (40000 < 65535). 0 would mean "unset" on the MCU,
/// so a real limit is never sent as 0 (`max_acc` is at least 1 after clamp).
fn stop_acc_wire(max_acc: u32) -> u16 {
    let capped = max_acc.min(fw_absolute::MAX_ACC_COUNT_S2).min(u32::from(u16::MAX));
    u16::try_from(capped).unwrap_or(u16::MAX)
}

impl FwTuning {
    /// `effective.duty_max_percent` with the operator-only values from the
    /// config. `duty_min` never exceeds `duty_max` (the MCU would reject it).
    /// `stop_acc_count_s2` is `effective.max_acc_count_s2` (already ceiling-
    /// clamped and stop-time-raised).
    pub fn new(cfg: &LimitsConfig, effective: &MotionLimits) -> Self {
        Self {
            duty_max_percent: effective.duty_max_percent,
            duty_min_percent: cfg.duty_min_percent.min(effective.duty_max_percent),
            kp_percent_per_100: cfg.kp_percent_per_100,
            ff_count_s_per_percent: cfg.ff_count_s_per_percent,
            deadband_count: cfg.deadband_count,
            deadband_out_count: cfg.deadband_out_count,
            stop_acc_count_s2: stop_acc_wire(effective.max_acc_count_s2),
        }
    }

    /// Duty / Kp / FF / deadband only. Acceleration is the controlled-stop
    /// rate and has its own confirmation (`LIMITS.stop_acc`), not this
    /// comparison. Issue #15 M3 still waits on the duty cap.
    pub fn same_duty(self, other: Self) -> bool {
        self.duty_max_percent == other.duty_max_percent
            && self.duty_min_percent == other.duty_min_percent
            && self.kp_percent_per_100 == other.kp_percent_per_100
            && self.ff_count_s_per_percent == other.ff_count_s_per_percent
            && self.deadband_count == other.deadband_count
            && self.deadband_out_count == other.deadband_out_count
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
            stop_acc_count_s2: self.stop_acc_count_s2,
        }
    }
}

/// `motion_limits` of the status snapshot.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct MotionLimitsSnapshot {
    /// In effect now: what the planner / SET_POSITION use and the duty cap
    /// the MCUs confirmed. Lags the clamped `requested` while `pending`.
    pub effective: MotionLimits,
    pub requested: MotionLimits,
    pub ceiling: Ceilings,
    /// The `[limits]` values (what a new session resets to).
    pub defaults: MotionLimits,
    /// The operator's cap on the game channel (the `[limits]` values): a
    /// field in `game_fields` never exceeds it (Issue #15).
    pub operator_max: MotionLimits,
    /// Fields the game channel (WS `limits`) wrote last: held to
    /// `operator_max`. Cleared by a new session; released field by field
    /// when `POST /v1/limits` writes them.
    pub game_fields: FieldSet,
    /// Fields of `requested` that [`clamp`] changed.
    pub limited: Vec<LimitedField>,
    /// Why (`ceiling_clamped` / `operator_limited` / `stop_time_limited`).
    pub reasons: Vec<LimitReason>,
    /// The clamped request changes the duty cap or the stop rate, and not
    /// every connected axis has confirmed it yet. `effective` keeps the
    /// confirmed duty cap (only lowered gain and velocity are taken at
    /// once). Acceleration may rise only after every axis has echoed that
    /// `stop_acc`; a lower rate applies only while it stays at or below the
    /// echoed stop deceleration.
    pub pending: bool,
}

/// The requested set (with its provenance) shared by the API (writer) and
/// the control loop (reader).
pub type SharedLimits = Arc<Mutex<LimitsRequest>>;

pub fn shared(initial: MotionLimits) -> SharedLimits {
    Arc::new(Mutex::new(LimitsRequest::new(initial)))
}

/// Lock, tolerating poisoning (the value is plain data).
pub fn read(s: &SharedLimits) -> LimitsRequest {
    *s.lock().unwrap_or_else(|e| e.into_inner())
}

pub fn write(s: &SharedLimits, v: LimitsRequest) {
    *s.lock().unwrap_or_else(|e| e.into_inner()) = v;
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ceilings() -> Ceilings {
        Ceilings { max_gain: 1.0, max_vel_count_s: 6000, max_acc_count_s2: 40000, duty_max_percent: 60 }
    }

    /// The operator's values (sim.toml) with nothing game-written.
    fn no_cap() -> OperatorCap {
        OperatorCap::none(MotionLimits { max_gain: 0.5, max_vel_count_s: 3000, max_acc_count_s2: 15000, duty_max_percent: 40 })
    }

    fn game_cap() -> OperatorCap {
        OperatorCap { fields: FieldSet::ALL, ..no_cap() }
    }

    /// `clamp` without an operator cap (admin / config values).
    fn clamp(requested: &MotionLimits, c: &Ceilings, d: Duration) -> Clamped {
        super::clamp(requested, c, &no_cap(), d)
    }

    /// 0.8 × min(1500, 3000) ms, the default timing.
    const DEADLINE: Duration = Duration::from_millis(1200);

    #[test]
    fn clamp_reports_each_field() {
        let req = MotionLimits { max_gain: 1.5, max_vel_count_s: 7000, max_acc_count_s2: 50000, duty_max_percent: 90 };
        let c = clamp(&req, &ceilings(), DEADLINE);
        assert_eq!(
            c.effective,
            MotionLimits { max_gain: 1.0, max_vel_count_s: 6000, max_acc_count_s2: 40000, duty_max_percent: 60 }
        );
        assert_eq!(
            c.limited,
            vec![LimitedField::MaxGain, LimitedField::MaxVelCountS, LimitedField::MaxAccCountS2, LimitedField::DutyMaxPercent]
        );
        assert_eq!(c.reasons, vec![LimitReason::CeilingClamped]);
        let ok = MotionLimits { max_gain: 0.5, max_vel_count_s: 3000, max_acc_count_s2: 15000, duty_max_percent: 40 };
        assert_eq!(clamp(&ok, &ceilings(), DEADLINE), Clamped { effective: ok, limited: vec![], reasons: vec![] });
    }

    #[test]
    fn clamp_degrades_bad_gain_to_zero() {
        let req = MotionLimits { max_gain: f64::NAN, max_vel_count_s: 1, max_acc_count_s2: 1, duty_max_percent: 1 };
        assert_eq!(clamp(&req, &ceilings(), DEADLINE).effective.max_gain, 0.0);
        let req = MotionLimits { max_gain: -0.1, ..req };
        assert_eq!(clamp(&req, &ceilings(), DEADLINE).effective.max_gain, 0.0);
    }

    #[test]
    fn stop_deadline_is_a_fraction_of_the_shorter_timeout() {
        let t = TimingConfig::default(); // mcu_stop 1500, stop_settle 3000
        assert_eq!(stop_deadline(&t), DEADLINE);
        let t = TimingConfig { stop_settle_timeout_ms: 1000, ..t };
        assert_eq!(stop_deadline(&t), Duration::from_millis(800));
        assert_eq!(stop_time_s(600, 1), 600.0);
        assert_eq!(stop_time_s(600, 0), f64::INFINITY);
        assert_eq!(min_acc_for_stop(600, DEADLINE), 500);
        assert_eq!(min_acc_for_stop(6000, DEADLINE), 5000);
        assert_eq!(min_acc_for_stop(0, DEADLINE), 0);
        assert_eq!(min_acc_for_stop(600, Duration::ZERO), u32::MAX);
    }

    #[test]
    fn clamp_raises_acc_to_the_stop_bound() {
        // Issue #15 H2: 600 count/s with acc 1 would take 600 s to stop.
        let req = MotionLimits { max_gain: 0.5, max_vel_count_s: 600, max_acc_count_s2: 1, duty_max_percent: 20 };
        let c = clamp(&req, &ceilings(), DEADLINE);
        assert_eq!(c.effective, MotionLimits { max_acc_count_s2: 500, ..req });
        assert_eq!(c.limited, vec![LimitedField::MaxAccCountS2]);
        assert_eq!(c.reasons, vec![LimitReason::StopTimeLimited]);
        // Exactly at the bound: untouched.
        let ok = MotionLimits { max_acc_count_s2: 500, ..req };
        assert_eq!(clamp(&ok, &ceilings(), DEADLINE), Clamped { effective: ok, limited: vec![], reasons: vec![] });
        // Judged on the ceiling-clamped velocity: 7000 → 6000 needs 5000.
        let req = MotionLimits { max_vel_count_s: 7000, max_acc_count_s2: 1000, ..req };
        let c = clamp(&req, &ceilings(), DEADLINE);
        assert_eq!((c.effective.max_vel_count_s, c.effective.max_acc_count_s2), (6000, 5000));
        assert_eq!(c.limited, vec![LimitedField::MaxVelCountS, LimitedField::MaxAccCountS2]);
        assert_eq!(c.reasons, vec![LimitReason::CeilingClamped, LimitReason::StopTimeLimited]);
    }

    #[test]
    fn clamp_lowers_vel_when_the_acc_ceiling_cannot_stop_it() {
        // A unit whose absolute acceleration ceiling is tiny: 6000 count/s
        // needs 5000 count/s², the ceiling gives 1000 → cap vel at 1200.
        let c = Ceilings { max_acc_count_s2: 1000, ..ceilings() };
        let req = MotionLimits { max_gain: 0.5, max_vel_count_s: 6000, max_acc_count_s2: 800, duty_max_percent: 20 };
        let r = clamp(&req, &c, DEADLINE);
        assert_eq!((r.effective.max_vel_count_s, r.effective.max_acc_count_s2), (1200, 1000));
        assert_eq!(r.limited, vec![LimitedField::MaxVelCountS, LimitedField::MaxAccCountS2]);
        assert_eq!(r.reasons, vec![LimitReason::StopTimeLimited]);
        // acc already at the ceiling: only the velocity moves.
        let req = MotionLimits { max_acc_count_s2: 1000, ..req };
        let r = clamp(&req, &c, DEADLINE);
        assert_eq!((r.effective.max_vel_count_s, r.effective.max_acc_count_s2), (1200, 1000));
        assert_eq!(r.limited, vec![LimitedField::MaxVelCountS]);
    }

    #[test]
    fn game_written_fields_are_held_to_the_operator_values() {
        // Issue #15: the game asks for more than `[limits]` on every field.
        let req = MotionLimits { max_gain: 0.8, max_vel_count_s: 5000, max_acc_count_s2: 30000, duty_max_percent: 50 };
        let c = super::clamp(&req, &ceilings(), &game_cap(), DEADLINE);
        assert_eq!(c.effective, no_cap().max);
        assert_eq!(c.limited, LimitedField::ALL.to_vec());
        assert_eq!(c.reasons, vec![LimitReason::OperatorLimited]);
        // Only the game-written fields are capped; the others (admin-written
        // or untouched) go up to the firmware ceiling.
        let mut fields = FieldSet::NONE;
        fields.insert(LimitedField::MaxGain);
        let c = super::clamp(&req, &ceilings(), &OperatorCap { fields, ..no_cap() }, DEADLINE);
        assert_eq!(c.effective, MotionLimits { max_gain: 0.5, ..req });
        assert_eq!(c.limited, vec![LimitedField::MaxGain]);
        assert_eq!(c.reasons, vec![LimitReason::OperatorLimited]);
        // Above both bounds, the tighter one names the reason: the operator's
        // value normally, the firmware ceiling on a unit that reports a lower one.
        let req = MotionLimits { max_vel_count_s: 7000, ..req };
        let c = super::clamp(&req, &ceilings(), &game_cap(), DEADLINE);
        assert_eq!(c.effective.max_vel_count_s, 3000);
        assert_eq!(c.reasons, vec![LimitReason::OperatorLimited]);
        let low = Ceilings { duty_max_percent: 30, ..ceilings() };
        let c = super::clamp(&req, &low, &game_cap(), DEADLINE);
        assert_eq!(c.effective.duty_max_percent, 30);
        assert_eq!(c.reasons, vec![LimitReason::CeilingClamped, LimitReason::OperatorLimited]);
        // Lowering is always allowed and nothing is reported.
        let lower = MotionLimits { max_gain: 0.2, max_vel_count_s: 600, max_acc_count_s2: 2000, duty_max_percent: 20 };
        let c = super::clamp(&lower, &ceilings(), &game_cap(), DEADLINE);
        assert_eq!(c, Clamped { effective: lower, limited: vec![], reasons: vec![] });
        // The stop-time rule still runs on the capped velocity.
        let slow = MotionLimits { max_vel_count_s: 9000, max_acc_count_s2: 1, ..lower };
        let c = super::clamp(&slow, &ceilings(), &game_cap(), DEADLINE);
        assert_eq!((c.effective.max_vel_count_s, c.effective.max_acc_count_s2), (3000, 2500));
        assert_eq!(c.reasons, vec![LimitReason::OperatorLimited, LimitReason::StopTimeLimited]);
    }

    #[test]
    fn request_tracks_who_wrote_each_field() {
        let mut r = LimitsRequest::new(no_cap().max);
        assert!(r.game_fields.is_empty());
        r.apply(&LimitsPatch { max_vel_count_s: Some(5000.0), max_gain: Some(0.8), ..Default::default() }, Source::Game)
            .unwrap();
        assert_eq!(r.game_fields.to_vec(), vec![LimitedField::MaxGain, LimitedField::MaxVelCountS]);
        assert_eq!((r.requested.max_gain, r.requested.max_vel_count_s), (0.8, 5000));
        // The admin writing a field releases it; untouched fields keep theirs.
        r.apply(&LimitsPatch { max_vel_count_s: Some(4000.0), ..Default::default() }, Source::Admin).unwrap();
        assert_eq!(r.game_fields.to_vec(), vec![LimitedField::MaxGain]);
        assert_eq!(r.requested.max_vel_count_s, 4000);
        // A rejected patch changes nothing.
        let before = r;
        assert_eq!(
            r.apply(&LimitsPatch { max_acc_count_s2: Some(0.0), ..Default::default() }, Source::Game),
            Err(PatchError::OutOfRange)
        );
        assert_eq!(r, before);
        assert_eq!(serde_json::to_value(r.game_fields).unwrap(), serde_json::json!(["max_gain"]));
        assert_eq!(r.operator_cap(no_cap().max).fields, r.game_fields);
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
        assert_eq!(t.stop_acc_count_s2, 15000);
        let msg = t.to_msg(9);
        assert_eq!(msg.request_id, 9);
        assert_eq!(msg.stop_acc_count_s2, 15000);
        let capped = FwTuning::new(
            &cfg,
            &MotionLimits { max_acc_count_s2: 99_999, ..e },
        );
        assert_eq!(capped.stop_acc_count_s2, 40000);
    }
}
