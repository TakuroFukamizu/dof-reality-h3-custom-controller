//! 3-axis target generator (DESIGN.md §4). Pure: no clock, `dt` is injected.
//!
//! Per tick, `step_track`:
//! 1. `v_raw = (target − pos) / dt` per axis.
//! 2. Clip to the stop-able speed, so the axis can still brake to the target
//!    at `a`. The discrete-time form `(−a·dt + sqrt((a·dt)² + 8·a·|err|)) / 2`
//!    is used: it is ≤ `sqrt(2·a·|err|)` and accounts for the step being taken
//!    before braking starts, so a later `step_stop` also ends before the target.
//! 3. One common scale `s = min(1, min_i(vmax_i / |v_i|))` for all three axes
//!    (keeps the direction in axis space). `s < 1` → `VelocityLimited`.
//! 4. Per-axis acceleration clip `|v_new − vel| ≤ a·dt` → `AccelerationLimited`.
//! 5. Never cross the target; within 0.5 count with ~zero velocity → snap.

use crate::types::{Counts, LimitReason, AXIS_COUNT};

/// Per-axis motion limits for one tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AxisLimits {
    pub max_vel_count_s: f64,
    pub max_acc_count_s2: f64,
}

impl AxisLimits {
    /// Finite, `max_vel ≥ 0`, `max_acc > 0`.
    pub fn is_valid(&self) -> bool {
        self.max_vel_count_s.is_finite()
            && self.max_vel_count_s >= 0.0
            && self.max_acc_count_s2.is_finite()
            && self.max_acc_count_s2 > 0.0
    }
}

/// Result of one planner tick.
#[derive(Debug, Clone, PartialEq)]
pub struct PlanOutput {
    /// `round(positions)`, saturated into i32.
    pub command_counts: [Counts; AXIS_COUNT],
    pub positions: [f64; AXIS_COUNT],
    pub velocities: [f64; AXIS_COUNT],
    /// Limits that changed this tick's step (deduplicated).
    pub limits: Vec<LimitReason>,
    /// Track: every axis is at its target with zero velocity.
    /// Stop: every axis has zero velocity.
    pub settled: bool,
}

/// Snap tolerance: within this many counts of the target …
pub const SETTLE_POS_COUNT: f64 = 0.5;
/// … and slower than this (count/s, i.e. float noise only).
pub const SETTLE_VEL_COUNT_S: f64 = 1e-6;

#[derive(Debug, Clone, Default)]
pub struct Planner {
    pos: [f64; AXIS_COUNT],
    vel: [f64; AXIS_COUNT],
    initialized: bool,
    /// Last valid `step_track` targets; `step_stop` never crosses them.
    last_targets: Option<[f64; AXIS_COUNT]>,
}

impl Planner {
    pub fn new() -> Self {
        Self::default()
    }

    /// Start from the measured positions at rest (on reaching HOLDING).
    pub fn init(&mut self, actual_counts: [Counts; AXIS_COUNT]) {
        self.pos = actual_counts.map(f64::from);
        self.vel = [0.0; AXIS_COUNT];
        self.last_targets = None;
        self.initialized = true;
    }

    /// Forget the state (e.g. after DISARM); steps do nothing until `init`.
    pub fn clear(&mut self) {
        *self = Self::default();
    }

    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// Stop in place: velocity 0, position kept.
    pub fn hold(&mut self) {
        self.vel = [0.0; AXIS_COUNT];
    }

    pub fn positions(&self) -> [f64; AXIS_COUNT] {
        self.pos
    }

    pub fn velocities(&self) -> [f64; AXIS_COUNT] {
        self.vel
    }

    pub fn command_counts(&self) -> [Counts; AXIS_COUNT] {
        self.pos.map(to_count)
    }

    /// Current state as an output, without stepping.
    pub fn output(&self, settled: bool) -> PlanOutput {
        PlanOutput {
            command_counts: self.command_counts(),
            positions: self.pos,
            velocities: self.vel,
            limits: Vec::new(),
            settled,
        }
    }

    /// Move toward `targets` (already soft-limit clipped). Invalid input
    /// (uninitialized, non-finite target, bad limits, `dt ≤ 0`/non-finite)
    /// leaves the state unchanged and reports `settled = false`.
    pub fn step_track(
        &mut self,
        targets: [f64; AXIS_COUNT],
        limits: [AxisLimits; AXIS_COUNT],
        dt_s: f64,
    ) -> PlanOutput {
        if !self.initialized
            || !valid_dt(dt_s)
            || targets.iter().any(|t| !t.is_finite())
            || limits.iter().any(|l| !l.is_valid())
        {
            return self.output(false);
        }
        self.last_targets = Some(targets);

        // Steps 1–2: desired velocity, limited to what can still brake in time.
        let mut v = [0.0; AXIS_COUNT];
        for i in 0..AXIS_COUNT {
            let err = targets[i] - self.pos[i];
            let a = limits[i].max_acc_count_s2;
            let v_raw = err / dt_s;
            let v_stop = stoppable_speed(err.abs(), a, dt_s);
            v[i] = if v_raw.abs() > v_stop { v_stop.copysign(err) } else { v_raw };
        }

        // Step 3: one scale for all axes.
        let mut s: f64 = 1.0;
        for i in 0..AXIS_COUNT {
            if v[i] != 0.0 {
                s = s.min(limits[i].max_vel_count_s / v[i].abs());
            }
        }
        let mut out_limits = Vec::new();
        if s < 1.0 {
            for vi in &mut v {
                *vi *= s;
            }
            out_limits.push(LimitReason::VelocityLimited);
        }

        // Steps 4–5 per axis.
        let mut accel_limited = false;
        let mut settled = true;
        for i in 0..AXIS_COUNT {
            let dv_max = limits[i].max_acc_count_s2 * dt_s;
            let lo = self.vel[i] - dv_max;
            let hi = self.vel[i] + dv_max;
            let mut v_new = v[i];
            if v_new < lo {
                v_new = lo;
                accel_limited = true;
            } else if v_new > hi {
                v_new = hi;
                accel_limited = true;
            }

            let target = targets[i];
            let err_old = target - self.pos[i];
            let mut new_pos = self.pos[i] + v_new * dt_s;
            if crosses(target, self.pos[i], new_pos) {
                new_pos = target;
                v_new = err_old / dt_s;
            }

            if (target - new_pos).abs() <= SETTLE_POS_COUNT && v_new.abs() <= SETTLE_VEL_COUNT_S {
                new_pos = target;
                v_new = 0.0;
            } else {
                settled = false;
            }
            self.pos[i] = new_pos;
            self.vel[i] = v_new;
        }
        if accel_limited {
            out_limits.push(LimitReason::AccelerationLimited);
        }

        PlanOutput { limits: out_limits, ..self.output(settled) }
    }

    /// Decelerate every axis at its `a` to zero velocity, without a target.
    /// Also never crosses the last `step_track` target. Invalid input leaves
    /// the state unchanged (`settled` then reflects the current velocity).
    pub fn step_stop(&mut self, limits: [AxisLimits; AXIS_COUNT], dt_s: f64) -> PlanOutput {
        if !self.initialized || !valid_dt(dt_s) || limits.iter().any(|l| !l.is_valid()) {
            let settled = self.initialized && self.vel.iter().all(|v| *v == 0.0);
            return self.output(settled);
        }
        for i in 0..AXIS_COUNT {
            let dv = limits[i].max_acc_count_s2 * dt_s;
            let v = self.vel[i];
            let v_new = if v.abs() <= dv { 0.0 } else { v - dv.copysign(v) };
            let mut new_pos = self.pos[i] + v_new * dt_s;
            let mut v_new = v_new;
            if let Some(t) = self.last_targets.map(|t| t[i]) {
                if crosses(t, self.pos[i], new_pos) {
                    new_pos = t;
                    v_new = 0.0;
                }
            }
            self.pos[i] = new_pos;
            self.vel[i] = v_new;
        }
        let settled = self.vel.iter().all(|v| *v == 0.0);
        self.output(settled)
    }
}

/// Moving `from → to` passes `target`, or leaves it while standing on it.
fn crosses(target: f64, from: f64, to: f64) -> bool {
    let before = target - from;
    let after = target - to;
    if before == 0.0 {
        after != 0.0
    } else {
        after != 0.0 && after.signum() != before.signum()
    }
}

fn valid_dt(dt_s: f64) -> bool {
    dt_s.is_finite() && dt_s > 0.0
}

/// Largest speed from which braking at `a` (one tick of travel first, then
/// `a·dt` less per tick) ends within `dist`. Solves `v² + a·dt·v − 2·a·dist = 0`.
fn stoppable_speed(dist: f64, a: f64, dt: f64) -> f64 {
    let u = a * dt;
    let x = 8.0 * a * dist;
    // = (−u + sqrt(u² + x)) / 2, rewritten to avoid cancellation when u² ≫ x.
    let v = x / (2.0 * (u + (u * u + x).sqrt()));
    if v.is_finite() { v.max(0.0) } else { 0.0 }
}

/// `round()` then saturate into i32 (NaN → 0).
fn to_count(x: f64) -> Counts {
    x.round() as Counts
}
