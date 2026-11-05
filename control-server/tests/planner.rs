//! planner: uniform velocity scaling, acceleration limit, no overshoot, stop mode.
#![allow(clippy::needless_range_loop)] // per-axis tests index several arrays in step

use h3cs::planner::{AxisLimits, PlanOutput, Planner};
use h3cs::types::LimitReason;

const DT: f64 = 0.01;

fn lim(v: f64, a: f64) -> AxisLimits {
    AxisLimits {
        max_vel_count_s: v,
        max_acc_count_s2: a,
    }
}

fn lims(v: f64, a: f64) -> [AxisLimits; 3] {
    [lim(v, a); 3]
}

fn planner_at(c: [i32; 3]) -> Planner {
    let mut p = Planner::new();
    p.init(c);
    p
}

/// Tiny deterministic PRNG (xorshift64*).
struct Rng(u64);
impl Rng {
    fn next_u64(&mut self) -> u64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    /// Uniform in [lo, hi).
    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * ((self.next_u64() >> 11) as f64 / (1u64 << 53) as f64)
    }
}

/// Run `step_track` until settled (or `max_steps`), returning every output.
fn run_track(
    p: &mut Planner,
    t: [f64; 3],
    l: [AxisLimits; 3],
    max_steps: usize,
) -> Vec<PlanOutput> {
    let mut outs = Vec::new();
    for _ in 0..max_steps {
        let o = p.step_track(t, l, DT);
        let done = o.settled;
        outs.push(o);
        if done {
            break;
        }
    }
    outs
}

#[test]
fn init_sets_position_and_zero_velocity() {
    let mut p = Planner::new();
    assert!(!p.is_initialized());
    p.init([10, -20, 30]);
    assert!(p.is_initialized());
    assert_eq!(p.positions(), [10.0, -20.0, 30.0]);
    assert_eq!(p.velocities(), [0.0; 3]);
    assert_eq!(p.command_counts(), [10, -20, 30]);
    p.clear();
    assert!(!p.is_initialized());
}

#[test]
fn uninitialized_planner_does_not_move() {
    let mut p = Planner::new();
    let o = p.step_track([1000.0; 3], lims(4000.0, 20000.0), DT);
    assert_eq!(o.positions, [0.0; 3]);
    assert!(!o.settled);
}

#[test]
fn one_axis_saturation_scales_all_steps_by_the_same_ratio() {
    // Huge acceleration: neither the brake envelope nor the accel clip binds.
    let mut p = planner_at([0, 0, 0]);
    let errs = [1000.0, 100.0, -50.0];
    let l = lims(4000.0, 1e12);
    let o = p.step_track(errs, l, DT);
    assert!(o.limits.contains(&LimitReason::VelocityLimited));
    assert!(!o.limits.contains(&LimitReason::AccelerationLimited));
    // Axis 1 wanted 100_000 count/s, limited to 4000 → s = 0.04 for every axis.
    let steps = o.positions;
    assert!((steps[0] - 40.0).abs() < 1e-9, "{steps:?}");
    for i in 0..3 {
        for j in 0..3 {
            let r_step = steps[i] / steps[j];
            let r_err = errs[i] / errs[j];
            assert!(
                (r_step - r_err).abs() < 1e-9 * r_err.abs().max(1.0),
                "{i}/{j}: {r_step} vs {r_err}"
            );
        }
    }
    // The slower axes are far below their own limit: they were scaled, not clipped.
    assert!((o.velocities[1] - 400.0).abs() < 1e-6);
    assert!((o.velocities[2] + 200.0).abs() < 1e-6);
}

#[test]
fn uniform_scale_uses_each_axis_own_limit() {
    // Axis 2 has the tighter limit relative to its demand.
    let mut p = planner_at([0, 0, 0]);
    let errs = [100.0, 100.0, 0.0];
    let l = [lim(4000.0, 1e12), lim(1000.0, 1e12), lim(4000.0, 1e12)];
    let o = p.step_track(errs, l, DT);
    assert!((o.velocities[1] - 1000.0).abs() < 1e-6);
    assert!((o.velocities[0] - 1000.0).abs() < 1e-6);
    assert_eq!(o.velocities[2], 0.0);
}

#[test]
fn no_limit_reported_when_step_is_free() {
    let mut p = planner_at([0, 0, 0]);
    let o = p.step_track([1.0, 0.0, -1.0], lims(4000.0, 1e12), DT);
    assert!(o.limits.is_empty(), "{:?}", o.limits);
    assert_eq!(o.positions, [1.0, 0.0, -1.0]);
}

#[test]
fn acceleration_ramps_from_rest() {
    let a = 20000.0;
    let mut p = planner_at([0, 0, 0]);
    let l = lims(4000.0, a);
    let mut prev_v = 0.0;
    for k in 1..=10 {
        let o = p.step_track([100_000.0, 0.0, 0.0], l, DT);
        let v = o.velocities[0];
        assert!(
            (v - prev_v - a * DT).abs() < 1e-9,
            "step {k}: {prev_v} → {v}"
        );
        assert!(o.limits.contains(&LimitReason::AccelerationLimited));
        prev_v = v;
    }
    // After 20 steps the ramp hits the velocity limit and stays there.
    for _ in 0..20 {
        p.step_track([100_000.0, 0.0, 0.0], l, DT);
    }
    let o = p.step_track([100_000.0, 0.0, 0.0], l, DT);
    assert!((o.velocities[0] - 4000.0).abs() < 1e-9);
    assert!(o.limits.contains(&LimitReason::VelocityLimited));
}

#[test]
fn every_step_respects_velocity_and_acceleration_except_final_stop() {
    let a = 20000.0;
    let vmax = 4000.0;
    let mut p = planner_at([0, 0, 0]);
    let outs = run_track(&mut p, [3000.0, -1500.0, 700.0], lims(vmax, a), 10_000);
    assert!(outs.last().unwrap().settled);
    let mut prev = [0.0; 3];
    for o in &outs {
        for i in 0..3 {
            assert!(o.velocities[i].abs() <= vmax + 1e-9);
            // Small tolerance for the last-step crossing clamp only.
            assert!((o.velocities[i] - prev[i]).abs() <= a * DT * 2.0 + 1e-9);
        }
        prev = o.velocities;
    }
}

#[test]
fn reaches_target_from_rest_without_overshoot_and_settles_exactly() {
    let target = [2500.0, -1234.0, 17.0];
    let mut p = planner_at([0, 0, 0]);
    let outs = run_track(&mut p, target, lims(4000.0, 20000.0), 10_000);
    let last = outs.last().unwrap();
    assert!(last.settled, "did not settle in {} steps", outs.len());
    assert_eq!(last.positions, target);
    assert_eq!(last.velocities, [0.0; 3]);
    assert_eq!(last.command_counts, [2500, -1234, 17]);
    for o in &outs {
        for i in 0..3 {
            // Same side of the target as the start (0), never past it.
            assert!(
                o.positions[i] * target[i].signum() <= target[i].abs() + 1e-9,
                "{:?}",
                o.positions
            );
        }
    }
    // Stays settled.
    let o = p.step_track(target, lims(4000.0, 20000.0), DT);
    assert!(o.settled);
    assert_eq!(o.positions, target);
}

#[test]
fn reaches_target_without_overshoot_when_moving_the_opposite_way() {
    let mut p = planner_at([0, 0, 0]);
    let l = lims(4000.0, 20000.0);
    // Build up negative velocity on every axis.
    for _ in 0..30 {
        p.step_track([-100_000.0; 3], l, DT);
    }
    assert!(p.velocities().iter().all(|v| *v < -3000.0));
    let start = p.positions();
    let target = [start[0] + 500.0, start[1] + 50.0, start[2] + 2000.0];
    let outs = run_track(&mut p, target, l, 10_000);
    let last = outs.last().unwrap();
    assert!(last.settled);
    assert_eq!(last.positions, target);
    for o in &outs {
        for i in 0..3 {
            assert!(
                o.positions[i] <= target[i] + 1e-9,
                "axis {i} overshot: {:?}",
                o.positions
            );
        }
    }
    // It really did first continue away (decelerating at a), then come back.
    let min0 = outs
        .iter()
        .map(|o| o.positions[0])
        .fold(f64::INFINITY, f64::min);
    assert!(min0 < start[0]);
}

#[test]
fn target_jumping_closer_than_braking_distance_is_not_crossed() {
    let mut p = planner_at([0, 0, 0]);
    let l = lims(4000.0, 20000.0);
    for _ in 0..30 {
        p.step_track([100_000.0, 0.0, 0.0], l, DT);
    }
    let x = p.positions()[0];
    let target = [x + 5.0, 0.0, 0.0];
    let outs = run_track(&mut p, target, l, 100);
    for o in &outs {
        assert!(o.positions[0] <= target[0]);
    }
    assert!(outs.last().unwrap().settled);
}

#[test]
fn step_stop_decelerates_at_a_and_settles() {
    let a = 20000.0;
    let mut p = planner_at([0, 0, 0]);
    let l = lims(4000.0, a);
    for _ in 0..40 {
        p.step_track([100_000.0, -100_000.0, 0.0], l, DT);
    }
    let v0 = p.velocities();
    assert!((v0[0] - 4000.0).abs() < 1e-9 && (v0[1] + 4000.0).abs() < 1e-9);
    let mut prev = v0;
    let mut steps = 0;
    loop {
        let o = p.step_stop(l, DT);
        steps += 1;
        for i in 0..3 {
            let dv = (o.velocities[i] - prev[i]).abs();
            assert!(dv <= a * DT + 1e-9);
            // Never reverses.
            assert!(o.velocities[i] * v0[i] >= 0.0);
        }
        prev = o.velocities;
        if o.settled {
            assert_eq!(o.velocities, [0.0; 3]);
            break;
        }
        assert!(steps < 1000);
    }
    // 4000 / (20000 · 0.01) = 20 ticks.
    assert_eq!(steps, 20);
    let o = p.step_stop(l, DT);
    assert!(o.settled);
}

#[test]
fn step_stop_after_full_speed_tracking_stops_before_the_target() {
    let l = lims(4000.0, 20000.0);
    let target = 3000.0;
    let mut p = planner_at([0, 0, 0]);
    // Track until the brake envelope starts to bind, then switch to stop at
    // every possible point of the approach.
    let mut k = 0;
    loop {
        let mut q = p.clone();
        let mut prev_v = q.velocities()[0];
        loop {
            let o = q.step_stop(l, DT);
            assert!(
                o.positions[0] <= target + 1e-9,
                "stop from step {k} overshot: {}",
                o.positions[0]
            );
            // Braking at a, not an abrupt stop at the target (the envelope holds).
            assert!(
                (prev_v - o.velocities[0]).abs() <= 20000.0 * DT + 1e-9,
                "step {k}: {prev_v} → {}",
                o.velocities[0]
            );
            prev_v = o.velocities[0];
            if o.settled {
                break;
            }
        }
        let o = p.step_track([target, 0.0, 0.0], l, DT);
        k += 1;
        if o.settled {
            break;
        }
        assert!(k < 10_000);
    }
}

#[test]
fn hold_zeroes_velocity_and_keeps_position() {
    let mut p = planner_at([0, 0, 0]);
    for _ in 0..5 {
        p.step_track([1000.0; 3], lims(4000.0, 20000.0), DT);
    }
    let pos = p.positions();
    p.hold();
    assert_eq!(p.velocities(), [0.0; 3]);
    assert_eq!(p.positions(), pos);
    let o = p.step_stop(lims(4000.0, 20000.0), DT);
    assert!(o.settled);
    assert_eq!(o.positions, pos);
}

#[test]
fn invalid_input_does_not_move_or_panic() {
    let l = lims(4000.0, 20000.0);
    let mut p = planner_at([5, 6, 7]);
    for _ in 0..3 {
        p.step_track([100.0; 3], l, DT);
    }
    let pos = p.positions();
    let vel = p.velocities();
    for dt in [0.0, -0.01, f64::NAN, f64::INFINITY] {
        let o = p.step_track([100.0; 3], l, dt);
        assert_eq!(o.positions, pos);
        let o = p.step_stop(l, dt);
        assert_eq!(o.positions, pos);
    }
    for t in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let o = p.step_track([0.0, t, 0.0], l, DT);
        assert_eq!(o.positions, pos);
    }
    for bad in [
        lim(f64::NAN, 1.0),
        lim(1.0, f64::NAN),
        lim(-1.0, 1.0),
        lim(1.0, 0.0),
        lim(1.0, f64::INFINITY),
    ] {
        let o = p.step_track([100.0; 3], [l[0], bad, l[2]], DT);
        assert_eq!(o.positions, pos);
        let o = p.step_stop([bad, l[1], l[2]], DT);
        assert_eq!(o.positions, pos);
    }
    assert_eq!(p.velocities(), vel);
    // Huge but finite values saturate the command instead of panicking.
    let o = p.step_track([1e300; 3], lims(1e300, 1e300), DT);
    assert!(o.positions.iter().all(|x| x.is_finite()));
    let _ = o.command_counts;
}

#[test]
fn zero_velocity_limit_freezes_every_axis() {
    let mut p = planner_at([0, 0, 0]);
    let l = [
        lim(4000.0, 20000.0),
        lim(0.0, 20000.0),
        lim(4000.0, 20000.0),
    ];
    let o = p.step_track([100.0, 100.0, 100.0], l, DT);
    assert_eq!(o.positions, [0.0; 3]);
    assert!(o.limits.contains(&LimitReason::VelocityLimited));
}

#[test]
fn monotonic_approach_over_random_targets() {
    let mut rng = Rng(0x9E37_79B9_7F4A_7C15);
    for case in 0..300 {
        let start = [0; 3].map(|_| rng.range(-3000.0, 3000.0).round() as i32);
        let target = [0; 3].map(|_| rng.range(-3000.0, 3000.0));
        let l = [0; 3].map(|_| lim(rng.range(100.0, 5000.0), rng.range(1000.0, 50000.0)));
        let mut p = planner_at(start);
        let mut prev_err = [0usize, 1, 2].map(|i| (target[i] - start[i] as f64).abs());
        let mut settled = false;
        for _ in 0..20_000 {
            let o = p.step_track(target, l, DT);
            for i in 0..3 {
                let err = (target[i] - o.positions[i]).abs();
                assert!(
                    err <= prev_err[i] + 1e-9,
                    "case {case} axis {i}: {} → {err}",
                    prev_err[i]
                );
                // Same side as the start: never crossed.
                let side0 = target[i] - start[i] as f64;
                assert!(
                    (target[i] - o.positions[i]) * side0 >= -1e-9,
                    "case {case} axis {i} crossed"
                );
                assert!(o.velocities[i].abs() <= l[i].max_vel_count_s + 1e-9);
                prev_err[i] = err;
            }
            if o.settled {
                assert_eq!(o.positions, target);
                settled = true;
                break;
            }
        }
        assert!(settled, "case {case} never settled");
    }
}
