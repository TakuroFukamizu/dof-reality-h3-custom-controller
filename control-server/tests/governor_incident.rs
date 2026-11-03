//! 2026-11 の追従事故の再現。
//!
//! 軸 2 は持ち上げが約 900 count/s で頭打ち、下げは速い。指令は 40 ms 遅れて
//! 届き、STATUS は 20 ms 周期。−1200 count/s で降下中に目標を +1200 側へ
//! 反転する（max_acc 20000）。ガバナ無しでは |FE| が 400 を超え、ありでは
//! 300 未満のまま最終目標へ着き、他軸も同じ s で遅くなる。停止の減速度は
//! s² を掛けない。
//!
//! `sim_axis` は方向で速度上限を分けられないので、ここだけ小さなプラントを使う。

use std::collections::VecDeque;
use std::time::Duration;

use h3cs::governor::{
    scale_track_limits, target_vel_sign, AxisSample, GovernorConfig, TrackingGovernor,
};
use h3cs::planner::{AxisLimits, Planner};
use h3cs::types::AXIS_COUNT;
use tokio::time::Instant;

const DT: f64 = 0.01;
const DELAY_TICKS: usize = 4;
const STATUS_EVERY: usize = 2;
const LIFT_CAP: f64 = 900.0;
const LOWER_CAP: f64 = 5000.0;
const HEALTHY_CAP: f64 = 5000.0;
const MAX_VEL: f64 = 1200.0;
const MAX_ACC: f64 = 20_000.0;
const DUTY: i16 = 400;
const FE_LIMIT: u32 = 400;
/// 事故の軸（axis id 2）。
const WEAK: usize = 1;
const DESCEND_S: f64 = 0.5;
const HORIZON_S: f64 = 10.0;
const SAMPLE_AFTER_REVERSE_S: f64 = 2.0;
const FINAL_TARGET: f64 = 5000.0;

struct AxisPlant {
    pos: f64,
    /// 再生待ちの指令位置。先頭が今 MCU が見ている setpoint。
    delay: VecDeque<f64>,
    prev_sp: f64,
    cap_up: f64,
    cap_down: f64,
}

impl AxisPlant {
    fn new(pos: f64, cap_up: f64, cap_down: f64) -> Self {
        Self {
            pos,
            delay: VecDeque::from(vec![pos; DELAY_TICKS]),
            prev_sp: pos,
            cap_up,
            cap_down,
        }
    }

    /// 新しい指令を積み、40 ms 前の setpoint に向かって 1 tick 進める。
    /// 戻り値は (適用した setpoint, その速度, 移動後の FE)。
    fn step(&mut self, command: f64) -> (f64, f64, i32) {
        self.delay.push_back(command);
        let sp = self.delay.pop_front().expect("delay line");
        let sv = (sp - self.prev_sp) / DT;
        self.prev_sp = sp;
        let v = ((sp - self.pos) / DT).clamp(-self.cap_down, self.cap_up);
        self.pos += v * DT;
        let fe = (self.pos - sp).round() as i32;
        (sp, sv, fe)
    }
}

fn pwm_for(fe: i32, cmd_vel: f64, cap_up: f64, cap_down: f64) -> i16 {
    let sign: i32 = if cmd_vel > 1.0 {
        1
    } else if cmd_vel < -1.0 {
        -1
    } else {
        0
    };
    let over = (sign > 0 && cmd_vel > cap_up + 1.0) || (sign < 0 && -cmd_vel > cap_down + 1.0);
    let behind = sign != 0 && fe != 0 && fe.signum() == -sign;
    if over && behind {
        (sign * i32::from(DUTY)) as i16
    } else {
        // 飽和マージン（duty_max − 5）より十分小さい。
        (sign * 80) as i16
    }
}

struct Outcome {
    weak_peak_fe: i32,
    healthy_peak_fe: i32,
    min_s: f64,
    /// 反転から SAMPLE_AFTER_REVERSE_S の、健全な軸のプランナー位置。
    pos_mid: f64,
    final_plant: [f64; AXIS_COUNT],
    vels_equal: bool,
    limits_shared: bool,
    stop_full_decel: bool,
}

fn simulate(govern: bool) -> Outcome {
    let mut planner = Planner::new();
    planner.init([0, 0, 0]);
    let mut gov = TrackingGovernor::new(
        GovernorConfig {
            enabled: govern,
            ..GovernorConfig::default()
        },
        FE_LIMIT,
    );
    let base = [AxisLimits {
        max_vel_count_s: MAX_VEL,
        max_acc_count_s2: MAX_ACC,
    }; AXIS_COUNT];
    let mut plants = [
        AxisPlant::new(0.0, HEALTHY_CAP, HEALTHY_CAP),
        AxisPlant::new(0.0, LIFT_CAP, LOWER_CAP),
        AxisPlant::new(0.0, HEALTHY_CAP, HEALTHY_CAP),
    ];
    let mut published_fe = [0i32; AXIS_COUNT];
    let mut published_pwm = [0i16; AXIS_COUNT];
    let mut age_ms = 0u64;

    let mut weak_peak_fe = 0i32;
    let mut healthy_peak_fe = 0i32;
    let mut min_s = 1.0f64;
    let mut pos_mid = 0.0;
    let mut vels_equal = true;
    let mut limits_shared = true;
    let mut stop_full_decel = false;

    let t0 = Instant::now();
    let n = (HORIZON_S / DT).round() as usize;
    let descend_ticks = (DESCEND_S / DT).round() as usize;
    let mid_tick = descend_ticks + (SAMPLE_AFTER_REVERSE_S / DT).round() as usize;

    for tick in 0..n {
        let now = t0 + Duration::from_millis((tick as u64) * 10);
        let targets = if tick < descend_ticks {
            [-8_000.0; AXIS_COUNT]
        } else {
            [FINAL_TARGET; AXIS_COUNT]
        };
        let vels = planner.velocities();
        let samples = std::array::from_fn(|i| AxisSample {
            following_error_count: published_fe[i],
            pwm_permille: published_pwm[i],
            duty_max_permille: DUTY,
            target_vel_sign: target_vel_sign(vels[i]),
            status_age_ms: age_ms,
            active: true,
        });
        let s = gov.update(now, &samples);
        min_s = min_s.min(s);
        let limits = scale_track_limits(base, s);
        if limits[0] != limits[1] || limits[1] != limits[2] {
            limits_shared = false;
        }
        // 渡している制限が「同じ s」そのものであること。
        let again = scale_track_limits(base, s);
        if limits != again {
            limits_shared = false;
        }

        if govern && !stop_full_decel && s < 0.9 && vels.iter().all(|v| *v > 500.0) {
            let mut stopped = planner.clone();
            let before = stopped.velocities();
            stopped.step_stop(base, DT);
            let after = stopped.velocities();
            stop_full_decel = (0..AXIS_COUNT).all(|i| {
                let dv = before[i] - after[i];
                (dv - MAX_ACC * DT).abs() < 1e-4 && dv > MAX_ACC * s * s * DT + 1.0
            });
        }

        planner.step_track(targets, limits, DT);
        let pv = planner.velocities();
        if (pv[0] - pv[1]).abs() > 1e-6 || (pv[1] - pv[2]).abs() > 1e-6 {
            vels_equal = false;
        }
        if tick == mid_tick {
            pos_mid = planner.positions()[0];
        }

        let mut measured_fe = [0i32; AXIS_COUNT];
        let mut measured_pwm = [0i16; AXIS_COUNT];
        for i in 0..AXIS_COUNT {
            let cmd = planner.positions()[i];
            let (_sp, sv, fe) = plants[i].step(cmd);
            measured_fe[i] = fe;
            measured_pwm[i] = pwm_for(fe, sv, plants[i].cap_up, plants[i].cap_down);
            let peak = fe.unsigned_abs();
            if i == WEAK {
                weak_peak_fe = weak_peak_fe.max(peak as i32);
            } else {
                healthy_peak_fe = healthy_peak_fe.max(peak as i32);
            }
        }
        if tick % STATUS_EVERY == STATUS_EVERY - 1 {
            published_fe = measured_fe;
            published_pwm = measured_pwm;
            age_ms = 0;
        } else {
            age_ms = age_ms.saturating_add(10);
        }
    }

    Outcome {
        weak_peak_fe,
        healthy_peak_fe,
        min_s,
        pos_mid,
        final_plant: std::array::from_fn(|i| plants[i].pos),
        vels_equal,
        limits_shared,
        stop_full_decel,
    }
}

#[test]
fn reverse_under_lift_cap_faults_without_governor_and_tracks_with_it() {
    let off = simulate(false);
    let on = simulate(true);

    assert!(
        off.weak_peak_fe > 400,
        "ガバナ無しの |FE| ピーク {} は 400 を超えること",
        off.weak_peak_fe
    );
    assert!(
        on.weak_peak_fe < 300,
        "ガバナありの |FE| ピーク {} は 300 未満",
        on.weak_peak_fe
    );
    assert!(
        on.healthy_peak_fe < 30,
        "健全軸の |FE| {}",
        on.healthy_peak_fe
    );
    for i in 0..AXIS_COUNT {
        let err = (on.final_plant[i] - FINAL_TARGET).abs();
        assert!(
            err < 1.0,
            "軸 {i} の最終位置 {}（目標 {FINAL_TARGET}）",
            on.final_plant[i]
        );
    }
    assert!(on.min_s < 0.9, "min s {}", on.min_s);
    assert!(on.vels_equal, "3 軸のプランナー速度が揃っていない");
    assert!(on.limits_shared, "制限が軸ごとに違う");
    assert!(
        off.pos_mid - on.pos_mid > 200.0,
        "健全軸も同じ s で遅れること（無し {}、あり {}）",
        off.pos_mid,
        on.pos_mid
    );
    assert!(on.stop_full_decel, "停止が s² の減速度になっている");
    assert!(off.min_s == 1.0);
    assert!(
        !off.stop_full_decel,
        "ガバナ無しでは s<0.9 にならないので停止検査は走らない"
    );
}
