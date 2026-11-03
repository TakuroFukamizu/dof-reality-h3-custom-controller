//! 追従ガバナ（tracking governor）。
//!
//! プランナーは `max_vel` / `max_acc` だけで目標を進める。負荷で実速度が
//! 頭打ちになると目標だけが先に走り、MCU の追従誤差（`actual − setpoint`）が
//! `following_error_limit_count` を超えて FOLLOWING_ERROR になる。
//!
//! ガバナは「目標の進行方向に対して実位置が遅れ、かつ出力が飽和している
//! （または大きな誤差が縮んでいない）」軸を見つけ、全軸共通の倍率 `s`
//! （`s_min` 以上 1 以下）を返す。呼び出し側は追跡の `max_vel` を `s` 倍、
//! `max_acc` を `s²` 倍する。停止の減速度には掛けない。
//!
//! `s` の目標値は遅れている軸の `|FE|` から
//! `1 − (|FE| − fe_start) / (fe_limit − fe_start)` を `[s_min, 1]` に
//! クランプし、軸間の最小を取る。`fe_limit` は設定の
//! `following_error_limit_count`（FAULT の手前で `s` が `s_min` に着く）。

use std::collections::VecDeque;
use std::time::Duration;

use serde::{Deserialize, Deserializer, Serialize};
use tokio::time::Instant;

use crate::planner::AxisLimits;
use crate::types::{AxisId, AXIS_COUNT};

/// `|pwm|` が `duty_max − この値` 以上なら飽和とみなす（permille）。
const SAT_MARGIN_PERMILLE: i32 = 5;
/// `|FE|` が `fe_hard` 以上で、この窓のあいだ減っていなければ硬い遅れ。
const FE_TREND_WINDOW_MS: u64 = 60;
/// 速度の符号を決める下限（count/s）。これ未満は「目標は動いていない」。
const VEL_SIGN_MIN_COUNT_S: f64 = 1.0;
/// `s` の積分に使う 1 回の dt の上限。遅れた tick が空き時間を速度にしない。
const MAX_INTEGRATE_DT: Duration = Duration::from_millis(20);
/// これより長い観測の空きは解放の滞在を捨てる（2 tick ぶん。ちょうど 20 ms は連続）。
const RELEASE_GAP: Duration = Duration::from_millis(20);

/// 工場出荷のしきい値。`following_error_limit_count` が `FACTORY_FE_HARD` 以下で、
/// 設定がこの 3 つを書いていないときだけ、limit に対する比率へ縮める。
pub const FACTORY_FE_START: u32 = 120;
pub const FACTORY_FE_HARD: u32 = 200;
pub const FACTORY_FE_RELEASE: u32 = 60;

/// `[governor]`。節が無いときは [`GovernorConfig::default`]（有効）。
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct GovernorConfig {
    pub enabled: bool,
    /// 飽和と組で減速を始める |FE|（count）。
    pub fe_start: u32,
    /// 飽和が無くても、縮んでいなければ減速する |FE|。
    pub fe_hard: u32,
    /// これを下回り、かつ非飽和が `release_ms` 続くまで `s` は上がらない。
    pub fe_release: u32,
    /// `s` の下限。(0, 1]。
    pub s_min: f64,
    /// `s` を下げる最大速度（1/s）。
    pub down_rate: f64,
    /// `s` を上げる最大速度（1/s）。
    pub up_rate: f64,
    /// 飽和とみなすまでに必要な連続時間（ms）。
    pub sat_ms: u64,
    /// 全軸が解放条件を満たしてから `s` を上げ始めるまでの時間（ms）。
    pub release_ms: u64,
    /// STATUS がこれより古ければ `s` を上げない（ms）。
    pub stale_ms: u64,
    /// 原因軸の実位置が目標へこれだけ進まなければサーバが coordinated stop する（ms）。
    pub no_progress_ms: u64,
    /// TOML が `fe_start` / `fe_hard` / `fe_release` のいずれかを書いた。
    /// 書いた値は比率へ縮めない。シリアライズしない。
    #[serde(skip)]
    pub thresholds_explicit: bool,
}

impl Default for GovernorConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            fe_start: FACTORY_FE_START,
            fe_hard: FACTORY_FE_HARD,
            fe_release: FACTORY_FE_RELEASE,
            s_min: 0.25,
            down_rate: 4.0,
            up_rate: 0.5,
            sat_ms: 60,
            release_ms: 300,
            stale_ms: 100,
            no_progress_ms: 1500,
            thresholds_explicit: false,
        }
    }
}

/// `fe_*` の省略と明示を分ける。既定値と同じ数を書いても明示。
#[derive(Deserialize)]
#[serde(deny_unknown_fields, default)]
struct GovernorConfigDe {
    enabled: Option<bool>,
    fe_start: Option<u32>,
    fe_hard: Option<u32>,
    fe_release: Option<u32>,
    s_min: Option<f64>,
    down_rate: Option<f64>,
    up_rate: Option<f64>,
    sat_ms: Option<u64>,
    release_ms: Option<u64>,
    stale_ms: Option<u64>,
    no_progress_ms: Option<u64>,
}

impl Default for GovernorConfigDe {
    fn default() -> Self {
        Self {
            enabled: None,
            fe_start: None,
            fe_hard: None,
            fe_release: None,
            s_min: None,
            down_rate: None,
            up_rate: None,
            sat_ms: None,
            release_ms: None,
            stale_ms: None,
            no_progress_ms: None,
        }
    }
}

impl<'de> Deserialize<'de> for GovernorConfig {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let raw = GovernorConfigDe::deserialize(deserializer)?;
        let mut cfg = GovernorConfig::default();
        if let Some(v) = raw.enabled {
            cfg.enabled = v;
        }
        if let Some(v) = raw.fe_start {
            cfg.fe_start = v;
        }
        if let Some(v) = raw.fe_hard {
            cfg.fe_hard = v;
        }
        if let Some(v) = raw.fe_release {
            cfg.fe_release = v;
        }
        if let Some(v) = raw.s_min {
            cfg.s_min = v;
        }
        if let Some(v) = raw.down_rate {
            cfg.down_rate = v;
        }
        if let Some(v) = raw.up_rate {
            cfg.up_rate = v;
        }
        if let Some(v) = raw.sat_ms {
            cfg.sat_ms = v;
        }
        if let Some(v) = raw.release_ms {
            cfg.release_ms = v;
        }
        if let Some(v) = raw.stale_ms {
            cfg.stale_ms = v;
        }
        if let Some(v) = raw.no_progress_ms {
            cfg.no_progress_ms = v;
        }
        cfg.thresholds_explicit =
            raw.fe_start.is_some() || raw.fe_hard.is_some() || raw.fe_release.is_some();
        Ok(cfg)
    }
}

impl GovernorConfig {
    fn thresholds_are_factory(&self) -> bool {
        self.fe_start == FACTORY_FE_START
            && self.fe_hard == FACTORY_FE_HARD
            && self.fe_release == FACTORY_FE_RELEASE
    }

    /// 有効、かつしきい値を書いておらず、工場値のまま、かつ
    /// `limit` が工場の `fe_hard` 以下のときだけ、
    /// `fe_release = 0.15·limit`、`fe_start = 0.3·limit`、`fe_hard = 0.5·limit`
    /// （四捨五入）にする。2 回目は工場値でなくなるので何もしない。
    pub fn apply_scaled_defaults(&mut self, limit: u32) {
        if !self.enabled || self.thresholds_explicit || !self.thresholds_are_factory() {
            return;
        }
        if limit == 0 || limit > FACTORY_FE_HARD {
            return;
        }
        self.fe_release = pct_half_up(limit, 15);
        self.fe_start = pct_half_up(limit, 30);
        self.fe_hard = pct_half_up(limit, 50);
        if self.fe_hard >= limit {
            self.fe_hard = limit.saturating_sub(1);
        }
        if self.fe_start >= self.fe_hard {
            self.fe_start = self.fe_hard.saturating_sub(1);
        }
        if self.fe_release >= self.fe_start {
            self.fe_release = self.fe_start.saturating_sub(1);
        }
    }

    /// 有効なときだけ `fe_release < fe_start < fe_hard < following_error_limit`。
    /// 工場値で limit が `fe_hard` 以下なら、検査は縮めた値で行う（self は変えない）。
    /// `0 < s_min ≤ 1`、両 rate が正、`no_progress_ms > 0` は無効でも見る。
    /// 違反の説明を返す（空なら合格）。
    pub fn validation_errors(&self, following_error_limit_count: u32) -> Vec<String> {
        let mut cfg = self.clone();
        cfg.apply_scaled_defaults(following_error_limit_count);
        let mut errs = Vec::new();
        if cfg.enabled {
            if cfg.fe_release >= cfg.fe_start {
                errs.push(format!(
                    "governor.fe_release ({}) must be < governor.fe_start ({})",
                    cfg.fe_release, cfg.fe_start
                ));
            }
            if cfg.fe_start >= cfg.fe_hard {
                errs.push(format!(
                    "governor.fe_start ({}) must be < governor.fe_hard ({})",
                    cfg.fe_start, cfg.fe_hard
                ));
            }
            if cfg.fe_hard >= following_error_limit_count {
                errs.push(format!(
                    "governor.fe_hard ({}) must be < limits.following_error_limit_count ({})",
                    cfg.fe_hard, following_error_limit_count
                ));
            }
        }
        if !(cfg.s_min.is_finite() && cfg.s_min > 0.0 && cfg.s_min <= 1.0) {
            errs.push(format!(
                "governor.s_min must be within (0, 1], got {}",
                cfg.s_min
            ));
        }
        if !(cfg.down_rate.is_finite() && cfg.down_rate > 0.0) {
            errs.push(format!(
                "governor.down_rate must be > 0, got {}",
                cfg.down_rate
            ));
        }
        if !(cfg.up_rate.is_finite() && cfg.up_rate > 0.0) {
            errs.push(format!("governor.up_rate must be > 0, got {}", cfg.up_rate));
        }
        if cfg.no_progress_ms == 0 {
            errs.push("governor.no_progress_ms must be > 0".into());
        }
        errs
    }
}

/// `numer` は百分率の分子（15 = 15%）。0.5 は切り上げ。
fn pct_half_up(limit: u32, numer: u32) -> u32 {
    ((u64::from(limit) * u64::from(numer) + 50) / 100) as u32
}

/// なぜ `s` を下げているか。status の `governor.reason`。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum GovernReason {
    /// `|pwm|` が Duty 上限付近に `sat_ms` 以上張り付き、`|FE| ≥ fe_start`。
    PwmSaturated,
    /// `|FE| ≥ fe_hard` が約 60 ms のあいだ減っていない。
    FollowingError,
}

/// status の `governor` オブジェクト。
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct GovernorStatus {
    pub scale: f64,
    /// `scale < 1` のあいだ true。
    pub active: bool,
    /// いちばん遅れている軸（1..=3）。制限中でなければ null。
    pub axis: Option<AxisId>,
    pub reason: Option<GovernReason>,
}

impl Default for GovernorStatus {
    fn default() -> Self {
        Self {
            scale: 1.0,
            active: false,
            axis: None,
            reason: None,
        }
    }
}

/// 1 軸分の入力。supervisor が最新 STATUS とプランナー速度から埋める。
#[derive(Debug, Clone, Copy)]
pub struct AxisSample {
    /// `actual − setpoint`（`sim_axis` と同じ符号）。
    pub following_error_count: i32,
    pub pwm_permille: i16,
    pub duty_max_permille: i16,
    /// 目標が動いている向き。+1 / −1 / 0（動いていない）。
    pub target_vel_sign: i8,
    pub status_age_ms: u64,
    /// 追跡中の軸だけ true。false は遅れ・古さ・解放判定の対象外。
    pub active: bool,
}

#[derive(Debug, Clone)]
struct AxisMem {
    /// 飽和が続いている開始時刻。非飽和・古い STATUS で消す。
    sat_since: Option<Instant>,
    /// `(時刻, |FE|)` を古い順。硬い遅れの「減っていない」判定に使う。
    fe_hist: VecDeque<(Instant, u32)>,
}

impl Default for AxisMem {
    fn default() -> Self {
        Self {
            sat_since: None,
            fe_hist: VecDeque::with_capacity(16),
        }
    }
}

/// 追従ガバナ。`update` が返す `s` は `[s_min, 1]`。
#[derive(Debug, Clone)]
pub struct TrackingGovernor {
    cfg: GovernorConfig,
    /// スケール式の分母。`following_error_limit_count`。
    fe_limit: u32,
    scale: f64,
    last_now: Option<Instant>,
    /// 全 active 軸が解放条件を満たし始めた時刻。
    clear_since: Option<Instant>,
    /// `s < 1` のあいだ残す、最後に効いた軸と理由。
    cause: Option<(AxisId, GovernReason)>,
    /// この tick で遅れと判定した軸。最悪の 1 軸だけではなく全部。
    lagging: [bool; AXIS_COUNT],
    mem: [AxisMem; AXIS_COUNT],
}

impl TrackingGovernor {
    pub fn new(mut cfg: GovernorConfig, fe_limit: u32) -> Self {
        // 手で組んだ Config でも、工場しきい値が limit を超えないようにする。
        cfg.apply_scaled_defaults(fe_limit);
        Self {
            cfg,
            fe_limit,
            scale: 1.0,
            last_now: None,
            clear_since: None,
            cause: None,
            lagging: [false; AXIS_COUNT],
            mem: Default::default(),
        }
    }

    pub fn config(&self) -> &GovernorConfig {
        &self.cfg
    }

    /// この tick で「目標方向に遅れ、かつ飽和または硬い誤差」だった軸。
    /// status の `axis` は最悪の 1 本だけだが、無進捗の窓は全部を見る。
    pub fn lagging_axes(&self) -> [bool; AXIS_COUNT] {
        self.lagging
    }

    pub fn status(&self) -> GovernorStatus {
        let (axis, reason) = match self.cause {
            Some((a, r)) if self.scale < 1.0 => (Some(a), Some(r)),
            _ => (None, None),
        };
        GovernorStatus {
            scale: self.scale,
            active: self.scale < 1.0,
            axis,
            reason,
        }
    }

    /// 1 tick。戻り値は適用すべき `s`。
    pub fn update(&mut self, now: Instant, axes: &[AxisSample; AXIS_COUNT]) -> f64 {
        if !self.cfg.enabled {
            self.reset_runtime();
            return 1.0;
        }

        let gap = match self.last_now {
            Some(prev) => now.saturating_duration_since(prev),
            None => Duration::ZERO,
        };
        self.last_now = Some(now);
        // 空いたあいだ「解放が続いていた」とは数えない。積分の dt も頭打ち。
        // 飽和の計時（sat_since）は壁時計のまま（空きではリセットしない）。
        if gap > RELEASE_GAP || gap > Duration::from_millis(self.cfg.stale_ms) {
            self.clear_since = None;
        }
        let dt = {
            let secs = gap.min(MAX_INTEGRATE_DT).as_secs_f64();
            if secs.is_finite() && secs > 0.0 {
                secs
            } else {
                0.0
            }
        };

        let mut any_stale = false;
        let mut all_clear = true;
        let mut any_active = false;
        let mut worst: Option<(usize, f64, GovernReason)> = None;

        for i in 0..AXIS_COUNT {
            self.lagging[i] = false;
            let sample = &axes[i];
            if !sample.active {
                self.mem[i] = AxisMem::default();
                continue;
            }
            any_active = true;
            let stale = sample.status_age_ms > self.cfg.stale_ms;
            if stale {
                // 古い PWM / FE では飽和も遅れも判定しない。上げることだけ禁じる。
                any_stale = true;
                all_clear = false;
                self.mem[i] = AxisMem::default();
                continue;
            }

            let abs_fe = sample.following_error_count.unsigned_abs();
            let saturated = pwm_saturated(sample.pwm_permille, sample.duty_max_permille);
            self.note_saturation(i, now, saturated);
            // 非飽和のとき経過は 0。sat_ms = 0 でも「今飽和している」ことが要る。
            let sat_long = saturated
                && self.saturation_elapsed(i, now) >= Duration::from_millis(self.cfg.sat_ms);
            let not_falling = trend_not_decreasing(&self.mem[i].fe_hist, now, abs_fe);
            self.push_fe(i, now, abs_fe);

            let behind = is_behind(sample.following_error_count, sample.target_vel_sign);
            let sat_lag = sat_long && abs_fe >= self.cfg.fe_start;
            let hard_lag = abs_fe >= self.cfg.fe_hard && not_falling;
            if behind && (sat_lag || hard_lag) {
                self.lagging[i] = true;
                let reason = if hard_lag {
                    GovernReason::FollowingError
                } else {
                    GovernReason::PwmSaturated
                };
                let st = self.s_target_for(abs_fe);
                let take = match worst {
                    None => true,
                    Some((j, w, _)) => st < w - 1e-12 || ((st - w).abs() <= 1e-12 && i < j),
                };
                if take {
                    worst = Some((i, st, reason));
                }
            }

            // 解放は「|FE| が fe_release 未満、かつ今は飽和していない」。
            let clear = abs_fe < self.cfg.fe_release && !saturated;
            if !clear {
                all_clear = false;
            }
        }

        // 追跡中の軸が無いときは真空に解放（停止中に s を戻す）。
        // 条件が続いているあいだ clear_since は消さない。毎 tick リセットすると
        // release_ms が永遠に埋まらない。
        if !any_active || (all_clear && !any_stale) {
            if self.clear_since.is_none() {
                self.clear_since = Some(now);
            }
        } else {
            self.clear_since = None;
        }

        let s_target = worst.map(|(_, s, _)| s).unwrap_or(1.0);
        if let Some((i, st, reason)) = worst {
            if st < 1.0 {
                if let Some(id) = AxisId::from_index(i) {
                    self.cause = Some((id, reason));
                }
            }
        }

        let release_ready = self.clear_since.is_some_and(|t| {
            now.saturating_duration_since(t) >= Duration::from_millis(self.cfg.release_ms)
        });
        let allow_rise = release_ready && !any_stale;

        let mut goal = s_target.clamp(self.cfg.s_min, 1.0);
        if goal > self.scale && !allow_rise {
            goal = self.scale;
        }
        let rate = if goal < self.scale {
            self.cfg.down_rate
        } else {
            self.cfg.up_rate
        };
        let step = rate * dt;
        if (goal - self.scale).abs() <= step {
            self.scale = goal;
        } else if goal > self.scale {
            self.scale += step;
        } else {
            self.scale -= step;
        }
        if !self.scale.is_finite() {
            self.scale = 1.0;
        }
        self.scale = self.scale.clamp(self.cfg.s_min, 1.0);
        if self.scale >= 1.0 - 1e-12 {
            self.scale = 1.0;
            self.cause = None;
        }
        self.scale
    }

    fn reset_runtime(&mut self) {
        self.scale = 1.0;
        self.last_now = None;
        self.clear_since = None;
        self.cause = None;
        self.lagging = [false; AXIS_COUNT];
        self.mem = Default::default();
    }

    fn s_target_for(&self, abs_fe: u32) -> f64 {
        let start = f64::from(self.cfg.fe_start);
        let limit = f64::from(self.fe_limit);
        let span = limit - start;
        let raw = if span > 0.0 {
            1.0 - (f64::from(abs_fe) - start) / span
        } else {
            self.cfg.s_min
        };
        raw.clamp(self.cfg.s_min, 1.0)
    }

    fn note_saturation(&mut self, i: usize, now: Instant, saturated: bool) {
        if saturated {
            if self.mem[i].sat_since.is_none() {
                self.mem[i].sat_since = Some(now);
            }
        } else {
            self.mem[i].sat_since = None;
        }
    }

    fn saturation_elapsed(&self, i: usize, now: Instant) -> Duration {
        self.mem[i]
            .sat_since
            .map(|t| now.saturating_duration_since(t))
            .unwrap_or(Duration::ZERO)
    }

    fn push_fe(&mut self, i: usize, now: Instant, abs_fe: u32) {
        let hist = &mut self.mem[i].fe_hist;
        hist.push_back((now, abs_fe));
        let keep_after = now.checked_sub(Duration::from_millis(500)).unwrap_or(now);
        while hist.len() >= 2 && hist[0].0 < keep_after {
            hist.pop_front();
        }
    }
}

/// 追跡用の制限。速度は `s`、加速度は `s²`。停止には使わない。
pub fn scale_track_limits(
    limits: [AxisLimits; AXIS_COUNT],
    scale: f64,
) -> [AxisLimits; AXIS_COUNT] {
    let s = if scale.is_finite() {
        scale.clamp(0.0, 1.0)
    } else {
        1.0
    };
    // s = 0 だと max_acc が 0 になりプランナーがステップを拒否する。
    // ガバナは s_min > 0 しか返さないが、ここでも加速度だけは正に保つ。
    let s_acc = s.max(1e-6);
    limits.map(|l| AxisLimits {
        max_vel_count_s: l.max_vel_count_s * s,
        max_acc_count_s2: l.max_acc_count_s2 * s_acc * s_acc,
    })
}

/// プランナー速度からガバナへ渡す符号。
pub fn target_vel_sign(velocity_count_s: f64) -> i8 {
    if !velocity_count_s.is_finite() || velocity_count_s.abs() < VEL_SIGN_MIN_COUNT_S {
        0
    } else if velocity_count_s > 0.0 {
        1
    } else {
        -1
    }
}

fn sign_i8(v: i8) -> i8 {
    if v > 0 {
        1
    } else if v < 0 {
        -1
    } else {
        0
    }
}

fn sign_i32(v: i32) -> i8 {
    if v > 0 {
        1
    } else if v < 0 {
        -1
    } else {
        0
    }
}

/// 実位置が目標の進行方向の後ろにいる。行き過ぎ（FE と速度が同符号）は遅れではない。
fn is_behind(following_error_count: i32, target_vel_sign: i8) -> bool {
    let vel = sign_i8(target_vel_sign);
    let fe = sign_i32(following_error_count);
    vel != 0 && fe != 0 && fe == -vel
}

fn pwm_saturated(pwm_permille: i16, duty_max_permille: i16) -> bool {
    let pwm = i32::from(pwm_permille.unsigned_abs());
    let thr = i32::from(duty_max_permille) - SAT_MARGIN_PERMILLE;
    pwm >= thr
}

/// `window` 以上前のサンプルより |FE| が小さくなっていなければ true。
/// 窓を埋めるサンプルが無ければ false（まだ「減っていない」とは言わない）。
fn trend_not_decreasing(hist: &VecDeque<(Instant, u32)>, now: Instant, abs_fe: u32) -> bool {
    let Some(cutoff) = now.checked_sub(Duration::from_millis(FE_TREND_WINDOW_MS)) else {
        return false;
    };
    let mut past = None;
    for &(t, fe) in hist {
        if t <= cutoff {
            past = Some(fe);
        } else {
            break;
        }
    }
    past.is_some_and(|p| abs_fe >= p)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg_fast() -> GovernorConfig {
        GovernorConfig {
            sat_ms: 0,
            release_ms: 100,
            down_rate: 10.0,
            up_rate: 1.0,
            ..GovernorConfig::default()
        }
    }

    fn gov(cfg: GovernorConfig) -> TrackingGovernor {
        TrackingGovernor::new(cfg, 400)
    }

    fn ax(fe: i32, pwm: i16, sign: i8, age: u64, active: bool) -> AxisSample {
        AxisSample {
            following_error_count: fe,
            pwm_permille: pwm,
            duty_max_permille: 400,
            target_vel_sign: sign,
            status_age_ms: age,
            active,
        }
    }

    fn idle() -> AxisSample {
        ax(0, 0, 0, 0, false)
    }

    fn only(a: AxisSample) -> [AxisSample; AXIS_COUNT] {
        [a, idle(), idle()]
    }

    /// 正方向へ進む目標に対し、実位置が後ろ（FE が負）で Duty 上限に張り付いている。
    fn lagging(fe_abs: i32) -> AxisSample {
        ax(-fe_abs, 400, 1, 0, true)
    }

    fn step(g: &mut TrackingGovernor, now: &mut Instant, ms: u64, axes: &[AxisSample; 3]) -> f64 {
        *now += Duration::from_millis(ms);
        g.update(*now, axes)
    }

    /// 20 ms 刻みで `s_min` まで下げる。1 回の長い step は dt 頭打ちで届かない。
    fn descend_to_floor(
        g: &mut TrackingGovernor,
        now: &mut Instant,
        axes: &[AxisSample; 3],
    ) -> f64 {
        g.update(*now, axes);
        let mut s = 1.0;
        for _ in 0..40 {
            s = step(g, now, 20, axes);
            if (s - g.config().s_min).abs() < 1e-9 {
                break;
            }
        }
        s
    }

    fn expected_s(fe_abs: u32) -> f64 {
        let raw = 1.0 - (f64::from(fe_abs) - 120.0) / (400.0 - 120.0);
        raw.clamp(0.25, 1.0)
    }

    #[test]
    fn defaults_match_the_incident_tuning() {
        let d = GovernorConfig::default();
        assert!(d.enabled);
        assert_eq!((d.fe_start, d.fe_hard, d.fe_release), (120, 200, 60));
        assert_eq!(d.s_min, 0.25);
        assert_eq!((d.down_rate, d.up_rate), (4.0, 0.5));
        assert_eq!((d.sat_ms, d.release_ms, d.stale_ms), (60, 300, 100));
        assert_eq!(d.no_progress_ms, 1500);
        assert!(!d.thresholds_explicit);
        assert!(d.validation_errors(400).is_empty());
        assert!(d.validation_errors(800).is_empty());
        // limit が工場の fe_hard 以下でも、検査は縮めた値で通り、self は工場値のまま。
        assert!(d.validation_errors(200).is_empty());
        assert_eq!(d.fe_hard, FACTORY_FE_HARD);
    }

    #[test]
    fn validation_rejects_inverted_thresholds_and_bad_rates() {
        let mut c = GovernorConfig::default();
        c.fe_release = 120;
        assert!(c
            .validation_errors(400)
            .iter()
            .any(|e| e.contains("fe_release")));
        c = GovernorConfig::default();
        c.fe_hard = 100;
        assert!(c
            .validation_errors(400)
            .iter()
            .any(|e| e.contains("fe_start")));
        c = GovernorConfig::default();
        c.fe_hard = 400;
        assert!(c
            .validation_errors(400)
            .iter()
            .any(|e| e.contains("following_error_limit")));
        c = GovernorConfig::default();
        c.s_min = 0.0;
        assert!(c.validation_errors(400).iter().any(|e| e.contains("s_min")));
        c.s_min = 1.1;
        assert!(c.validation_errors(400).iter().any(|e| e.contains("s_min")));
        c = GovernorConfig::default();
        c.down_rate = 0.0;
        c.up_rate = -1.0;
        let errs = c.validation_errors(400);
        assert!(errs.iter().any(|e| e.contains("down_rate")));
        assert!(errs.iter().any(|e| e.contains("up_rate")));
        c = GovernorConfig::default();
        c.s_min = 1.0;
        assert!(c.validation_errors(400).is_empty());
        c = GovernorConfig::default();
        c.no_progress_ms = 0;
        assert!(c
            .validation_errors(400)
            .iter()
            .any(|e| e.contains("no_progress_ms")));
        // 無効でも no_progress_ms と rate は見る。しきい値の順序は見ない。
        c.enabled = false;
        c.fe_hard = 500;
        assert!(c
            .validation_errors(200)
            .iter()
            .any(|e| e.contains("no_progress_ms")));
        assert!(!c
            .validation_errors(200)
            .iter()
            .any(|e| e.contains("fe_hard")));
    }

    #[test]
    fn factory_thresholds_scale_only_when_enabled_and_not_explicit() {
        let mut c = GovernorConfig::default();
        c.apply_scaled_defaults(200);
        assert_eq!((c.fe_release, c.fe_start, c.fe_hard), (30, 60, 100));
        assert!(c.validation_errors(200).is_empty());
        // 工場値ではなくなるので 2 回目は上書きしない。
        c.fe_hard = 90;
        c.apply_scaled_defaults(200);
        assert_eq!(c.fe_hard, 90);

        let mut wide = GovernorConfig::default();
        wide.apply_scaled_defaults(400);
        assert_eq!(
            (wide.fe_release, wide.fe_start, wide.fe_hard),
            (60, 120, 200)
        );
        wide.apply_scaled_defaults(800);
        assert_eq!(wide.fe_hard, 200);

        // 無効、または TOML が数を書いたときは縮めない。明示の違反は検査が落とす。
        let mut off = GovernorConfig::default();
        off.enabled = false;
        off.fe_hard = 500;
        off.apply_scaled_defaults(200);
        assert_eq!(off.fe_hard, 500);
        assert!(off.validation_errors(200).is_empty());

        let mut explicit = GovernorConfig::default();
        explicit.thresholds_explicit = true;
        explicit.apply_scaled_defaults(200);
        assert_eq!(explicit.fe_hard, FACTORY_FE_HARD);
        explicit.fe_hard = 500;
        assert!(explicit
            .validation_errors(200)
            .iter()
            .any(|e| e.contains("fe_hard")));

        // Rust 側で工場値を壊しただけ（明示フラグなし）は縮めず、順序で落とす。
        let mut edited = GovernorConfig::default();
        edited.fe_hard = 400;
        edited.apply_scaled_defaults(200);
        assert_eq!(edited.fe_hard, 400);
        assert!(edited
            .validation_errors(400)
            .iter()
            .any(|e| e.contains("following_error_limit")));
    }

    #[test]
    fn omitted_thresholds_deserialize_as_factory_and_explicit_numbers_stay() {
        let omitted: GovernorConfig = toml::from_str("enabled = true\n").unwrap();
        assert!(!omitted.thresholds_explicit);
        assert_eq!(omitted.fe_hard, FACTORY_FE_HARD);
        assert_eq!(omitted.no_progress_ms, 1500);

        let explicit: GovernorConfig = toml::from_str(
            "fe_start = 120\nfe_hard = 200\nfe_release = 60\nno_progress_ms = 800\n",
        )
        .unwrap();
        assert!(explicit.thresholds_explicit);
        assert_eq!(
            (explicit.fe_release, explicit.fe_start, explicit.fe_hard),
            (60, 120, 200)
        );
        assert_eq!(explicit.no_progress_ms, 800);
        let mut kept = explicit;
        kept.apply_scaled_defaults(200);
        assert_eq!(kept.fe_hard, 200);
    }

    #[test]
    fn overshoot_does_not_count_as_lag() {
        let mut g = gov(cfg_fast());
        let mut now = Instant::now();
        // 目標は + 方向。FE が正 = 実位置が setpoint より前（行き過ぎ）。
        let overshoot = only(ax(300, 400, 1, 0, true));
        let s = step(&mut g, &mut now, 100, &overshoot);
        assert_eq!(s, 1.0);
        assert!(!g.status().active);
    }

    #[test]
    fn saturation_with_small_following_error_does_not_trigger() {
        let mut g = gov(GovernorConfig::default());
        let mut now = Instant::now();
        let mild = only(ax(-50, 400, 1, 0, true));
        g.update(now, &mild);
        for _ in 0..20 {
            let s = step(&mut g, &mut now, 10, &mild);
            assert_eq!(s, 1.0, "small FE must not govern");
        }
    }

    #[test]
    fn saturation_must_be_continuous_for_sat_ms() {
        let mut g = gov(GovernorConfig::default());
        let mut now = Instant::now();
        let hot = only(lagging(180));
        g.update(now, &hot);
        // 50 ms ではまだ sat_ms（60）に届かない。
        let s = step(&mut g, &mut now, 50, &hot);
        assert_eq!(s, 1.0);
        // さらに 10 ms で 60 ms 連続。ここから下がり始める（このステップの dt は 10 ms）。
        let s = step(&mut g, &mut now, 10, &hot);
        assert!(s < 1.0, "s = {s}");
        assert_eq!(g.status().reason, Some(GovernReason::PwmSaturated));
        assert_eq!(g.status().axis, AxisId::new(1));
        assert!(g.status().active);
    }

    #[test]
    fn saturation_gap_resets_the_timer() {
        let mut g = gov(GovernorConfig::default());
        let mut now = Instant::now();
        let hot = only(lagging(180));
        let cool = only(ax(-180, 100, 1, 0, true));
        g.update(now, &hot);
        step(&mut g, &mut now, 50, &hot);
        step(&mut g, &mut now, 10, &cool);
        // 飽和が途切れた。次の飽和サンプルで計時をやり直すので、
        // そこから 50 ms では sat_ms（60）に届かない。
        step(&mut g, &mut now, 10, &hot);
        let s = step(&mut g, &mut now, 50, &hot);
        assert_eq!(s, 1.0);
        let s = step(&mut g, &mut now, 10, &hot);
        assert!(s < 1.0);
    }

    #[test]
    fn scale_tracks_formula_and_takes_the_worst_axis() {
        let mut g = gov(cfg_fast());
        let mut now = Instant::now();
        // 軸 1 の |FE| 150、軸 2 の |FE| 180（fe_hard 未満なので飽和経路）。軸 2 が律速。
        let axes = [lagging(150), ax(-180, 400, 1, 0, true), idle()];
        g.update(now, &axes);
        let want = expected_s(180);
        let mut s = 1.0;
        for _ in 0..10 {
            s = step(&mut g, &mut now, 20, &axes);
            if (s - want).abs() < 1e-9 {
                break;
            }
        }
        assert!((s - want).abs() < 1e-9, "s {s} want {want}");
        assert_eq!(g.status().axis, AxisId::new(2));
        assert_eq!(g.status().reason, Some(GovernReason::PwmSaturated));
    }

    #[test]
    fn scale_floors_at_s_min() {
        let mut g = gov(cfg_fast());
        let mut now = Instant::now();
        let axes = only(lagging(10_000));
        // 500 ms を 1 回で渡しても積分は 20 ms まで。rate 10 なら 0.2 しか落ちない。
        g.update(now, &axes);
        let one = step(&mut g, &mut now, 500, &axes);
        let capped = 1.0 - 10.0 * Duration::from_millis(20).as_secs_f64();
        assert!(
            (one - capped).abs() < 1e-6,
            "capped step s = {one}, want {capped}"
        );
        let s = descend_to_floor(&mut g, &mut now, &axes);
        assert_eq!(s, 0.25);
        assert!((expected_s(10_000) - 0.25).abs() < 1e-12);
    }

    #[test]
    fn down_rate_bounds_one_step_and_a_delayed_tick_does_not_integrate_the_gap() {
        let mut cfg = GovernorConfig::default();
        cfg.sat_ms = 0;
        cfg.down_rate = 4.0;
        let mut g = gov(cfg);
        let mut now = Instant::now();
        let axes = only(lagging(10_000));
        g.update(now, &axes);
        // 100 ms の空きは 20 ms として積分する。4/s × 0.02 = 0.08。0.6 までは落ちない。
        let s = step(&mut g, &mut now, 100, &axes);
        let one = 1.0 - 4.0 * Duration::from_millis(20).as_secs_f64();
        assert!((s - one).abs() < 1e-6, "s = {s}");
        // 観測した 20 ms を 5 回足すと 100 ms ぶん（0.4）下がる。
        let mut s = 1.0;
        let mut g = gov(GovernorConfig {
            sat_ms: 0,
            down_rate: 4.0,
            ..GovernorConfig::default()
        });
        let mut now = Instant::now();
        g.update(now, &axes);
        for _ in 0..5 {
            s = step(&mut g, &mut now, 20, &axes);
        }
        let mut want = 1.0;
        let dt = Duration::from_millis(20).as_secs_f64();
        for _ in 0..5 {
            want -= 4.0 * dt;
        }
        assert!((s - want).abs() < 1e-6, "s = {s} want {want}");
        assert!((want - 0.6).abs() < 1e-9);
    }

    #[test]
    fn hard_following_error_triggers_without_saturation_when_not_falling() {
        let mut g = gov(cfg_fast());
        let mut now = Instant::now();
        // PWM は低い。|FE| = 250 ≥ fe_hard。60 ms のあいだ減っていない。
        let stuck = only(ax(-250, 50, 1, 0, true));
        g.update(now, &stuck);
        let early = step(&mut g, &mut now, 50, &stuck);
        assert_eq!(early, 1.0, "window not full");
        let s = step(&mut g, &mut now, 10, &stuck);
        assert!(s < 1.0, "s = {s}");
        assert_eq!(g.status().reason, Some(GovernReason::FollowingError));
    }

    #[test]
    fn hard_path_does_not_fire_while_error_is_decreasing() {
        let mut g = gov(cfg_fast());
        let mut now = Instant::now();
        g.update(now, &only(ax(-300, 0, 1, 0, true)));
        // 60 ms 前より |FE| が小さい → 硬い遅れではない。飽和もしていない。
        let s = step(&mut g, &mut now, 60, &only(ax(-250, 0, 1, 0, true)));
        assert_eq!(s, 1.0);
    }

    #[test]
    fn zero_target_velocity_is_not_a_direction() {
        let mut g = gov(cfg_fast());
        let mut now = Instant::now();
        let axes = only(ax(-300, 400, 0, 0, true));
        g.update(now, &axes);
        let s = step(&mut g, &mut now, 100, &axes);
        assert_eq!(s, 1.0);
    }

    #[test]
    fn inactive_axis_is_ignored() {
        let mut g = gov(cfg_fast());
        let mut now = Instant::now();
        let axes = only(ax(-300, 400, 1, 0, false));
        g.update(now, &axes);
        assert_eq!(step(&mut g, &mut now, 100, &axes), 1.0);
    }

    #[test]
    fn disabled_governor_stays_at_one() {
        let mut cfg = cfg_fast();
        cfg.enabled = false;
        let mut g = gov(cfg);
        let mut now = Instant::now();
        let axes = only(lagging(500));
        g.update(now, &axes);
        assert_eq!(step(&mut g, &mut now, 200, &axes), 1.0);
        assert!(!g.status().active);
    }

    #[test]
    fn scale_does_not_rise_until_release_window_and_then_only_at_up_rate() {
        let mut g = gov(cfg_fast());
        let mut now = Instant::now();
        let hot = only(lagging(10_000));
        assert_eq!(descend_to_floor(&mut g, &mut now, &hot), 0.25);

        let clear = only(ax(-10, 0, 1, 0, true));
        // この tick で計時開始（経過 0）。release_ms = 100。10 ms 刻み。
        step(&mut g, &mut now, 10, &clear);
        for _ in 0..9 {
            let held = step(&mut g, &mut now, 10, &clear);
            assert_eq!(held, 0.25);
        }
        // 計時から 100 ms。dt = 10 ms、up_rate 1/s → +0.01。
        let s = step(&mut g, &mut now, 10, &clear);
        assert!((s - 0.26).abs() < 1e-9, "s = {s}");
    }

    #[test]
    fn release_evidence_resets_across_observation_gaps() {
        let clear = only(ax(-10, 0, 1, 0, true));
        let hot = only(lagging(10_000));

        // 21 ms の空きは 2 tick 超。stale_ms が 100 でも解放の計時は捨てる。
        let mut g = gov(cfg_fast());
        let mut now = Instant::now();
        assert_eq!(descend_to_floor(&mut g, &mut now, &hot), 0.25);
        step(&mut g, &mut now, 10, &clear);
        step(&mut g, &mut now, 21, &clear);
        for _ in 0..9 {
            assert_eq!(step(&mut g, &mut now, 10, &clear), 0.25);
        }

        // 16 ms は 20 ms 以下でも stale_ms を超えたら捨てる。
        let mut cfg = cfg_fast();
        cfg.stale_ms = 15;
        let mut g = gov(cfg);
        let mut now = Instant::now();
        assert_eq!(descend_to_floor(&mut g, &mut now, &hot), 0.25);
        step(&mut g, &mut now, 10, &clear);
        step(&mut g, &mut now, 16, &clear);
        for _ in 0..9 {
            assert_eq!(step(&mut g, &mut now, 10, &clear), 0.25);
        }

        // ちょうど 20 ms は連続。そのあと 10 ms を足して 100 ms で上がり始める。
        let mut g = gov(cfg_fast());
        let mut now = Instant::now();
        assert_eq!(descend_to_floor(&mut g, &mut now, &hot), 0.25);
        step(&mut g, &mut now, 10, &clear); // 計時開始
        let still = step(&mut g, &mut now, 20, &clear); // 経過 20 ms。空きでは切らない。
        assert_eq!(still, 0.25);
        for _ in 0..7 {
            assert_eq!(step(&mut g, &mut now, 10, &clear), 0.25); // 経過 90 ms
        }
        let s = step(&mut g, &mut now, 10, &clear); // 経過 100 ms
        assert!((s - 0.26).abs() < 1e-9, "s = {s}");
    }

    #[test]
    fn scale_does_not_rise_while_any_axis_is_above_fe_release_or_saturated() {
        let mut g = gov(cfg_fast());
        let mut now = Instant::now();
        let hot = only(lagging(10_000));
        assert_eq!(descend_to_floor(&mut g, &mut now, &hot), 0.25);

        // |FE| は fe_release 以上。飽和は解けている。10 ms 刻みで release_ms を超えても上げない。
        let mid = only(ax(-80, 0, 1, 0, true));
        for _ in 0..30 {
            assert_eq!(
                step(&mut g, &mut now, 10, &mid),
                0.25,
                "fe_release hysteresis"
            );
        }

        // |FE| は小さいが、まだ飽和。
        let sat = only(ax(-10, 400, 1, 0, true));
        for _ in 0..30 {
            assert_eq!(step(&mut g, &mut now, 10, &sat), 0.25, "still saturated");
        }
    }

    #[test]
    fn stale_status_blocks_increase_and_does_not_cause_a_decrease() {
        let mut g = gov(cfg_fast());
        let mut now = Instant::now();
        // 古い巨大 FE では下げない。
        let stale_hot = only(ax(-500, 400, 1, 101, true));
        g.update(now, &stale_hot);
        assert_eq!(step(&mut g, &mut now, 20, &stale_hot), 1.0);

        let hot = only(lagging(10_000));
        assert_eq!(descend_to_floor(&mut g, &mut now, &hot), 0.25);

        // 誤差は解消したように見えるが STATUS が古い。10 ms 刻みでも上げない
        // （空きで計時が切れたから、ではなく古さそのもので上げない）。
        let stale_clear = only(ax(0, 0, 1, 150, true));
        let mut s = 0.25;
        for _ in 0..30 {
            s = step(&mut g, &mut now, 10, &stale_clear);
        }
        assert_eq!(s, 0.25);
        assert!(g.status().active);
        // 原因の軸は、s が 1 に戻るまで残す。
        assert_eq!(g.status().axis, AxisId::new(1));
    }

    #[test]
    fn fresh_boundary_is_age_equal_to_stale_ms() {
        let mut g = gov(cfg_fast());
        let mut now = Instant::now();
        // age == stale_ms は古くない（仕様は「より大きい」）。
        let axes = only(ax(-400, 400, 1, 100, true));
        g.update(now, &axes);
        let s = step(&mut g, &mut now, 200, &axes);
        assert!(s < 1.0);
    }

    #[test]
    fn lowering_direction_uses_the_opposite_error_sign() {
        let mut g = gov(cfg_fast());
        let mut now = Instant::now();
        // 目標速度が負。遅れ = 実位置が setpoint より大きい = FE 正。
        let behind = only(ax(180, -400, -1, 0, true));
        g.update(now, &behind);
        assert!(step(&mut g, &mut now, 100, &behind) < 1.0);
        // 同符号は行き過ぎ。
        let mut g = gov(cfg_fast());
        let over = only(ax(-180, -400, -1, 0, true));
        g.update(now, &over);
        assert_eq!(step(&mut g, &mut now, 100, &over), 1.0);
    }

    #[test]
    fn scale_track_limits_uses_s_and_s_squared() {
        let base = [AxisLimits {
            max_vel_count_s: 1200.0,
            max_acc_count_s2: 20_000.0,
        }; AXIS_COUNT];
        let out = scale_track_limits(base, 0.5);
        assert_eq!(out[0].max_vel_count_s, 600.0);
        assert_eq!(out[0].max_acc_count_s2, 5_000.0);
        assert_eq!(out[1], out[0]);
        assert_eq!(out[2], out[0]);
        let full = scale_track_limits(base, 1.0);
        assert_eq!(full[0], base[0]);
        assert_eq!(target_vel_sign(0.5), 0);
        assert_eq!(target_vel_sign(-1200.0), -1);
        assert_eq!(target_vel_sign(3.0), 1);
    }
}
