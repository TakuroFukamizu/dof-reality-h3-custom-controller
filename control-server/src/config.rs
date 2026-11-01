//! Versioned TOML configuration. Units are in the field names.
//! See `config/example.toml` for an annotated example.
//!
//! Rules (architecture §6, §11):
//! - Calibration, signs and limits may only change while DISARMED (the server
//!   loads config at start; there is no hot reload in v1).
//! - Every axis is assigned from the axis the operator selected on the MCU
//!   (HELLO `selected_axis`, protocol §1.1); the configured `device_id` is an
//!   optional cross-check (0 = none) and the fallback for a MCU that reports
//!   no selection. Never by USB port order.
//! - Axis 2/3 values must be measured on their own; do not copy axis 1.

use crate::calibration::AxisCalibration;
use crate::types::{AxisId, DeviceId, AXIS_COUNT, VIRTUAL_MAX, VIRTUAL_MIN};
use serde::{Deserialize, Serialize};
use std::net::SocketAddr;
use std::path::Path;

pub const SCHEMA_VERSION: u32 = 2;

/// Upper bound for every `*_ms` setting (1 h). Larger values are typos and
/// would overflow `Instant + Duration`.
pub const MAX_TIMING_MS: u64 = 3_600_000;

/// `ms` as a Duration, clamped to [`MAX_TIMING_MS`] (safe for `Instant +`
/// even on a config that skipped `validate()`).
pub fn bounded_ms(ms: u64) -> std::time::Duration {
    std::time::Duration::from_millis(ms.min(MAX_TIMING_MS))
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Config {
    pub schema_version: u32,
    /// Machine identifier; part of the calibration hash.
    pub rig_id: String,
    #[serde(default)]
    pub server: ServerConfig,
    #[serde(default)]
    pub timing: TimingConfig,
    #[serde(default)]
    pub game: GameConfig,
    /// Motion limits: the PC-side current set's initial values plus the
    /// values distributed to the MCUs (design 2026-11-01 §2).
    #[serde(default)]
    pub limits: LimitsConfig,
    pub backend: BackendConfig,
    /// The only axis whose MCU drives the shared main-power relay (G2).
    pub power_controller_axis: AxisId,
    #[serde(rename = "axis")]
    pub axes: Vec<AxisConfig>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct ServerConfig {
    /// Must be a loopback address unless `allow_non_loopback` is set.
    pub bind: String,
    pub allow_non_loopback: bool,
    /// Require `Authorization: Bearer <token>` on admin routes. Default
    /// `false`: a PC-local setup (loopback bind) needs no token; the Origin
    /// check still applies. Must be `true` when `allow_non_loopback` is set.
    pub require_admin_token: bool,
    /// Admin token is read from this environment variable at start (only when
    /// `require_admin_token`). If the variable is unset a random token is
    /// generated and printed once.
    pub admin_token_env: String,
    /// Allowed browser Origins for HTTP/WS. Requests without an Origin header
    /// (non-browser clients such as bridge.py) are allowed.
    pub allowed_origins: Vec<String>,
    pub max_body_bytes: usize,
    /// Max admin requests per second (token bucket); 0 disables.
    pub admin_rate_limit_per_s: u32,
    /// Status broadcast period on WS /v1/motion.
    pub status_period_ms: u64,
}

impl Default for ServerConfig {
    fn default() -> Self {
        Self {
            bind: "127.0.0.1:8892".into(),
            allow_non_loopback: false,
            require_admin_token: false,
            admin_token_env: "H3CS_ADMIN_TOKEN".into(),
            allowed_origins: vec!["http://localhost:8891".into(), "http://127.0.0.1:8891".into()],
            max_body_bytes: 4096,
            admin_rate_limit_per_s: 20,
            status_period_ms: 100,
        }
    }
}

/// Proposed bench values (architecture §8), not validated limits.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct TimingConfig {
    pub planner_period_ms: u64,
    pub set_position_valid_for_ms: u16,
    pub mcu_command_timeout_ms: u16,
    pub mcu_stop_timeout_ms: u16,
    /// Axis considered disconnected if no STATUS for this long.
    pub axis_status_timeout_ms: u64,
    /// Game pose older than this (by source_age + server receive time) is stale.
    pub source_freshness_ms: u64,
    /// ARMING must reach HOLDING on all axes within this.
    pub arm_timeout_ms: u64,
    /// STOPPING must settle within this.
    pub stop_settle_timeout_ms: u64,
    /// After a stop settles, hold this long before applying brakes (BRAKED_DISARMED).
    pub stop_hold_ms: u64,
    /// Max allowed spread of last_applied_frame across axes while ACTIVE.
    pub max_frame_skew: u32,
    /// Center: once the planner is at neutral, every axis's MEASURED position must
    /// be within `game.center_tolerance_count` of neutral within this, or the rig
    /// faults (StopTimeout) instead of reporting `center_reached` on a lagging axis.
    pub center_reach_timeout_ms: u64,
    /// Measured rest (Issue #15 H1): an axis is "at rest" when STATUS reports
    /// |velocity_count_s| <= `rest_velocity_count_s` AND
    /// |following_error_count| <= `rest_error_count`, continuously for
    /// `rest_dwell_ms`. STOPPING (Decel -> Hold, Hold -> DISARM) and the
    /// post-center settle require every axis at rest, not just a trajectory
    /// at velocity 0: the MCU declares HOLDING when its SETPOINT stops, while a
    /// gravity-loaded axis can still be moving / hundreds of counts off.
    pub rest_velocity_count_s: u32,
    /// Must cover the firmware deadband (`limits.deadband_out_count`).
    pub rest_error_count: u32,
    pub rest_dwell_ms: u64,
    /// Handshake: HELLO_REQ retry period and overall timeout per port.
    pub hello_retry_ms: u64,
    pub hello_timeout_ms: u64,
}

impl Default for TimingConfig {
    fn default() -> Self {
        Self {
            planner_period_ms: 10,
            set_position_valid_for_ms: 50,
            mcu_command_timeout_ms: 100,
            mcu_stop_timeout_ms: 1500,
            axis_status_timeout_ms: 200,
            source_freshness_ms: 250,
            arm_timeout_ms: 5000,
            stop_settle_timeout_ms: 3000,
            stop_hold_ms: 1000,
            max_frame_skew: 5,
            center_reach_timeout_ms: 5000,
            rest_velocity_count_s: 50,
            rest_error_count: 50,
            rest_dwell_ms: 200,
            hello_retry_ms: 200,
            hello_timeout_ms: 2000,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct GameConfig {
    /// When false, a pose without `frame_seq` is rejected. When true it is accepted
    /// and freshness is "timer only" (cannot detect a frozen game loop).
    pub allow_missing_frame_seq: bool,
    /// Compat: clamp pose components outside -1..=1 (and report). When false, reject.
    pub clamp_out_of_range: bool,
    /// Virtual-position limits applied after mixing (inside 0..=1023).
    pub virtual_min: f64,
    pub virtual_max: f64,
    /// Planner scale for the admin `center` move (fraction of axis max velocity).
    pub center_speed_fraction: f64,
    /// Center is complete only when every axis's MEASURED count (STATUS.actual_count)
    /// is within this many counts of neutral. Must cover the firmware deadband
    /// (`limits.deadband_out_count`) with margin; 2026-11-01 a lagging axis was
    /// declared centered 280 counts short because only the setpoint was checked.
    pub center_tolerance_count: u32,
    /// Rig orientation (compat::Orientation): the game sends pitch / roll / yaw
    /// in its own sense; these map them onto this rig's mixer inputs. The
    /// operator sets them after the orientation check (hardware-bringup.md §7);
    /// the game never needs to know. Rebuilt H3 2026-11-02: swap_pitch_roll = true.
    pub swap_pitch_roll: bool,
    pub invert_pitch: bool,
    pub invert_roll: bool,
    pub invert_yaw: bool,
    /// START (game) is refused (`pose_not_near_hold`) when the fresh pose
    /// would command any axis farther than this from where it is held: a
    /// forgotten tab streaming an extreme pose must not launch the seat at
    /// START (Issue #15 M4). The game should be at neutral when START is
    /// pressed. Measured in counts along the same path as tracking
    /// (mix → saturate → calibration → soft limits).
    pub start_max_offset_count: u32,
}

impl GameConfig {
    pub fn orientation(&self) -> crate::compat::Orientation {
        crate::compat::Orientation {
            swap_pitch_roll: self.swap_pitch_roll,
            invert_pitch: self.invert_pitch,
            invert_roll: self.invert_roll,
            invert_yaw: self.invert_yaw,
        }
    }
}

impl Default for GameConfig {
    fn default() -> Self {
        Self {
            allow_missing_frame_seq: false,
            clamp_out_of_range: true,
            virtual_min: 0.0,
            virtual_max: 1023.0,
            center_speed_fraction: 0.25,
            center_tolerance_count: 50,
            swap_pitch_roll: false,
            invert_pitch: false,
            invert_roll: false,
            invert_yaw: false,
            start_max_offset_count: 300,
        }
    }
}

/// `[limits]`: one set for all three axes. The first four are the runtime
/// set's initial values (changed by WS `limits` / `POST /v1/limits`, reset
/// on `POST /v1/session`); the rest are distributed to the MCUs (SET_LIMITS,
/// CONFIG) and not changeable at runtime. Every value is an estimate not yet
/// verified on the rig (`docs/control-server/limits.md`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct LimitsConfig {
    /// Gain ceiling for the game (≤ 1.0; 0.5 keeps a full pitch+roll inside
    /// virtual 0..=1023).
    pub max_gain: f64,
    pub max_vel_count_s: u32,
    pub max_acc_count_s2: u32,
    /// Position-loop duty cap sent to the MCUs (clamped there to the
    /// firmware's absolute ceiling).
    pub duty_max_percent: u8,
    /// Smallest non-zero duty the loop commands (the firmware default 7 %
    /// made slow motion step).
    pub duty_min_percent: u8,
    pub kp_percent_per_100: u8,
    /// Feed-forward: trajectory speed [count/s] ÷ this = duty [%].
    pub ff_count_s_per_percent: u16,
    pub deadband_count: u16,
    pub deadband_out_count: u16,
    /// Protection thresholds (CONFIG).
    pub following_error_limit_count: u32,
    /// Must be ≥ 1.5 × `max_vel_count_s`; also caps runtime velocity
    /// requests at `overspeed / 1.5`.
    pub overspeed_count_s: u32,
}

impl Default for LimitsConfig {
    fn default() -> Self {
        Self {
            max_gain: 0.5,
            max_vel_count_s: 3000,
            max_acc_count_s2: 15000,
            duty_max_percent: 40,
            duty_min_percent: 3,
            kp_percent_per_100: 2,
            ff_count_s_per_percent: 100,
            deadband_count: 10,
            deadband_out_count: 20,
            following_error_limit_count: 800,
            overspeed_count_s: 9000,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum BackendConfig {
    /// Three in-process simulated axes (no hardware).
    Sim(SimBackendConfig),
    /// Real CoreS3 axes over USB CDC.
    Serial(SerialBackendConfig),
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct SimBackendConfig {
    /// Simulated MCUs establish a position reference at boot (bench convenience).
    pub start_referenced: bool,
    /// Simulated plant: max achievable velocity (count/s).
    pub plant_max_vel_count_s: f64,
    /// Simulated STATUS period.
    pub status_period_ms: u64,
    /// Simulated one-way link latency.
    pub link_latency_ms: u64,
    /// Simulated axis N reports `selected_axis = N` (the assignment comes from
    /// the selection; `device_id` is only cross-checked). `false`: it reports
    /// 0 and the axis is found through the configured `device_id` alone (the
    /// fallback path for firmware without axis selection).
    pub report_selected_axis: bool,
    /// Mechanical end stops of the simulated mechanism (boot coordinate). The
    /// plant cannot pass them; CALIBRATE finds them (`end_min_count` is the
    /// P end, so a calibrated sim reports `n_end_count = max − min`).
    pub end_min_count: i32,
    pub end_max_count: i32,
}

impl Default for SimBackendConfig {
    fn default() -> Self {
        Self {
            start_referenced: true,
            plant_max_vel_count_s: 4000.0,
            status_period_ms: 20,
            link_latency_ms: 1,
            report_selected_axis: true,
            end_min_count: -3000,
            end_max_count: 3000,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SerialBackendConfig {
    /// Ports to probe. Each is assigned from its HELLO (`selected_axis`, else
    /// the `[[axis]]` with its `device_id`) before use; a port that resolves
    /// to no axis is closed without sending anything except HELLO_REQ.
    pub ports: Vec<String>,
    #[serde(default = "default_baud")]
    pub baud: u32,
    /// Axes served by an in-process simulated unit instead of a real one
    /// (bring-up with fewer real units: the rig still needs three axes; a
    /// simulated axis follows its target perfectly, so with a pose that keeps
    /// it at its neutral it just sits there). Its `[[axis]]` must be a valid
    /// calibration like in the sim backend. A real unit that selects a
    /// simulated axis becomes an `axis_conflict` (the simulated unit is
    /// identified first).
    #[serde(default)]
    pub sim_axes: Vec<AxisId>,
    /// Parameters of those simulated units.
    #[serde(default)]
    pub sim: SimBackendConfig,
}

fn default_baud() -> u32 {
    115_200
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AxisConfig {
    pub axis_id: AxisId,
    /// Optional cross-check: the MCU expected on this axis (0 = none). An
    /// MCU that selects this axis with another device_id blocks ARM
    /// (`axis_device_mismatch`); an MCU reporting `selected_axis = 0` is
    /// assigned to the axis whose device_id matches (fallback).
    pub device_id: DeviceId,
    pub name: String,
    /// Operator assertion that the calibration below was measured on THIS axis.
    /// The runtime calibration is valid only if this is true AND the MCU reports
    /// POSITION_REFERENCED for its current boot.
    pub calibration_valid: bool,
    /// Virtual position → count at virtual 0, 512, 1023 (piecewise linear).
    /// Strictly monotonic (increasing or decreasing); direction may differ per axis.
    pub count_at_virtual_0: i32,
    pub count_at_virtual_512: i32,
    pub count_at_virtual_1023: i32,
    /// Measured mechanical end stops (not targets).
    pub mechanical_min_count: i32,
    pub mechanical_max_count: i32,
    /// Operational soft limits; must lie strictly inside the mechanical range by
    /// at least `decel_zone_count`, and contain all three calibration points.
    pub soft_min_count: i32,
    pub soft_max_count: i32,
    pub decel_zone_count: i32,
}

#[derive(Debug, thiserror::Error)]
pub enum ConfigError {
    #[error("read {path}: {source}")]
    Io { path: String, source: std::io::Error },
    #[error("parse: {0}")]
    Parse(#[from] toml::de::Error),
    #[error("invalid config: {0}")]
    Invalid(String),
}

impl Config {
    pub fn load(path: &Path) -> Result<Self, ConfigError> {
        let text = std::fs::read_to_string(path)
            .map_err(|source| ConfigError::Io { path: path.display().to_string(), source })?;
        Self::from_toml_str(&text)
    }

    pub fn from_toml_str(text: &str) -> Result<Self, ConfigError> {
        let cfg: Config = toml::from_str(text)?;
        cfg.validate()?;
        Ok(cfg)
    }

    /// Structural validation. Must reject: wrong schema_version; not exactly one
    /// axis per AxisId 1..=3; duplicate device_id; non-loopback bind without
    /// allow_non_loopback; virtual_min/max outside 0..=1023 or min >= max;
    /// timing values of 0; `[limits]` rules (below); per-axis calibration
    /// invariants (see `crate::calibration::AxisCalibration::validate`).
    ///
    /// All-zero device_ids (`DeviceId::is_unassigned`) mean "no cross-check"
    /// and are allowed on any number of axes (the axis is then assigned from
    /// the MCU's `selected_axis` alone). Duplicate non-zero device_ids are
    /// rejected.
    ///
    /// All problems are collected and reported together, each naming its field.
    pub fn validate(&self) -> Result<(), ConfigError> {
        let mut errs: Vec<String> = Vec::new();

        if self.schema_version != SCHEMA_VERSION {
            errs.push(format!("schema_version must be {SCHEMA_VERSION}, got {}", self.schema_version));
        }
        if self.rig_id.trim().is_empty() {
            errs.push("rig_id must not be empty".into());
        }

        // Axes: exactly one per AxisId; non-zero device_ids unique.
        if self.axes.len() != AXIS_COUNT {
            errs.push(format!("expected exactly {AXIS_COUNT} [[axis]] entries, got {}", self.axes.len()));
        }
        for id in AxisId::ALL {
            match self.axes.iter().filter(|a| a.axis_id == id).count() {
                0 => errs.push(format!("[[axis]] axis_id = {} is missing", id.get())),
                1 => {}
                n => errs.push(format!("[[axis]] axis_id = {} appears {n} times", id.get())),
            }
        }
        for (i, a) in self.axes.iter().enumerate() {
            if a.device_id.is_unassigned() {
                continue;
            }
            if let Some(b) = self.axes[..i].iter().find(|b| b.device_id == a.device_id) {
                errs.push(format!(
                    "duplicate device_id {} on axis_id = {} and axis_id = {}",
                    a.device_id,
                    b.axis_id.get(),
                    a.axis_id.get()
                ));
            }
        }
        for a in &self.axes {
            if let Err(e) = AxisCalibration::from_config(a).validate() {
                errs.push(format!("[[axis]] axis_id = {}: {e}", a.axis_id.get()));
            }
        }

        // Server.
        let s = &self.server;
        match s.bind.parse::<SocketAddr>() {
            Ok(addr) if !addr.ip().is_loopback() && !s.allow_non_loopback => errs.push(format!(
                "server.bind {} is not a loopback address (set server.allow_non_loopback to allow)",
                s.bind
            )),
            Ok(_) => {}
            Err(_) => errs.push(format!("server.bind must be an IP:port literal (e.g. 127.0.0.1:8892), got {:?}", s.bind)),
        }
        if s.admin_token_env.trim().is_empty() {
            errs.push("server.admin_token_env must not be empty".into());
        }
        if s.allow_non_loopback && !s.require_admin_token {
            errs.push("server.require_admin_token must be true when allow_non_loopback is set".into());
        }
        if s.max_body_bytes == 0 {
            errs.push("server.max_body_bytes must be > 0".into());
        }
        if s.status_period_ms == 0 {
            errs.push("server.status_period_ms must be > 0".into());
        }

        // Timing: every value must be non-zero.
        let t = &self.timing;
        for (name, v) in [
            ("timing.planner_period_ms", t.planner_period_ms),
            ("timing.set_position_valid_for_ms", u64::from(t.set_position_valid_for_ms)),
            ("timing.mcu_command_timeout_ms", u64::from(t.mcu_command_timeout_ms)),
            ("timing.mcu_stop_timeout_ms", u64::from(t.mcu_stop_timeout_ms)),
            ("timing.axis_status_timeout_ms", t.axis_status_timeout_ms),
            ("timing.source_freshness_ms", t.source_freshness_ms),
            ("timing.arm_timeout_ms", t.arm_timeout_ms),
            ("timing.stop_settle_timeout_ms", t.stop_settle_timeout_ms),
            ("timing.stop_hold_ms", t.stop_hold_ms),
            ("timing.max_frame_skew", u64::from(t.max_frame_skew)),
            ("timing.center_reach_timeout_ms", t.center_reach_timeout_ms),
            ("timing.rest_velocity_count_s", u64::from(t.rest_velocity_count_s)),
            ("timing.rest_error_count", u64::from(t.rest_error_count)),
            ("timing.rest_dwell_ms", t.rest_dwell_ms),
            ("timing.hello_retry_ms", t.hello_retry_ms),
            ("timing.hello_timeout_ms", t.hello_timeout_ms),
        ] {
            if v == 0 {
                errs.push(format!("{name} must be > 0"));
            }
        }
        for (name, v) in [
            ("timing.planner_period_ms", t.planner_period_ms),
            ("timing.set_position_valid_for_ms", u64::from(t.set_position_valid_for_ms)),
            ("timing.mcu_command_timeout_ms", u64::from(t.mcu_command_timeout_ms)),
            ("timing.mcu_stop_timeout_ms", u64::from(t.mcu_stop_timeout_ms)),
            ("timing.axis_status_timeout_ms", t.axis_status_timeout_ms),
            ("timing.source_freshness_ms", t.source_freshness_ms),
            ("timing.arm_timeout_ms", t.arm_timeout_ms),
            ("timing.stop_settle_timeout_ms", t.stop_settle_timeout_ms),
            ("timing.stop_hold_ms", t.stop_hold_ms),
            ("timing.center_reach_timeout_ms", t.center_reach_timeout_ms),
            ("timing.rest_dwell_ms", t.rest_dwell_ms),
            ("timing.hello_retry_ms", t.hello_retry_ms),
            ("timing.hello_timeout_ms", t.hello_timeout_ms),
            ("server.status_period_ms", s.status_period_ms),
        ] {
            if v > MAX_TIMING_MS {
                errs.push(format!("{name} must be <= {MAX_TIMING_MS}, got {v}"));
            }
        }
        // A command must outlive one planner period, or every SET_POSITION expires
        // before the next one arrives.
        if t.planner_period_ms > 0 && u64::from(t.set_position_valid_for_ms) <= t.planner_period_ms {
            errs.push(format!(
                "timing.set_position_valid_for_ms ({}) must be > timing.planner_period_ms ({})",
                t.set_position_valid_for_ms, t.planner_period_ms
            ));
        }
        if t.planner_period_ms > 0 && u64::from(t.mcu_command_timeout_ms) <= t.planner_period_ms {
            errs.push(format!(
                "timing.mcu_command_timeout_ms ({}) must be > timing.planner_period_ms ({})",
                t.mcu_command_timeout_ms, t.planner_period_ms
            ));
        }
        // The rest dwell is part of every stop: it must fit in the settle window.
        if t.stop_settle_timeout_ms > 0 && t.rest_dwell_ms >= t.stop_settle_timeout_ms {
            errs.push(format!(
                "timing.rest_dwell_ms ({}) must be < timing.stop_settle_timeout_ms ({})",
                t.rest_dwell_ms, t.stop_settle_timeout_ms
            ));
        }
        if t.rest_error_count > 0 && u32::from(self.limits.deadband_out_count) > t.rest_error_count {
            // The firmware stops correcting inside deadband_out, so a rest
            // tolerance tighter than that could never be met: every STOP would
            // time out.
            errs.push(format!(
                "timing.rest_error_count ({}) must be >= limits.deadband_out_count ({})",
                t.rest_error_count, self.limits.deadband_out_count
            ));
        }

        // Game.
        let g = &self.game;
        let vrange = VIRTUAL_MIN..=VIRTUAL_MAX;
        if !vrange.contains(&g.virtual_min) {
            errs.push(format!("game.virtual_min must be within 0..=1023, got {}", g.virtual_min));
        }
        if !vrange.contains(&g.virtual_max) {
            errs.push(format!("game.virtual_max must be within 0..=1023, got {}", g.virtual_max));
        }
        if g.virtual_min.partial_cmp(&g.virtual_max) != Some(std::cmp::Ordering::Less) {
            errs.push(format!(
                "game.virtual_min ({}) must be < game.virtual_max ({})",
                g.virtual_min, g.virtual_max
            ));
        }
        if g.center_tolerance_count == 0 {
            errs.push("game.center_tolerance_count must be > 0".into());
        } else if u32::from(self.limits.deadband_out_count) > g.center_tolerance_count {
            // The firmware stops correcting inside deadband_out, so a tolerance
            // tighter than that could never be met: CENTER would always time out.
            errs.push(format!(
                "game.center_tolerance_count ({}) must be >= limits.deadband_out_count ({})",
                g.center_tolerance_count, self.limits.deadband_out_count
            ));
        }
        if !(g.center_speed_fraction > 0.0 && g.center_speed_fraction <= 1.0) {
            errs.push(format!("game.center_speed_fraction must be within (0, 1], got {}", g.center_speed_fraction));
        }
        if g.start_max_offset_count == 0 {
            errs.push("game.start_max_offset_count must be > 0".into());
        }

        // Limits (design 2026-11-01 §2).
        let l = &self.limits;
        if !(l.max_gain.is_finite() && l.max_gain > 0.0 && l.max_gain <= 1.0) {
            errs.push(format!("limits.max_gain must be within (0, 1], got {}", l.max_gain));
        }
        for (name, v) in [
            ("limits.max_vel_count_s", l.max_vel_count_s),
            ("limits.max_acc_count_s2", l.max_acc_count_s2),
            ("limits.following_error_limit_count", l.following_error_limit_count),
            ("limits.overspeed_count_s", l.overspeed_count_s),
        ] {
            if v == 0 {
                errs.push(format!("{name} must be > 0"));
            }
        }
        if !(1..=100).contains(&l.duty_max_percent) {
            errs.push(format!("limits.duty_max_percent must be within 1..=100, got {}", l.duty_max_percent));
        }
        if l.duty_min_percent > l.duty_max_percent {
            errs.push(format!(
                "limits.duty_min_percent ({}) must be <= limits.duty_max_percent ({})",
                l.duty_min_percent, l.duty_max_percent
            ));
        }
        if l.ff_count_s_per_percent == 0 {
            errs.push("limits.ff_count_s_per_percent must be > 0".into());
        }
        if l.deadband_out_count < l.deadband_count {
            errs.push(format!(
                "limits.deadband_out_count ({}) must be >= limits.deadband_count ({})",
                l.deadband_out_count, l.deadband_count
            ));
        }
        if l.kp_percent_per_100 == 0 {
            // With kp = 0 the loop outputs nothing at rest (FF is zero when the
            // trajectory stands still), so the axis cannot hold position at all.
            errs.push("limits.kp_percent_per_100 must be > 0".into());
        }
        // Cross-check (2026-11-01 on hardware): the position loop's output before
        // FOLLOWING_ERROR trips is at most kp x following_error_limit / 100 (+ FF,
        // which is 0 against a static load). If that is below duty_min, the loop
        // can never command more than the floor before the fault: a load that the
        // floor cannot move is either stuck (static) or faults (moving target),
        // with no third outcome. kp 2 x 400 / 100 = 8 % vs a measured 9 % breakaway
        // did exactly that. duty_min_percent should be the measured breakaway duty.
        let authority = f64::from(l.kp_percent_per_100) * f64::from(l.following_error_limit_count) / 100.0;
        if authority < f64::from(l.duty_min_percent) {
            errs.push(format!(
                "limits.kp_percent_per_100 ({}) x limits.following_error_limit_count ({}) / 100 = {authority:.1} % \
                 must be >= limits.duty_min_percent ({} %): the position loop would never exceed the duty floor \
                 before FOLLOWING_ERROR (raise kp or following_error_limit_count, or lower duty_min)",
                l.kp_percent_per_100, l.following_error_limit_count, l.duty_min_percent
            ));
        }
        if f64::from(l.overspeed_count_s) < crate::limits::OVERSPEED_MARGIN * f64::from(l.max_vel_count_s) {
            errs.push(format!(
                "limits.overspeed_count_s ({}) must be >= {} x limits.max_vel_count_s ({})",
                l.overspeed_count_s,
                crate::limits::OVERSPEED_MARGIN,
                l.max_vel_count_s
            ));
        }
        // Stop-time rule (Issue #15 H2): a STOP decelerates at max_acc; if that
        // takes longer than the stop timeouts the rig faults and brakes at speed.
        // Checked on the initial set and on the firmware ceilings (a runtime
        // request may raise max_vel to the ceiling; the clamp then needs an
        // acceleration bound it can actually reach).
        {
            use crate::limits::{min_acc_for_stop, stop_deadline, stop_time_s, Ceilings, STOP_TIME_FRACTION};
            let deadline = stop_deadline(t);
            let deadline_s = deadline.as_secs_f64();
            let budget = format!(
                "{deadline_s:.3} s ({STOP_TIME_FRACTION} x min(timing.mcu_stop_timeout_ms {}, timing.stop_settle_timeout_ms {}))",
                t.mcu_stop_timeout_ms, t.stop_settle_timeout_ms
            );
            if l.max_acc_count_s2 > 0 {
                let stop_s = stop_time_s(l.max_vel_count_s, l.max_acc_count_s2);
                if stop_s > deadline_s {
                    errs.push(format!(
                        "limits.max_vel_count_s ({}) / limits.max_acc_count_s2 ({}) = {stop_s:.3} s worst-case stop time \
                         must be <= {budget}: a STOP would run into the timeout and brake at speed \
                         (max_acc_count_s2 must be >= {})",
                        l.max_vel_count_s,
                        l.max_acc_count_s2,
                        min_acc_for_stop(l.max_vel_count_s, deadline)
                    ));
                }
            }
            let c = Ceilings::fallback(l.overspeed_count_s);
            let stop_s = stop_time_s(c.max_vel_count_s, c.max_acc_count_s2);
            if stop_s > deadline_s {
                errs.push(format!(
                    "firmware ceilings: max_vel {} / max_acc {} = {stop_s:.3} s worst-case stop time must be <= {budget} \
                     (protocol::fw_absolute and limits.overspeed_count_s / {})",
                    c.max_vel_count_s,
                    c.max_acc_count_s2,
                    crate::limits::OVERSPEED_MARGIN
                ));
            }
        }

        // Backend.
        match &self.backend {
            BackendConfig::Sim(b) => {
                if !(b.plant_max_vel_count_s.is_finite() && b.plant_max_vel_count_s > 0.0) {
                    errs.push(format!("backend.plant_max_vel_count_s must be > 0, got {}", b.plant_max_vel_count_s));
                }
                if b.status_period_ms == 0 {
                    errs.push("backend.status_period_ms must be > 0".into());
                }
                if b.end_min_count >= b.end_max_count {
                    errs.push(format!(
                        "backend.end_min_count ({}) must be < backend.end_max_count ({})",
                        b.end_min_count, b.end_max_count
                    ));
                }
                for (name, v) in [("backend.status_period_ms", b.status_period_ms), ("backend.link_latency_ms", b.link_latency_ms)] {
                    if v > MAX_TIMING_MS {
                        errs.push(format!("{name} must be <= {MAX_TIMING_MS}, got {v}"));
                    }
                }
            }
            BackendConfig::Serial(b) => {
                if b.ports.is_empty() && b.sim_axes.len() < AXIS_COUNT {
                    errs.push("backend.ports must list at least one port (or sim_axes must cover every axis)".into());
                }
                if b.baud == 0 {
                    errs.push("backend.baud must be > 0".into());
                }
                let mut seen = std::collections::BTreeSet::new();
                for a in &b.sim_axes {
                    if !seen.insert(*a) {
                        errs.push(format!("backend.sim_axes lists axis {} twice", a.get()));
                    }
                }
                for (name, v) in [("backend.sim.status_period_ms", b.sim.status_period_ms), ("backend.sim.link_latency_ms", b.sim.link_latency_ms)] {
                    if v > MAX_TIMING_MS {
                        errs.push(format!("{name} must be <= {MAX_TIMING_MS}, got {v}"));
                    }
                }
            }
        }

        if errs.is_empty() {
            Ok(())
        } else {
            Err(ConfigError::Invalid(errs.join("; ")))
        }
    }

    /// The `[[axis]]` entry for `id`. Always `Some` after `validate()`; `None`
    /// on a deserialized-but-unvalidated config that lacks the axis.
    pub fn axis(&self, id: AxisId) -> Option<&AxisConfig> {
        self.axes.iter().find(|a| a.axis_id == id)
    }
}
