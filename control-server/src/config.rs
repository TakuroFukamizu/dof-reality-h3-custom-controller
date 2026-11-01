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

pub const SCHEMA_VERSION: u32 = 1;

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
    /// Admin token is read from this environment variable at start. If the
    /// variable is unset a random token is generated and printed once.
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
            hello_retry_ms: 200,
            hello_timeout_ms: 2000,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct GameConfig {
    /// Max effective gain allowed by this machine profile. The old UI max 0.6 is
    /// NOT a validated limit for the new rig.
    pub max_gain: f64,
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
}

impl Default for GameConfig {
    fn default() -> Self {
        Self {
            max_gain: 0.35,
            allow_missing_frame_seq: false,
            clamp_out_of_range: true,
            virtual_min: 0.0,
            virtual_max: 1023.0,
            center_speed_fraction: 0.25,
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
    pub max_vel_count_s: u32,
    pub max_acc_count_s2: u32,
    pub following_error_limit_count: u32,
    pub overspeed_count_s: u32,
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
    /// allow_non_loopback; max_gain outside 0..=1; virtual_min/max outside
    /// 0..=1023 or min >= max; timing values of 0; per-axis calibration
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

        // Game.
        let g = &self.game;
        if !(g.max_gain.is_finite() && (0.0..=1.0).contains(&g.max_gain)) {
            errs.push(format!("game.max_gain must be within 0..=1, got {}", g.max_gain));
        }
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
        if !(g.center_speed_fraction > 0.0 && g.center_speed_fraction <= 1.0) {
            errs.push(format!("game.center_speed_fraction must be within (0, 1], got {}", g.center_speed_fraction));
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
                if b.ports.is_empty() {
                    errs.push("backend.ports must list at least one port".into());
                }
                if b.baud == 0 {
                    errs.push("backend.baud must be > 0".into());
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
