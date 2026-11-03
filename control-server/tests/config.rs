//! config: loading and structural validation.

use h3cs::config::{BackendConfig, Config, ConfigError, LimitsConfig};
use h3cs::types::{AxisId, DeviceId};
use std::path::Path;

const EXAMPLE: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/config/example.toml");
const SIM: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/config/sim.toml");

fn sim() -> Config {
    Config::load(Path::new(SIM)).unwrap()
}

fn invalid(cfg: &Config) -> String {
    match cfg.validate() {
        Err(ConfigError::Invalid(m)) => m,
        Err(e) => panic!("unexpected error kind: {e}"),
        Ok(()) => panic!("expected validation failure"),
    }
}

fn assert_invalid(cfg: &Config, needle: &str) {
    let m = invalid(cfg);
    assert!(m.contains(needle), "{needle:?} not in {m:?}");
}

#[test]
fn shipped_configs_load_and_validate() {
    for p in [EXAMPLE, SIM] {
        let cfg = Config::load(Path::new(p)).unwrap_or_else(|e| panic!("{p}: {e}"));
        cfg.validate().unwrap();
        for id in AxisId::ALL {
            assert_eq!(cfg.axis(id).map(|a| a.axis_id), Some(id));
        }
    }
    assert!(matches!(sim().backend, BackendConfig::Sim(_)));
    let ex = Config::load(Path::new(EXAMPLE)).unwrap();
    assert!(matches!(ex.backend, BackendConfig::Serial(_)));
}

#[test]
fn placeholder_device_ids_are_allowed_but_unassigned() {
    let ex = Config::load(Path::new(EXAMPLE)).unwrap();
    assert!(ex.axes.iter().all(|a| a.device_id.is_unassigned()));
    assert!(sim().axes.iter().all(|a| !a.device_id.is_unassigned()));
    assert!(DeviceId(0).is_unassigned());
    assert!(!DeviceId(1).is_unassigned());

    // Mixed: one placeholder, two distinct real ids → valid.
    let mut cfg = sim();
    cfg.axes[1].device_id = DeviceId(0);
    cfg.validate().unwrap();
}

#[test]
fn duplicate_nonzero_device_id_rejected() {
    let mut cfg = sim();
    cfg.axes[2].device_id = cfg.axes[0].device_id;
    assert_invalid(&cfg, "duplicate device_id 5131000000000001");
    assert_invalid(&cfg, "axis_id = 3");
}

#[test]
fn missing_and_duplicate_axis_rejected() {
    let mut cfg = sim();
    cfg.axes.retain(|a| a.axis_id.get() != 2);
    assert_invalid(&cfg, "axis_id = 2 is missing");

    let mut cfg = sim();
    cfg.axes[2].axis_id = AxisId::new(1).unwrap();
    cfg.axes[2].device_id = DeviceId(0x99);
    let m = invalid(&cfg);
    assert!(m.contains("axis_id = 1 appears 2 times"), "{m}");
    assert!(m.contains("axis_id = 3 is missing"), "{m}");

    let mut cfg = sim();
    let extra = cfg.axes[0].clone();
    cfg.axes.push(extra);
    assert_invalid(&cfg, "exactly 3");
}

#[test]
fn axis_4_rejected_at_parse() {
    let text = std::fs::read_to_string(SIM).unwrap();
    let bad = text.replacen("axis_id = 3", "axis_id = 4", 1);
    assert_ne!(bad, text);
    let e = Config::from_toml_str(&bad).unwrap_err();
    assert!(matches!(e, ConfigError::Parse(_)), "{e}");
    assert!(e.to_string().contains("axis_id must be 1..=3"), "{e}");

    let bad = text.replacen("power_controller_axis = 1", "power_controller_axis = 0", 1);
    assert!(Config::from_toml_str(&bad).is_err());
}

#[test]
fn non_loopback_bind_rejected_unless_allowed() {
    let mut cfg = sim();
    cfg.server.bind = "0.0.0.0:8892".into();
    assert_invalid(&cfg, "server.bind");
    cfg.server.bind = "192.168.1.10:8892".into();
    assert_invalid(&cfg, "not a loopback");
    cfg.server.allow_non_loopback = true;
    assert_invalid(&cfg, "require_admin_token");
    cfg.server.require_admin_token = true;
    cfg.validate().unwrap();

    let mut cfg = sim();
    cfg.server.bind = "[::1]:8892".into();
    cfg.validate().unwrap();
    cfg.server.bind = "localhost:8892".into();
    assert_invalid(&cfg, "server.bind must be an IP:port");
    cfg.server.bind = "garbage".into();
    assert_invalid(&cfg, "server.bind");
}

#[test]
fn max_gain_bounds() {
    for bad in [1.5, 0.0, -0.01, f64::NAN, f64::INFINITY] {
        let mut cfg = sim();
        cfg.limits.max_gain = bad;
        assert_invalid(&cfg, "limits.max_gain");
    }
    for ok in [0.1, 1.0, 0.6] {
        let mut cfg = sim();
        cfg.limits.max_gain = ok;
        cfg.validate().unwrap();
    }
}

#[test]
fn limits_rules() {
    type Case = (fn(&mut Config), &'static str);
    let cases: &[Case] = &[
        (|c| c.limits.max_vel_count_s = 0, "limits.max_vel_count_s"),
        (|c| c.limits.max_acc_count_s2 = 0, "limits.max_acc_count_s2"),
        (|c| c.limits.following_error_limit_count = 0, "limits.following_error_limit_count"),
        (|c| c.limits.overspeed_count_s = 0, "limits.overspeed_count_s"),
        (|c| c.limits.duty_max_percent = 0, "limits.duty_max_percent"),
        (|c| c.limits.duty_max_percent = 101, "limits.duty_max_percent"),
        (|c| c.limits.duty_min_percent = 50, "limits.duty_min_percent"),
        (|c| c.limits.ff_count_s_per_percent = 0, "limits.ff_count_s_per_percent"),
        (|c| c.limits.deadband_out_count = 5, "limits.deadband_out_count"),
        // overspeed must be >= 1.5 x max_vel (3000 -> 4500)
        (|c| c.limits.overspeed_count_s = 4499, "limits.overspeed_count_s"),
        // kp = 0 cannot hold position at rest.
        (|c| c.limits.kp_percent_per_100 = 0, "limits.kp_percent_per_100"),
        // kp x following_error / 100 must reach duty_min, or the loop never
        // exceeds the floor before FOLLOWING_ERROR (2026-11-01: 2 x 400 / 100 = 8 %
        // against a 9 % breakaway). sim: 2 x 800 / 100 = 16 >= 3; make it 2 x 100 = 2 < 3.
        (|c| c.limits.following_error_limit_count = 100, "limits.kp_percent_per_100"),
        (
            |c| {
                c.limits.kp_percent_per_100 = 1;
                c.limits.duty_min_percent = 9;
            },
            "limits.kp_percent_per_100",
        ),
        // CENTER's measured tolerance must cover the firmware deadband.
        (|c| c.game.center_tolerance_count = 0, "game.center_tolerance_count"),
        (|c| c.game.center_tolerance_count = 19, "game.center_tolerance_count"),
        // So must the measured-rest tolerance of STOP (Issue #15 H1), and the
        // rest dwell must fit inside the settle window.
        (|c| c.timing.rest_error_count = 19, "timing.rest_error_count (19) must be >= limits.deadband_out_count (20)"),
        (|c| c.timing.rest_dwell_ms = 3000, "timing.rest_dwell_ms (3000) must be < timing.stop_settle_timeout_ms (3000)"),
        // START's pose interlock needs a positive window.
        (|c| c.game.start_max_offset_count = 0, "game.start_max_offset_count"),
        // Stop-time rule (Issue #15 H2): 600 count/s at 1 count/s² takes 600 s to stop.
        (
            |c| {
                c.limits.max_vel_count_s = 600;
                c.limits.max_acc_count_s2 = 1;
            },
            "worst-case stop time",
        ),
    ];
    for (mutate, needle) in cases {
        let mut cfg = sim();
        mutate(&mut cfg);
        assert_invalid(&cfg, needle);
    }
    let mut cfg = sim();
    cfg.limits.overspeed_count_s = 4500;
    cfg.validate().unwrap();
    // Exactly at the authority boundary is accepted (kp 1 x 300 / 100 = 3 % = duty_min 3).
    let mut cfg = sim();
    cfg.limits.kp_percent_per_100 = 1;
    cfg.limits.following_error_limit_count = 300;
    cfg.validate().unwrap();
    // The bring-up numbers verified on hardware 2026-11-02 pass: kp 6 x 400 / 100 = 24 >= 9.
    let mut cfg = sim();
    cfg.limits.kp_percent_per_100 = 6;
    cfg.limits.following_error_limit_count = 400;
    cfg.limits.duty_min_percent = 9;
    cfg.limits.duty_max_percent = 20;
    cfg.validate().unwrap();
    assert_eq!(cfg.game.center_tolerance_count, 50);
    assert_eq!(cfg.timing.center_reach_timeout_ms, 5000);
    assert_eq!(h3cs::config::GameConfig::default().start_max_offset_count, 300);
    assert_eq!(
        (cfg.timing.rest_velocity_count_s, cfg.timing.rest_error_count, cfg.timing.rest_dwell_ms),
        (50, 50, 200)
    );
    // Exactly at the deadband is accepted.
    let mut cfg = sim();
    cfg.timing.rest_error_count = 20;
    cfg.validate().unwrap();
    // Defaults are the documented initial values.
    let d = LimitsConfig::default();
    assert_eq!((d.max_gain, d.max_vel_count_s, d.max_acc_count_s2, d.duty_max_percent), (0.5, 3000, 15000, 40));
    assert_eq!((d.duty_min_percent, d.kp_percent_per_100, d.ff_count_s_per_percent), (3, 2, 100));
    assert_eq!((d.deadband_count, d.deadband_out_count), (10, 20));
    assert_eq!((d.following_error_limit_count, d.overspeed_count_s), (800, 9000));
}

#[test]
fn governor_section_defaults_and_rejects_bad_thresholds() {
    let cfg = sim();
    assert_eq!(cfg.governor, h3cs::governor::GovernorConfig::default());
    let mut bad = cfg.clone();
    bad.governor.fe_hard = cfg.limits.following_error_limit_count;
    assert_invalid(&bad, "governor.fe_hard");
    bad = cfg.clone();
    bad.governor.fe_release = bad.governor.fe_start;
    assert_invalid(&bad, "governor.fe_release");
    bad = cfg.clone();
    bad.governor.s_min = 0.0;
    assert_invalid(&bad, "governor.s_min");
    bad.governor.s_min = 1.0;
    bad.governor.down_rate = 0.0;
    assert_invalid(&bad, "governor.down_rate");
    let text = std::fs::read_to_string(SIM).unwrap();
    let with = format!("{text}\n[governor]\nfe_start = 100\nfe_hard = 180\nfe_release = 40\n");
    let loaded = Config::from_toml_str(&with).unwrap();
    assert_eq!((loaded.governor.fe_release, loaded.governor.fe_start, loaded.governor.fe_hard), (40, 100, 180));
    assert_eq!(loaded.governor.s_min, h3cs::governor::GovernorConfig::default().s_min);
}

#[test]
fn live_bringup_accepts_default_governor() {
    // 節が無い実機設定。既定の fe_hard が following_error_limit_count 未満であること。
    let live = concat!(env!("CARGO_MANIFEST_DIR"), "/config/h3-bringup-01-live-20261102.toml");
    let cfg = Config::load(Path::new(live)).unwrap_or_else(|e| panic!("{live}: {e}"));
    cfg.validate().unwrap();
    assert_eq!(cfg.governor, h3cs::governor::GovernorConfig::default());
    assert!(cfg.limits.following_error_limit_count > cfg.governor.fe_hard);
}

/// Issue #15 H2: `max_vel / max_acc` must fit in 0.8 x the shorter stop
/// timeout, for the initial set and for the firmware ceilings.
#[test]
fn stop_time_rule_names_the_numbers() {
    let mut cfg = sim(); // mcu_stop 1500, stop_settle 3000 → 1.2 s
    cfg.limits.max_vel_count_s = 600;
    cfg.limits.max_acc_count_s2 = 1;
    let m = invalid(&cfg);
    assert!(m.contains("limits.max_vel_count_s (600) / limits.max_acc_count_s2 (1) = 600.000 s"), "{m}");
    assert!(m.contains("<= 1.200 s (0.8 x min(timing.mcu_stop_timeout_ms 1500, timing.stop_settle_timeout_ms 3000))"), "{m}");
    assert!(m.contains("max_acc_count_s2 must be >= 500"), "{m}");
    // Exactly at the bound passes; one below fails.
    cfg.limits.max_acc_count_s2 = 500;
    cfg.validate().unwrap();
    cfg.limits.max_acc_count_s2 = 499;
    assert_invalid(&cfg, "worst-case stop time");
    // The shorter of the two timeouts rules: stop_settle 1000 ms → 0.8 s → needs 750.
    cfg.limits.max_acc_count_s2 = 500;
    cfg.timing.stop_settle_timeout_ms = 1000;
    assert_invalid(&cfg, "max_acc_count_s2 must be >= 750");
    // The firmware ceilings (6000 / 40000 = 0.15 s) must pass too, or a runtime
    // request at the velocity ceiling could not be stopped by any allowed acc.
    let mut cfg = sim();
    cfg.timing.stop_settle_timeout_ms = 100; // 0.08 s
    let m = invalid(&cfg);
    assert!(m.contains("firmware ceilings: max_vel 6000 / max_acc 40000 = 0.150 s"), "{m}");
    // The shipped sets pass: sim 3000 / 15000 = 0.2 s, bring-up 600 / 2000 = 0.3 s.
    sim().validate().unwrap();
    let bringup = concat!(env!("CARGO_MANIFEST_DIR"), "/config/bringup.toml");
    Config::load(std::path::Path::new(bringup)).unwrap();
}

#[test]
fn schema_1_files_with_per_axis_limits_are_rejected() {
    // An old (schema 1) file: `[game].max_gain` and per-axis limits are unknown fields now.
    let text = std::fs::read_to_string(SIM).unwrap();
    let old = text.replacen("schema_version = 2", "schema_version = 1", 1);
    assert!(matches!(Config::from_toml_str(&old), Err(ConfigError::Invalid(_))));
    let with_axis_limit = text.replacen("decel_zone_count = 300", "decel_zone_count = 300\nmax_vel_count_s = 600", 1);
    assert!(matches!(Config::from_toml_str(&with_axis_limit), Err(ConfigError::Parse(_))));
    let with_game_gain = text.replacen("clamp_out_of_range = true", "clamp_out_of_range = true\nmax_gain = 0.35", 1);
    assert!(matches!(Config::from_toml_str(&with_game_gain), Err(ConfigError::Parse(_))));
}

#[test]
fn virtual_limits_bounds() {
    let cases: [(f64, f64, &str); 5] = [
        (-1.0, 1023.0, "game.virtual_min"),
        (0.0, 1024.0, "game.virtual_max"),
        (600.0, 600.0, "must be < game.virtual_max"),
        (700.0, 600.0, "must be < game.virtual_max"),
        (f64::NAN, 1023.0, "game.virtual_min"),
    ];
    for (lo, hi, needle) in cases {
        let mut cfg = sim();
        cfg.game.virtual_min = lo;
        cfg.game.virtual_max = hi;
        assert_invalid(&cfg, needle);
    }
}

#[test]
fn center_speed_fraction_bounds() {
    for bad in [0.0, -0.1, 1.5, f64::NAN] {
        let mut cfg = sim();
        cfg.game.center_speed_fraction = bad;
        assert_invalid(&cfg, "game.center_speed_fraction");
    }
}

#[test]
fn zero_timing_rejected() {
    type Set = fn(&mut Config);
    let cases: [(Set, &str); 15] = [
        (|c| c.timing.rest_velocity_count_s = 0, "timing.rest_velocity_count_s"),
        (|c| c.timing.rest_error_count = 0, "timing.rest_error_count"),
        (|c| c.timing.rest_dwell_ms = 0, "timing.rest_dwell_ms"),
        (|c| c.timing.planner_period_ms = 0, "timing.planner_period_ms"),
        (|c| c.timing.set_position_valid_for_ms = 0, "timing.set_position_valid_for_ms"),
        (|c| c.timing.mcu_command_timeout_ms = 0, "timing.mcu_command_timeout_ms"),
        (|c| c.timing.mcu_stop_timeout_ms = 0, "timing.mcu_stop_timeout_ms"),
        (|c| c.timing.axis_status_timeout_ms = 0, "timing.axis_status_timeout_ms"),
        (|c| c.timing.source_freshness_ms = 0, "timing.source_freshness_ms"),
        (|c| c.timing.arm_timeout_ms = 0, "timing.arm_timeout_ms"),
        (|c| c.timing.stop_settle_timeout_ms = 0, "timing.stop_settle_timeout_ms"),
        (|c| c.timing.stop_hold_ms = 0, "timing.stop_hold_ms"),
        (|c| c.timing.max_frame_skew = 0, "timing.max_frame_skew"),
        (|c| c.timing.hello_retry_ms = 0, "timing.hello_retry_ms"),
        (|c| c.timing.hello_timeout_ms = 0, "timing.hello_timeout_ms"),
    ];
    for (set, needle) in cases {
        let mut cfg = sim();
        set(&mut cfg);
        assert_invalid(&cfg, &format!("{needle} must be > 0"));
    }
}

#[test]
fn command_lifetime_must_exceed_planner_period() {
    let mut cfg = sim();
    cfg.timing.set_position_valid_for_ms = 10;
    assert_invalid(&cfg, "timing.set_position_valid_for_ms (10) must be > timing.planner_period_ms (10)");
    let mut cfg = sim();
    cfg.timing.mcu_command_timeout_ms = 5;
    assert_invalid(&cfg, "timing.mcu_command_timeout_ms");
}

#[test]
fn server_and_backend_zero_values_rejected() {
    let mut cfg = sim();
    cfg.server.max_body_bytes = 0;
    assert_invalid(&cfg, "server.max_body_bytes");
    let mut cfg = sim();
    cfg.server.status_period_ms = 0;
    assert_invalid(&cfg, "server.status_period_ms");
    let mut cfg = sim();
    if let BackendConfig::Sim(b) = &mut cfg.backend {
        b.plant_max_vel_count_s = 0.0;
    }
    assert_invalid(&cfg, "backend.plant_max_vel_count_s");

    let mut ex = Config::load(Path::new(EXAMPLE)).unwrap();
    if let BackendConfig::Serial(b) = &mut ex.backend {
        b.ports.clear();
    }
    assert_invalid(&ex, "backend.ports");
}

#[test]
fn schema_version_and_rig_id() {
    let mut cfg = sim();
    cfg.schema_version = 1;
    assert_invalid(&cfg, "schema_version must be 2");
    let mut cfg = sim();
    cfg.rig_id = " ".into();
    assert_invalid(&cfg, "rig_id");
}

#[test]
fn soft_limit_outside_mechanical_rejected() {
    let mut cfg = sim();
    cfg.axes[1].soft_max_count = 1200; // mechanical_max_count = 1000
    let m = invalid(&cfg);
    assert!(m.contains("axis_id = 2") && m.contains("soft_max_count"), "{m}");

    // Inside, but not by decel_zone_count (300).
    let mut cfg = sim();
    cfg.axes[0].soft_min_count = -2500; // mechanical_min_count = -2742
    let m = invalid(&cfg);
    assert!(m.contains("axis_id = 1") && m.contains("soft_min_count"), "{m}");
}

#[test]
fn non_monotonic_points_rejected() {
    let mut cfg = sim();
    cfg.axes[2].count_at_virtual_512 = 500;
    let m = invalid(&cfg);
    assert!(m.contains("axis_id = 3") && m.contains("monotonic"), "{m}");

    // Decreasing is fine.
    let mut cfg = sim();
    cfg.axes[2].count_at_virtual_0 = 400;
    cfg.axes[2].count_at_virtual_1023 = -400;
    cfg.validate().unwrap();
}

#[test]
fn multiple_errors_are_all_reported() {
    let mut cfg = sim();
    cfg.limits.max_gain = 2.0;
    cfg.timing.stop_hold_ms = 0;
    let m = invalid(&cfg);
    assert!(m.contains("limits.max_gain") && m.contains("timing.stop_hold_ms"), "{m}");
}

#[test]
fn from_toml_str_runs_validation() {
    let text = std::fs::read_to_string(SIM).unwrap();
    let bad = text.replacen("max_gain = 0.5", "max_gain = 1.5", 1);
    assert!(matches!(Config::from_toml_str(&bad), Err(ConfigError::Invalid(_))));
    let bad = text.replacen("bind = \"127.0.0.1:8892\"", "bind = \"0.0.0.0:8892\"", 1);
    assert!(matches!(Config::from_toml_str(&bad), Err(ConfigError::Invalid(_))));
    assert!(matches!(Config::from_toml_str("not toml ["), Err(ConfigError::Parse(_))));
}

#[test]
fn axis_lookup_on_unvalidated_config_does_not_panic() {
    // Deserialize without validate(), then drop axis 2: lookup must return None.
    let text = std::fs::read_to_string(SIM).unwrap();
    let mut cfg: Config = toml::from_str(&text).unwrap();
    cfg.axes.retain(|a| a.axis_id.get() != 2);
    let two = AxisId::new(2).unwrap();
    assert!(cfg.axis(two).is_none());
    assert!(cfg.axis(AxisId::new(1).unwrap()).is_some());
    assert!(cfg.validate().is_err());
}

#[test]
fn huge_timing_values_rejected() {
    use h3cs::config::MAX_TIMING_MS;
    type Set = fn(&mut Config, u64);
    let cases: [(Set, &str); 10] = [
        (|c, v| c.timing.rest_dwell_ms = v, "timing.rest_dwell_ms"),
        (|c, v| c.timing.planner_period_ms = v, "timing.planner_period_ms"),
        (|c, v| c.timing.axis_status_timeout_ms = v, "timing.axis_status_timeout_ms"),
        (|c, v| c.timing.source_freshness_ms = v, "timing.source_freshness_ms"),
        (|c, v| c.timing.arm_timeout_ms = v, "timing.arm_timeout_ms"),
        (|c, v| c.timing.stop_settle_timeout_ms = v, "timing.stop_settle_timeout_ms"),
        (|c, v| c.timing.stop_hold_ms = v, "timing.stop_hold_ms"),
        (|c, v| c.timing.hello_retry_ms = v, "timing.hello_retry_ms"),
        (|c, v| c.timing.hello_timeout_ms = v, "timing.hello_timeout_ms"),
        (|c, v| c.server.status_period_ms = v, "server.status_period_ms"),
    ];
    for (set, needle) in cases {
        let mut cfg = sim();
        set(&mut cfg, MAX_TIMING_MS);
        assert!(!cfg.validate().is_err_and(|e| e.to_string().contains(&format!("{needle} must be <="))), "{needle} at the max");
        for v in [MAX_TIMING_MS + 1, u64::MAX] {
            let mut cfg = sim();
            set(&mut cfg, v);
            assert_invalid(&cfg, &format!("{needle} must be <= {MAX_TIMING_MS}"));
        }
    }
    assert_eq!(h3cs::config::bounded_ms(u64::MAX), std::time::Duration::from_millis(MAX_TIMING_MS));
}

#[test]
fn sim_backend_end_stops_default_and_are_ordered() {
    let BackendConfig::Sim(b) = sim().backend else { panic!("sim backend") };
    assert_eq!((b.end_min_count, b.end_max_count), (-3000, 3000));
    let text = std::fs::read_to_string(SIM).unwrap();
    assert!(text.contains("end_min_count = -3000") && text.contains("end_max_count = 3000"), "sim.toml documents the keys");
    let omitted = text.replace("end_min_count = -3000", "").replace("end_max_count = 3000", "");
    let BackendConfig::Sim(b) = Config::from_toml_str(&omitted).unwrap().backend else { panic!("sim backend") };
    assert_eq!((b.end_min_count, b.end_max_count), (-3000, 3000));
    let narrow = text.replace("end_min_count = -3000", "end_min_count = -500").replace("end_max_count = 3000", "end_max_count = 500");
    let BackendConfig::Sim(b) = Config::from_toml_str(&narrow).unwrap().backend else { panic!("sim backend") };
    assert_eq!((b.end_min_count, b.end_max_count), (-500, 500));
    let inverted = text.replace("end_min_count = -3000", "end_min_count = 3000");
    let err = Config::from_toml_str(&inverted).unwrap_err().to_string();
    assert!(err.contains("backend.end_min_count (3000) must be < backend.end_max_count (3000)"), "{err}");
}

#[test]
fn sim_backend_report_selected_axis_defaults_to_true() {
    let BackendConfig::Sim(b) = sim().backend else { panic!("sim backend") };
    assert!(b.report_selected_axis);
    let text = std::fs::read_to_string(SIM).unwrap();
    assert!(text.contains("report_selected_axis = true"), "sim.toml documents the key");
    let off = text.replace("report_selected_axis = true", "report_selected_axis = false");
    let BackendConfig::Sim(b) = Config::from_toml_str(&off).unwrap().backend else { panic!("sim backend") };
    assert!(!b.report_selected_axis);
    // Omitted → default true.
    let omitted = text.replace("report_selected_axis = true", "");
    let BackendConfig::Sim(b) = Config::from_toml_str(&omitted).unwrap().backend else { panic!("sim backend") };
    assert!(b.report_selected_axis);
}

#[test]
fn all_zero_device_ids_mean_no_cross_check_and_validate() {
    // Three axes without a device_id: valid (assignment comes from selected_axis).
    let mut cfg = sim();
    for a in &mut cfg.axes {
        a.device_id = DeviceId(0);
    }
    cfg.validate().unwrap();
}

#[test]
fn non_loopback_bind_requires_admin_token() {
    let base = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/config/sim.toml")).unwrap();
    let lan = base.replacen("bind = \"127.0.0.1:8892\"", "bind = \"0.0.0.0:8892\"\nallow_non_loopback = true", 1);
    assert!(h3cs::config::Config::from_toml_str(&lan).is_err(), "LAN bind without require_admin_token must be rejected");
    let ok = lan.replacen("allow_non_loopback = true", "allow_non_loopback = true\nrequire_admin_token = true", 1);
    assert!(h3cs::config::Config::from_toml_str(&ok).is_ok());
}

#[test]
fn serial_sim_axes_validation() {
    let base = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/config/bringup.toml")).unwrap();
    let with = |extra: &str| base.replacen("baud = 115200", &format!("baud = 115200\n{extra}"), 1);
    assert!(h3cs::config::Config::from_toml_str(&with("sim_axes = [3]")).is_ok());
    let dup = h3cs::config::Config::from_toml_str(&with("sim_axes = [3, 3]"));
    assert!(dup.is_err(), "duplicate sim axis must be rejected");
    let bad = h3cs::config::Config::from_toml_str(&with("sim_axes = [4]"));
    assert!(bad.is_err(), "axis 4 does not exist");
    // No ports at all is fine only when every axis is simulated.
    let no_ports = base.replacen("ports = [", "ports = [] # [", 1);
    assert!(h3cs::config::Config::from_toml_str(&no_ports).is_err());
    assert!(h3cs::config::Config::from_toml_str(&no_ports.replacen("baud = 115200", "baud = 115200\nsim_axes = [1, 2, 3]", 1)).is_ok());
}
