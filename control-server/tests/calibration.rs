//! calibration: piecewise-linear virtual→count map, inverse, soft limits,
//! invariants, hash.

use h3cs::calibration::{calibration_hash, from_config, AxisCalibration, CalibrationError};
use h3cs::config::Config;
use h3cs::types::{AxisId, VirtualPos};

const SIM_TOML: &str = include_str!("../config/sim.toml");

fn axis(id: u8) -> AxisId {
    AxisId::new(id).unwrap()
}

/// Asymmetric, increasing calibration (different slope per segment).
fn inc() -> AxisCalibration {
    AxisCalibration {
        axis_id: axis(1),
        count_at: [-1000, 200, 2500],
        mechanical_min_count: -1600,
        mechanical_max_count: 3100,
        soft_min_count: -1200,
        soft_max_count: 2700,
        decel_zone_count: 300,
        operator_marked_valid: true,
    }
}

/// Decreasing calibration (axis mounted the other way round).
fn dec() -> AxisCalibration {
    AxisCalibration {
        axis_id: axis(2),
        count_at: [2500, 200, -1000],
        ..inc()
    }
}

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() <= 1e-9 * (1.0 + a.abs().max(b.abs()))
}

#[test]
fn passes_through_the_three_points() {
    for cal in [inc(), dec()] {
        assert_eq!(
            cal.virtual_to_count_f(VirtualPos(0.0)),
            f64::from(cal.count_at[0])
        );
        assert_eq!(
            cal.virtual_to_count_f(VirtualPos(512.0)),
            f64::from(cal.count_at[1])
        );
        assert_eq!(
            cal.virtual_to_count_f(VirtualPos(1023.0)),
            f64::from(cal.count_at[2])
        );
        assert_eq!(cal.neutral_count(), 200);
    }
}

#[test]
fn piecewise_linear_per_segment() {
    let cal = inc();
    // Lower segment: slope 1200/512.
    assert!(close(cal.virtual_to_count_f(VirtualPos(256.0)), -400.0));
    // Upper segment: slope 2300/511.
    assert!(close(
        cal.virtual_to_count_f(VirtualPos(512.0 + 511.0 / 2.0)),
        1350.0
    ));
    let d = dec();
    assert!(close(d.virtual_to_count_f(VirtualPos(256.0)), 1350.0));
    assert!(close(
        d.virtual_to_count_f(VirtualPos(512.0 + 511.0 / 2.0)),
        -400.0
    ));
}

#[test]
fn extrapolates_linearly_outside_range() {
    let cal = inc();
    assert!(close(cal.virtual_to_count_f(VirtualPos(-512.0)), -2200.0));
    assert!(close(
        cal.virtual_to_count_f(VirtualPos(1023.0 + 511.0)),
        4800.0
    ));
    let d = dec();
    assert!(close(d.virtual_to_count_f(VirtualPos(-512.0)), 4800.0));
    assert!(close(d.virtual_to_count_f(VirtualPos(1534.0)), -2200.0));
}

#[test]
fn map_is_strictly_monotonic_in_axis_direction() {
    for (cal, sign) in [(inc(), 1.0), (dec(), -1.0)] {
        let mut prev = cal.virtual_to_count_f(VirtualPos(-100.0));
        let mut v = -99.5;
        while v <= 1123.0 {
            let c = cal.virtual_to_count_f(VirtualPos(v));
            assert!((c - prev) * sign > 0.0, "not monotonic at v={v}");
            prev = c;
            v += 0.5;
        }
    }
}

#[test]
fn inverse_round_trips_both_directions() {
    for cal in [inc(), dec()] {
        let mut v = -200.0;
        while v <= 1223.0 {
            let c = cal.virtual_to_count_f(VirtualPos(v));
            let back = cal.count_to_virtual(c).0;
            assert!(close(back, v), "v={v} c={c} back={back}");
            v += 0.25;
        }
        for c in [-1000.0, 200.0, 2500.0, -3000.0, 4000.0] {
            let v = cal.count_to_virtual(c);
            assert!(close(cal.virtual_to_count_f(v), c));
        }
        assert_eq!(cal.count_to_virtual(200.0), VirtualPos(512.0));
    }
}

#[test]
fn clamp_soft_limits() {
    let cal = dec();
    assert_eq!(cal.clamp_soft(0.0), (0.0, false));
    assert_eq!(cal.clamp_soft(-1200.0), (-1200.0, false));
    assert_eq!(cal.clamp_soft(2700.0), (2700.0, false));
    assert_eq!(cal.clamp_soft(-1500.0), (-1200.0, true));
    assert_eq!(cal.clamp_soft(9999.5), (2700.0, true));
    assert_eq!(cal.clamp_soft(f64::NAN), (200.0, true));
    assert_eq!(cal.clamp_soft(f64::INFINITY), (2700.0, true));
}

#[test]
fn valid_calibrations_pass() {
    assert_eq!(inc().validate(), Ok(()));
    assert_eq!(dec().validate(), Ok(()));
    // Exactly decel_zone_count margin is allowed.
    let edge = AxisCalibration {
        soft_min_count: -1300,
        mechanical_min_count: -1600,
        ..inc()
    };
    assert_eq!(edge.validate(), Ok(()));
    // operator_marked_valid is not a structural invariant.
    assert_eq!(
        AxisCalibration {
            operator_marked_valid: false,
            ..inc()
        }
        .validate(),
        Ok(())
    );
}

fn invalid_msg(cal: AxisCalibration) -> String {
    match cal.validate() {
        Err(CalibrationError::Invalid { msg, .. }) => msg,
        Ok(()) => panic!("expected invalid: {cal:?}"),
    }
}

#[test]
fn rejects_invalid_calibrations_naming_the_field() {
    let base = inc();
    let cases: Vec<(AxisCalibration, &str)> = vec![
        (
            AxisCalibration {
                count_at: [-1000, 200, 200],
                ..base.clone()
            },
            "monotonic",
        ),
        (
            AxisCalibration {
                count_at: [-1000, 300, 200],
                ..base.clone()
            },
            "monotonic",
        ),
        (
            AxisCalibration {
                count_at: [0, 0, 0],
                ..base.clone()
            },
            "monotonic",
        ),
        (
            AxisCalibration {
                count_at: [-1300, 200, 2500],
                ..base.clone()
            },
            "count_at_virtual_0",
        ),
        (
            AxisCalibration {
                count_at: [-1000, 200, 2701],
                ..base.clone()
            },
            "count_at_virtual_1023",
        ),
        (
            AxisCalibration {
                decel_zone_count: -1,
                ..base.clone()
            },
            "decel_zone_count",
        ),
        (
            AxisCalibration {
                soft_min_count: -1400,
                ..base.clone()
            },
            "soft_min_count",
        ),
        (
            AxisCalibration {
                soft_max_count: 2900,
                ..base.clone()
            },
            "soft_max_count",
        ),
        (
            AxisCalibration {
                soft_min_count: -1700,
                ..base.clone()
            },
            "soft_min_count",
        ),
        (
            AxisCalibration {
                soft_max_count: 3200,
                ..base.clone()
            },
            "soft_max_count",
        ),
        (
            AxisCalibration {
                mechanical_min_count: 5000,
                ..base.clone()
            },
            "mechanical_min_count",
        ),
        (
            AxisCalibration {
                soft_min_count: 3000,
                soft_max_count: 2000,
                ..base.clone()
            },
            "soft_min_count",
        ),
    ];
    for (cal, needle) in cases {
        let msg = invalid_msg(cal.clone());
        assert!(
            msg.contains(needle),
            "{needle:?} not in {msg:?} for {cal:?}"
        );
    }
}

#[test]
fn extreme_values_do_not_overflow() {
    let cal = AxisCalibration {
        count_at: [i32::MIN, 0, i32::MAX],
        mechanical_min_count: i32::MIN,
        mechanical_max_count: i32::MAX,
        soft_min_count: i32::MIN,
        soft_max_count: i32::MAX,
        decel_zone_count: i32::MAX,
        ..inc()
    };
    assert!(cal.validate().is_err());
    let _ = cal.virtual_to_count_f(VirtualPos(1e300));
    let _ = cal.count_to_virtual(f64::NAN);
    let _ = calibration_hash("", &cal);
}

#[test]
fn error_display_names_axis() {
    let e = AxisCalibration {
        decel_zone_count: -1,
        ..dec()
    }
    .validate()
    .unwrap_err();
    assert!(e.to_string().starts_with("axis2: "), "{e}");
}

// ---------------------------------------------------------------------- hash

#[test]
fn hash_golden_values_for_sim_config() {
    // Expected values computed independently with Python zlib.crc32 over the
    // documented layout (u32 len | rig_id | u8 axis | 3×i32 | 5×i32, LE; geometry only).
    let cfg = Config::from_toml_str(SIM_TOML).unwrap();
    let cals = from_config(&cfg);
    let got: Vec<u32> = cals
        .iter()
        .map(|c| calibration_hash(&cfg.rig_id, c))
        .collect();
    assert_eq!(got, vec![0x2c2a_144a, 0xf84d_5380, 0xcebf_c373]);
}

#[test]
fn hash_is_stable_and_changes_with_every_hashed_field() {
    let base = inc();
    let h0 = calibration_hash("rig", &base);
    assert_eq!(h0, calibration_hash("rig", &base.clone()));

    let variants = [
        AxisCalibration {
            axis_id: axis(3),
            ..base.clone()
        },
        AxisCalibration {
            count_at: [-999, 200, 2500],
            ..base.clone()
        },
        AxisCalibration {
            count_at: [-1000, 201, 2500],
            ..base.clone()
        },
        AxisCalibration {
            count_at: [-1000, 200, 2501],
            ..base.clone()
        },
        AxisCalibration {
            mechanical_min_count: -1601,
            ..base.clone()
        },
        AxisCalibration {
            mechanical_max_count: 3101,
            ..base.clone()
        },
        AxisCalibration {
            soft_min_count: -1201,
            ..base.clone()
        },
        AxisCalibration {
            soft_max_count: 2701,
            ..base.clone()
        },
        AxisCalibration {
            decel_zone_count: 301,
            ..base.clone()
        },
    ];
    for v in &variants {
        assert_ne!(calibration_hash("rig", v), h0, "{v:?}");
    }
    assert_ne!(calibration_hash("rig2", &base), h0);
    // Length prefix: moving a byte between rig_id and the payload changes the hash.
    assert_ne!(
        calibration_hash("", &base),
        calibration_hash("\u{1}", &base)
    );
    // Operator flag is policy, not an MCU parameter: excluded on purpose.
    assert_eq!(
        calibration_hash(
            "rig",
            &AxisCalibration {
                operator_marked_valid: false,
                ..base
            }
        ),
        h0
    );
}

#[test]
fn hash_independent_of_axis_order_in_config() {
    // Reverse the [[axis]] blocks in the TOML text.
    let (head, rest) = SIM_TOML.split_at(SIM_TOML.find("[[axis]]").unwrap());
    let mut blocks: Vec<&str> = rest
        .split("[[axis]]")
        .filter(|b| !b.trim().is_empty())
        .collect();
    assert_eq!(blocks.len(), 3);
    blocks.reverse();
    let reordered = format!(
        "{head}{}",
        blocks
            .iter()
            .map(|b| format!("[[axis]]{b}\n"))
            .collect::<String>()
    );

    let a = Config::from_toml_str(SIM_TOML).unwrap();
    let b = Config::from_toml_str(&reordered).unwrap();
    assert_eq!(b.axes[0].axis_id, axis(3));
    let ca = from_config(&a);
    let cb = from_config(&b);
    assert_eq!(ca, cb);
    for (x, y) in ca.iter().zip(cb.iter()) {
        assert_eq!(
            calibration_hash(&a.rig_id, x),
            calibration_hash(&b.rig_id, y)
        );
    }
}

// --------------------------------------------------------------- from_config

#[test]
fn from_config_indexes_by_axis_id_and_copies_fields() {
    let mut cfg = Config::from_toml_str(SIM_TOML).unwrap();
    cfg.axes.reverse();
    let cals = from_config(&cfg);
    for (i, cal) in cals.iter().enumerate() {
        assert_eq!(cal.axis_id.index(), i);
        let a = cfg
            .axis(cal.axis_id)
            .expect("validated config has every axis");
        assert_eq!(cal, &AxisCalibration::from_config(a));
        assert_eq!(
            cal.count_at,
            [
                a.count_at_virtual_0,
                a.count_at_virtual_512,
                a.count_at_virtual_1023
            ]
        );
        assert_eq!(cal.soft_min_count, a.soft_min_count);
        assert_eq!(cal.decel_zone_count, a.decel_zone_count);
        assert_eq!(cal.operator_marked_valid, a.calibration_valid);
    }
}

#[test]
fn from_config_missing_axis_yields_invalid_placeholder() {
    let mut cfg = Config::from_toml_str(SIM_TOML).unwrap();
    cfg.axes.retain(|a| a.axis_id != axis(2));
    let cals = from_config(&cfg);
    assert_eq!(cals[1].axis_id, axis(2));
    assert!(!cals[1].operator_marked_valid);
    assert!(cals[1].validate().is_err());
    assert!(cals[0].validate().is_ok());
}
