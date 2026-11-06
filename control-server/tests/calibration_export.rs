//! calibration_export: CAL_RESULT → config values, and the comment-preserving
//! TOML rewrite on the real config texts (DESIGN.md §11).

use h3cs::calibration_export::{derive, file_name, header, rewrite_toml, utc_timestamp, AxisExport, REWRITTEN_KEYS};
use h3cs::config::Config;
use h3cs::protocol::CalResultCode;
use h3cs::supervisor::CalibrationResult;
use h3cs::types::{AxisId, DeviceId};

const BRINGUP: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/config/bringup.toml");
const SIM: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/config/sim.toml");

fn ax(n: u8) -> AxisId {
    AxisId::new(n).unwrap()
}

fn ok(n_end: i32, neutral: i32, sign: i8) -> CalibrationResult {
    CalibrationResult {
        device_id: DeviceId(0x5131_0000_0000_0009),
        result: CalResultCode::Ok,
        step: 3,
        n_end_count: n_end,
        neutral_count: neutral,
        sign,
        reason: "ok".into(),
        boot_id: 0x89AB_CDEF,
        at_ms: 12.5,
    }
}

#[test]
fn derive_follows_protocol_1_2_for_both_signs() {
    // sign +1: P end 0, N end +5551 (axis 1 of bringup.toml).
    let e = derive(ax(1), DeviceId(0x5131_1cdb_d4ba_8d58), 300, &ok(5551, 2775, 1)).unwrap();
    assert_eq!(
        e,
        AxisExport {
            axis_id: ax(1),
            device_id: DeviceId(0x5131_1cdb_d4ba_8d58),
            n_end_count: 5551,
            neutral_count: 2775,
            sign: 1,
            decel_zone_count: 300,
            mechanical_min_count: 0,
            mechanical_max_count: 5551,
            soft_min_count: 300,
            soft_max_count: 5251,
            count_at_virtual_0: 300,
            count_at_virtual_512: 2775,
            count_at_virtual_1023: 5251,
        }
    );
    // sign −1: the N end is negative; virtual 0 is still the P-end side.
    let e = derive(ax(3), DeviceId(1), 300, &ok(-6138, -3069, -1)).unwrap();
    assert_eq!((e.mechanical_min_count, e.mechanical_max_count), (-6138, 0));
    assert_eq!((e.soft_min_count, e.soft_max_count), (-5838, -300));
    assert_eq!((e.count_at_virtual_0, e.count_at_virtual_512, e.count_at_virtual_1023), (-300, -3069, -5838));
    // Neutral is taken from the MCU, not recomputed.
    let e = derive(ax(2), DeviceId(2), 100, &ok(1000, 480, 1)).unwrap();
    assert_eq!(e.count_at_virtual_512, 480);
    // Leaving i32 is an error, not a wrap (only reachable with an unvalidated
    // negative decel zone; a validated config can never get here).
    assert!(derive(ax(2), DeviceId(2), -1, &ok(i32::MIN, 0, -1)).is_err());
    assert!(derive(ax(2), DeviceId(2), 0, &ok(i32::MIN, 0, -1)).is_ok());
}

#[test]
fn rewrite_bringup_toml_replaces_values_keeps_comments_and_validates() {
    let text = std::fs::read_to_string(BRINGUP).unwrap();
    let a1 = derive(ax(1), DeviceId(0x5131_1cdb_d4ba_8d58), 300, &ok(5553, 2776, 1)).unwrap();
    let a3 = derive(ax(3), DeviceId(0x5131_30ed_a00b_2040), 300, &ok(6138, 3069, 1)).unwrap();
    let rw = rewrite_toml(&text, &[a1.clone(), a3.clone()]).unwrap();
    assert_eq!(rw.replaced.len(), 2 * REWRITTEN_KEYS.len());
    // Only the targeted lines changed; every other line is byte-identical.
    let before: Vec<&str> = text.lines().collect();
    let after: Vec<&str> = rw.text.lines().collect();
    assert_eq!(before.len(), after.len());
    let changed: Vec<(&str, &str)> = before.iter().zip(&after).filter(|(b, a)| b != a).map(|(b, a)| (*b, *a)).collect();
    assert!(changed.len() <= 2 * REWRITTEN_KEYS.len(), "{changed:#?}");
    for (b, a) in &changed {
        // Same key, same trailing comment.
        let key = |l: &str| l.split('=').next().unwrap().trim().to_string();
        assert_eq!(key(b), key(a), "{b} -> {a}");
        let comment = |l: &str| l.find('#').map(|i| l[i..].to_string());
        assert_eq!(comment(b), comment(a), "comment kept: {b} -> {a}");
    }
    // The documented comment on the stroke line survives next to the new value.
    assert!(rw.text.contains("mechanical_max_count = 5553     # 従来 env の UPPER / CAL で当てた N 端"), "{}", rw.text);
    assert!(rw.text.contains("count_at_virtual_512 = 2776     # = stroke / 2"), "{}", rw.text);
    assert!(rw.text.contains("device_id = \"51311cdbd4ba8d58\"   # 2026-11-01 に接続して確認した個体"));
    // Axis 2's block is untouched.
    assert!(rw.text.contains("count_at_virtual_512 = 2731     # = stroke / 2"));
    // The header plus the text is a valid config with the new numbers.
    let ts = utc_timestamp(std::time::SystemTime::UNIX_EPOCH);
    let full = format!("{}{}", header(&ts, BRINGUP, &[a1, a3], &rw.replaced), rw.text);
    let cfg = Config::from_toml_str(&full).unwrap();
    let c1 = cfg.axis(ax(1)).unwrap();
    assert_eq!((c1.mechanical_max_count, c1.soft_max_count, c1.count_at_virtual_1023, c1.count_at_virtual_512), (5553, 5253, 5253, 2776));
    assert!(!c1.calibration_valid);
    let c3 = cfg.axis(ax(3)).unwrap();
    assert_eq!((c3.mechanical_max_count, c3.count_at_virtual_512), (6138, 3069));
    assert_eq!(cfg.axis(ax(2)).unwrap().mechanical_max_count, 5462);
    assert!(full.starts_with("# h3-control-server"));
    assert!(full.contains("19700101-000000（UTC）"));
}

#[test]
fn rewrite_sim_toml_and_commented_axis_markers_are_not_headers() {
    let text = std::fs::read_to_string(SIM).unwrap();
    let a2 = derive(ax(2), DeviceId(0x5131_0000_0000_0002), 300, &ok(1000, 500, 1)).unwrap();
    let rw = rewrite_toml(&text, &[a2]).unwrap();
    assert!(rw.text.contains("calibration_valid = false   # 模擬軸のみ"), "{}", rw.text);
    let cfg = Config::from_toml_str(&rw.text).unwrap();
    let c2 = cfg.axis(ax(2)).unwrap();
    assert_eq!((c2.mechanical_min_count, c2.mechanical_max_count, c2.soft_min_count, c2.soft_max_count), (0, 1000, 300, 700));
    assert_eq!((c2.count_at_virtual_0, c2.count_at_virtual_512, c2.count_at_virtual_1023), (300, 500, 700));
    assert!(cfg.axis(ax(1)).unwrap().calibration_valid, "axis 1 untouched");

    // A `[[axis]]` inside a comment does not open a block; a block without
    // axis_id is skipped; a missing key or a duplicate axis is an error.
    let odd = "\
# [[axis]] in a comment\n\
[[axis]]\n\
name = \"no id\"\n\
[[axis]]\n\
axis_id = 2   # the real one\n\
device_id = \"0\"\n\
calibration_valid = true\n\
count_at_virtual_0 = 1\n\
count_at_virtual_512 = 2\n\
count_at_virtual_1023 = 3\n\
mechanical_min_count = 0\n\
mechanical_max_count = 9\n\
soft_min_count = 1\n\
soft_max_count = 8\n\
decel_zone_count = 1\n";
    let a2 = derive(ax(2), DeviceId(0xAB), 1, &ok(10, 5, 1)).unwrap();
    let rw = rewrite_toml(odd, std::slice::from_ref(&a2)).unwrap();
    assert!(rw.text.contains("device_id = \"00000000000000ab\"\n"));
    assert!(rw.text.contains("soft_max_count = 9\n"));
    assert!(rw.text.starts_with("# [[axis]] in a comment\n[[axis]]\nname = \"no id\"\n"));
    let missing = odd.replace("soft_max_count = 8\n", "");
    assert!(rewrite_toml(&missing, std::slice::from_ref(&a2)).unwrap_err().contains("soft_max_count"));
    let twice = format!("{odd}{}", odd.trim_start_matches("# [[axis]] in a comment\n"));
    assert!(rewrite_toml(&twice, std::slice::from_ref(&a2)).unwrap_err().contains("more than once"));
    assert!(rewrite_toml(odd, &[AxisExport { axis_id: ax(1), ..a2 }]).unwrap_err().contains("axis_id = 1"));
}

#[test]
fn file_name_uses_rig_id_and_timestamp() {
    assert_eq!(file_name("h3-bringup-01", "20261101-053012"), "h3-bringup-01.calibration-20261101-053012.toml");
}

const LIVE: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/config/h3-bringup-01-live-20261102.toml");

#[test]
fn derive_keeping_preserves_the_live_350_margin_and_the_direction() {
    use h3cs::calibration_export::{derive_keeping, Margins};
    let live = Config::load(std::path::Path::new(LIVE)).unwrap();
    let a1 = live.axis(ax(1)).unwrap();
    assert_eq!(Margins::of(a1), Margins { p: 350, n: 350, p_to_n: true });
    let e = derive_keeping(a1, a1.device_id, &ok(5560, 2780, 1)).unwrap();
    assert_eq!((e.mechanical_min_count, e.mechanical_max_count, e.soft_min_count, e.soft_max_count), (0, 5560, 50, 5510));
    assert_eq!((e.count_at_virtual_0, e.count_at_virtual_512, e.count_at_virtual_1023), (400, 2780, 5160));
    // Plain derive would have put the end points on the soft limits.
    let plain = derive(ax(1), a1.device_id, 50, &ok(5560, 2780, 1)).unwrap();
    assert_eq!((plain.count_at_virtual_0, plain.count_at_virtual_1023), (50, 5510));

    // Operator-reversed direction is kept, with each side's margin.
    let mut rev = a1.clone();
    rev.count_at_virtual_0 = rev.soft_max_count - 400; // soft_max − 400（ライブ設定の値から。校正の反映で soft_max は変わる）
    rev.count_at_virtual_1023 = 400; // soft_min + 350
    assert_eq!(Margins::of(&rev), Margins { p: 350, n: 400, p_to_n: false });
    let e = derive_keeping(&rev, rev.device_id, &ok(5560, 2780, 1)).unwrap();
    assert_eq!((e.count_at_virtual_0, e.count_at_virtual_1023), (5110, 400));

    // sign −1: margins measured the same way from the P end (0).
    let e = derive_keeping(a1, a1.device_id, &ok(-6000, -3000, -1)).unwrap();
    assert_eq!((e.soft_min_count, e.soft_max_count), (-5950, -50));
    assert_eq!((e.count_at_virtual_0, e.count_at_virtual_512, e.count_at_virtual_1023), (-400, -3000, -5600));

    // Margins that do not fit the measured window are an error, not a crossing.
    assert!(derive_keeping(a1, a1.device_id, &ok(800, 400, 1)).is_err());
    // Crossed margins that would still be "monotonic" the other way round.
    let mut wide = a1.clone();
    wide.count_at_virtual_0 = wide.soft_min_count + 600;
    wide.count_at_virtual_1023 = wide.soft_max_count - 600;
    assert!(derive_keeping(&wide, wide.device_id, &ok(1000, 500, 1)).is_err(), "600 + 600 inside [50, 950] around 500");
    assert!(derive_keeping(&wide, wide.device_id, &ok(-1000, -500, -1)).is_err());
}

#[test]
fn rewrite_axis_values_replaces_only_the_given_keys() {
    use h3cs::calibration_export::rewrite_axis_values;
    let text = std::fs::read_to_string(LIVE).unwrap();
    let rw = rewrite_axis_values(&text, &[(ax(2), vec![("calibration_valid", "false".into())])]).unwrap();
    let changed: Vec<(&str, &str)> =
        text.lines().zip(rw.text.lines()).filter(|(a, b)| a != b).collect();
    assert_eq!(changed.len(), 1, "{changed:?}");
    assert_eq!(changed[0].1, "calibration_valid = false   # #1 / #2: 2026-11-01 の /v1/calibrate の結果。#3 は模擬軸");
    let cfg = Config::from_toml_str(&rw.text).unwrap();
    assert!(!cfg.axis(ax(2)).unwrap().calibration_valid);
    assert!(cfg.axis(ax(1)).unwrap().calibration_valid);
    assert!(rewrite_axis_values(&text, &[(ax(2), vec![("no_such_key", "1".into())])]).is_err());
}
