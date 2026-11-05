//! `POST /v1/calibration/export` (DESIGN.md §11): turn CAL_RESULTs into a new
//! configuration file next to the running one. Pure: no I/O here, the API
//! handler reads / writes the files.
//!
//! The counts in a CAL_RESULT are in the MCU's `actual_count` coordinate with
//! the P end at 0 (protocol §1.2): `n_end_count` is the N end (negative for a
//! sign −1 unit), `neutral_count` the neutral. From these, with the axis's
//! current `decel_zone_count`:
//!
//! ```text
//! mechanical = [min(0, n_end), max(0, n_end)]
//! soft       = [mechanical_min + decel, mechanical_max − decel]
//! count_at_virtual_0    = the soft limit on the P-end side   (virtual grows P → N)
//! count_at_virtual_512  = neutral_count
//! count_at_virtual_1023 = the soft limit on the N-end side
//! ```
//!
//! The file is the running config's text with those values, `device_id` (the
//! connected unit) and `calibration_valid = false` replaced line by line inside
//! each `[[axis]]` block, comments kept, and a header comment with the
//! timestamp (UTC), the raw results and the previous values. The operator
//! still has to review it and set `calibration_valid = true` by hand.

use serde::Serialize;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use crate::supervisor::CalibrationResult;
use crate::types::{AxisId, Counts, DeviceId};

/// Values written for one axis.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct AxisExport {
    pub axis_id: AxisId,
    pub device_id: DeviceId,
    pub n_end_count: Counts,
    pub neutral_count: Counts,
    pub sign: i8,
    pub decel_zone_count: i32,
    pub mechanical_min_count: Counts,
    pub mechanical_max_count: Counts,
    pub soft_min_count: Counts,
    pub soft_max_count: Counts,
    pub count_at_virtual_0: Counts,
    pub count_at_virtual_512: Counts,
    pub count_at_virtual_1023: Counts,
}

/// Derive the config values of one axis from its CAL_RESULT. `Err` when the
/// arithmetic leaves i32 (absurd counts).
pub fn derive(
    axis_id: AxisId,
    device_id: DeviceId,
    decel_zone_count: i32,
    r: &CalibrationResult,
) -> Result<AxisExport, String> {
    let n = i64::from(r.n_end_count);
    let decel = i64::from(decel_zone_count);
    let mech_min = n.min(0);
    let mech_max = n.max(0);
    let soft_min = mech_min + decel;
    let soft_max = mech_max - decel;
    // Virtual position increases from the P end (count 0) toward the N end.
    let (cv0, cv1023) = if n >= 0 {
        (soft_min, soft_max)
    } else {
        (soft_max, soft_min)
    };
    let to_i32 = |name: &str, v: i64| {
        i32::try_from(v).map_err(|_| format!("{axis_id}: {name} {v} does not fit in i32"))
    };
    Ok(AxisExport {
        axis_id,
        device_id,
        n_end_count: r.n_end_count,
        neutral_count: r.neutral_count,
        sign: r.sign,
        decel_zone_count,
        mechanical_min_count: to_i32("mechanical_min_count", mech_min)?,
        mechanical_max_count: to_i32("mechanical_max_count", mech_max)?,
        soft_min_count: to_i32("soft_min_count", soft_min)?,
        soft_max_count: to_i32("soft_max_count", soft_max)?,
        count_at_virtual_0: to_i32("count_at_virtual_0", cv0)?,
        count_at_virtual_512: r.neutral_count,
        count_at_virtual_1023: to_i32("count_at_virtual_1023", cv1023)?,
    })
}

/// One replaced `key = value` line.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Replaced {
    pub axis_id: AxisId,
    pub key: &'static str,
    pub old: String,
    pub new: String,
}

/// Result of [`rewrite_toml`]: the new text (no header yet) and what changed.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Rewrite {
    pub text: String,
    pub replaced: Vec<Replaced>,
}

/// Keys rewritten in an axis block, in the order they are reported.
pub const REWRITTEN_KEYS: [&str; 10] = [
    "device_id",
    "calibration_valid",
    "count_at_virtual_0",
    "count_at_virtual_512",
    "count_at_virtual_1023",
    "mechanical_min_count",
    "mechanical_max_count",
    "soft_min_count",
    "soft_max_count",
    "decel_zone_count",
];

impl AxisExport {
    fn value_of(&self, key: &str) -> Option<String> {
        Some(match key {
            "device_id" => format!("\"{}\"", self.device_id),
            "calibration_valid" => "false".to_string(),
            "count_at_virtual_0" => self.count_at_virtual_0.to_string(),
            "count_at_virtual_512" => self.count_at_virtual_512.to_string(),
            "count_at_virtual_1023" => self.count_at_virtual_1023.to_string(),
            "mechanical_min_count" => self.mechanical_min_count.to_string(),
            "mechanical_max_count" => self.mechanical_max_count.to_string(),
            "soft_min_count" => self.soft_min_count.to_string(),
            "soft_max_count" => self.soft_max_count.to_string(),
            "decel_zone_count" => self.decel_zone_count.to_string(),
            _ => return None,
        })
    }
}

/// A `key = value  # comment` line split into parts (whitespace kept).
struct KeyLine<'a> {
    /// Everything up to and including `=`.
    head: &'a str,
    key: &'a str,
    /// Whitespace between `=` and the value.
    pad: &'a str,
    value: &'a str,
    /// Whitespace between the value and the comment (or end of line).
    gap: &'a str,
    /// `# …` (empty when none).
    comment: &'a str,
}

/// Parse a line as `key = value [# comment]`. Table headers, blank lines and
/// comment lines are `None`. A `#` inside a quoted string is not a comment.
fn parse_key_line(line: &str) -> Option<KeyLine<'_>> {
    let trimmed = line.trim_start();
    if trimmed.is_empty() || trimmed.starts_with('#') || trimmed.starts_with('[') {
        return None;
    }
    let eq = line.find('=')?;
    let key = line[..eq].trim();
    if key.is_empty()
        || !key
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-')
    {
        return None; // quoted / dotted keys are not rewritten
    }
    let rest = &line[eq + 1..];
    let mut in_str = false;
    let mut escaped = false;
    let mut comment_at = rest.len();
    for (i, c) in rest.char_indices() {
        match c {
            '\\' if in_str => escaped = !escaped,
            '"' if !escaped => in_str = !in_str,
            '#' if !in_str => {
                comment_at = i;
                break;
            }
            _ => escaped = false,
        }
    }
    let before = &rest[..comment_at];
    let value = before.trim();
    let pad_len = before.len() - before.trim_start().len();
    let pad = &before[..pad_len];
    let gap = &before[pad_len + value.len()..];
    Some(KeyLine {
        head: &line[..=eq],
        key,
        pad,
        value,
        gap,
        comment: &rest[comment_at..],
    })
}

/// Replace the calibration values of `axes` inside the `[[axis]]` blocks of
/// `text` (line by line; everything else, comments included, is kept). Every
/// key in [`REWRITTEN_KEYS`] must exist once in each updated block.
pub fn rewrite_toml(text: &str, axes: &[AxisExport]) -> Result<Rewrite, String> {
    let lines: Vec<&str> = text.split_inclusive('\n').collect();
    // Block = (first line index after the header, one past the last line).
    let mut blocks: Vec<(usize, usize)> = Vec::new();
    let mut open: Option<usize> = None;
    for (i, line) in lines.iter().enumerate() {
        let t = line.trim_start();
        if t.starts_with('[') {
            if let Some(start) = open.take() {
                blocks.push((start, i));
            }
            if t.trim_end().trim_end_matches('\r') == "[[axis]]" {
                open = Some(i + 1);
            }
        }
    }
    if let Some(start) = open {
        blocks.push((start, lines.len()));
    }
    let axis_of_block = |(start, end): (usize, usize)| -> Option<AxisId> {
        lines[start..end]
            .iter()
            .filter_map(|l| parse_key_line(l))
            .find(|k| k.key == "axis_id")
            .and_then(|k| k.value.parse::<u8>().ok())
            .and_then(AxisId::new)
    };
    let mut out: Vec<String> = lines.iter().map(|l| l.to_string()).collect();
    let mut replaced = Vec::new();
    for ax in axes {
        let found: Vec<(usize, usize)> = blocks
            .iter()
            .copied()
            .filter(|b| axis_of_block(*b) == Some(ax.axis_id))
            .collect();
        let (start, end) = match found.as_slice() {
            [b] => *b,
            [] => {
                return Err(format!(
                    "no [[axis]] block with axis_id = {}",
                    ax.axis_id.get()
                ))
            }
            _ => {
                return Err(format!(
                    "[[axis]] axis_id = {} appears more than once",
                    ax.axis_id.get()
                ))
            }
        };
        for key in REWRITTEN_KEYS {
            let Some(new) = ax.value_of(key) else {
                continue;
            };
            let hits: Vec<usize> = (start..end)
                .filter(|&i| parse_key_line(lines[i]).is_some_and(|k| k.key == key))
                .collect();
            let i = match hits.as_slice() {
                [i] => *i,
                [] => {
                    return Err(format!(
                        "axis {}: key {key} not found in its [[axis]] block",
                        ax.axis_id.get()
                    ))
                }
                _ => {
                    return Err(format!(
                        "axis {}: key {key} appears more than once",
                        ax.axis_id.get()
                    ))
                }
            };
            let Some(k) = parse_key_line(lines[i]) else {
                continue;
            };
            let eol = if lines[i].ends_with("\r\n") {
                "\r\n"
            } else if lines[i].ends_with('\n') {
                "\n"
            } else {
                ""
            };
            let gap = if k.comment.is_empty() {
                ""
            } else if k.gap.is_empty() {
                " "
            } else {
                k.gap
            };
            let pad = if k.pad.is_empty() { " " } else { k.pad };
            let comment = k.comment.trim_end_matches(['\r', '\n']);
            out[i] = format!("{}{pad}{new}{gap}{comment}{eol}", k.head);
            replaced.push(Replaced {
                axis_id: ax.axis_id,
                key,
                old: k.value.to_string(),
                new,
            });
        }
    }
    Ok(Rewrite {
        text: out.concat(),
        replaced,
    })
}

/// Header comment for the exported file.
pub fn header(timestamp: &str, source: &str, axes: &[AxisExport], replaced: &[Replaced]) -> String {
    let mut h = String::new();
    h.push_str(
        "# h3-control-server が校正結果から書き出した設定（POST /v1/calibration/export）。\n",
    );
    h.push_str(&format!(
        "# 書き出し: {timestamp}（UTC）。元の設定: {source}\n"
    ));
    h.push_str("# calibration_valid は false のまま。値を確認し、hardware-bringup.md §5.5 の手順で true にすること。\n");
    for a in axes {
        h.push_str(&format!(
            "# axis {}: device_id {} / CAL_RESULT n_end_count {} neutral_count {} sign {:+} / \
             mechanical [{}, {}] soft [{}, {}] count_at_virtual [{}, {}, {}] (decel_zone {})\n",
            a.axis_id.get(),
            a.device_id,
            a.n_end_count,
            a.neutral_count,
            a.sign,
            a.mechanical_min_count,
            a.mechanical_max_count,
            a.soft_min_count,
            a.soft_max_count,
            a.count_at_virtual_0,
            a.count_at_virtual_512,
            a.count_at_virtual_1023,
            a.decel_zone_count
        ));
    }
    for r in replaced.iter().filter(|r| r.old != r.new) {
        h.push_str(&format!(
            "#   axis {} {}: {} -> {}\n",
            r.axis_id.get(),
            r.key,
            r.old,
            r.new
        ));
    }
    h.push('\n');
    h
}

/// File name: `<rig_id>.calibration-<timestamp>.toml` with the rig id reduced
/// to `[A-Za-z0-9._-]` (no path separators).
pub fn file_name(rig_id: &str, timestamp: &str) -> String {
    let safe: String = rig_id
        .chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || matches!(c, '.' | '_' | '-') {
                c
            } else {
                '_'
            }
        })
        .collect();
    let safe = if safe.is_empty() {
        "rig".to_string()
    } else {
        safe
    };
    format!("{safe}.calibration-{timestamp}.toml")
}

/// `YYYYmmdd-HHMMSS` of `t` in UTC (the server has no time-zone database).
pub fn utc_timestamp(t: SystemTime) -> String {
    let secs = t
        .duration_since(UNIX_EPOCH)
        .unwrap_or(Duration::ZERO)
        .as_secs();
    let days = secs / 86_400;
    let rem = secs % 86_400;
    let (h, m, s) = (rem / 3600, (rem % 3600) / 60, rem % 60);
    let (y, mo, d) = civil_from_days(i64::try_from(days).unwrap_or(0));
    format!("{y:04}{mo:02}{d:02}-{h:02}{m:02}{s:02}")
}

/// Days since 1970-01-01 → (year, month, day), proleptic Gregorian
/// (Howard Hinnant's `civil_from_days`).
fn civil_from_days(z: i64) -> (i64, u32, u32) {
    let z = z + 719_468;
    let era = if z >= 0 { z } else { z - 146_096 } / 146_097;
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let m = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    (if m <= 2 { y + 1 } else { y }, m, d)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn timestamp_is_utc_civil() {
        assert_eq!(utc_timestamp(UNIX_EPOCH), "19700101-000000");
        // 2026-11-01T05:30:12Z
        assert_eq!(
            utc_timestamp(UNIX_EPOCH + Duration::from_secs(1_793_511_012)),
            "20261101-053012"
        );
        // 2000-02-29T23:59:59Z (leap day).
        assert_eq!(
            utc_timestamp(UNIX_EPOCH + Duration::from_secs(951_868_799)),
            "20000229-235959"
        );
    }

    #[test]
    fn file_name_has_no_path_separators() {
        assert_eq!(
            file_name("h3/bringup 01", "20261101-000000"),
            "h3_bringup_01.calibration-20261101-000000.toml"
        );
        assert_eq!(file_name("", "t"), "rig.calibration-t.toml");
    }

    #[test]
    fn key_line_keeps_comment_and_quoted_hash() {
        let k = parse_key_line("device_id = \"51#31\"   # 2026 (照合用)\n").unwrap();
        assert_eq!(k.key, "device_id");
        assert_eq!(k.value, "\"51#31\"");
        assert_eq!(k.gap, "   ");
        assert_eq!(k.comment, "# 2026 (照合用)\n");
        assert!(parse_key_line("[[axis]]").is_none());
        assert!(parse_key_line("# soft_min_count = 1").is_none());
    }
}
