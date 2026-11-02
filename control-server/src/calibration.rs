//! Per-axis calibration: legacy virtual position (0..=1023) → encoder count
//! (architecture §6). Three-point piecewise linear through (0, c0), (512, c512),
//! (1023, c1023); strictly monotonic, direction may differ per axis.
//! 0 and 1023 are operational soft targets, not the mechanical end stops.

use crate::config::{AxisConfig, Config};
use crate::types::{AxisId, Counts, VirtualPos, VIRTUAL_CENTER, VIRTUAL_MAX};
use crc::{Crc, CRC_32_ISO_HDLC};

#[derive(Debug, Clone, PartialEq)]
pub struct AxisCalibration {
    pub axis_id: AxisId,
    pub count_at: [i32; 3], // virtual 0, 512, 1023
    pub mechanical_min_count: Counts,
    pub mechanical_max_count: Counts,
    pub soft_min_count: Counts,
    pub soft_max_count: Counts,
    pub decel_zone_count: i32,
    pub operator_marked_valid: bool,
}

#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum CalibrationError {
    #[error("{axis}: {msg}")]
    Invalid { axis: AxisId, msg: String },
}

impl AxisCalibration {
    pub fn from_config(a: &AxisConfig) -> Self {
        Self {
            axis_id: a.axis_id,
            count_at: [a.count_at_virtual_0, a.count_at_virtual_512, a.count_at_virtual_1023],
            mechanical_min_count: a.mechanical_min_count,
            mechanical_max_count: a.mechanical_max_count,
            soft_min_count: a.soft_min_count,
            soft_max_count: a.soft_max_count,
            decel_zone_count: a.decel_zone_count,
            operator_marked_valid: a.calibration_valid,
        }
    }
    /// Invariants: mechanical_min < soft_min, soft_max < mechanical_max, with at
    /// least decel_zone_count margin on each side; decel_zone_count >= 0;
    /// count_at strictly monotonic; all three points inside [soft_min, soft_max].
    /// Velocity / acceleration / protection thresholds live in `[limits]`
    /// (design 2026-11-01), not here.
    ///
    /// Reports the first violated invariant, naming the field.
    pub fn validate(&self) -> Result<(), CalibrationError> {
        let err = |msg: String| Err(CalibrationError::Invalid { axis: self.axis_id, msg });
        // i64 so differences of extreme i32 values cannot overflow.
        let [c0, c512, c1023] = self.count_at.map(i64::from);
        let mech_min = i64::from(self.mechanical_min_count);
        let mech_max = i64::from(self.mechanical_max_count);
        let soft_min = i64::from(self.soft_min_count);
        let soft_max = i64::from(self.soft_max_count);
        let decel = i64::from(self.decel_zone_count);

        if decel < 0 {
            return err(format!("decel_zone_count must be >= 0, got {decel}"));
        }
        if mech_min >= mech_max {
            return err(format!("mechanical_min_count ({mech_min}) must be < mechanical_max_count ({mech_max})"));
        }
        if soft_min >= soft_max {
            return err(format!("soft_min_count ({soft_min}) must be < soft_max_count ({soft_max})"));
        }
        if soft_min <= mech_min || soft_min - mech_min < decel {
            return err(format!(
                "soft_min_count ({soft_min}) must be above mechanical_min_count ({mech_min}) by at least decel_zone_count ({decel})"
            ));
        }
        if soft_max >= mech_max || mech_max - soft_max < decel {
            return err(format!(
                "soft_max_count ({soft_max}) must be below mechanical_max_count ({mech_max}) by at least decel_zone_count ({decel})"
            ));
        }
        let increasing = c0 < c512 && c512 < c1023;
        let decreasing = c0 > c512 && c512 > c1023;
        if !increasing && !decreasing {
            return err(format!(
                "count_at_virtual_0/512/1023 ({c0}, {c512}, {c1023}) must be strictly monotonic"
            ));
        }
        for (name, c) in [("count_at_virtual_0", c0), ("count_at_virtual_512", c512), ("count_at_virtual_1023", c1023)] {
            if c < soft_min || c > soft_max {
                return err(format!("{name} ({c}) must lie inside soft limits [{soft_min}, {soft_max}]"));
            }
        }
        Ok(())
    }

    fn points(&self) -> [f64; 3] {
        self.count_at.map(f64::from)
    }

    /// Map a virtual position to a count (f64, unrounded), extrapolating linearly
    /// outside 0..=1023 from the nearest segment (callers saturate first).
    pub fn virtual_to_count_f(&self, v: VirtualPos) -> f64 {
        let [c0, c512, c1023] = self.points();
        let v = v.0;
        if v <= VIRTUAL_CENTER {
            c0 + (c512 - c0) * (v / VIRTUAL_CENTER)
        } else {
            c512 + (c1023 - c512) * ((v - VIRTUAL_CENTER) / (VIRTUAL_MAX - VIRTUAL_CENTER))
        }
    }
    /// Inverse map (for status display); exact inverse of `virtual_to_count_f`
    /// (up to float rounding). Works for increasing and decreasing axes.
    pub fn count_to_virtual(&self, c: f64) -> VirtualPos {
        let [c0, c512, c1023] = self.points();
        // Upper segment iff c lies on the 1023 side of the neutral count.
        if (c - c512) * (c1023 - c512) > 0.0 {
            VirtualPos(VIRTUAL_CENTER + (VIRTUAL_MAX - VIRTUAL_CENTER) * ((c - c512) / (c1023 - c512)))
        } else {
            VirtualPos(VIRTUAL_CENTER * ((c - c0) / (c512 - c0)))
        }
    }
    /// Clamp to soft limits; returns (count, was_clamped). NaN maps to the
    /// neutral count and is reported as clamped.
    pub fn clamp_soft(&self, c: f64) -> (f64, bool) {
        let lo = f64::from(self.soft_min_count);
        let hi = f64::from(self.soft_max_count);
        if c.is_nan() {
            (f64::from(self.neutral_count()), true)
        } else if c < lo {
            (lo, true)
        } else if c > hi {
            (hi, true)
        } else {
            (c, false)
        }
    }
    /// Count at virtual 512.
    pub fn neutral_count(&self) -> Counts {
        self.count_at[1]
    }
}

/// CRC-32/ISO-HDLC over a canonical little-endian serialization of
/// (rig_id bytes, axis_id, count_at[3], mechanical/soft limits, decel zone).
/// Sent in CONFIG; echoed in HELLO. Geometry only: the motion limits moved
/// to `[limits]` (design 2026-11-01) and are not part of the hash.
///
/// Layout: u32 len(rig_id) | rig_id UTF-8 | u8 axis_id | i32 count_at[0..3] |
/// i32 mechanical_min, mechanical_max, soft_min, soft_max, decel_zone.
/// `operator_marked_valid` is deliberately excluded: it is server-side
/// policy, not an MCU parameter.
pub fn calibration_hash(rig_id: &str, cal: &AxisCalibration) -> u32 {
    let mut buf = Vec::with_capacity(64 + rig_id.len());
    let id = rig_id.as_bytes();
    buf.extend_from_slice(&u32::try_from(id.len()).unwrap_or(u32::MAX).to_le_bytes());
    buf.extend_from_slice(id);
    buf.push(cal.axis_id.get());
    for c in cal.count_at {
        buf.extend_from_slice(&c.to_le_bytes());
    }
    for c in [
        cal.mechanical_min_count,
        cal.mechanical_max_count,
        cal.soft_min_count,
        cal.soft_max_count,
        cal.decel_zone_count,
    ] {
        buf.extend_from_slice(&c.to_le_bytes());
    }
    Crc::<u32>::new(&CRC_32_ISO_HDLC).checksum(&buf)
}

/// All three axes' calibrations, indexed by AxisId::index(). Expects a
/// validated config; a missing axis (unvalidated input) yields a placeholder
/// that fails `validate()` and is not operator-marked valid, instead of panicking.
pub fn from_config(cfg: &Config) -> [AxisCalibration; 3] {
    AxisId::ALL.map(|id| match cfg.axes.iter().find(|a| a.axis_id == id) {
        Some(a) => AxisCalibration::from_config(a),
        None => AxisCalibration {
            axis_id: id,
            count_at: [0; 3],
            mechanical_min_count: 0,
            mechanical_max_count: 0,
            soft_min_count: 0,
            soft_max_count: 0,
            decel_zone_count: 0,
            operator_marked_valid: false,
        },
    })
}
