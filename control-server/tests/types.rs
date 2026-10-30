//! types: AxisId index conversions.

use h3cs::types::{AxisId, AXIS_COUNT};

#[test]
fn from_index_round_trips_valid_indices() {
    for i in 0..AXIS_COUNT {
        let id = AxisId::from_index(i).expect("valid index");
        assert_eq!(id.index(), i);
        assert_eq!(usize::from(id.get()), i + 1);
    }
}

#[test]
fn from_index_rejects_out_of_range_without_truncation() {
    // 255 used to overflow `as u8 + 1`; 256 / 257 used to wrap to axis 1 / 2.
    for i in [3, 4, 254, 255, 256, 257, 258, usize::MAX] {
        assert_eq!(AxisId::from_index(i), None, "index {i}");
    }
}
