//! types: AxisId index conversions, FaultCode wire / JSON names.

use h3cs::types::{AxisId, FaultCode, McuState, AXIS_COUNT};

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

#[test]
fn fault_code_wire_values_round_trip_and_local_estop_is_12() {
    // protocol.md §3.2: 0..=12 are defined, everything else is UNKNOWN.
    for v in 0..=12u16 {
        let code = FaultCode::from_u16(v);
        assert_ne!(code, FaultCode::Unknown, "value {v} is defined");
        assert_eq!(code.to_u16(), v);
    }
    assert_eq!(FaultCode::from_u16(12), FaultCode::LocalEstop);
    assert_eq!(FaultCode::LocalEstop.to_u16(), 12);
    for v in [13u16, 100, 0xFFFE, 0xFFFF] {
        assert_eq!(FaultCode::from_u16(v), FaultCode::Unknown, "value {v}");
    }
    assert_eq!(FaultCode::Unknown.to_u16(), 0xFFFF);
}

#[test]
fn fault_code_json_names_are_screaming_snake_case() {
    assert_eq!(serde_json::to_value(FaultCode::LocalEstop).unwrap(), "LOCAL_ESTOP");
    assert_eq!(serde_json::from_value::<FaultCode>("LOCAL_ESTOP".into()).unwrap(), FaultCode::LocalEstop);
    assert_eq!(serde_json::to_value(FaultCode::HostFault).unwrap(), "HOST_FAULT");
    assert_eq!(serde_json::to_value(FaultCode::None).unwrap(), "NONE");
}

#[test]
fn mcu_state_calibrating_is_8() {
    // protocol.md §3.1: 0..=8 are defined; CALIBRATING = 8.
    for v in 0..=8u8 {
        let s = McuState::from_u8(v).expect("defined");
        assert_eq!(s as u8, v);
    }
    assert_eq!(McuState::from_u8(8), Some(McuState::Calibrating));
    assert_eq!(McuState::from_u8(9), None);
    assert_eq!(serde_json::to_value(McuState::Calibrating).unwrap(), "CALIBRATING");
    assert_eq!(serde_json::from_value::<McuState>("CALIBRATING".into()).unwrap(), McuState::Calibrating);
}
