//! `backend.ports = "auto"`: find the CoreS3 units among the serial ports
//! without opening anything.
//!
//! Port names (`/dev/cu.usbmodemXXXX`, `COMn`) follow the physical USB port
//! and change when a cable moves to another hub socket. The USB serial number
//! does not: the CoreS3's USB JTAG/serial unit (VID:PID 303A:1001) reports its
//! eFuse MAC there (`1C:DB:D4:BA:8D:58`), and the v1 firmware's `device_id`
//! is `0x5131` followed by the same 6 bytes in written order
//! (`51311cdbd4ba8d58`). So the port of a configured unit can be found from
//! the OS port list alone. Opening a port can reset whatever is attached
//! (another ESP32-S3 has shown up under a CoreS3's old port name), so only
//! ports that match a configured `device_id` are ever opened; the HELLO
//! handshake still checks the unit after opening.

use crate::types::DeviceId;

/// Espressif USB JTAG/serial debug unit (ESP32-S3 native USB).
pub const CORES3_USB_VID: u16 = 0x303A;
pub const CORES3_USB_PID: u16 = 0x1001;
/// Upper 16 bits of every v1 firmware `device_id`.
pub const DEVICE_ID_PREFIX: u64 = 0x5131 << 48;

/// One entry of the OS serial port list.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ListedPort {
    pub name: String,
    /// Present for USB ports.
    pub usb: Option<UsbIds>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UsbIds {
    pub vid: u16,
    pub pid: u16,
    pub serial: Option<String>,
}

impl ListedPort {
    /// The `device_id` this port's unit reports, if it is an ESP32-S3 USB
    /// JTAG/serial unit with a MAC serial number.
    pub fn cores3_device_id(&self) -> Option<DeviceId> {
        let usb = self.usb.as_ref()?;
        if usb.vid != CORES3_USB_VID || usb.pid != CORES3_USB_PID {
            return None;
        }
        device_id_from_usb_serial(usb.serial.as_deref()?)
    }

    /// macOS lists every device twice; `/dev/tty.*` waits for DCD on open.
    /// Only the `/dev/cu.*` name is used.
    pub fn is_dialin_alias(&self) -> bool {
        self.name.starts_with("/dev/tty.")
    }
}

/// Serial port list from the OS (enumeration only; nothing is opened).
/// Sorted by name.
pub fn list_ports() -> Vec<ListedPort> {
    let mut v: Vec<ListedPort> = tokio_serial::available_ports()
        .unwrap_or_default()
        .into_iter()
        .map(|p| ListedPort {
            usb: match p.port_type {
                tokio_serial::SerialPortType::UsbPort(u) => {
                    Some(UsbIds { vid: u.vid, pid: u.pid, serial: u.serial_number })
                }
                _ => None,
            },
            name: p.port_name,
        })
        .collect();
    v.sort_by(|a, b| a.name.cmp(&b.name));
    v.dedup_by(|a, b| a.name == b.name);
    v
}

/// `"1C:DB:D4:BA:8D:58"` (or without separators, any case) → `51311cdbd4ba8d58`.
/// `None` unless it is exactly 6 bytes of hex.
pub fn device_id_from_usb_serial(serial: &str) -> Option<DeviceId> {
    let hex: String = serial.chars().filter(|c| !matches!(c, ':' | '-' | ' ')).collect();
    if hex.len() != 12 || !hex.chars().all(|c| c.is_ascii_hexdigit()) {
        return None;
    }
    let mac = u64::from_str_radix(&hex, 16).ok()?;
    Some(DeviceId(DEVICE_ID_PREFIX | mac))
}

/// Ports (name, device_id) of the `wanted` units, sorted by name. Only
/// CoreS3 USB ports whose serial number maps to a wanted id; never a
/// `/dev/tty.*` alias.
pub fn find_units(listed: &[ListedPort], wanted: &[DeviceId]) -> Vec<(String, DeviceId)> {
    let mut v: Vec<(String, DeviceId)> = listed
        .iter()
        .filter(|p| !p.is_dialin_alias())
        .filter_map(|p| p.cores3_device_id().filter(|id| wanted.contains(id)).map(|id| (p.name.clone(), id)))
        .collect();
    v.sort();
    v
}

/// CoreS3-looking ports that are not opened because no wanted `device_id`
/// matches (a swapped-in unit, or a unit of another project). For logging.
pub fn unmatched_units(listed: &[ListedPort], wanted: &[DeviceId]) -> Vec<(String, String)> {
    listed
        .iter()
        .filter(|p| !p.is_dialin_alias())
        .filter(|p| matches!(&p.usb, Some(u) if u.vid == CORES3_USB_VID && u.pid == CORES3_USB_PID))
        .filter(|p| p.cores3_device_id().is_none_or(|id| !wanted.contains(&id)))
        .map(|p| (p.name.clone(), p.usb.as_ref().and_then(|u| u.serial.clone()).unwrap_or_default()))
        .collect()
}

/// Does `name` still carry the unit `id` in this list?
pub fn still_matches(listed: &[ListedPort], name: &str, id: DeviceId) -> bool {
    listed.iter().any(|p| p.name == name && p.cores3_device_id() == Some(id))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn usb(name: &str, vid: u16, pid: u16, serial: Option<&str>) -> ListedPort {
        ListedPort { name: name.into(), usb: Some(UsbIds { vid, pid, serial: serial.map(Into::into) }) }
    }

    fn cores3(name: &str, serial: &str) -> ListedPort {
        usb(name, 0x303A, 0x1001, Some(serial))
    }

    /// The three units on 2026-11-02 after the hub re-plug (both macOS names).
    fn rig() -> Vec<ListedPort> {
        vec![
            ListedPort { name: "/dev/cu.Bluetooth-Incoming-Port".into(), usb: None },
            cores3("/dev/cu.usbmodem14101", "68:EE:8F:D8:42:F0"),
            cores3("/dev/cu.usbmodem14301", "30:ED:A0:0B:20:40"),
            cores3("/dev/cu.usbmodem14501", "1C:DB:D4:BA:8D:58"),
            cores3("/dev/tty.usbmodem14101", "68:EE:8F:D8:42:F0"),
            cores3("/dev/tty.usbmodem14301", "30:ED:A0:0B:20:40"),
            cores3("/dev/tty.usbmodem14501", "1C:DB:D4:BA:8D:58"),
        ]
    }

    const D1: DeviceId = DeviceId(0x5131_1cdb_d4ba_8d58);
    const D2: DeviceId = DeviceId(0x5131_68ee_8fd8_42f0);
    const D3: DeviceId = DeviceId(0x5131_30ed_a00b_2040);

    #[test]
    fn usb_serial_maps_to_the_firmware_device_id() {
        assert_eq!(device_id_from_usb_serial("1C:DB:D4:BA:8D:58"), Some(D1));
        assert_eq!(device_id_from_usb_serial("30:ed:a0:0b:20:40"), Some(D3));
        assert_eq!(device_id_from_usb_serial("68EE8FD842F0"), Some(D2));
        assert_eq!(device_id_from_usb_serial("68-EE-8F-D8-42-F0"), Some(D2));
        assert_eq!(device_id_from_usb_serial(""), None);
        assert_eq!(device_id_from_usb_serial("1C:DB:D4:BA:8D"), None);
        assert_eq!(device_id_from_usb_serial("1C:DB:D4:BA:8D:58:00"), None);
        assert_eq!(device_id_from_usb_serial("ZZ:DB:D4:BA:8D:58"), None);
    }

    #[test]
    fn finds_each_configured_unit_on_its_cu_port_only() {
        let got = find_units(&rig(), &[D1, D2, D3]);
        assert_eq!(
            got,
            vec![
                ("/dev/cu.usbmodem14101".to_string(), D2),
                ("/dev/cu.usbmodem14301".to_string(), D3),
                ("/dev/cu.usbmodem14501".to_string(), D1),
            ]
        );
    }

    #[test]
    fn never_matches_units_that_are_not_configured_or_other_devices() {
        let mut listed = rig();
        // Another project's ESP32-S3 (same VID:PID, other MAC), a non-Espressif
        // adapter carrying a MAC-looking serial, and a CoreS3 without serial.
        listed.push(cores3("/dev/cu.usbmodem2101", "DC:54:75:00:11:22"));
        listed.push(usb("/dev/cu.usbserial-1", 0x10C4, 0xEA60, Some("1C:DB:D4:BA:8D:58")));
        listed.push(usb("/dev/cu.usbmodem3101", 0x303A, 0x1001, None));
        let got = find_units(&listed, &[D1]);
        assert_eq!(got, vec![("/dev/cu.usbmodem14501".to_string(), D1)]);

        let un = unmatched_units(&listed, &[D1]);
        let names: Vec<&str> = un.iter().map(|(n, _)| n.as_str()).collect();
        assert_eq!(
            names,
            ["/dev/cu.usbmodem14101", "/dev/cu.usbmodem14301", "/dev/cu.usbmodem2101", "/dev/cu.usbmodem3101"]
        );
        assert!(find_units(&listed, &[]).is_empty());
    }

    #[test]
    fn still_matches_follows_the_serial_not_the_name() {
        let listed = rig();
        assert!(still_matches(&listed, "/dev/cu.usbmodem14501", D1));
        // The name now carries another unit, or is gone.
        assert!(!still_matches(&listed, "/dev/cu.usbmodem14501", D2));
        assert!(!still_matches(&listed, "/dev/cu.usbmodem11501", D1));
    }
}
