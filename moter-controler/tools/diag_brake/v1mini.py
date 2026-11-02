"""USB プロトコル v1 の最小限の符号化（計測専用の道具 diag_brake.py 用）。

moter-controler/include/v1_codec.h と同じバイト列を作る・読む（tests/test_v1mini.py が C++ のテストベクタと突き合わせる）。
使うのは HELLO_REQ / CONFIG / DIAG_START / DIAG_READ の送信と、HELLO / ACK / STATUS / FAULT_EVENT / DIAG_RESULT / DIAG_DATA の受信だけ。
外部依存なし（pyserial は diag_brake.py 側）。
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field

PROTOCOL_VERSION = 1

# 種別
HELLO_REQ = 0x01
CONFIG = 0x02
DISARM = 0x07
FAULT_RESET = 0x08
DIAG_START = 0x40
DIAG_READ = 0x41
HELLO = 0x81
ACK = 0x82
STATUS = 0x83
FAULT_EVENT = 0x84
AXIS_SELECT = 0x86
CAL_RESULT = 0x87
LIMITS = 0x88
DIAG_RESULT = 0xC0
DIAG_DATA = 0xC1

CAP_DIAG_BRAKE = 1 << 8
ROLE_POWER_CONTROLLER = 1 << 0

FLAG_POWER_ON_CMD = 1 << 0
FLAG_BRAKE_RELEASE_CMD = 1 << 1
FLAG_POSITION_REFERENCED = 1 << 2
FLAG_CONFIGURED = 1 << 3
FLAG_POWER_CONTROLLER = 1 << 4

MODE_CANCEL, MODE_SAG, MODE_ENGAGE, MODE_POWER_HOLD = 0, 1, 2, 3
MODE_NAMES = {0: "CANCEL", 1: "SAG", 2: "ENGAGE", 3: "POWER_HOLD"}
RESULT_NAMES = {0: "OK", 1: "FAULT", 2: "CANCELLED", 3: "ABORTED"}
RESULT_OK, RESULT_FAULT, RESULT_CANCELLED, RESULT_ABORTED = 0, 1, 2, 3
STOP_NAMES = {
    0: "NONE", 1: "T_MAX", 2: "D_MAX", 3: "END_MARGIN", 4: "V_TARGET", 5: "D_RUN", 6: "RUN_TIMEOUT",
    7: "REVERSE", 8: "NO_PULSE", 9: "CANCEL", 10: "FAULT", 11: "SAG_GUARD", 12: "OVERSPEED", 13: "ROOM_LOST",
    14: "BUFFER_FULL",
}
STATE_NAMES = {
    0: "BOOT", 1: "DISARMED", 2: "ARMING", 3: "HOLDING", 4: "ACTIVE", 5: "STOPPING", 6: "DISARMING",
    7: "FAULT", 8: "CALIBRATING", 9: "DIAG",
}
FAULT_NAMES = {
    0: "NONE", 1: "COMMAND_TIMEOUT", 2: "FOLLOWING_ERROR", 3: "OVERSPEED", 4: "SOFT_LIMIT", 5: "ENCODER_FAULT",
    6: "CONTROL_OVERRUN", 7: "CONFIG_INVALID", 8: "STOP_TIMEOUT", 9: "POWER_FAULT", 10: "HOST_FAULT",
    11: "NOT_REFERENCED", 12: "LOCAL_ESTOP",
}
ACK_RESULT_NAMES = {0: "ACCEPTED", 1: "REJECTED", 2: "DUPLICATE"}
ACK_REASON_NAMES = {
    0: "NONE", 1: "BAD_STATE", 2: "EPOCH_MISMATCH", 3: "NOT_CONFIGURED", 4: "INVALID_PARAM",
    5: "NOT_REFERENCED", 6: "FAULT_ACTIVE", 7: "FAULT_CAUSE_PRESENT",
}
SAMPLE_FLAG_BRAKE_RELEASE = 1 << 0
SAMPLE_FLAG_POWER_ON = 1 << 1
RESULT_FLAG_TRUNCATED = 1 << 0
RESULT_FLAG_NOT_SETTLED = 1 << 1
RESULT_FLAG_POST_MARGIN = 1 << 2   # ブレーキ作動の後に端の余裕を越えた（区間は閉じた）
RESULT_FLAG_SESSION_KEPT = 1 << 3  # 結果の後も DIAG の待機（主電源 ON・ブレーキ作動）に残っている
SESSION_ACTIVE = 1 << 0            # DIAG_DATA の session_flags
SESSION_POWER_CMD = 1 << 1
SESSION_SETTLED = 1 << 2
SESSION_TEST_BUSY = 1 << 3
DIAG_READ_STATUS_ONLY = 0xFFFF     # 範囲外の offset: 記録は返さず区間の状態だけ読む
POWER_SETTLE_MS = 5000             # ファームの V1_DIAG_POWER_SETTLE_MS
TIME_NONE = 0xFFFFFFFF
SAMPLES_PER_CHUNK = 16
SAMPLE_BYTES = 13

MCU_PAYLOAD_LEN = {HELLO: 32, ACK: 8, STATUS: 56, FAULT_EVENT: 20, AXIS_SELECT: 8, CAL_RESULT: 28, LIMITS: 24,
                   DIAG_RESULT: 64, DIAG_DATA: 16 + SAMPLES_PER_CHUNK * SAMPLE_BYTES, 0x85: 28}


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE（poly 0x1021、init 0xFFFF）"""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    out = bytearray([0])
    code_idx = 0
    code = 1
    for b in data:
        if code == 0xFF:
            out[code_idx] = code
            code_idx = len(out)
            out.append(0)
            code = 1
        if b == 0:
            out[code_idx] = code
            code_idx = len(out)
            out.append(0)
            code = 1
        else:
            out.append(b)
            code += 1
    out[code_idx] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes | None:
    out = bytearray()
    i = 0
    n = len(data)
    if n == 0:
        return None
    while i < n:
        code = data[i]
        if code == 0:
            return None
        i += 1
        block = code - 1
        if i + block > n:
            return None
        chunk = data[i:i + block]
        if 0 in chunk:
            return None
        out += chunk
        i += block
        if code < 0xFF and i < n:
            out.append(0)
    return bytes(out)


def frame(msg_type: int, payload: bytes) -> bytes:
    head = struct.pack("<BBH", PROTOCOL_VERSION, msg_type, len(payload)) + payload
    return head + struct.pack("<H", crc16(head))


def wire(msg_type: int, payload: bytes) -> bytes:
    """COBS 符号化 + 区切り 0x00"""
    return cobs_encode(frame(msg_type, payload)) + b"\x00"


def parse_frame(f: bytes) -> tuple[int, bytes] | None:
    if len(f) < 6 or len(f) > 256:
        return None
    body = f[:-2]
    if crc16(body) != struct.unpack("<H", f[-2:])[0]:
        return None
    if body[0] != PROTOCOL_VERSION:
        return None
    plen = struct.unpack("<H", body[2:4])[0]
    if plen != len(body) - 4:
        return None
    return body[1], bytes(body[4:])


# --- 送信 ---
def hello_req(request_id: int) -> bytes:
    return wire(HELLO_REQ, struct.pack("<I", request_id))


def config(request_id: int, session_epoch: int, calibration_hash: int, soft_min: int, soft_max: int,
           max_vel: int, max_acc: int, fe_limit: int, overspeed: int, command_timeout_ms: int,
           stop_timeout_ms: int, role_flags: int) -> bytes:
    p = struct.pack("<IIIiiIIIIHHB", request_id, session_epoch, calibration_hash, soft_min, soft_max, max_vel,
                    max_acc, fe_limit, overspeed, command_timeout_ms, stop_timeout_ms, role_flags)
    p += b"\x00" * (44 - len(p))
    return wire(CONFIG, p)


def fault_reset(request_id: int) -> bytes:
    return wire(FAULT_RESET, struct.pack("<I", request_id))


def diag_start_payload(request_id: int, mode: int, dir_: int = 0, duty_permille: int = 0, t_max_ms: int = 0,
                       d_max_count: int = 0, v_target_count_s: int = 0, delay_ms: int = 0, hold_ms: int = 0) -> bytes:
    return struct.pack("<IBbHHHHHI", request_id, mode, dir_, duty_permille, t_max_ms, d_max_count, v_target_count_s,
                       delay_ms, hold_ms)


def diag_start(request_id: int, mode: int, **kw) -> bytes:
    return wire(DIAG_START, diag_start_payload(request_id, mode, **kw))


def diag_read(request_id: int, offset: int) -> bytes:
    return wire(DIAG_READ, struct.pack("<IH", request_id, offset) + b"\x00\x00")


# --- 受信 ---
@dataclass
class Msg:
    type: int
    fields: dict = field(default_factory=dict)


def decode(encoded: bytes) -> Msg | None:
    """COBS 符号化済み（区切りを除く）の 1 フレーム。未知の種別・長さ違い・CRC 不一致は None。"""
    f = cobs_decode(encoded)
    if f is None:
        return None
    parsed = parse_frame(f)
    if parsed is None:
        return None
    t, p = parsed
    need = MCU_PAYLOAD_LEN.get(t)
    if need is None or need != len(p):
        return None
    if t == HELLO:
        (rid, ver, axis, state, dev, boot, fmaj, fmin, fpat) = struct.unpack_from("<IBBBxQIBBB", p, 0)
        caps, cal_hash = struct.unpack_from("<II", p, 24)
        return Msg(t, dict(request_id=rid, protocol_version=ver, selected_axis=axis, mcu_state=state, device_id=dev,
                           boot_id=boot, fw=(fmaj, fmin, fpat), capabilities=caps, calibration_hash=cal_hash))
    if t == ACK:
        rid, acked, result, reason = struct.unpack("<IBBH", p)
        return Msg(t, dict(request_id=rid, acked_type=acked, result=result, reason=reason))
    if t == STATUS:
        (boot, mcu_us, state, axis, fault, flags, pwm, actual, target, vel, fe) = struct.unpack_from(
            "<IQBBHHhiiii", p, 0)
        return Msg(t, dict(boot_id=boot, mcu_time_us=mcu_us, mcu_state=state, selected_axis=axis, fault_code=fault,
                           flags=flags, pwm_permille=pwm, actual_count=actual, target_count=target,
                           velocity_count_s=vel, following_error_count=fe))
    if t == FAULT_EVENT:
        boot, mcu_us, code, before = struct.unpack_from("<IQHB", p, 0)
        actual = struct.unpack_from("<i", p, 16)[0]
        return Msg(t, dict(boot_id=boot, mcu_time_us=mcu_us, fault_code=code, mcu_state_before=before,
                           actual_count=actual))
    if t == DIAG_RESULT:
        v = struct.unpack("<IIBBBBHHHHiIIiiiiIibBHhH", p)
        names = ["test_id", "boot_id", "mode", "result", "stop_reason", "referenced_after", "fault_code",
                 "sample_count", "sample_period_us", "max_sample_gap_us", "start_count", "t_drive_us", "t_engage_us",
                 "disp_at_engage", "final_disp", "peak_disp", "v_at_engage_count_s", "t_settle_us",
                 "disp_engage_to_settle", "sign", "flags", "adc_first", "drive_pwm_permille", "session_ms_at_release"]
        return Msg(t, dict(zip(names, v)))
    if t == DIAG_DATA:
        rid, test_id, offset, total, n, session_flags, session_ms = struct.unpack_from("<IIHHBBH", p, 0)
        if n > SAMPLES_PER_CHUNK:
            return None
        samples = []
        for i in range(n):
            ts, count, pwm, adc, flags = struct.unpack_from("<IihHB", p, 16 + i * SAMPLE_BYTES)
            samples.append(dict(t_us=ts, count=count, pwm_permille=pwm, adc_raw=adc, flags=flags))
        return Msg(t, dict(request_id=rid, test_id=test_id, offset=offset, total=total, n=n, session_flags=session_flags,
                           session_ms=session_ms, samples=samples))
    return Msg(t, {})


class Splitter:
    """受信バイト列を 0x00 で区切る"""

    def __init__(self) -> None:
        self.buf = bytearray()

    def push(self, data: bytes) -> list[bytes]:
        frames = []
        for b in data:
            if b == 0:
                if self.buf:
                    frames.append(bytes(self.buf))
                self.buf.clear()
            elif len(self.buf) < 300:
                self.buf.append(b)
            else:
                self.buf.clear()  # 長すぎる。次の区切りまで捨てる（簡易）
        return frames


def device_id_from_mac(mac: str) -> int:
    """'1C:DB:D4:BA:8D:58' → 0x51311cdbd4ba8d58（v1_device_id_from_mac と同じ: 5131 + MAC を表記順）"""
    h = normalize_mac(mac)
    if len(h) != 12:
        raise ValueError(f"MAC は 12 桁の 16 進: {mac!r}")
    return (0x5131 << 48) | int(h, 16)


def normalize_mac(text: str) -> str:
    return "".join(c for c in text.lower() if c in "0123456789abcdef")
