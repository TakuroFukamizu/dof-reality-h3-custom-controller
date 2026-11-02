"""v1mini.py を C++（v1_codec.h）のテストベクタと突き合わせる。実機・シリアルは使わない。

    ~/.platformio/penv/bin/python -m unittest discover -s moter-controler/tools/diag_brake -p 'test_*.py'
"""

import struct
import time
import types
import unittest

import diag_brake
import v1mini


def unhex(text: str) -> bytes:
    return bytes.fromhex(text.replace(" ", ""))


class CodecVectors(unittest.TestCase):
    def test_crc(self):
        self.assertEqual(v1mini.crc16(b"123456789"), 0x29B1)

    def test_hello_req_vector(self):  # test_v1_codec.h 7.1
        self.assertEqual(v1mini.frame(v1mini.HELLO_REQ, (1).to_bytes(4, "little")), unhex("01 01 04 00 01 00 00 00 99 BE"))
        self.assertEqual(v1mini.hello_req(1), unhex("04 01 01 04 02 01 01 01 03 99 BE 00"))

    def test_diag_start_vector(self):  # test_v1_diag.h test_diag_codec_roundtrip と同じバイト列
        f = v1mini.frame(v1mini.DIAG_START, v1mini.diag_start_payload(
            0x01020304, v1mini.MODE_ENGAGE, dir_=-1, duty_permille=200, t_max_ms=150, d_max_count=400,
            v_target_count_s=800, delay_ms=50, hold_ms=60000))
        self.assertEqual(f[:24], unhex("01 40 14 00 04 03 02 01 02 FF C8 00 96 00 90 01 20 03 32 00 60 EA 00 00"))
        self.assertEqual(len(f), 26)

    def test_status_vector(self):  # test_v1_codec.h 7.15
        w = unhex("04 01 83 38 09 EF CD AB 89 D2 02 96 49 01 01 01 03 04 01 01 02 1F 10 06 FF 32 FB FF FF 2E FB FF FF "
                  "6A FF FF FF 04 01 01 03 D2 04 01 03 E8 03 01 03 E8 03 01 01 01 01 01 02 01 02 02 03 49 36 00")
        m = v1mini.decode(w[:-1])
        self.assertIsNotNone(m)
        self.assertEqual(m.type, v1mini.STATUS)
        self.assertEqual(m.fields["boot_id"], 2309737967)
        self.assertEqual(m.fields["mcu_state"], 4)
        self.assertEqual(m.fields["flags"], 31)
        self.assertEqual(m.fields["pwm_permille"], -250)
        self.assertEqual(m.fields["actual_count"], -1230)
        self.assertEqual(m.fields["velocity_count_s"], -150)

    def test_diag_result_vector(self):  # C++ の encode_mcu_wire で作った（genvec）
        w = unhex("0401C0400265010108EFCDAB8902010802050D8B02E803EB0324FAFFFF50C30104400D0315FEFFFFFFFDFFFFFFFCFFFFFF"
                  "CEFFFFFFFFFFFFFF01010106FF02FF0796010103A75500")
        m = v1mini.decode(w[:-1])
        self.assertIsNotNone(m)
        f = m.fields
        self.assertEqual((f["test_id"], f["mode"], f["result"], f["stop_reason"], f["fault_code"]), (101, 2, 1, 8, 5))
        self.assertEqual((f["sample_count"], f["max_sample_gap_us"], f["start_count"]), (651, 1003, -1500))
        self.assertEqual((f["t_drive_us"], f["t_engage_us"], f["t_settle_us"]), (50000, 200000, v1mini.TIME_NONE))
        self.assertEqual((f["disp_at_engage"], f["final_disp"], f["peak_disp"], f["v_at_engage_count_s"]), (-2, -3, -4, -50))
        self.assertEqual((f["sign"], f["flags"], f["adc_first"], f["drive_pwm_permille"]), (-1, 2, 2047, 150))

    def test_diag_data_vector(self):
        w = unhex("0401C1E002070101026501010210048B0202010103803E010524FAFFFF0106FF07036842010723FAFFFF6AFF0102"
                  + "01" * 182 + "03852100")
        m = v1mini.decode(w[:-1])
        self.assertIsNotNone(m)
        f = m.fields
        self.assertEqual((f["request_id"], f["test_id"], f["offset"], f["total"], f["n"]), (7, 101, 16, 651, 2))
        self.assertEqual(f["samples"][0], dict(t_us=16000, count=-1500, pwm_permille=0, adc_raw=2047, flags=3))
        self.assertEqual(f["samples"][1], dict(t_us=17000, count=-1501, pwm_permille=-150, adc_raw=0, flags=1))

    def test_diag_result_session_fields_vector(self):  # genvec: ABORTED / ROOM_LOST、SESSION_KEPT、解除時の区間 5005 ms
        w = unhex("0401C0400266010109EFCDAB8901030D0101010103E803010324090109FFFFFFFFFFFFFFFF0101010101010101010101010101"
                  "0105FFFFFFFF010101030108010101058D1394D900")
        m = v1mini.decode(w[:-1])
        self.assertIsNotNone(m)
        f = m.fields
        self.assertEqual((f["test_id"], f["result"], f["stop_reason"], f["start_count"]), (102, 3, 13, 2340))
        self.assertEqual(v1mini.RESULT_NAMES[f["result"]], "ABORTED")
        self.assertEqual(v1mini.STOP_NAMES[f["stop_reason"]], "ROOM_LOST")
        self.assertEqual(f["flags"], v1mini.RESULT_FLAG_SESSION_KEPT)
        self.assertEqual(f["session_ms_at_release"], 5005)

    def test_diag_data_session_status_vector(self):  # genvec: DIAG_READ(offset 0xFFFF) への応答（区間の状態だけ）
        w = unhex("0401C1E0020801010266010103FFFF010104078F19" + "01" * 207 + "0391BB00")
        m = v1mini.decode(w[:-1])
        self.assertIsNotNone(m)
        f = m.fields
        self.assertEqual((f["request_id"], f["test_id"], f["offset"], f["n"]), (8, 102, 0xFFFF, 0))
        self.assertEqual(f["session_flags"], v1mini.SESSION_ACTIVE | v1mini.SESSION_POWER_CMD | v1mini.SESSION_SETTLED)
        self.assertEqual(f["session_ms"], 6543)

    def test_cobs_roundtrip(self):
        for data in (b"", b"\x00", b"\x11\x00\x22", bytes(range(1, 256)) * 2, bytes(300)):
            enc = v1mini.cobs_encode(data)
            self.assertNotIn(0, enc)
            if data:
                self.assertEqual(v1mini.cobs_decode(enc), data)

    def test_device_id(self):
        self.assertEqual(v1mini.device_id_from_mac("1C:DB:D4:BA:8D:58"), 0x51311CDBD4BA8D58)


class Analysis(unittest.TestCase):
    def test_sag_metrics(self):
        # 0〜200 ms 解除（10 ms 遅れて 200 count/s で沈む）→ 200 ms で作動 → 30 ms 滑って止まる
        samples = []
        c = 0.0
        for t in range(0, 500):
            released = t < 200
            if 10 <= t < 230:
                c -= 0.2
            samples.append(dict(t_us=t * 1000, count=round(c), pwm_permille=0, adc_raw=0,
                                flags=v1mini.SAMPLE_FLAG_BRAKE_RELEASE if released else 0))
        at_engage = samples[200]["count"]
        res = dict(t_engage_us=200000, start_count=0, disp_at_engage=at_engage, final_disp=samples[-1]["count"],
                   v_at_engage_count_s=-200)
        m = diag_brake.host_metrics(samples, res)
        self.assertTrue(17.0 <= m["release_to_motion_ms"] <= 19.0)  # |変位| ≥ 2（10 ms 遅れ + 約 2 count / 0.2）
        self.assertTrue(190 <= m["released_max_rate_100ms_count_s"] <= 210)
        self.assertEqual(m["stop_disp_after_engage"], samples[-1]["count"] - at_engage)
        self.assertTrue(-7 <= m["stop_disp_after_engage"] <= -5)
        self.assertTrue(20 <= m["engage_to_still_ms"] <= 32)
        self.assertTrue(28 <= m["engage_to_half_speed_ms"] <= 40)



# ---------------------------------------------------------------------------
# 区間の流れ（run_session）を、メッセージの層で答える偽の MCU で確かめる（実機・シリアルは使わない）
# ---------------------------------------------------------------------------
class FakeMcu:
    def __init__(self, name: str, events: list, results=(), settle_polls: int = 3, power: bool = False,
                 session_drops: bool = False):
        self.name = name
        self.events = events
        self.results = list(results)
        self.settle_polls = settle_polls
        self.power = power
        self.session = False
        self.polls = 0
        self.session_drops = session_drops

    def handle(self, data: bytes, link: "FakeLink") -> None:
        f = v1mini.cobs_decode(data[:-1])
        t, p = v1mini.parse_frame(f)
        if t == v1mini.DIAG_START:
            rid, mode = struct.unpack_from("<IB", p, 0)
            self.events.append((self.name, v1mini.MODE_NAMES[mode]))
            link.inbox.append(v1mini.Msg(v1mini.ACK, dict(request_id=rid, acked_type=t, result=0, reason=0)))
            if mode == v1mini.MODE_POWER_HOLD:
                self.session = True
            elif mode == v1mini.MODE_CANCEL:
                self.session = False
                link.status = dict(link.status, mcu_state=1)
            else:
                self.session = True
                res = dict(self.results.pop(0))
                res["test_id"] = rid
                link.inbox.append(v1mini.Msg(v1mini.DIAG_RESULT, res))
        elif t == v1mini.DIAG_READ:
            rid, off = struct.unpack_from("<IH", p, 0)
            flags = 0
            if off == v1mini.DIAG_READ_STATUS_ONLY:
                self.polls += 1
                if self.session and not (self.session_drops and self.polls > self.settle_polls):
                    flags = v1mini.SESSION_ACTIVE | (v1mini.SESSION_POWER_CMD if self.power else 0)
                    if self.polls >= self.settle_polls:
                        flags |= v1mini.SESSION_SETTLED
                self.events.append((self.name, "status", flags))
            link.inbox.append(v1mini.Msg(v1mini.DIAG_DATA, dict(request_id=rid, test_id=0, offset=off, total=0, n=0,
                                                                  session_flags=flags, session_ms=0, samples=[])))


class FakeLink(diag_brake.Link):
    def __init__(self, name: str, mcu: FakeMcu, actual: int = 2778):  # noqa: super().__init__ はポートを開くので呼ばない
        self.name = name
        self.port = "/dev/null"
        self.mac = "00:00:00:00:00:00"
        self.log = lambda m: None
        self.rid = 1000
        self.inbox = []
        self.fault_events = []
        self.status = dict(mcu_state=1, actual_count=actual, flags=0)
        self.hello = dict(device_id=0)
        self.last_keepalive = time.monotonic()
        self.mcu = mcu

    def send(self, data: bytes) -> None:
        self.mcu.handle(data, self)

    def pump(self) -> None:
        pass

    def close(self) -> None:
        pass


def ok_result(start=2778, final=-3, kept=True, result=0, **extra):
    r = dict(mode=1, result=result, stop_reason=1, referenced_after=1, fault_code=0, sample_count=0,
             sample_period_us=1000, max_sample_gap_us=1000, start_count=start, t_drive_us=v1mini.TIME_NONE,
             t_engage_us=150000, disp_at_engage=-2, final_disp=final, peak_disp=final, v_at_engage_count_s=-20,
             t_settle_us=5000, disp_engage_to_settle=-1, sign=1, flags=v1mini.RESULT_FLAG_SESSION_KEPT if kept else 0,
             adc_first=0, drive_pwm_permille=0, session_ms_at_release=5003)
    r.update(extra)
    return r


AX = dict(soft_min_count=300, soft_max_count=5253)


def session_args(**kw):
    a = dict(repeat=1, power_hold_ms=120000, settle_timeout=3.0, result_timeout=1.0, min_room=2000)
    a.update(kw)
    return types.SimpleNamespace(**a)


class Session(unittest.TestCase):
    def run_session(self, target, power, args):
        got = []
        rc = diag_brake.run_session(target, power, AX, v1mini.MODE_SAG, dict(t_max_ms=150, d_max_count=60), args,
                                    lambda m: None, lambda i, res, samples: got.append(res))
        return rc, got

    def test_power_holder_settled_before_target_start(self):
        ev = []
        power = FakeLink("ax1", FakeMcu("ax1", ev, power=True, settle_polls=3))
        target = FakeLink("ax2", FakeMcu("ax2", ev, results=[ok_result(), ok_result()]))
        rc, got = self.run_session(target, power, session_args(repeat=2))
        self.assertEqual(rc, 0)
        self.assertEqual(len(got), 2)
        first_settled = next(i for i, e in enumerate(ev)
                             if e[0] == "ax1" and e[1] == "status" and e[2] & v1mini.SESSION_SETTLED)
        first_start = ev.index(("ax2", "SAG"))
        self.assertLess(ev.index(("ax1", "POWER_HOLD")), first_settled)
        self.assertLess(first_settled, first_start)  # #1 が SETTLED を名乗る前に #2 を始めない
        # 計測ごとに POWER_HOLD を延ばし、SETTLED を見直してから始める
        starts = [i for i, e in enumerate(ev) if e == ("ax2", "SAG")]
        for i in starts:
            before = ev[:i]
            self.assertEqual(before[-2], ("ax1", "POWER_HOLD"))
            self.assertEqual(before[-1][:2], ("ax1", "status"))
            self.assertTrue(before[-1][2] & v1mini.SESSION_SETTLED)

    def test_power_session_lost_stops_before_start(self):
        ev = []
        power = FakeLink("ax1", FakeMcu("ax1", ev, power=True, settle_polls=1, session_drops=True))
        target = FakeLink("ax2", FakeMcu("ax2", ev, results=[ok_result(), ok_result()]))
        with self.assertRaises(diag_brake.ToolError):
            self.run_session(target, power, session_args(repeat=2))
        self.assertNotIn(("ax2", "SAG"), ev)

    def test_power_never_settles_times_out(self):
        ev = []
        power = FakeLink("ax1", FakeMcu("ax1", ev, power=True, settle_polls=10 ** 6))
        target = FakeLink("ax2", FakeMcu("ax2", ev, results=[ok_result()]))
        with self.assertRaises(diag_brake.ToolError):
            self.run_session(target, power, session_args(settle_timeout=0.5))
        self.assertNotIn(("ax2", "SAG"), ev)

    def test_stops_on_unsuccessful_result(self):
        ev = []
        target = FakeLink("ax1", FakeMcu("ax1", ev, results=[ok_result(), ok_result(result=3, stop_reason=13),
                                                             ok_result(), ok_result()]))
        rc, got = self.run_session(target, None, session_args(repeat=4))
        self.assertEqual(rc, 2)
        self.assertEqual(len(got), 2)
        self.assertEqual(ev.count(("ax1", "SAG")), 2)

    def test_stops_when_session_not_kept(self):
        ev = []
        target = FakeLink("ax1", FakeMcu("ax1", ev, results=[ok_result(kept=False), ok_result()]))
        rc, got = self.run_session(target, None, session_args(repeat=2))
        self.assertEqual(rc, 2)
        self.assertEqual(len(got), 1)

    def test_min_room_checked_before_each_run(self):
        ev = []
        # 1 回目の後に 600 count ずれて、ソフトリミット（5253）まで 2000 を割った
        target = FakeLink("ax1", FakeMcu("ax1", ev, results=[ok_result(start=2778, final=500), ok_result()]))
        with self.assertRaises(diag_brake.ToolError):
            self.run_session(target, None, session_args(repeat=2))
        self.assertEqual(ev.count(("ax1", "SAG")), 1)
        # 最初から近い
        near = FakeLink("ax1", FakeMcu("ax1", [], results=[ok_result()]), actual=4000)
        with self.assertRaises(diag_brake.ToolError):
            self.run_session(near, None, session_args())

    def test_warnings(self):
        self.assertEqual(diag_brake.run_warnings(ok_result()), [])
        w = diag_brake.run_warnings(ok_result(max_sample_gap_us=2600))
        self.assertTrue(any("max_sample_gap_us" in x for x in w))
        w = diag_brake.run_warnings(ok_result(session_ms_at_release=2500))
        self.assertTrue(any("電源待ち" in x for x in w))
        w = diag_brake.run_warnings(ok_result(result=3, flags=v1mini.RESULT_FLAG_POST_MARGIN))
        self.assertTrue(any("端の余裕" in x for x in w))
        self.assertIn("slide=-1", diag_brake.summary_line(1, "sag", "x", ok_result(), {}, 0))


if __name__ == "__main__":
    unittest.main()
