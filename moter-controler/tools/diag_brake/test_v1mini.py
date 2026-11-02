"""v1mini.py を C++（v1_codec.h）のテストベクタと突き合わせる。実機・シリアルは使わない。

    ~/.platformio/penv/bin/python -m unittest discover -s moter-controler/tools/diag_brake -p 'test_*.py'
"""

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


if __name__ == "__main__":
    unittest.main()
