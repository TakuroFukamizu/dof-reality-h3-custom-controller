"""analyze.py を合成の記録で確かめる。実機・シリアルは使わない。

    ~/.platformio/penv/bin/python -m unittest discover -s moter-controler/tools/diag_brake -p 'test_*.py'

合成の記録は diag_brake.py の run() と同じ書き方（CSV の列・書式、JSON の summary のキー）で書く（write_run）。
host_metrics は diag_brake.host_metrics をそのまま呼んで埋める（道具と同じ値になる）。
"""

from __future__ import annotations

import csv
import io
import json
import os
import shutil
import tempfile
import unittest
from contextlib import redirect_stdout

import analyze
import diag_brake
import v1mini as v1

C0 = 2778


# ---------------------------------------------------------------------------
# 合成の記録
# ---------------------------------------------------------------------------
def settle_index(samples: list[dict], ie: int) -> int | None:
    """ファームの settle_index と同じ（作動指令の点から 20 ms の間 ±1 count）"""
    for i in range(ie, len(samples)):
        if samples[-1]["t_us"] - samples[i]["t_us"] < 20000:
            return None
        if all(abs(samples[j]["count"] - samples[i]["count"]) <= 1
               for j in range(i + 1, len(samples)) if samples[j]["t_us"] - samples[i]["t_us"] <= 20000):
            return i
    return None


def make_result(samples: list[dict], mode: int, te_us: int | None, td_us: int | None = None, stop: int = 1,
                result: int = 0, fault: int = 0, flags_extra: int = 0, v_engage: int = 0, drive_pwm: int = 0,
                sample_count: int | None = None) -> dict:
    """DIAG_RESULT（v1mini.decode が返す dict と同じキー）をファームの finish() と同じ規則で作る"""
    c0 = samples[0]["count"]
    gaps = [samples[i + 1]["t_us"] - samples[i]["t_us"] for i in range(len(samples) - 1)]
    peak = 0
    for s in samples:
        if abs(s["count"] - c0) > abs(peak):
            peak = s["count"] - c0
    flags = flags_extra
    t_settle = v1.TIME_NONE
    d_settle = 0
    disp_at_engage = 0
    if te_us is not None:
        ie = next(i for i, s in enumerate(samples) if s["t_us"] >= te_us)
        disp_at_engage = samples[ie]["count"] - c0
        si = settle_index(samples, ie)
        if si is None:
            flags |= v1.RESULT_FLAG_NOT_SETTLED
        else:
            t_settle = samples[si]["t_us"] - samples[ie]["t_us"]
            d_settle = samples[si]["count"] - samples[ie]["count"]
    return dict(test_id=1234, boot_id=99, mode=mode, result=result, stop_reason=stop, referenced_after=1, fault_code=fault,
                sample_count=len(samples) if sample_count is None else sample_count, sample_period_us=1000,
                max_sample_gap_us=max(gaps) if gaps else 0, start_count=c0,
                t_drive_us=v1.TIME_NONE if td_us is None else td_us,
                t_engage_us=v1.TIME_NONE if te_us is None else te_us, disp_at_engage=disp_at_engage,
                final_disp=samples[-1]["count"] - c0, peak_disp=peak, v_at_engage_count_s=v_engage, t_settle_us=t_settle,
                disp_engage_to_settle=d_settle, sign=1, flags=flags, adc_first=samples[0]["adc_raw"],
                drive_pwm_permille=drive_pwm)


def sag_samples(te_ms: int, rate: float, slip: int = 0, settle_ms: int = 10, post_ms: int = 400, c0: int = C0,
                gap_at_ms: int | None = None, gap_us: int = 0, end_creep: int = 0) -> list[dict]:
    """PWM 0 で解除（t = 0）→ rate [count/s] で沈む → te_ms で作動 → settle_ms の間に slip だけ滑って止まる"""
    out = []
    t = 0
    sgn = 1 if rate >= 0 else -1
    while t <= (te_ms + post_ms) * 1000:
        ms = t / 1000.0
        if ms < te_ms:  # 作動指令の周期の点は brake_release = 0（ファームの engage() は record() の前に解除指令を落とす）
            c = c0 + int(rate * ms / 1000.0)
            fl = v1.SAMPLE_FLAG_BRAKE_RELEASE | v1.SAMPLE_FLAG_POWER_ON
            adc = 1800
        else:
            base = c0 + int(rate * te_ms / 1000.0)
            k = min(1.0, (ms - te_ms) / settle_ms) if settle_ms else 1.0
            c = base + sgn * int(round(slip * k))
            if end_creep and ms > te_ms + post_ms - 100:
                c += sgn * int((ms - (te_ms + post_ms - 100)) * end_creep / 100)
            fl = v1.SAMPLE_FLAG_POWER_ON
            adc = 1700
        out.append(dict(t_us=t, count=c, pwm_permille=0, adc_raw=adc, flags=fl))
        t += 1000
        if gap_at_ms is not None and t == gap_at_ms * 1000:
            t += gap_us
    return out


def engage_samples(delay_ms: int, dir_: int, duty: int, v_engage: float, accel_ms: int, slip: int, settle_ms: int,
                   post_ms: int = 500, c0: int = C0, never_still: bool = False) -> tuple[list[dict], int, int]:
    """解除（t = 0）→ delay_ms 待つ（動かない）→ 走行（accel_ms で v_engage まで等加速）→ 作動 → settle_ms で slip 滑る"""
    out = []
    te = delay_ms + accel_ms
    t = 0
    pos = float(c0)
    while t <= (te + post_ms) * 1000:
        ms = t / 1000.0
        if ms < delay_ms:
            pwm, fl, adc = 0, v1.SAMPLE_FLAG_BRAKE_RELEASE | v1.SAMPLE_FLAG_POWER_ON, 1800
            pos = c0
        elif ms < te:
            tr = (ms - delay_ms) / 1000.0
            a = v_engage / (accel_ms / 1000.0)
            pos = c0 + dir_ * 0.5 * a * tr * tr
            pwm, fl, adc = dir_ * duty, v1.SAMPLE_FLAG_BRAKE_RELEASE | v1.SAMPLE_FLAG_POWER_ON, 2100
        else:
            run_disp = 0.5 * v_engage * accel_ms / 1000.0
            k = min(1.0, (ms - te) / settle_ms)
            pos = c0 + dir_ * (run_disp + slip * k)
            if never_still:
                pos += dir_ * (ms - te) * 0.2  # 200 count/s で這い続ける
            pwm, fl, adc = 0, v1.SAMPLE_FLAG_POWER_ON, 1700
        out.append(dict(t_us=t, count=int(pos), pwm_permille=pwm, adc_raw=adc, flags=fl))
        t += 1000
    return out, delay_ms * 1000, te * 1000


def write_run(d: str, stamp: str, axis: int, mode: str, label: str, samples: list[dict], res: dict, params: dict,
              csv_rows: list[dict] | None = None, fault_events: list | None = None, write_csv: bool = True) -> str:
    """diag_brake.run() と同じ書き方（ファイル名・CSV の列と書式・JSON の summary のキー）"""
    base = os.path.join(d, f"{stamp}_ax{axis}_{mode}_{label}")
    with open(base + ".log", "w") as f:
        f.write("12:00:00.000 [ax1] HELLO ...\n")
    rows = samples if csv_rows is None else csv_rows
    if write_csv:
        with open(base + ".csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_us", "t_ms", "count", "disp", "pwm_permille", "brake_release", "power_on", "adc_raw"])
            for s in rows:
                w.writerow([s["t_us"], f"{s['t_us'] / 1000:.3f}", s["count"], s["count"] - res["start_count"],
                            s["pwm_permille"], int(bool(s["flags"] & v1.SAMPLE_FLAG_BRAKE_RELEASE)),
                            int(bool(s["flags"] & v1.SAMPLE_FLAG_POWER_ON)), s["adc_raw"]])
    metrics = diag_brake.host_metrics(samples, res)
    named = dict(res)
    named["mode_name"] = v1.MODE_NAMES.get(res["mode"])
    named["result_name"] = v1.RESULT_NAMES.get(res["result"])
    named["stop_reason_name"] = v1.STOP_NAMES.get(res["stop_reason"])
    named["fault_name"] = v1.FAULT_NAMES.get(res["fault_code"])
    status = dict(boot_id=99, mcu_time_us=1, mcu_state=1, selected_axis=axis, fault_code=0, flags=12, pwm_permille=0,
                  actual_count=res["start_count"], target_count=0, velocity_count_s=0, following_error_count=0)
    cfg = dict(session_epoch=5, soft_min=300, soft_max=5300, max_vel=1200, max_acc=6000, fe_limit=400, overspeed=2500,
               command_timeout_ms=400, stop_timeout_ms=1000, role_flags=v1.ROLE_POWER_CONTROLLER if axis == 1 else 0)
    summary = dict(
        when=stamp, label=label, axis=axis, port="/dev/cu.usbmodem12501", mac="1C:DB:D4:BA:8D:58",
        device_id="51311cdbd4ba8d58", boot_id=99, mode=mode, params=params, config=cfg,
        config_file="control-server/config/h3-bringup-01-live-20261102.toml",
        status_before=status, status_after=status, firmware_result=named, host_metrics=metrics,
        samples_read=len(rows), fault_events=fault_events or [],
    )
    with open(base + ".json", "w") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)
    return base


_seq = [0]


def stamp() -> str:
    _seq[0] += 1
    return f"20261103-10{_seq[0] // 60:02d}{_seq[0] % 60:02d}"


def add_sag(d, axis=1, label="N0_empty_S1", te_ms=150, rate=-120.0, slip=3, stop=1, t_max=150, d_max=60, **kw):
    res_kw = {k: kw.pop(k) for k in ("result", "fault", "flags_extra", "sample_count") if k in kw}
    s = sag_samples(te_ms, rate, slip=slip, **kw)
    res = make_result(s, v1.MODE_SAG, te_ms * 1000, stop=stop, v_engage=int(rate), **res_kw)
    return write_run(d, stamp(), axis, "sag", label, s, res, dict(t_max_ms=t_max, d_max_count=d_max)), s, res


def add_engage(d, axis=1, label="N0_empty_E1", dir_=1, duty=120, v_engage=300.0, accel_ms=100, slip=10, settle_ms=30,
               stop=4, never_still=False, **res_kw):
    s, td, te = engage_samples(50, dir_, duty, v_engage, accel_ms, slip, settle_ms, never_still=never_still)
    res = make_result(s, v1.MODE_ENGAGE, te, td_us=td, stop=stop, v_engage=int(dir_ * v_engage), drive_pwm=dir_ * duty,
                      **res_kw)
    params = dict(dir_=dir_, duty_permille=duty, d_max_count=300, v_target_count_s=int(v_engage), delay_ms=50)
    return write_run(d, stamp(), axis, "engage", label, s, res, params), s, res


class Base(unittest.TestCase):
    def setUp(self):
        self.d = tempfile.mkdtemp(prefix="brake_sag_test_")

    def tearDown(self):
        shutil.rmtree(self.d, ignore_errors=True)

    def run_analyze(self, **kw):
        return analyze.analyze([self.d], analyze.Opts(**kw))

    def group(self, rep, **key):
        gs = [g for g in rep["groups"] if all(g["key"][k] == v for k, v in key.items())]
        self.assertEqual(len(gs), 1, f"{key}: {[g['key'] for g in rep['groups']]}")
        return gs[0]

    def run_by_base(self, rep, base):
        return next(r for r in rep["runs"] if r["id"] == os.path.basename(base))


# ---------------------------------------------------------------------------
class FormatAndMetrics(Base):
    def test_fixture_matches_tool_writer(self):
        """合成の CSV が道具の書き方と同じ（列・t_ms の書式・disp）であること"""
        base, s, res = add_sag(self.d)
        with open(base + ".csv") as f:
            lines = f.read().splitlines()
        self.assertEqual(lines[0], "t_us,t_ms,count,disp,pwm_permille,brake_release,power_on,adc_raw")
        self.assertEqual(lines[1], f"0,0.000,{C0},0,0,1,1,1800")
        with open(base + ".json") as f:
            js = json.load(f)
        for k in ("when", "label", "axis", "mode", "params", "firmware_result", "host_metrics", "samples_read", "fault_events"):
            self.assertIn(k, js)
        self.assertEqual(js["firmware_result"]["stop_reason_name"], "T_MAX")

    def test_sag_metrics_agree_with_host_metrics(self):
        base, s, res = add_sag(self.d, te_ms=300, rate=-150.0, t_max=300, d_max=100, label="N0_empty_S2")
        hm = diag_brake.host_metrics(s, res)
        rep = self.run_analyze()
        m = self.run_by_base(rep, base)["metrics"]
        self.assertEqual(m["sag_rate_100ms_count_s"], hm["released_max_rate_100ms_count_s"])
        self.assertEqual(m["release_to_motion_ms"], hm["release_to_motion_ms"])
        self.assertEqual(m["engage_to_still_ms"], hm["engage_to_still_ms"])
        self.assertEqual(m["stop_disp_max_abs"], hm["max_abs_disp_after_engage"])
        self.assertEqual(m["stop_disp_final"], hm["stop_disp_after_engage"])
        self.assertEqual(m["sag_dir"], "-")
        self.assertAlmostEqual(m["sag_rate_100ms_count_s"], 150.0, delta=10.0)
        # 時刻ごとの沈み: −150 count/s で 50 ms = −7、300 ms = −45
        self.assertEqual(m["sag_disp_at_ms"], {"50": -7, "100": -15, "150": -22, "300": -45})
        self.assertEqual(m["t_settle_fw_ms"], res["t_settle_us"] / 1000)

    def test_engage_metrics_agree_with_host_metrics(self):
        base, s, res = add_engage(self.d, v_engage=600.0, slip=25, settle_ms=40, label="N0_empty_E2", duty=150)
        hm = diag_brake.host_metrics(s, res)
        rep = self.run_analyze()
        r = self.run_by_base(rep, base)
        self.assertEqual(r["excluded"], [])
        m = r["metrics"]
        self.assertEqual(m["engage_to_still_ms"], hm["engage_to_still_ms"])
        self.assertEqual(m["stop_disp_max_abs"], hm["max_abs_disp_after_engage"])
        self.assertEqual(m["v_at_engage_fw_count_s"], 600)
        self.assertAlmostEqual(m["v_at_engage_host_count_s"], 600, delta=80)  # 20 ms 窓（等加速なので窓の中央の速さ寄り）
        self.assertGreaterEqual(m["engage_to_still_ms"], 30)
        self.assertEqual(m["t_drive_ms"], 50.0)
        self.assertIsNotNone(m["motion_onset_ms"])
        self.assertGreater(m["motion_onset_ms"], 50.0)
        self.assertEqual(m["release_to_motion_ms"], m["motion_onset_ms"])  # 待ちの間は動かない合成
        vi = m["viout"]
        self.assertEqual(vi["release"]["max"], 1800)
        self.assertEqual(vi["drive"]["min"], 2100)
        self.assertEqual(vi["rest"]["median"], 1700)
        self.assertEqual(vi["adc_first"], 1800)

    def test_sag_horizons_not_reached_are_none(self):
        """S1（150 ms で作動）では 300 ms の沈みは分からない（0 にも作動後の値にもしない）"""
        for _ in range(3):
            add_sag(self.d, te_ms=150, rate=-200.0)
        rep = self.run_analyze()
        g = self.group(rep, axis=1, mode="sag", stage="S1")
        self.assertEqual(g["sag_disp_at_ms"]["150"], dict(max_abs=30, worst_signed=-30, n=3, n_not_reached=0))
        self.assertEqual(g["sag_disp_at_ms"]["300"], dict(max_abs=None, worst_signed=None, n=0, n_not_reached=3))

    def test_d_max_stop_is_curtailed_not_excluded(self):
        base, _, _ = add_sag(self.d, te_ms=80, rate=-800.0, stop=2, d_max=60)  # D_MAX で 80 ms に作動
        rep = self.run_analyze()
        r = self.run_by_base(rep, base)
        self.assertEqual(r["excluded"], [])
        self.assertTrue(any("D_MAX" in f for f in r["flags"]))
        self.assertIsNone(r["metrics"]["sag_disp_at_ms"]["100"])
        self.assertEqual(r["metrics"]["sag_disp_at_ms"]["50"], -40)
        self.assertEqual(self.group(rep, stage="S1")["sag_curtailed"], 1)


class Quality(Base):
    def test_exclusions_with_reasons(self):
        ok, _, _ = add_sag(self.d)
        fault, _, _ = add_sag(self.d, result=1, fault=12, stop=10)
        trunc, _, _ = add_sag(self.d, flags_extra=v1.RESULT_FLAG_TRUNCATED)
        short, s, res = add_sag(self.d, sample_count=2000)
        nopulse, _, _ = add_engage(self.d, stop=8, result=1, fault=5)
        reverse, _, _ = add_engage(self.d, stop=7)
        nocsv_s = sag_samples(150, -100.0)
        nocsv = write_run(self.d, stamp(), 1, "sag", "N0_empty_S1", nocsv_s, make_result(nocsv_s, 1, 150000),
                          dict(t_max_ms=150, d_max_count=60), write_csv=False)
        with open(os.path.join(self.d, "20261103-110000_ax2_sag_N0_empty_S1.log"), "w") as f:
            f.write("error\n")
        rep = self.run_analyze()
        reasons = {os.path.basename(b): self.run_by_base(rep, b)["excluded"] for b in
                   (ok, fault, trunc, short, nopulse, reverse, nocsv)}
        self.assertEqual(reasons[os.path.basename(ok)], [])
        self.assertTrue(any("result=FAULT" in x for x in reasons[os.path.basename(fault)]))
        self.assertTrue(any("TRUNCATED" in x for x in reasons[os.path.basename(trunc)]))
        self.assertTrue(any("読み出しの欠け" in x for x in reasons[os.path.basename(short)]))
        self.assertTrue(any("NO_PULSE" in x for x in reasons[os.path.basename(nopulse)]))
        self.assertTrue(any("REVERSE" in x for x in reasons[os.path.basename(reverse)]))
        self.assertTrue(any("CSV が無い" in x for x in reasons[os.path.basename(nocsv)]))
        self.assertEqual(len(rep["incomplete"]), 1)
        self.assertEqual(rep["n_included"], 1)
        md = analyze.to_markdown(rep)
        self.assertIn("除外した回", md)
        self.assertIn("20261103-110000_ax2_sag_N0_empty_S1.log", md)

    def test_gaps_flag_and_exclude(self):
        late, _, _ = add_sag(self.d, gap_at_ms=60, gap_us=2000)      # 3 ms の間隔 → 注記
        huge, _, _ = add_sag(self.d, gap_at_ms=60, gap_us=25000)     # 26 ms の間隔 → 除外
        rep = self.run_analyze()
        rl = self.run_by_base(rep, late)
        self.assertEqual(rl["excluded"], [])
        self.assertEqual(rl["late_samples"], 1)
        self.assertEqual(rl["max_sample_gap_us"], 3000)
        rh = self.run_by_base(rep, huge)
        self.assertTrue(any("記録の間隔" in x for x in rh["excluded"]))
        self.assertEqual(self.group(rep, stage="S1")["max_sample_gap_us"], 26000)

    def test_control_overrun_from_fault_events(self):
        base, _, _ = add_sag(self.d)
        with open(base + ".json") as f:
            js = json.load(f)
        js["fault_events"] = [dict(boot_id=99, mcu_time_us=5, fault_code=6, mcu_state_before=9, actual_count=C0)]
        with open(base + ".json", "w") as f:
            json.dump(js, f)
        rep = self.run_analyze()
        r = self.run_by_base(rep, base)
        self.assertTrue(r["control_overrun"])
        self.assertIn("CONTROL_OVERRUN", r["excluded"])
        self.assertEqual(self.group(rep, stage="S1")["control_overrun"], 1)

    def test_csv_row_mismatch_excluded(self):
        s = sag_samples(150, -100.0)
        res = make_result(s, 1, 150000)
        base = write_run(self.d, stamp(), 1, "sag", "N0_empty_S1", s, res, {}, csv_rows=s[:-5])
        with open(base + ".json") as f:
            js = json.load(f)
        js["samples_read"] = len(s)
        with open(base + ".json", "w") as f:
            json.dump(js, f)
        rep = self.run_analyze()
        self.assertTrue(any("CSV の行数" in x for x in self.run_by_base(rep, base)["excluded"]))


class Grouping(Base):
    def test_group_keys(self):
        add_engage(self.d, dir_=1)
        add_engage(self.d, dir_=-1)
        add_engage(self.d, axis=2, dir_=1)
        add_sag(self.d, label="P15_ballast20kg_S1")
        add_sag(self.d, label="N0_empty_S1_r2")
        weird, _, _ = add_sag(self.d, label="run")
        rep = self.run_analyze()
        self.group(rep, axis=1, mode="engage", dir=1, stage="E1")
        self.group(rep, axis=1, mode="engage", dir=-1)
        self.group(rep, axis=2, mode="engage", dir=1)
        g = self.group(rep, axis=1, position="P15", load="ballast20kg", stage="S1")
        self.assertIsNone(g["key"]["dir"])
        self.group(rep, axis=1, position="N0", load="empty", stage="S1")
        self.group(rep, position="run", load="?", stage="?")
        self.assertTrue(any("ラベル" in x for x in self.run_by_base(rep, weird)["flags"]))

    def test_parse_label(self):
        self.assertEqual(analyze.parse_label("M15_ballast_10kg_E4"),
                         dict(position="M15", load="ballast_10kg", stage="E4", label_ok=True))
        self.assertFalse(analyze.parse_label("n0_empty")["label_ok"])
        self.assertEqual(analyze.load_kind("empty"), "empty")
        self.assertEqual(analyze.load_kind("ballast20kg"), "ballast")
        self.assertEqual(analyze.load_kind("?"), "unknown")


class Bounds(Base):
    def test_bound_small_n_has_no_sigma(self):
        b = analyze.bound([100, 120, 110], 10.0, None)
        self.assertEqual(b["n"], 3)
        self.assertIsNone(b["mean_plus_3sd"])
        self.assertEqual(b["conservative"], 130.0)

    def test_bound_n5_uses_max_of_both(self):
        vals = [100, 100, 100, 100, 160]
        b = analyze.bound(vals, 10.0, 25.0)
        import statistics
        m3 = round(statistics.fmean(vals) + 3 * statistics.stdev(vals), 2)
        self.assertEqual(b["mean_plus_3sd"], m3)
        self.assertEqual(b["conservative"], max(170.0, m3))
        self.assertEqual(b["conservative_with_margin"], round(b["conservative"] * 1.25, 2))

    def test_bound_empty(self):
        self.assertIsNone(analyze.bound([], 1.0, None))
        self.assertIsNone(analyze.bound([None, None], 1.0, None))

    def test_upper_envelope_bounds_all_points(self):
        pts = [(300, 12), (310, 15), (600, 25), (620, 22), (1000, 48), (1200, 51)]
        e = analyze.upper_envelope(pts)
        for v, d in pts:
            self.assertLessEqual(d, e["a_count"] + e["b_count_per_count_s"] * v + 1e-9)
        self.assertEqual(e["v_max_covered"], 1200)
        self.assertGreaterEqual(e["a_count"], 0)
        self.assertGreaterEqual(e["b_count_per_count_s"], 0)
        self.assertIsNone(analyze.upper_envelope([(300, 10), (300, 12)]))


class Recommend(Base):
    def test_insufficient_when_empty(self):
        rep = self.run_analyze()
        for name in analyze.R10_CONSTANTS:
            self.assertEqual(rep["recommended"][name]["derived"], analyze.INSUFFICIENT)
            self.assertEqual(rep["recommended"][name]["n"], 0)
        self.assertEqual(rep["recommended"]["stop_disp_envelope"]["derived"], analyze.INSUFFICIENT)
        md = analyze.to_markdown(rep)
        self.assertIn("insufficient data", md)

    def test_backdrive_uses_down_direction_only(self):
        for r in (-100.0, -150.0, -180.0, -120.0, -130.0):
            add_sag(self.d, te_ms=300, rate=r, t_max=300, label="N0_empty_S2")
        add_sag(self.d, te_ms=300, rate=400.0, t_max=300, label="P15_empty_S2")  # 上向き（別の所見）
        add_sag(self.d, te_ms=300, rate=0.0, t_max=300, label="M15_empty_S2")    # 動かない（n に入れない）
        rep = self.run_analyze()
        e = rep["recommended"]["V1_SUP_BACKDRIVE_COUNT_S"]
        self.assertEqual(e["n"], 5)
        self.assertEqual(e["n_up"], 1)
        self.assertEqual(e["n_no_motion"], 1)
        self.assertLess(e["derived_max"], 200)
        self.assertGreaterEqual(e["derived_max"], 170)
        self.assertIsNotNone(e["bound"]["mean_plus_3sd"])
        self.assertGreaterEqual(e["up_max_count_s"], 390)
        self.assertFalse(e["exceeds_current"])
        # バラストの回が無い: 保持の包絡線は作らない
        self.assertEqual(rep["recommended"]["V1_SUP_HOLD_ENV_COUNT_S"]["derived"], analyze.INSUFFICIENT)
        # SAG だけ: 作動中の滑りの包絡線は作らない
        self.assertEqual(rep["recommended"]["stop_disp_envelope"]["derived"], analyze.INSUFFICIENT)
        # 解除待ち: 動かなかった回は n に入れない
        rw = rep["recommended"]["BRAKE_RELEASE_WAIT_MS"]
        self.assertEqual(rw["n_sag"], 6)
        self.assertEqual(rw["n_sag_no_motion"], 1)

    def test_exceeds_current_flagged(self):
        add_sag(self.d, te_ms=300, rate=-260.0, t_max=300, d_max=100, label="N0_empty_S2")
        rep = self.run_analyze()
        e = rep["recommended"]["V1_SUP_BACKDRIVE_COUNT_S"]
        self.assertTrue(e["exceeds_current"])
        self.assertIn("仮値を超えた", analyze.to_markdown(rep))

    def test_engage_ms_lower_bound_when_not_still(self):
        add_engage(self.d, slip=10, settle_ms=30)
        add_engage(self.d, never_still=True)
        rep = self.run_analyze()
        e = rep["recommended"]["V1_BRAKE_ENGAGE_MS"]
        self.assertTrue(e["derived_is_lower_bound"])
        self.assertEqual(e["n_not_still"], 1)
        # 静止しなかった回を落とさない: 記録の終わり（作動後 500 ms）が下限として最大になる
        self.assertEqual(e["n"], 2)
        self.assertEqual(e["derived_max"], 500.0)
        self.assertTrue(e["exceeds_current"])
        self.assertIn("≥", analyze.to_markdown(rep))

    def test_hold_envelope_from_ballast(self):
        add_sag(self.d, te_ms=300, rate=-90.0, t_max=300, label="N0_ballast20kg_S2")
        add_sag(self.d, te_ms=300, rate=-300.0, t_max=300, label="N0_empty_S2")  # 無負荷は使わない
        rep = self.run_analyze()
        e = rep["recommended"]["V1_SUP_HOLD_ENV_COUNT_S"]
        self.assertEqual(e["n_ballast_runs"], 1)
        self.assertLess(e["derived_max"], 100)

    def test_post_still_creep_detected(self):
        base, _, _ = add_sag(self.d, te_ms=150, rate=-100.0, end_creep=10)  # 静止の後、終わりの 100 ms で 10 count 滑る
        rep = self.run_analyze()
        m = self.run_by_base(rep, base)["metrics"]
        self.assertGreaterEqual(m["post_still_max_v_count_s"], 50)

    def test_stop_disp_envelope_from_engage(self):
        for v, slip in ((300, 8), (600, 20), (1000, 40)):
            for d in (1, -1):
                add_engage(self.d, dir_=d, v_engage=float(v), slip=slip, settle_ms=30)
        rep = self.run_analyze()
        env = rep["recommended"]["stop_disp_envelope"]
        self.assertEqual(env["n"], 6)
        for r in rep["runs"]:
            m = r["metrics"]
            self.assertLessEqual(m["stop_disp_max_abs"],
                                 env["a_count"] + env["b_count_per_count_s"] * abs(m["v_at_engage_fw_count_s"]) + 1e-9)
        eng = rep["recommended"]["V1_BRAKE_ENGAGE_MS"]
        self.assertEqual(eng["n_engage"], 6)
        self.assertEqual(eng["v_at_engage_abs_max"], 1000)


class Cli(Base):
    def test_main_writes_md_and_json(self):
        add_sag(self.d)
        add_engage(self.d)
        md = os.path.join(self.d, "out.md")
        js = os.path.join(self.d, "out.json")
        self.assertEqual(analyze.main([self.d, "--md", md, "--json", js, "--margin-pct", "20"]), 0)
        with open(js) as f:
            rep = json.load(f)
        self.assertEqual(rep["n_runs"], 2)
        self.assertEqual(rep["options"]["margin_pct"], 20.0)
        with open(md) as f:
            text = f.read()
        self.assertIn("置き換え候補", text)
        self.assertIn("V1_SUP_BACKDRIVE_COUNT_S", text)
        # 出力先を同じディレクトリに置いても、次の解析で入力として拾わない（.md / .json の summary でない JSON は除外）
        rep2 = self.run_analyze()
        self.assertEqual(rep2["n_runs"], 3)
        self.assertTrue(any("firmware_result" in "".join(r["excluded"]) for r in rep2["runs"]))

    def test_individual_files_and_stdout(self):
        base, _, _ = add_sag(self.d)
        buf = io.StringIO()
        with redirect_stdout(buf):
            self.assertEqual(analyze.main([base + ".csv"]), 0)
        self.assertIn("ブレーキ・沈みの計測のまとめ", buf.getvalue())
        self.assertIn("回数 1", buf.getvalue())


if __name__ == "__main__":
    unittest.main()
