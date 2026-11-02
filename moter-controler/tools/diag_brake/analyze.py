#!/usr/bin/env python3
"""ブレーキ・沈みの計測（diag_brake.py の出力）を条件ごとにまとめ、r10 の仮値の置き換え候補を出す。実機・シリアルは使わない。

手順: moter-controler/docs/brake_sag_measurement.md、読み方・定義: moter-controler/docs/brake_sag_analysis.md

    PY=~/.platformio/penv/bin/python   # 標準ライブラリだけ（numpy 不要）
    $PY moter-controler/tools/diag_brake/analyze.py moter-controler/docs/data/brake_sag \\
        --md /tmp/brake_sag_summary.md --json /tmp/brake_sag_summary.json

入力: ディレクトリ（中の *.json）か、個別の .json / .csv / .log。1 回 = <日時>_ax<n>_<mode>_<label>.{json,csv,log}
（diag_brake.py の run() が書く。JSON の axis / mode / label を使い、ファイル名は解釈しない）。
.log だけ残っている回（道具が拒否・通信の失敗で JSON を書かなかった）は「不完全」として一覧に出す。

まとめ方の要点（詳しくは docs/brake_sag_analysis.md）:
  - 条件 = 軸 × 位置 × 荷重 × モード × 段 × 向き（ENGAGE のみ）。ラベルは「位置_荷重_段」（例 N0_empty_S1）
  - 打ち切り・FAULT・中止・記録の欠け・ENGAGE の無効な止まり方は除外し、理由を全部出す（黙って捨てない）
  - 境界は最悪側: 最大・最大 + 分解能・（n ≥ 5 なら）平均 + 3σ。余裕の掛け方は r10 の側で決める（--margin-pct は表示だけ）
  - データが無い・足りない値は作らない（"insufficient data" と出す）
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import statistics
import sys

import v1mini as v1

# ---------------------------------------------------------------------------
# 定数（ファーム・道具と同じ意味のもの）
# ---------------------------------------------------------------------------
TIME_NONE = v1.TIME_NONE
FAULT_CONTROL_OVERRUN = 6  # v1.FAULT_NAMES[6]
SAG_HORIZONS_MS = (50, 100, 150, 300)
CSV_REQUIRED = ("t_us", "count", "pwm_permille", "brake_release", "adc_raw")
# ENGAGE で、作動の解析に使わない止まり方（走り切っていない・荷重に負けた・故障の疑い）
ENGAGE_EXCLUDE_STOPS = {"NO_PULSE", "REVERSE", "SAG_GUARD", "OVERSPEED", "CANCEL", "FAULT", "NONE"}
SAG_CURTAILED_STOPS = {"D_MAX", "END_MARGIN"}  # 沈みの途中で作動させた（その後の時刻の沈みは分からない）
EMPTY_LOADS = {"empty", "noload", "none", "0kg"}

# r10 の仮値（origin/hw-r10-integrated-test:moter-controler/include/config.h、2026-11-02 時点）
R10_CONSTANTS = {
    "V1_SUP_BACKDRIVE_COUNT_S": dict(current=200, unit="count/s", meaning="受動的な沈み（ブレーキ解除・PWM 0、下向き = カウント減）"),
    "V1_BRAKE_ENGAGE_MS": dict(current=150, unit="ms", meaning="ブレーキ作動指令から掴むまで"),
    "BRAKE_RELEASE_WAIT_MS": dict(current=300, unit="ms", meaning="ブレーキ解除指令から離れるまで（ARMING の待ち）"),
    "V1_SUP_HOLD_ENV_COUNT_S": dict(current=150, unit="count/s", meaning="保持の包絡線（バラスト条件の最悪）"),
}

INSUFFICIENT = "insufficient data"


class Opts:
    def __init__(self, still_thr: float = 50.0, still_window_ms: float = 20.0, motion_count: int = 2,
                 gap_exclude_us: int = 20000, margin_pct: float | None = None):
        self.still_thr = still_thr
        self.still_window_us = int(round(still_window_ms * 1000))
        self.motion_count = motion_count
        self.gap_exclude_us = gap_exclude_us
        self.margin_pct = margin_pct

    @property
    def still_band(self) -> int:
        # 窓の中の変位の許容。50 count/s × 20 ms = 1 count（ファームの t_settle と同じ ±1 count / 20 ms）
        return max(1, int(math.floor(self.still_thr * self.still_window_us / 1e6 + 1e-9)))


# ---------------------------------------------------------------------------
# 入力
# ---------------------------------------------------------------------------
def discover(paths: list[str]) -> tuple[list[str], list[str]]:
    """(JSON の一覧, JSON の無い .log の一覧)"""
    bases: dict[str, set[str]] = {}
    for p in paths:
        if os.path.isdir(p):
            for name in sorted(os.listdir(p)):
                stem, ext = os.path.splitext(name)
                if ext in (".json", ".csv", ".log"):
                    bases.setdefault(os.path.join(p, stem), set()).add(ext)
        else:
            stem, ext = os.path.splitext(p)
            if ext not in (".json", ".csv", ".log"):
                raise SystemExit(f"error: {p}: .json / .csv / .log かディレクトリを渡す")
            have = {e for e in (".json", ".csv", ".log") if os.path.exists(stem + e)}
            if not have:
                raise SystemExit(f"error: {p} が無い")
            bases.setdefault(stem, set()).update(have)
    jsons = sorted(b + ".json" for b, e in bases.items() if ".json" in e)
    logs = sorted(b + ".log" for b, e in bases.items() if ".json" not in e and ".log" in e)
    orphan_csv = sorted(b + ".csv" for b, e in bases.items() if ".json" not in e and ".log" not in e)
    return jsons, logs + orphan_csv


LABEL_POS = re.compile(r"^(N0|[PM]\d+)$")
LABEL_STAGE = re.compile(r"^[SE]\d+$")
LABEL_REPEAT = re.compile(r"^(r?\d+)$")


def parse_label(label: str) -> dict:
    """「位置_荷重_段」。末尾に回数（_r2 / _2）が付いていてもよい。合わなければ label そのものを条件にする"""
    toks = label.split("_")
    if len(toks) >= 4 and LABEL_REPEAT.match(toks[-1]) and LABEL_STAGE.match(toks[-2]):
        toks = toks[:-1]
    if len(toks) >= 3 and LABEL_POS.match(toks[0]) and LABEL_STAGE.match(toks[-1]):
        return dict(position=toks[0], load="_".join(toks[1:-1]), stage=toks[-1], label_ok=True)
    return dict(position=label, load="?", stage="?", label_ok=False)


def load_kind(load: str) -> str:
    if load == "?":
        return "unknown"
    return "empty" if load.lower() in EMPTY_LOADS else "ballast"


def read_csv(path: str) -> list[dict]:
    with open(path, newline="") as f:
        r = csv.DictReader(f)
        cols = r.fieldnames or []
        missing = [c for c in CSV_REQUIRED if c not in cols]
        if missing:
            raise ValueError(f"CSV の列が無い: {missing}")
        out = []
        for row in r:
            out.append(dict(t_us=int(row["t_us"]), count=int(row["count"]), pwm=int(row["pwm_permille"]),
                            brake=int(row["brake_release"]), adc=int(row["adc_raw"]),
                            power=int(row["power_on"]) if row.get("power_on") not in (None, "") else None))
    return out


def tnone(v) -> int | None:
    return None if v is None or v == TIME_NONE else int(v)


# ---------------------------------------------------------------------------
# 1 回の解析
# ---------------------------------------------------------------------------
def vel20(samples: list[dict], i: int, window_us: int) -> float | None:
    """ファームの velocity() と同じ: 点 i と、window 以上前の最も新しい点（無ければ最初の点）の差 [count/s]。履歴 10 ms 未満は None"""
    if i <= 0:
        return None
    k = i - 1
    while k > 0 and samples[i]["t_us"] - samples[k]["t_us"] < window_us:
        k -= 1
    dt = samples[i]["t_us"] - samples[k]["t_us"]
    if dt < 10_000:
        return None
    return (samples[i]["count"] - samples[k]["count"]) * 1e6 / dt


def released_max_rate(samples: list[dict]) -> tuple[float | None, float | None]:
    """解除中（brake_release = 1）の最悪の 100 ms 窓の速さ（短ければその長さの窓）。diag_brake.host_metrics と同じ定義"""
    rel = [s for s in samples if s["brake"]]
    if len(rel) < 2:
        return None, None
    span = rel[-1]["t_us"] - rel[0]["t_us"]
    w = min(100_000, span)
    best = 0.0
    j = 0
    for i, si in enumerate(rel):
        j = max(j, i)
        while j + 1 < len(rel) and rel[j + 1]["t_us"] - si["t_us"] <= w:
            j += 1
        dtw = rel[j]["t_us"] - si["t_us"]
        if dtw >= w - 1000 and dtw > 0:
            best = max(best, abs(rel[j]["count"] - si["count"]) * 1e6 / dtw)
    return round(best, 1), w / 1000.0


def still_index(samples: list[dict], start: int, opts: Opts) -> int | None:
    """start 以降で、その点から window の間 ±band に収まり始めた最初の点（窓が記録の終わりを越えたら None）"""
    band = opts.still_band
    w = opts.still_window_us
    for i in range(start, len(samples)):
        if samples[-1]["t_us"] - samples[i]["t_us"] < w:
            return None
        ok = True
        for j in range(i + 1, len(samples)):
            if samples[j]["t_us"] - samples[i]["t_us"] > w:
                break
            if abs(samples[j]["count"] - samples[i]["count"]) > band:
                ok = False
                break
        if ok:
            return i
    return None


def adc_stats(vals: list[int]) -> dict | None:
    if not vals:
        return None
    return dict(n=len(vals), min=min(vals), max=max(vals), mean=round(statistics.fmean(vals), 1),
                median=statistics.median(vals))


def sign_name(d: int) -> str:
    return "+" if d > 0 else "-" if d < 0 else "0"


def analyze_run(json_path: str, opts: Opts) -> dict:
    base = json_path[:-5]
    run: dict = dict(id=os.path.basename(base), json=json_path, csv=base + ".csv", excluded=[], flags=[], metrics={})
    try:
        with open(json_path) as f:
            js = json.load(f)
    except (OSError, ValueError) as e:
        run["excluded"].append(f"JSON を読めない: {e}")
        return run
    fr = js.get("firmware_result")
    if not isinstance(fr, dict) or "mode" not in js or "axis" not in js:
        run["excluded"].append("JSON に firmware_result / mode / axis が無い（diag_brake.py の summary ではない）")
        return run
    label = str(js.get("label", "run"))
    run.update(label=label, axis=js["axis"], mode=js["mode"], when=js.get("when"), **parse_label(label))
    run["load_kind"] = load_kind(run["load"])
    params = js.get("params") or {}
    run["params"] = params
    run["dir"] = None
    if run["mode"] == "engage":
        d = params.get("dir_", params.get("dir"))
        run["dir"] = int(d) if d is not None else None
        dp = fr.get("drive_pwm_permille", 0)
        if run["dir"] is not None and dp and (dp > 0) != (run["dir"] > 0):
            run["flags"].append(f"params.dir_={run['dir']} と drive_pwm_permille={dp} の向きが違う")
    if not run["label_ok"]:
        run["flags"].append("ラベルが「位置_荷重_段」でない（ラベルそのものを条件にした）")
    elif (run["stage"][0] == "S") != (run["mode"] == "sag"):
        run["flags"].append(f"段 {run['stage']} とモード {run['mode']} が合わない")

    stop = fr.get("stop_reason_name") or v1.STOP_NAMES.get(fr.get("stop_reason"), str(fr.get("stop_reason")))
    result = fr.get("result_name") or v1.RESULT_NAMES.get(fr.get("result"), str(fr.get("result")))
    run["stop_reason"] = stop
    run["result"] = result
    flags = int(fr.get("flags", 0))
    period = int(fr.get("sample_period_us") or 1000)
    te = tnone(fr.get("t_engage_us"))
    run["t_engage_ms"] = None if te is None else te / 1000.0

    # --- 記録の品質（除外） ---
    if fr.get("result") != 0:
        run["excluded"].append(f"result={result}（stop={stop}, fault={fr.get('fault_name')}）")
    if flags & v1.RESULT_FLAG_TRUNCATED:
        run["excluded"].append("TRUNCATED（記録が 1024 点を超えた）")
    if js.get("samples_read", 0) < fr.get("sample_count", 0):
        run["excluded"].append(f"読み出しの欠け（samples_read {js.get('samples_read')} < sample_count {fr.get('sample_count')}）")
    if run["mode"] == "engage" and stop in ENGAGE_EXCLUDE_STOPS:
        run["excluded"].append(f"ENGAGE の止まり方 {stop}（作動の解析に使えない）")
    if te is None:
        run["excluded"].append("ブレーキ作動指令の時刻が無い（t_engage_us = NONE）")
    overrun_fw = fr.get("fault_code") == FAULT_CONTROL_OVERRUN or any(
        e.get("fault_code") == FAULT_CONTROL_OVERRUN for e in js.get("fault_events") or [])
    run["control_overrun"] = overrun_fw
    if overrun_fw:
        run["excluded"].append("CONTROL_OVERRUN")
    if js.get("fault_events"):
        run["flags"].append(f"FAULT_EVENT {len(js['fault_events'])} 件")

    samples: list[dict] = []
    if not os.path.exists(run["csv"]):
        run["excluded"].append("CSV が無い")
    else:
        try:
            samples = read_csv(run["csv"])
        except (OSError, ValueError, KeyError) as e:
            run["excluded"].append(f"CSV を読めない: {e}")
    if samples:
        if len(samples) != js.get("samples_read", len(samples)):
            run["excluded"].append(f"CSV の行数 {len(samples)} ≠ samples_read {js.get('samples_read')}")
        if any(samples[i + 1]["t_us"] <= samples[i]["t_us"] for i in range(len(samples) - 1)):
            run["excluded"].append("t_us が単調増加でない")
        if samples[0]["t_us"] != 0 or not samples[0]["brake"]:
            run["flags"].append("最初の点が解除指令（t = 0, brake_release = 1）でない")

    # --- 記録の間隔 ---
    gaps = [samples[i + 1]["t_us"] - samples[i]["t_us"] for i in range(len(samples) - 1)]
    host_gap = max(gaps) if gaps else 0
    fw_gap = int(fr.get("max_sample_gap_us") or 0)
    run["max_sample_gap_us"] = max(host_gap, fw_gap)
    run["late_samples"] = sum(1 for g in gaps if g > 2 * period)  # 公称周期の 2 倍を超えた間隔（制御タスクの遅れ）の数
    if run["late_samples"]:
        run["flags"].append(f"間隔 > {2 * period} us が {run['late_samples']} 回（最大 {run['max_sample_gap_us']} us）")
    if run["max_sample_gap_us"] > opts.gap_exclude_us:
        run["excluded"].append(f"記録の間隔 {run['max_sample_gap_us']} us > {opts.gap_exclude_us} us（20 ms 窓の値が出せない）")
    if flags & v1.RESULT_FLAG_NOT_SETTLED:
        run["flags"].append("NOT_SETTLED（記録の終わりまでに静止しなかった。作動時間・滑りは下限）")
    if run["mode"] == "sag" and stop in SAG_CURTAILED_STOPS:
        run["flags"].append(f"沈みの途中で作動（{stop}）")

    if not samples or te is None or any(e.startswith("CSV") or e.startswith("t_us") for e in run["excluded"]):
        return run
    run["metrics"] = compute_metrics(samples, fr, run["mode"], te, opts)
    return run


def compute_metrics(samples: list[dict], fr: dict, mode: str, te: int, opts: Opts) -> dict:
    m: dict = {}
    c0 = int(fr.get("start_count", samples[0]["count"]))
    w = opts.still_window_us

    # --- 解除中（作動指令の点まで。作動指令の点の count は指令の瞬間の測定値） ---
    upto = [s for s in samples if s["t_us"] <= te]
    t_motion = next((s["t_us"] for s in upto if abs(s["count"] - c0) >= opts.motion_count), None)
    m["release_to_motion_ms"] = None if t_motion is None else t_motion / 1000.0
    if mode == "sag":
        rate, win = released_max_rate(samples)
        m["sag_rate_100ms_count_s"] = rate
        m["sag_rate_window_ms"] = win
        d_eng = int(fr.get("disp_at_engage", upto[-1]["count"] - c0 if upto else 0))
        m["sag_disp_at_engage"] = d_eng
        m["sag_dir"] = sign_name(d_eng) if abs(d_eng) >= opts.motion_count else "0"
        m["sag_mean_rate_count_s"] = round(abs(d_eng) * 1e6 / te, 1) if te > 0 else None
        disp_at = {}
        for T in SAG_HORIZONS_MS:
            if te >= T * 1000:
                last = [s for s in upto if s["t_us"] <= T * 1000]
                disp_at[str(T)] = last[-1]["count"] - c0 if last else None
            else:
                disp_at[str(T)] = None  # その時刻にはもう作動していた（分からない。0 ではない）
        m["sag_disp_at_ms"] = disp_at
        # 加速の目安: 解除の後半の沈みが前半の 1.5 倍を超え、かつ 3 count 以上
        half = te // 2
        c_half = next((s["count"] for s in reversed(upto) if s["t_us"] <= half), c0)
        first = abs(c_half - c0)
        second = abs(d_eng - (c_half - c0))
        m["sag_accelerating"] = bool(second >= 3 and second > 1.5 * first)

    # --- ENGAGE: 走り始めから動き出しまで（解除待ちの補い: 動き出しの時刻はブレーキが離れた時刻以上） ---
    if mode == "engage":
        td = tnone(fr.get("t_drive_us"))
        m["t_drive_ms"] = None if td is None else td / 1000.0
        m["motion_onset_ms"] = None
        if td is not None:
            idr = next((i for i, s in enumerate(samples) if s["t_us"] >= td), None)
            if idr is not None:
                cd = samples[idr]["count"]
                mo = next((s["t_us"] for s in samples[idr:] if s["t_us"] <= te and abs(s["count"] - cd) >= opts.motion_count), None)
                if mo is not None:
                    m["motion_onset_ms"] = mo / 1000.0
                    m["drive_to_motion_ms"] = (mo - td) / 1000.0

    # --- 作動 ---
    ie = next((i for i, s in enumerate(samples) if s["t_us"] >= te), None)
    if ie is None:
        return m
    ce = samples[ie]["count"]
    m["v_at_engage_fw_count_s"] = fr.get("v_at_engage_count_s")
    v_host = vel20(samples, ie, w)
    m["v_at_engage_host_count_s"] = None if v_host is None else round(v_host, 1)
    m["stop_disp_final"] = int(fr.get("final_disp", samples[-1]["count"] - c0)) - int(fr.get("disp_at_engage", ce - c0))
    m["stop_disp_max_abs"] = max(abs(s["count"] - ce) for s in samples[ie:])
    si = still_index(samples, ie, opts)
    m["engage_to_still_ms"] = None if si is None else (samples[si]["t_us"] - te) / 1000.0
    m["engage_still_censored_ms"] = (samples[-1]["t_us"] - te) / 1000.0 if si is None else None  # 静止しなかった: 少なくともこれ以上
    ts = tnone(fr.get("t_settle_us"))
    m["t_settle_fw_ms"] = None if ts is None else ts / 1000.0
    cands = [x for x in (m["engage_to_still_ms"], m["t_settle_fw_ms"]) if x is not None]
    m["engage_ms_worst"] = max(cands) if cands else None
    # 境界に使う値: 静止しなかった回は「記録の終わりまで」を下限として入れる（最悪の回を落とさない）
    m["engage_ms_is_lower_bound"] = m["engage_ms_worst"] is None
    m["engage_ms_for_bound"] = m["engage_ms_worst"] if m["engage_ms_worst"] is not None else m["engage_still_censored_ms"]
    # 静止した後の這い（ブレーキが掴んだ後に荷重で滑るか）: 静止の窓の後の 20 ms 窓の最大 |v|
    m["post_still_max_v_count_s"] = None
    if si is not None:
        t_after = samples[si]["t_us"] + w
        vs = [abs(v) for k in range(si, len(samples)) if samples[k]["t_us"] >= t_after
              for v in [vel20(samples, k, w)] if v is not None]
        m["post_still_max_v_count_s"] = round(max(vs), 1) if vs else None

    # --- VIOUT の生値（区間ごと） ---
    t_still = samples[si]["t_us"] if si is not None else None
    ph: dict[str, list[int]] = {"release": [], "drive": [], "brake": [], "rest": []}
    for s in samples:
        if s["pwm"] != 0:
            ph["drive"].append(s["adc"])
        elif s["brake"]:
            ph["release"].append(s["adc"])
        elif t_still is not None and s["t_us"] >= t_still:
            ph["rest"].append(s["adc"])
        else:
            ph["brake"].append(s["adc"])
    m["viout"] = {k: adc_stats(v) for k, v in ph.items()}
    m["viout"]["adc_first"] = fr.get("adc_first")
    return m


# ---------------------------------------------------------------------------
# まとめ
# ---------------------------------------------------------------------------
def bound(values: list[float], resolution: float, margin_pct: float | None) -> dict | None:
    """最悪側の境界。max・max + 分解能・（n ≥ 5）平均 + 3σ（標本標準偏差）。conservative = その大きい方"""
    vals = [float(v) for v in values if v is not None]
    if not vals:
        return None
    b: dict = dict(n=len(vals), max=max(vals), min=min(vals), mean=round(statistics.fmean(vals), 2))
    b["max_plus_resolution"] = round(b["max"] + resolution, 2)
    b["resolution"] = resolution
    b["mean_plus_3sd"] = None
    if len(vals) >= 2:
        b["sd"] = round(statistics.stdev(vals), 2)
    if len(vals) >= 5:
        b["mean_plus_3sd"] = round(b["mean"] + 3 * statistics.stdev(vals), 2)
    b["conservative"] = max(x for x in (b["max_plus_resolution"], b["mean_plus_3sd"]) if x is not None)
    if margin_pct is not None:
        b["with_margin_pct"] = margin_pct
        b["conservative_with_margin"] = round(b["conservative"] * (1 + margin_pct / 100.0), 2)
    return b


def group_key(r: dict) -> tuple:
    return (r["axis"], r["position"], r["load"], r["mode"], r["stage"], r["dir"])


def key_dict(k: tuple) -> dict:
    return dict(axis=k[0], position=k[1], load=k[2], mode=k[3], stage=k[4], dir=k[5])


def rate_resolution(win_ms: float | None) -> float:
    # 1 count / 窓（100 ms 窓なら 10 count/s）
    return round(1000.0 / win_ms, 1) if win_ms else 10.0


def summarize_group(runs: list[dict], opts: Opts) -> dict:
    inc = [r for r in runs if not r["excluded"] and r["metrics"]]
    g: dict = dict(n_runs=len(runs), n_included=len(inc), n_excluded=len(runs) - len(inc),
                   runs=[r["id"] for r in runs], excluded=[r["id"] for r in runs if r not in inc])
    g["max_sample_gap_us"] = max((r.get("max_sample_gap_us", 0) for r in runs), default=0)
    g["late_samples"] = sum(r.get("late_samples", 0) for r in runs)
    g["control_overrun"] = sum(1 for r in runs if r.get("control_overrun"))
    if not inc:
        return g
    ms = [r["metrics"] for r in inc]
    mode = inc[0]["mode"]
    if mode == "sag":
        disp = {}
        for T in SAG_HORIZONS_MS:
            vals = [m["sag_disp_at_ms"][str(T)] for m in ms if m["sag_disp_at_ms"].get(str(T)) is not None]
            worst = max(vals, key=abs) if vals else None
            disp[str(T)] = dict(max_abs=None if worst is None else abs(worst), worst_signed=worst, n=len(vals),
                                n_not_reached=len(ms) - len(vals))
        g["sag_disp_at_ms"] = disp
        rates = [m["sag_rate_100ms_count_s"] for m in ms if m["sag_rate_100ms_count_s"] is not None]
        win = min((m["sag_rate_window_ms"] for m in ms if m.get("sag_rate_window_ms")), default=None)
        g["sag_rate_100ms"] = bound(rates, rate_resolution(win), opts.margin_pct)
        g["sag_rate_window_ms_min"] = win
        g["sag_dir_counts"] = {d: sum(1 for m in ms if m["sag_dir"] == d) for d in ("+", "-", "0")}
        g["sag_accelerating"] = sum(1 for m in ms if m["sag_accelerating"])
        g["sag_curtailed"] = sum(1 for r in inc if r["stop_reason"] in SAG_CURTAILED_STOPS)
    rtm = [m["release_to_motion_ms"] for m in ms if m.get("release_to_motion_ms") is not None]
    g["release_to_motion_ms"] = bound(rtm, 1.0, opts.margin_pct)
    g["release_no_motion"] = sum(1 for m in ms if m.get("release_to_motion_ms") is None)
    if mode == "engage":
        g["motion_onset_ms"] = bound([m.get("motion_onset_ms") for m in ms], 1.0, opts.margin_pct)
    eng = [m["engage_ms_for_bound"] for m in ms if m.get("engage_ms_for_bound") is not None]
    g["engage_ms"] = bound(eng, 1.0, opts.margin_pct)  # 静止しなかった回は下限の値で入る（engage_not_still > 0 なら max は下限）
    cens = [m["engage_still_censored_ms"] for m in ms if m.get("engage_still_censored_ms") is not None]
    g["engage_not_still"] = len(cens)
    g["engage_not_still_min_ms"] = min(cens) if cens else None
    va = [abs(m["v_at_engage_fw_count_s"]) for m in ms if m.get("v_at_engage_fw_count_s") is not None]
    g["v_at_engage_abs"] = dict(min=min(va), max=max(va), n=len(va)) if va else None
    g["stop_disp_max_abs"] = bound([m.get("stop_disp_max_abs") for m in ms], 1.0, opts.margin_pct)
    g["post_still_max_v"] = bound([m.get("post_still_max_v_count_s") for m in ms], 50.0, opts.margin_pct)
    viout = {}
    for ph in ("release", "drive", "brake", "rest"):
        vals = [m["viout"][ph] for m in ms if m.get("viout", {}).get(ph)]
        if vals:
            viout[ph] = dict(n=sum(v["n"] for v in vals), min=min(v["min"] for v in vals), max=max(v["max"] for v in vals),
                             median_of_runs=statistics.median(v["median"] for v in vals))
    first = [m["viout"].get("adc_first") for m in ms if m.get("viout", {}).get("adc_first") is not None]
    if first:
        viout["adc_first"] = dict(min=min(first), max=max(first))
    g["viout"] = viout
    return g


def upper_envelope(points: list[tuple[float, float]]) -> dict | None:
    """全点を上から押さえる直線 d ≤ a + b·|v|（a ≥ 0, b ≥ 0）のうち、[0, v_max] の下の面積が最小のもの。速さが 2 種類未満なら None"""
    pts = sorted(set((float(v), float(d)) for v, d in points))
    if len({p[0] for p in pts}) < 2:
        return None
    vmax = max(p[0] for p in pts)
    hull: list[tuple[float, float]] = []
    for p in pts:  # 上側の凸包（v の昇順）
        while len(hull) >= 2 and ((hull[-1][0] - hull[-2][0]) * (p[1] - hull[-2][1])
                                  - (hull[-1][1] - hull[-2][1]) * (p[0] - hull[-2][0])) >= 0:
            hull.pop()
        hull.append(p)
    cands = [(max(p[1] for p in pts), 0.0)]
    for (v1_, d1), (v2_, d2) in zip(hull, hull[1:]):
        if v2_ > v1_:
            b = (d2 - d1) / (v2_ - v1_)
            cands.append((d1 - b * v1_, b))
    nz = [p for p in pts if p[0] > 0]
    if nz:
        cands.append((0.0, max(p[1] / p[0] for p in nz)))
    best = None
    for a, b in cands:
        if a < 0 or b < 0:
            continue
        if any(d > a + b * v + 1e-9 for v, d in pts):
            continue
        obj = a + b * vmax / 2
        if best is None or obj < best[0]:
            best = (obj, a, b)
    if best is None:
        return None
    a = math.ceil(best[1] * 10) / 10
    b = math.ceil(best[2] * 1e5) / 1e5
    return dict(a_count=a, b_count_per_count_s=b, v_max_covered=vmax, n=len(points),
                note="v_max_covered より速い速度へは外挿しない")


def recommend(runs: list[dict], opts: Opts) -> dict:
    inc = [r for r in runs if not r["excluded"] and r["metrics"]]
    rec: dict = {}

    def entry(name: str, b: dict | None, source: str, extra: dict | None = None, lower_bound: bool = False) -> dict:
        c = R10_CONSTANTS[name]
        e = dict(current=c["current"], unit=c["unit"], meaning=c["meaning"], source=source)
        if b is None:
            e.update(derived=INSUFFICIENT, n=0)
        else:
            e.update(derived_max=b["max"], derived_conservative=b["conservative"], n=b["n"], bound=b,
                     exceeds_current=b["max"] > c["current"])
            if lower_bound:
                e["derived_is_lower_bound"] = True
        if extra:
            e.update(extra)
        return e

    sag = [r for r in inc if r["mode"] == "sag"]
    down = [r for r in sag if r["metrics"]["sag_dir"] == "-"]
    up = [r for r in sag if r["metrics"]["sag_dir"] == "+"]
    still = [r for r in sag if r["metrics"]["sag_dir"] == "0"]
    win = min((r["metrics"]["sag_rate_window_ms"] for r in down if r["metrics"].get("sag_rate_window_ms")), default=None)
    b = bound([r["metrics"]["sag_rate_100ms_count_s"] for r in down], rate_resolution(win), opts.margin_pct)
    ex = dict(n_up=len(up), n_no_motion=len(still), axes=sorted({r["axis"] for r in down}),
              accelerating=sum(1 for r in down if r["metrics"]["sag_accelerating"]))
    if up:
        ex["up_max_count_s"] = max(r["metrics"]["sag_rate_100ms_count_s"] or 0 for r in up)
    if win is not None and win < 100:
        ex["note_window"] = f"解除が 100 ms より短い回がある（最短の窓 {win} ms）"
    rec["V1_SUP_BACKDRIVE_COUNT_S"] = entry(
        "V1_SUP_BACKDRIVE_COUNT_S", b, "SAG・カウント減の向きの回の、解除中の最悪の 100 ms 窓の速さ（全軸・全位置・全荷重）", ex)

    eng_vals = [r["metrics"]["engage_ms_for_bound"] for r in inc if r["metrics"].get("engage_ms_for_bound") is not None]
    not_still = [r for r in inc if r["metrics"].get("engage_still_censored_ms") is not None]
    b = bound(eng_vals, 1.0, opts.margin_pct)
    va = [abs(r["metrics"]["v_at_engage_fw_count_s"]) for r in inc if r["metrics"].get("v_at_engage_fw_count_s") is not None]
    ex = dict(n_not_still=len(not_still), v_at_engage_abs_max=max(va) if va else None,
              n_sag=sum(1 for r in inc if r["mode"] == "sag" and r["metrics"].get("engage_ms_for_bound") is not None),
              n_engage=sum(1 for r in inc if r["mode"] == "engage" and r["metrics"].get("engage_ms_for_bound") is not None))
    if not_still:
        ex["note_not_still"] = (f"{len(not_still)} 回は記録の終わりまで静止しなかった（その回は記録の終わりまでの "
                                f"{min(r['metrics']['engage_still_censored_ms'] for r in not_still)} ms 以上を下限として入れた）。"
                                "最大は下限。ブレーキが掴めていない疑い")
    rec["V1_BRAKE_ENGAGE_MS"] = entry(
        "V1_BRAKE_ENGAGE_MS", b,
        f"作動指令から静止（{opts.still_window_us // 1000} ms の間 ±{opts.still_band} count）までと firmware t_settle の大きい方（SAG・ENGAGE の全回）",
        ex, lower_bound=bool(not_still))

    rel_sag = [r["metrics"]["release_to_motion_ms"] for r in sag if r["metrics"].get("release_to_motion_ms") is not None]
    rel_eng = [r["metrics"]["motion_onset_ms"] for r in inc
               if r["mode"] == "engage" and r["metrics"].get("motion_onset_ms") is not None]
    b = bound(rel_sag + rel_eng, 1.0, opts.margin_pct)
    ex = dict(n_sag=len(rel_sag), n_engage_onset=len(rel_eng), max_sag=max(rel_sag) if rel_sag else None,
              max_engage_onset=max(rel_eng) if rel_eng else None,
              n_sag_no_motion=sum(1 for r in sag if r["metrics"].get("release_to_motion_ms") is None))
    if ex["n_sag_no_motion"]:
        ex["note_no_motion"] = f"SAG {ex['n_sag_no_motion']} 回は解除中に動かなかった（離れたか分からない。n に入れない）"
    rec["BRAKE_RELEASE_WAIT_MS"] = entry(
        "BRAKE_RELEASE_WAIT_MS", b,
        f"SAG: 解除指令から |変位| ≥ {opts.motion_count} まで。ENGAGE: 解除指令から走行後の動き出しまで（離れた時刻の上側）", ex)

    bal = [r for r in inc if r["load_kind"] == "ballast"]
    sag_v = [r["metrics"]["sag_rate_100ms_count_s"] for r in bal if r["mode"] == "sag"]
    creep_v = [r["metrics"]["post_still_max_v_count_s"] for r in bal if r["metrics"].get("post_still_max_v_count_s") is not None]
    # 分解能は値ごと（100 ms 窓 = 10 count/s、静止後の 20 ms 窓 = 50 count/s）。保守側は両方の境界の大きい方
    bs = [x for x in (bound(sag_v, 10.0, opts.margin_pct), bound(creep_v, 50.0, opts.margin_pct)) if x is not None]
    b = max(bs, key=lambda x: x["conservative"]) if bs else None
    if b is not None:
        b = dict(b, n=len(sag_v) + len(creep_v), max=max(x["max"] for x in bs))
    ex = dict(n_ballast_runs=len(bal), n_empty_runs=sum(1 for r in inc if r["load_kind"] == "empty"),
              n_sag_values=len(sag_v), n_creep_values=len(creep_v))
    if not bal:
        ex["note"] = "バラストの回が無い（無負荷の値はこの定数には使わない）"
    rec["V1_SUP_HOLD_ENV_COUNT_S"] = entry(
        "V1_SUP_HOLD_ENV_COUNT_S", b, "バラスト条件の SAG 最悪 100 ms 窓と、作動して静止した後の 20 ms 窓の最大 |v|", ex)

    pts = [(abs(r["metrics"]["v_at_engage_fw_count_s"]), r["metrics"]["stop_disp_max_abs"]) for r in inc
           if r["mode"] == "engage" and r["metrics"].get("v_at_engage_fw_count_s") is not None]
    env = upper_envelope(pts)
    rec["stop_disp_envelope"] = env if env is not None else dict(derived=INSUFFICIENT, n=len(pts),
                                                                 note="ENGAGE の速さが 2 種類以上要る")
    return rec


def analyze(paths: list[str], opts: Opts) -> dict:
    jsons, incomplete = discover(paths)
    runs = [analyze_run(p, opts) for p in jsons]
    groups: dict[tuple, list[dict]] = {}
    for r in runs:
        if "axis" in r:
            groups.setdefault(group_key(r), []).append(r)
    gl = []
    for k in sorted(groups, key=lambda k: tuple("" if x is None else str(x) for x in k)):
        g = summarize_group(groups[k], opts)
        g["key"] = key_dict(k)
        gl.append(g)
    return dict(
        options=dict(still_thr_count_s=opts.still_thr, still_window_ms=opts.still_window_us / 1000, still_band_count=opts.still_band,
                     motion_count=opts.motion_count, gap_exclude_us=opts.gap_exclude_us, margin_pct=opts.margin_pct),
        n_runs=len(runs), n_included=sum(1 for r in runs if not r["excluded"] and r["metrics"]),
        incomplete=[dict(file=p, reason="JSON が無い（道具が拒否・通信の失敗で書かなかった。.log を見る）") for p in incomplete],
        runs=runs,
        groups=gl,
        recommended=recommend(runs, opts),
    )


# ---------------------------------------------------------------------------
# Markdown
# ---------------------------------------------------------------------------
def f(v, nd: int = 1) -> str:
    if v is None:
        return "—"
    if isinstance(v, float):
        return f"{v:.{nd}f}".rstrip("0").rstrip(".") if nd else f"{v:.0f}"
    return str(v)


def fb(b: dict | None, key: str = "max") -> str:
    return "—" if b is None else f(b.get(key))


def cond(k: dict) -> str:
    d = "" if k["dir"] is None else (" dir=+1" if k["dir"] > 0 else " dir=-1")
    return f"ax{k['axis']} {k['position']} {k['load']} {k['stage']}{d}"


def to_markdown(rep: dict) -> str:
    o = rep["options"]
    L: list[str] = []
    L.append("# ブレーキ・沈みの計測のまとめ（analyze.py が生成）")
    L.append("")
    L.append(f"回数 {rep['n_runs']}（解析に使った {rep['n_included']}、除外 {rep['n_runs'] - rep['n_included']}、"
             f"JSON の無い回 {len(rep['incomplete'])}）。静止 = {f(o['still_window_ms'])} ms の間 ±{o['still_band_count']} count"
             f"（{f(o['still_thr_count_s'])} count/s）、動き出し = |変位| ≥ {o['motion_count']} count。"
             "境界は最悪側（max・max + 分解能・n ≥ 5 で平均 + 3σ）。平均は参考。")
    L.append("")

    L.append("## 置き換え候補（測った値から。余裕の掛け方は r10 の側で決める）")
    L.append("")
    L.append("| 定数 | 仮値 | 実測の最大 | 保守側 | n | 元 | 注 |")
    L.append("| --- | --- | --- | --- | --- | --- | --- |")
    for name, e in rep["recommended"].items():
        if name == "stop_disp_envelope":
            continue
        notes = []
        if e.get("derived") == INSUFFICIENT:
            mx = cons = INSUFFICIENT
        else:
            lb = "≥ " if e.get("derived_is_lower_bound") else ""
            mx = lb + f(e["derived_max"])
            cons = lb + f(e["derived_conservative"])
            if "conservative_with_margin" in e["bound"]:
                cons += f"（+{f(e['bound']['with_margin_pct'])} %: {f(e['bound']['conservative_with_margin'])}）"
            if e["exceeds_current"]:
                notes.append("**仮値を超えた**")
        for k in ("note", "note_window", "note_not_still", "note_no_motion"):
            if e.get(k):
                notes.append(e[k])
        if name == "V1_SUP_BACKDRIVE_COUNT_S":
            if e.get("n_up"):
                notes.append(f"上向き（カウント増）の沈み {e['n_up']} 回、最大 {f(e.get('up_max_count_s'))} count/s（別の所見）")
            if e.get("n_no_motion"):
                notes.append(f"動かなかった回 {e['n_no_motion']}（n に入れない）")
            if e.get("accelerating"):
                notes.append(f"解除が長いほど速くなる回 {e['accelerating']}（速さでなく時刻ごとの沈みで見る）")
        if name == "BRAKE_RELEASE_WAIT_MS" and e.get("n", 0):
            notes.append(f"SAG の最大 {f(e.get('max_sag'))}（n {e['n_sag']}）、ENGAGE の動き出しの最大 {f(e.get('max_engage_onset'))}"
                         f"（n {e['n_engage_onset']}。--delay と走り出しを含む上側）")
        if name == "V1_BRAKE_ENGAGE_MS" and e.get("v_at_engage_abs_max") is not None:
            notes.append(f"作動時の |v| ≤ {e['v_at_engage_abs_max']} count/s の範囲")
        L.append(f"| `{name}` | {e['current']} {e['unit']} | {mx} | {cons} | {e['n']} | {e['source']} | {'; '.join(notes)} |")
    env = rep["recommended"]["stop_disp_envelope"]
    L.append("")
    if env.get("derived") == INSUFFICIENT:
        L.append(f"作動中の滑りの包絡線 d ≤ a + b·|v|: {INSUFFICIENT}（n = {env['n']}、{env['note']}）")
    else:
        L.append(f"作動中の滑りの包絡線（ENGAGE、作動後の最大 |変位|）: d ≤ {f(env['a_count'])} + {env['b_count_per_count_s']}·|v| "
                 f"[count]、n = {env['n']}、|v| ≤ {f(env['v_max_covered'])} count/s の範囲（外挿しない）")
    L.append("")

    sag = [g for g in rep["groups"] if g["key"]["mode"] == "sag"]
    if sag:
        L.append("## SAG（ブレーキ解除・PWM 0）")
        L.append("")
        L.append("|変位| の最大 [count]（括弧は解除がその時刻まで続いた回数。届かなかった回は数えない）")
        L.append("")
        hz = " | ".join(f"@{T} ms" for T in SAG_HORIZONS_MS)
        L.append(f"| 条件 | n（除外） | 向き +/−/0 | {hz} | 最悪 100 ms 窓 max [count/s] | 平均 + 3σ | 解除→動き出し max [ms] | 動かず | 加速 | 途中で作動 |")
        L.append("| --- | --- | --- | " + " | ".join("---" for _ in SAG_HORIZONS_MS) + " | --- | --- | --- | --- | --- | --- |")
        for g in sag:
            if not g["n_included"]:
                L.append(f"| {cond(g['key'])} | 0（{g['n_excluded']}） | " + " | ".join("—" for _ in range(len(SAG_HORIZONS_MS) + 7)) + " |")
                continue
            dc = g["sag_dir_counts"]
            cells = []
            for T in SAG_HORIZONS_MS:
                d = g["sag_disp_at_ms"][str(T)]
                cells.append("—" if d["max_abs"] is None else f"{d['max_abs']}（{d['n']}）")
            L.append(f"| {cond(g['key'])} | {g['n_included']}（{g['n_excluded']}） | {dc['+']}/{dc['-']}/{dc['0']} | "
                     + " | ".join(cells)
                     + f" | {fb(g['sag_rate_100ms'])} | {fb(g['sag_rate_100ms'], 'mean_plus_3sd')} | {fb(g['release_to_motion_ms'])} | "
                     f"{g['release_no_motion']} | {g['sag_accelerating']} | {g['sag_curtailed']} |")
        L.append("")

    groups_with_engage = [g for g in rep["groups"] if g["n_included"]]
    if groups_with_engage:
        L.append("## 作動（ブレーキ作動指令 + PWM 0 の後）")
        L.append("")
        L.append("| 条件 | モード | n | 作動時 \\|v\\| min–max [count/s] | 作動→静止 max [ms] | 平均 + 3σ | 静止せず | 作動後の滑り max [count] | 静止後の最大 \\|v\\| [count/s] | 動き出し（ENGAGE）max [ms] |")
        L.append("| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |")
        for g in groups_with_engage:
            va = g.get("v_at_engage_abs")
            vr = "—" if not va else f"{va['min']}–{va['max']}"
            ns = str(g["engage_not_still"]) + ("" if not g["engage_not_still"] else f"（≥ {f(g['engage_not_still_min_ms'])} ms）")
            lb = "≥ " if g["engage_not_still"] else ""
            L.append(f"| {cond(g['key'])} | {g['key']['mode']} | {g['n_included']} | {vr} | {lb}{fb(g['engage_ms'])} | "
                     f"{fb(g['engage_ms'], 'mean_plus_3sd')} | {ns} | {fb(g['stop_disp_max_abs'])} | {fb(g['post_still_max_v'])} | "
                     f"{fb(g.get('motion_onset_ms'))} |")
        L.append("")

        L.append("## VIOUT の ADC 生値（換算なし）")
        L.append("")
        L.append("区間: release = ブレーキ解除・PWM 0、drive = PWM ≠ 0、brake = 作動指令から静止まで、rest = 静止の後。min / 回ごとの中央値の中央値 / max")
        L.append("")
        L.append("| 条件 | release | drive | brake | rest | 最初の点 |")
        L.append("| --- | --- | --- | --- | --- | --- |")
        for g in groups_with_engage:
            vi = g.get("viout", {})
            cells = []
            for ph in ("release", "drive", "brake", "rest"):
                s = vi.get(ph)
                cells.append("—" if not s else f"{s['min']} / {f(float(s['median_of_runs']))} / {s['max']}")
            af = vi.get("adc_first")
            cells.append("—" if not af else f"{af['min']}–{af['max']}")
            L.append(f"| {cond(g['key'])} {g['key']['mode']} | " + " | ".join(cells) + " |")
        L.append("")

    L.append("## 記録の品質")
    L.append("")
    L.append("| 条件 | モード | 回数 | 除外 | 間隔の最大 [us] | 間隔 > 2 周期 | CONTROL_OVERRUN |")
    L.append("| --- | --- | --- | --- | --- | --- | --- |")
    for g in rep["groups"]:
        L.append(f"| {cond(g['key'])} | {g['key']['mode']} | {g['n_runs']} | {g['n_excluded']} | {g['max_sample_gap_us']} | "
                 f"{g['late_samples']} | {g['control_overrun']} |")
    L.append("")
    excl = [r for r in rep["runs"] if r["excluded"]]
    if excl or rep["incomplete"]:
        L.append("### 除外した回（理由）")
        L.append("")
        for r in excl:
            L.append(f"- `{r['id']}`: " + "; ".join(r["excluded"]))
        for i in rep["incomplete"]:
            L.append(f"- `{os.path.basename(i['file'])}`: {i['reason']}")
        L.append("")
    flagged = [r for r in rep["runs"] if r["flags"] and not r["excluded"]]
    if flagged:
        L.append("### 注記のある回（解析には使った）")
        L.append("")
        for r in flagged:
            L.append(f"- `{r['id']}`: " + "; ".join(r["flags"]))
        L.append("")
    return "\n".join(L) + "\n"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", nargs="+", help="ディレクトリ（中の *.json）か、個別の .json / .csv / .log")
    ap.add_argument("--md", help="Markdown の出力先（無ければ標準出力）")
    ap.add_argument("--json", dest="json_out", help="JSON の出力先")
    ap.add_argument("--still-thr", type=float, default=50.0,
                    help="静止の速さ [count/s]。窓の中の許容変位 = max(1, floor(これ × 窓))。既定 50 = ファームの t_settle と同じ ±1 count")
    ap.add_argument("--still-window-ms", type=float, default=20.0, help="静止の窓 [ms]（既定 20）")
    ap.add_argument("--motion-count", type=int, default=2, help="動き出しとみなす |変位| [count]（既定 2 = host_metrics と同じ）")
    ap.add_argument("--gap-exclude-us", type=int, default=20000, help="記録の間隔がこれを超えた回は除外（既定 20000 = 静止の窓）")
    ap.add_argument("--margin-pct", type=float, default=None,
                    help="保守側の値に掛ける余裕 [%%]（表示だけ。既定なし: 余裕の掛け方は r10 の側で決める）")
    args = ap.parse_args(argv)
    opts = Opts(args.still_thr, args.still_window_ms, args.motion_count, args.gap_exclude_us, args.margin_pct)
    rep = analyze(args.paths, opts)
    md = to_markdown(rep)
    if args.md:
        with open(args.md, "w") as fo:
            fo.write(md)
    else:
        sys.stdout.write(md)
    if args.json_out:
        with open(args.json_out, "w") as fo:
            json.dump(rep, fo, ensure_ascii=False, indent=2)
    return 0


if __name__ == "__main__":
    sys.exit(main())
