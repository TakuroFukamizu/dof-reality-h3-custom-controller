#!/usr/bin/env python3
"""samples_<label>.csv の #1 正弦波区間を、軌道の向き（上り=カウント増 / 下り）と軌道速度の帯で分け、
各帯の「速度のムラ」を出す。ムラ = |実測速度（ファームの 100 ms 窓）− 軌道速度| の平均と p90、
および |Δ実測速度|（100 ms ごと）の p90。どの条件でガタつくかを見る。
使い方: segment_jerk.py <csv> [...]"""
import csv, sys
BANDS = [(0, 50), (50, 150), (150, 300), (300, 9999)]
def p90(v): v = sorted(v); return v[int(0.9 * (len(v) - 1))] if v else 0
for path in sys.argv[1:]:
    rows = [r for r in csv.DictReader(open(path)) if r["phase"] == "sine" and r["axis"] == "1"]
    t = [float(r["t"]) for r in rows]; tg = [int(r["target"]) for r in rows]; vm = [int(r["vel"]) for r in rows]; pw = [int(r["pwm"]) for r in rows]
    seg = {}
    for i in range(2, len(rows)):
        dt = t[i] - t[i-1]
        if dt <= 0.03: continue
        vt = (tg[i] - tg[i-2]) / (t[i] - t[i-2])
        d = "up" if vt > 0 else "down"
        b = next(k for k, (lo, hi) in enumerate(BANDS) if lo <= abs(vt) < hi)
        s = seg.setdefault((d, b), {"ve": [], "dv": [], "pw": []})
        s["ve"].append(abs(vm[i] - vt)); s["dv"].append(abs(vm[i] - vm[i-1])); s["pw"].append(abs(pw[i]) / 10)
    name = path.rsplit("samples_", 1)[-1].rsplit(".", 1)[0]
    print(f"== {name}")
    print(f"{'dir':5s} {'|v_traj|':>10s} {'n':>4s} {'vel_err mean':>12s} {'p90':>5s} {'|dv| p90':>8s} {'|pwm| mean':>10s}")
    for d in ("up", "down"):
        for b, (lo, hi) in enumerate(BANDS):
            s = seg.get((d, b))
            if not s: continue
            n = len(s["ve"])
            print(f"{d:5s} {f'{lo}-{hi if hi < 9999 else chr(8734)}':>10s} {n:4d} {sum(s['ve'])/n:12.0f} {p90(s['ve']):5.0f} {p90(s['dv']):8.0f} {sum(s['pw'])/n:9.1f}%")
