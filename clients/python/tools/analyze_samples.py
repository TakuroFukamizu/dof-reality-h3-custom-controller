#!/usr/bin/env python3
"""samples_<label>.csv（hw_ab3.py の出力、10 Hz）から #1 の正弦波区間を解析する。
- motion_rev: 実測位置の向きの反転回数（2 count のヒステリシス）。traj_rev: 目標の向きの反転回数（同じ判定）
- extra_rev = motion_rev − traj_rev（軌道にない往復 = ガタつき・振動の目安）
- stick%: 目標が 30 count/s 以上で動いているのに実測が 100 ms で 1 count も動かないサンプルの割合
- jerk_p95: 実測速度（count 差分）の 100 ms ごとの変化の 95 パーセンタイル [count/s]（体感の段付きの目安）
- dpwm_p95: |ΔPWM| の 95 パーセンタイル [%]
使い方: analyze_samples.py <csv> [<csv> ...]
"""
import csv, sys
def revs(xs, hyst=2):
    n = 0; d = 0; ext = xs[0]
    for x in xs[1:]:
        if d >= 0 and x < ext - hyst:
            if d > 0: n += 1
            d = -1; ext = x
        elif d <= 0 and x > ext + hyst:
            if d < 0: n += 1
            d = 1; ext = x
        elif (d > 0 and x > ext) or (d < 0 and x < ext): ext = x
    return n
def p95(v): v = sorted(v); return v[int(0.95 * (len(v) - 1))] if v else 0
for path in sys.argv[1:]:
    rows = [r for r in csv.DictReader(open(path)) if r["phase"] == "sine" and r["axis"] == "1"]
    t = [float(r["t"]) for r in rows]; tg = [int(r["target"]) for r in rows]; ac = [int(r["actual"]) for r in rows]; pw = [int(r["pwm"]) for r in rows]
    mr = revs(ac); tr = revs(tg)
    stick = moving = 0; vel = []
    for i in range(1, len(rows)):
        dt = t[i] - t[i-1]
        if dt <= 0: continue
        vt = (tg[i] - tg[i-1]) / dt; va = (ac[i] - ac[i-1]) / dt; vel.append(va)
        if abs(vt) >= 30:
            moving += 1
            if ac[i] == ac[i-1]: stick += 1
    jerk = [abs(vel[i] - vel[i-1]) for i in range(1, len(vel))]
    dp = [abs(pw[i] - pw[i-1]) / 10 for i in range(1, len(pw))]
    name = path.rsplit("samples_", 1)[-1].rsplit(".", 1)[0]
    print(f"{name:16s} n={len(rows)} motion_rev={mr} traj_rev={tr} extra_rev={mr-tr} stick={100*stick/max(moving,1):.0f}% jerk_p95={p95(jerk):.0f} count/s dpwm_p95={p95(dp):.1f}%")
