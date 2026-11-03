#!/usr/bin/env python
"""ヨー（と前後・左右）の衝撃試験。READY_TO_ARM から ARM → CENTER → START、limits を指定して
「片側 amp の 1.5 Hz を cycles 回」をヨー・ピッチ・ロールの順に出し、軸ごとの実際の振れ幅（目標・実測）、追従誤差、Duty の張り付き、FAULT を測る。
使い方: shake_test.py <label> <logfile> <max_acc> <duty_max> [amp_pose] [freq] [cycles] [max_vel]"""
import json, sys, time, math, urllib.request
sys.path.insert(0, "/Users/takuro/.superset/worktrees/dof-reality-h3-custom-controller/quilted-pendulum/clients/python")
from h3_motion_client import MotionClient
B = "http://127.0.0.1:8892"
LABEL, LOG = sys.argv[1], open(sys.argv[2], "a", buffering=1)
ACC, DUTY = int(sys.argv[3]), int(sys.argv[4])
AMP = float(sys.argv[5]) if len(sys.argv) > 5 else 0.3
FREQ = float(sys.argv[6]) if len(sys.argv) > 6 else 1.5
CYC = float(sys.argv[7]) if len(sys.argv) > 7 else 2
VEL = int(sys.argv[8]) if len(sys.argv) > 8 else 1200
def log(*a):
    l = time.strftime("%H:%M:%S") + f" [{LABEL}] " + " ".join(str(x) for x in a); print(l, flush=True); LOG.write(l + "\n")
def post(p, b=None):
    r = urllib.request.Request(B + p, data=json.dumps(b or {}).encode(), headers={"Content-Type": "application/json"}, method="POST")
    try: return json.loads(urllib.request.urlopen(r, timeout=3).read())
    except urllib.error.HTTPError as e: return json.loads(e.read() or b"{}")
def st(): return json.load(urllib.request.urlopen(B + "/v1/status", timeout=2))
def ax(s): return {a["axis_id"]: a for a in s["axes"]}
def wait(t, to):
    t0 = time.time()
    while time.time() - t0 < to:
        s = st()
        if s["rig_state"] in t or s["rig_state"] == "FAULT_LATCHED": return s
        time.sleep(0.05)
    return st()
s = st()
if s["session"]["fresh"] or s["armed"]: log("someone is driving - abort"); sys.exit(1)
if s["rig_state"] not in ("READY_TO_ARM", "BRAKED_DISARMED"): log("not armable", s["rig_state"]); sys.exit(1)
c = MotionClient(B); c.start(); t = time.time()
while time.time() - t < 10 and not c.connected: time.sleep(0.05)
fs = 0
def send(p, r, y, n=1):
    global fs
    for _ in range(n): fs += 1; c.submit_pose(p, r, y, 1.0, frame_seq=fs); time.sleep(0.01)
send(0, 0, 0, 20)
lim = post("/v1/limits", {"max_vel_count_s": VEL, "max_acc_count_s2": ACC, "duty_max_percent": DUTY, "max_gain": 1.0})
log("limits ->", json.dumps(lim.get("effective") or lim)[:200], "reasons", json.dumps(lim.get("reasons"))[:200])
post("/v1/arm"); s = wait(("HOLDING",), 8)
if s["rig_state"] != "HOLDING": log("ARM failed", s["rig_state"], s["fault"]); c.close(); sys.exit(1)
post("/v1/center"); time.sleep(1.0); s = wait(("HOLDING",), 15)
send(0, 0, 0, 20); r = post("/v1/start"); s = wait(("ACTIVE",), 3)
if s["rig_state"] != "ACTIVE": log("START failed", s["rig_state"], r); c.close(); sys.exit(1)
for name, mk in (("yaw", lambda v: (0, 0, v)), ("pitch", lambda v: (v, 0, 0)), ("roll", lambda v: (0, v, 0))):
    send(0, 0, 0, 30)
    base = {i: ax(st())[i]["actual_count"] for i in (1, 2, 3)}
    mn = {i: 1e9 for i in (1, 2, 3)}; mx = {i: -1e9 for i in (1, 2, 3)}; tmn = dict(mn); tmx = dict(mx)
    fe = {i: 0 for i in (1, 2, 3)}; sat = {i: 0 for i in (1, 2, 3)}; n = 0
    t0 = time.time(); dur = CYC / FREQ + 0.8; last = 0; aborted = None
    while time.time() - t0 < dur:
        tt = time.time() - t0
        v = AMP * math.sin(2 * math.pi * FREQ * tt) if tt < CYC / FREQ else 0.0
        send(*mk(v), 1)
        if time.time() - last >= 0.04:
            last = time.time(); s = st(); a = ax(s); n += 1
            if s["rig_state"] != "ACTIVE": aborted = (s["rig_state"], s["fault"]); break
            for i in (1, 2, 3):
                mn[i] = min(mn[i], a[i]["actual_count"]); mx[i] = max(mx[i], a[i]["actual_count"])
                tmn[i] = min(tmn[i], a[i]["target_count"]); tmx[i] = max(tmx[i], a[i]["target_count"])
                fe[i] = max(fe[i], abs(a[i]["following_error_count"] or 0))
                if abs(a[i]["pwm_permille"] or 0) >= DUTY * 10 - 5: sat[i] += 1
    if aborted: log(name, "ABORT", aborted); break
    log(f"{name}: actual p-p={ {i: mx[i]-mn[i] for i in (1,2,3)} } target p-p={ {i: tmx[i]-tmn[i] for i in (1,2,3)} } fe_max={fe} sat%={ {i: round(100*sat[i]/max(n,1)) for i in (1,2,3)} }")
    time.sleep(0.5)
log("STOP", post("/v1/stop").get("ok")); s = wait(("BRAKED_DISARMED",), 15); log("end", s["rig_state"], s["fault"])
c.close()
