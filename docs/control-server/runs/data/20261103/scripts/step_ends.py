#!/usr/bin/env python
"""端寄りでの衝撃試験。各 DOF を base（±）までゆっくり寄せ、そこで片側 amp・freq Hz の揺れを cycles 回入れて、
実測の最小・最大（ソフトリミットとの距離）、追従誤差、FAULT を測る。
使い方: shake_ends.py <label> <log> <max_acc> <duty> [base] [amp] [freq] [cycles] [max_vel]"""
import json, sys, time, math, urllib.request
sys.path.insert(0, "/Users/takuro/.superset/worktrees/dof-reality-h3-custom-controller/quilted-pendulum/clients/python")
from h3_motion_client import MotionClient
B = "http://127.0.0.1:8892"
LABEL, LOG = sys.argv[1], open(sys.argv[2], "a", buffering=1)
ACC, DUTY = int(sys.argv[3]), int(sys.argv[4])
BASE = float(sys.argv[5]) if len(sys.argv) > 5 else 0.85
AMP = float(sys.argv[6]) if len(sys.argv) > 6 else 0.1
FREQ = float(sys.argv[7]) if len(sys.argv) > 7 else 4
CYC = float(sys.argv[8]) if len(sys.argv) > 8 else 4
VEL = int(sys.argv[9]) if len(sys.argv) > 9 else 1200
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
log("limits ->", json.dumps(lim.get("effective") or lim)[:200], "reasons", json.dumps(lim.get("reasons"))[:120])
post("/v1/arm"); s = wait(("HOLDING",), 8)
if s["rig_state"] != "HOLDING": log("ARM failed", s["rig_state"], s["fault"]); c.close(); sys.exit(1)
post("/v1/center"); time.sleep(1.0); s = wait(("HOLDING",), 15)
send(0, 0, 0, 20); r = post("/v1/start"); s = wait(("ACTIVE",), 3)
if s["rig_state"] != "ACTIVE": log("START failed", s["rig_state"], r); c.close(); sys.exit(1)
mk = {"yaw": lambda v: (0, 0, v), "pitch": lambda v: (v, 0, 0), "roll": lambda v: (0, v, 0)}
aborted = None
for name in ("yaw", "pitch", "roll"):
    mn = {i: 1e9 for i in (1, 2, 3)}; mx = {i: -1e9 for i in (1, 2, 3)}; tmn = dict(mn); tmx = dict(mx); fe = {i: 0 for i in (1, 2, 3)}; lims = set()
    for v in (-1.0, 1.0, 0.0):
        t0 = time.time(); last = 0
        while time.time() - t0 < 5.0:
            send(*mk[name](v), 1)
            if time.time() - last >= 0.03:
                last = time.time(); s = st(); a = ax(s); lims |= set(s.get("limits") or [])
                if s["rig_state"] != "ACTIVE": aborted = (s["rig_state"], s["fault"]); break
                for i in (1, 2, 3):
                    mn[i] = min(mn[i], a[i]["actual_count"]); mx[i] = max(mx[i], a[i]["actual_count"])
                    tmn[i] = min(tmn[i], a[i]["target_count"]); tmx[i] = max(tmx[i], a[i]["target_count"])
                    fe[i] = max(fe[i], abs(a[i]["following_error_count"] or 0))
        if aborted: break
    if aborted: log(name, "ABORT", aborted); break
    log(f"{name} steps ±1: actual min/max={ {i: (mn[i], mx[i]) for i in (1,2,3)} } target min/max={ {i: (tmn[i], tmx[i]) for i in (1,2,3)} } fe_max={fe} limits={sorted(lims)}")
log("STOP", post("/v1/stop").get("ok")); s = wait(("BRAKED_DISARMED",), 15); log("end", s["rig_state"], s["fault"])
post("/v1/limits", {"max_vel_count_s": 1200, "max_acc_count_s2": 6000, "duty_max_percent": 20, "max_gain": 1.0})
c.close()
