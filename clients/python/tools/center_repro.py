#!/usr/bin/env python
"""kp を上げたときの CENTER 開始時 ENCODER_FAULT の再現（11-02）。#1 を pose でずらして STOP → ARM → CENTER を 20 Hz で記録。
使い方: center_repro.py <label> <logfile> <pitch> [<pitch> ...]   （pitch は gain 0.3 での pose。符号でずらす向き）"""
import json, sys, time, urllib.request
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[1]))
from h3_motion_client import MotionClient
B = __import__("os").environ.get("H3CS_URL", "http://127.0.0.1:8892"); LABEL = sys.argv[1]; LOG = open(sys.argv[2], "a", buffering=1)
def ts(): return time.strftime("%H:%M:%S") + ".%03d" % int((time.time() % 1) * 1000)
def log(*a):
    line = ts() + f" [{LABEL}] " + " ".join(str(x) for x in a); print(line, flush=True); LOG.write(line + "\n")
def post(p, body=None):
    req = urllib.request.Request(B + p, data=json.dumps(body or {}).encode(), headers={"Content-Type": "application/json"}, method="POST")
    try: r = urllib.request.urlopen(req, timeout=3); return r.status, json.loads(r.read().decode())
    except urllib.error.HTTPError as e: return e.code, json.loads(e.read().decode() or "{}")
def st(): return json.load(urllib.request.urlopen(B + "/v1/status", timeout=2))
def a1(s): return [a for a in s["axes"] if a["axis_id"] == 1][0]
def wait(target, timeout):
    t0 = time.time()
    while time.time() - t0 < timeout:
        s = st()
        if s["rig_state"] in target or s["rig_state"] == "FAULT_LATCHED": return s
        time.sleep(0.05)
    return st()
def row(s):
    a = a1(s); return f'{s["rig_state"]:12s} t={a["target_count"]} c={a["actual_count"]} e={a["following_error_count"]} pwm={a["pwm_permille"]} v={a["velocity_count_s"]} mcu={a["mcu_state"]}'
for pitch in [float(x) for x in sys.argv[3:]]:
    s = st()
    if s["rig_state"] not in ("READY_TO_ARM", "BRAKED_DISARMED"): log("not armable", s["rig_state"], s["fault"]); sys.exit(1)
    post("/v1/arm"); s = wait(("HOLDING",), 10)
    if s["rig_state"] != "HOLDING": log("ARM failed", s["rig_state"], s["fault"]); sys.exit(1)
    post("/v1/center"); time.sleep(1.2); s = wait(("HOLDING",), 10)
    # ずらす
    c = MotionClient(B); c.start(); t0 = time.time()
    while time.time() - t0 < 10 and not c.connected: time.sleep(0.1)
    fs = 0
    for _ in range(10): fs += 1; c.submit_pose(0.0, 0.0, 0.0, 0.3, frame_seq=fs); time.sleep(0.02)
    post("/v1/start"); s = wait(("ACTIVE",), 5)
    if s["rig_state"] != "ACTIVE": log("START failed", s["rig_state"], s["fault"]); c.close(); sys.exit(1)
    t0 = time.time()
    while time.time() - t0 < 3.0:
        k = min((time.time() - t0) / 1.5, 1.0); fs += 1; c.submit_pose(pitch * k, 0.0, 0.0, 0.3, frame_seq=fs); time.sleep(0.02)
    s = st(); log(f"pitch {pitch}: offset row", row(s))
    if s["rig_state"] != "ACTIVE": log("fault while offsetting", s["fault"]); c.close(); sys.exit(1)
    post("/v1/stop"); s = wait(("BRAKED_DISARMED",), 20); c.close()
    log("stopped:", row(s))
    if s["rig_state"] != "BRAKED_DISARMED": sys.exit(1)
    time.sleep(1.0)
    # ARM → CENTER を 20 Hz で記録
    post("/v1/arm"); s = wait(("HOLDING",), 10); log("armed:", row(s))
    if s["rig_state"] != "HOLDING": sys.exit(1)
    log("CENTER ->", post("/v1/center")[1].get("rig_state")); t0 = time.time()
    while time.time() - t0 < 6:
        s = st(); log(f"  +{time.time()-t0:4.2f}s", row(s))
        if s["rig_state"] == "FAULT_LATCHED": log("FAULT", s["fault"]); sys.exit(2)
        if s["rig_state"] == "HOLDING" and time.time() - t0 > 1.5: break
        time.sleep(0.05)
    post("/v1/stop"); s = wait(("BRAKED_DISARMED", "READY_TO_ARM"), 20); log("end:", row(s)); time.sleep(1.0)
log("done")
