#!/usr/bin/env python
"""Issue #25 の合意手順用の計測（基準 → A）。READY_TO_ARM / BRAKED_DISARMED から:
H: ARM → 中立へ CENTER → 保持 60 s（補正バースト = Duty 0 → 非 0 の立ち上がり回数、最大誤差、位置の振れ）
S: 自前 WS で正弦波 120 s（pitch ±0.4 @0.07 Hz、roll ±0.4 @0.10 Hz、gain 0.3）: 最大誤差、速度追従 RMS 誤差
   （実測速度 − 軌道速度の差の RMS。軌道速度は target_count の差分から推定）、余分な反転（Duty 符号反転のうち軌道速度の符号反転と一致しないもの）、床張りつき率
中止: FAULT、|誤差| > 250、|誤差| > 20 のまま 1 s 無パルス。
使い方: hw_ab2.py <label> <logfile> [hold_s] [sine_s]
"""
import json, math, sys, time, urllib.request
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[1]))
from h3_motion_client import MotionClient

B = __import__("os").environ.get("H3CS_URL", "http://127.0.0.1:8892")
LABEL = sys.argv[1]; LOG = open(sys.argv[2], "a", buffering=1)
HOLD_S = float(sys.argv[3]) if len(sys.argv) > 3 else 60.0
SINE_S = float(sys.argv[4]) if len(sys.argv) > 4 else 120.0
REAL = (1, 2)

def ts(): return time.strftime("%H:%M:%S") + ".%03d" % int((time.time() % 1) * 1000)
def log(*a):
    line = ts() + f" [{LABEL}] " + " ".join(str(x) for x in a); print(line); LOG.write(line + "\n")
def post(p, body=None):
    req = urllib.request.Request(B + p, data=json.dumps(body or {}).encode(), headers={"Content-Type": "application/json"}, method="POST")
    try:
        r = urllib.request.urlopen(req, timeout=3); return r.status, json.loads(r.read().decode())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read().decode() or "{}")
import collections
RING = collections.deque(maxlen=40)
def st():
    s = json.load(urllib.request.urlopen(B + "/v1/status", timeout=2))
    try:
        a = [x for x in s["axes"] if x["axis_id"] == 1][0]
        RING.append(f'{time.strftime("%H:%M:%S")}.{int((time.time()%1)*1000):03d} {s["rig_state"]} t={a["target_count"]} c={a["actual_count"]} e={a["following_error_count"]} pwm={a["pwm_permille"]} v={a["velocity_count_s"]} pw={a.get("power_on_cmd")} br={a.get("brake_release_cmd")}')
    except Exception: pass
    return s
def axes(s): return {a["axis_id"]: a for a in s["axes"]}
def wait_state(target, timeout):
    t0 = time.time()
    while time.time() - t0 < timeout:
        s = st()
        if s["rig_state"] in target or s["rig_state"] == "FAULT_LATCHED": return s
        time.sleep(0.1)
    return st()
def bail(msg):
    log("!!! ABORT:", msg)
    for r in RING: log("   ring a1:", r)
    log("STOP ->", post("/v1/stop"))
    s = wait_state(("BRAKED_DISARMED",), 20); log("after stop:", s["rig_state"], s["fault"]); sys.exit(1)

class Track:
    """1 軸分の時系列統計。"""
    def __init__(self, floor_permille):
        self.n = 0; self.max_e = 0; self.sum_e2 = 0; self.bursts = 0; self.last_p = 0
        self.flips = 0; self.extra_flips = 0; self.floor = 0; self.fp = floor_permille
        self.prev_t = None; self.prev_target = None; self.sum_v2 = 0; self.nv = 0
        self.minc = 1e9; self.maxc = -1e9; self.no_move_since = None; self.last_count = None
        self.dp = []; self.rows = []
    def add(self, t, a):
        e = a["following_error_count"] or 0; p = a["pwm_permille"] or 0; c = a["actual_count"]; v = a["velocity_count_s"] or 0
        self.n += 1; self.max_e = max(self.max_e, abs(e)); self.sum_e2 += e * e
        if self.n > 1: self.dp.append(abs(p - self.last_p))
        self.rows.append((round(t, 3), a["target_count"], c, e, p, v))
        self.minc = min(self.minc, c); self.maxc = max(self.maxc, c)
        if p and not self.last_p: self.bursts += 1
        sg = (p > 0) - (p < 0); lsg = (self.last_p > 0) - (self.last_p < 0)
        traj_v = None
        if self.prev_t is not None and t > self.prev_t:
            traj_v = (a["target_count"] - self.prev_target) / (t - self.prev_t)
            self.sum_v2 += (v - traj_v) ** 2; self.nv += 1
        if sg and lsg and sg != lsg:
            self.flips += 1
            # 軌道速度の符号と Duty の符号が一致していれば「意図した反転」
            if traj_v is None or (traj_v > 0) != (sg > 0): self.extra_flips += 1
        if p and self.fp - 3 <= abs(p) <= self.fp + 3: self.floor += 1
        if p and abs(p) < self.fp - 3: self.subfloor = getattr(self, "subfloor", 0) + 1
        if abs(e) > 10: self.err_gt10 = getattr(self, "err_gt10", 0) + 1
        self.sum_absp = getattr(self, "sum_absp", 0) + abs(p)
        # 無パルス監視: |誤差| > 20 で count が 1 s 変わらない
        if self.last_count is not None and c == self.last_count and abs(e) > 20:
            if self.no_move_since is None: self.no_move_since = t
        else:
            self.no_move_since = None
        self.last_p = p; self.prev_t = t; self.prev_target = a["target_count"]; self.last_count = c
    def stuck(self, t): return self.no_move_since is not None and t - self.no_move_since > 1.0
    def text(self):
        if not self.n: return "n=0"
        rms_e = math.sqrt(self.sum_e2 / self.n); rms_v = math.sqrt(self.sum_v2 / self.nv) if self.nv else float("nan")
        sub = getattr(self, "subfloor", 0); gt = getattr(self, "err_gt10", 0); sp = getattr(self, "sum_absp", 0)
        dps = sorted(self.dp); p95 = dps[int(0.95 * (len(dps) - 1))] / 10 if dps else 0
        return (f"dpwm_p95={p95:.1f}% n={self.n} |err| max={self.max_e} rms={rms_e:.1f} err>10={100*gt/self.n:.0f}% vel_rms_err={rms_v:.0f} bursts={self.bursts} "
                f"stall_s={getattr(self, 'stall_s', 0):.1f} flips={self.flips} extra={self.extra_flips} at_floor={100*self.floor/self.n:.0f}% subfloor={100*sub/self.n:.0f}% mean|pwm|={sp/self.n/10:.1f}% range={self.minc}..{self.maxc}")

s = st(); fw = {i: axes(s)[i]["fw_limits"] for i in REAL}
floor = {i: (fw[i]["duty_min_percent"] if fw[i] else 9) * 10 for i in REAL}
log("start:", s["rig_state"], "limits", s["motion_limits"]["effective"], "fw", {k: fw[1][k] for k in ("duty_min_percent", "kp_percent_per_100", "duty_max_percent", "ff_count_s_per_percent")} if fw[1] else None)
if s["rig_state"] not in ("READY_TO_ARM", "BRAKED_DISARMED"): bail("not armable")

# ---- H: ARM → CENTER → hold
log("H: ARM ->", post("/v1/arm")[1].get("rig_state"))
s = wait_state(("HOLDING",), 10)
if s["rig_state"] != "HOLDING": bail(f"ARM ended in {s['rig_state']} {s['fault']}")
log("H: CENTER ->", post("/v1/center")[1].get("rig_state"))
t0 = time.time()
while time.time() - t0 < 40:
    s = st()
    if s["rig_state"] == "FAULT_LATCHED": bail(f"fault during CENTER {s['fault']}")
    if s["rig_state"] == "HOLDING" and time.time() - t0 > 1: break
    time.sleep(0.1)
log(f"H: centered after {time.time()-t0:.1f}s; holding {HOLD_S:.0f}s")
hold = {i: Track(floor[i]) for i in REAL}; t0 = time.time(); t_settle = 5.0
while time.time() - t0 < HOLD_S:
    s = st(); a = axes(s); t = time.time()
    if s["rig_state"] != "HOLDING": bail(f"left HOLDING {s['rig_state']} {s['fault']}")
    if t - t0 > t_settle:
        for i in REAL:
            hold[i].add(t, a[i])
            if hold[i].stuck(t):
                if i == 1: bail("axis 1 no pulses for 1 s with |err| > 20 during hold")
                hold[i].stall_s = getattr(hold[i], "stall_s", 0) + 0.1
    time.sleep(0.1)
for i in REAL: log(f"H: hold a{i}:", hold[i].text())

# ---- S: sine
client = MotionClient(B); client.start(); t0 = time.time()
while time.time() - t0 < 10 and not client.connected: time.sleep(0.1)
if not client.connected: client.close(); bail("client could not connect")
fs = 0
for _ in range(10): fs += 1; client.submit_pose(0.0, 0.0, 0.0, 0.3, frame_seq=fs); time.sleep(0.02)
log("S: START ->", post("/v1/start"))
s = wait_state(("ACTIVE",), 5)
if s["rig_state"] != "ACTIVE": client.close(); bail(f"START ended in {s['rig_state']} {s['fault']}")
sine = {i: Track(floor[i]) for i in REAL}; AMP = 0.4; GAIN = 0.3; t0 = time.time(); nxt = 0; prog = 0
while True:
    t = time.time() - t0
    if t > SINE_S: break
    fs += 1; client.submit_pose(AMP * math.sin(2 * math.pi * 0.07 * t), AMP * math.sin(2 * math.pi * 0.10 * t), 0.0, GAIN, frame_seq=fs)
    if t >= nxt:
        nxt = t + 0.1; s = st(); a = axes(s); now = time.time()
        if s["rig_state"] != "ACTIVE": client.close(); bail(f"left ACTIVE {s['rig_state']} {s['fault']}")
        for i in REAL:
            sine[i].add(now, a[i])
            if abs(a[i]["following_error_count"] or 0) > 250: client.close(); bail(f"following error {a[i]['following_error_count']} on axis {i}")
            if sine[i].stuck(now):
                if i == 1: client.close(); bail("axis 1 no pulses for 1 s with |err| > 20 during sine")
                sine[i].stall_s = getattr(sine[i], "stall_s", 0) + 0.1
        if t >= prog: prog = t + 30; log(f"S: {t:5.1f}s", " | ".join(f"a{i} e={a[i]['following_error_count']} pwm={a[i]['pwm_permille']}" for i in REAL))
    time.sleep(0.02)
for i in REAL: log(f"S: sine a{i}:", sine[i].text())
import csv
with open(f"{sys.argv[2].rsplit('/',1)[0]}/samples_{LABEL}.csv", "w", newline="") as f:
    w = csv.writer(f); w.writerow(["phase", "axis", "t", "target", "actual", "err", "pwm", "vel"])
    for ph, tr in (("hold", hold), ("sine", sine)):
        for i in REAL:
            for r in tr[i].rows: w.writerow([ph, i, *r])
log("S: dec_err", [x["decode_errors"] for x in s["axes"]], "rx_err", [x["rx_error_count"] for x in s["axes"]])

# ---- STOP
log("STOP ->", post("/v1/stop")[1].get("rig_state")); t0 = time.time()
s = wait_state(("BRAKED_DISARMED",), 20)
log(f"end: {s['rig_state']} after {time.time()-t0:.1f}s fault={s['fault']}")
client.close()
