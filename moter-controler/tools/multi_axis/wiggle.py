"""3 軸同時の複合動作（目視確認用、約 60 s）。
usage: wiggle.py <logfile> <label=port> ...
HOME(原点が無い軸のみ) → UPPER → NEUTRAL → 手動速度で往復 25 s（位置を監視して端の 350 手前で折り返し）→ UPPER → NEUTRAL
"""
import serial, time, sys, re, datetime, os

log = open(sys.argv[1], "a", buffering=1)
def now(): return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
def out(msg):
    line = f"{now()} {msg}"; log.write(line + "\n"); print(line, flush=True)

class Axis:
    def __init__(self, label, port):
        self.label, self.port = label, port
        s = serial.Serial(); s.port = port; s.baudrate = 115200; s.timeout = 0
        s.dtr = True; s.rts = True; s.open()
        self.s = s; self.buf = b""; self.lines = []; self.stat = {}; self.enc = {}
        self.failed = None; self.ready = False; self.speed = 0; self.last_q = 0
    def send(self, cmd):
        out(f"[{self.label}] > {cmd}"); self.s.write((cmd + "\n").encode())
    def pump(self):
        chunk = self.s.read(4096)
        if not chunk: return
        self.buf += chunk
        while b"\n" in self.buf:
            raw, self.buf = self.buf.split(b"\n", 1)
            line = raw.decode("utf-8", "replace").rstrip()
            if not line: continue
            self.lines.append(line)
            if line.startswith("STAT ") or line.startswith("OK R="):
                self.stat = dict(kv.split("=", 1) for kv in line.split()[1:] if "=" in kv)
            elif line.startswith("ENC "):
                self.enc = dict(kv.split("=", 1) for kv in line.split()[1:] if "=" in kv)
            if line.startswith(("ENC ", "STAT ")):
                log.write(f"{now()} [{self.label}] {line}\n")
            else:
                out(f"[{self.label}] {line}")
    def query(self, wait=0.8):
        self.send("?"); t0 = time.time()
        while time.time() - t0 < wait: self.pump(); time.sleep(0.02)
        return self.stat, self.enc
    def pos(self):
        try: return int(self.enc.get("pos"))
        except (TypeError, ValueError): return None
    def stroke(self):
        try: return int(self.enc.get("stroke"))
        except (TypeError, ValueError): return 0

def prepare(ax):
    stat, enc = ax.query()
    out(f"[{ax.label}] state motor={enc.get('motor')} cal={enc.get('cal')} stroke={enc.get('stroke')} origin={enc.get('origin')} "
        f"R={stat.get('R')} B={stat.get('B')} LIVE={stat.get('LIVE')} pos={enc.get('pos')} auto={enc.get('auto')}/{enc.get('phase')}")
    if not enc:
        ax.failed = "no response"; return False
    if enc.get("cal") != "1":
        ax.failed = "not calibrated"; return False
    if enc.get("phase") in ("moving", "dwell"):
        ax.failed = "already moving"; return False
    if stat.get("S") not in (None, "0"): ax.send("S 0"); time.sleep(0.3)
    on = False
    if stat.get("R") != "1": ax.send("R 1"); on = True; time.sleep(0.3)
    if stat.get("B") != "1": ax.send("B 1"); on = True; time.sleep(0.3)
    if on:
        out(f"[{ax.label}] 電源/ブレーキを入れたので 6 s 待つ"); time.sleep(6)
    stat, enc = ax.query()
    if stat.get("LIVE") != "1":
        ax.failed = f"not live (R={stat.get('R')} B={stat.get('B')})"; return False
    ax.ready = True; return True

def auto_all(axes, cmd, kind, timeout, only=None):
    """全軸に同時に自動移動を出し、全部の結果を待つ。"""
    targets = [a for a in axes if a.ready and not a.failed and (only is None or only(a))]
    if not targets: return
    for a in targets:
        a.seen = len(a.lines); a.send(cmd); a.res = None
    deadline = time.time() + timeout
    while time.time() < deadline and any(a.res is None for a in targets):
        for a in targets:
            a.pump()
            if a.res is not None: continue
            for line in a.lines[a.seen:]:
                if line.startswith("ERR auto_"): a.res = ("rejected", line); break
                m = re.match(rf"AUTO {kind} (done|failed|cancelled) ?(\S*) pos=(-?\d+)", line)
                if m: a.res = (m.group(1), line); break
            a.seen = len(a.lines)
        time.sleep(0.02)
    for a in targets:
        if a.res is None: a.res = ("timeout", "")
        out(f"[{a.label}] {cmd} -> {a.res[0]} {a.res[1]}")
        if a.res[0] != "done":
            a.failed = f"{cmd}: {a.res[0]}"; a.send("X")

# 手動往復のパターン（speed, 秒）。speed + は P 端向き（pos 減、座席は沈む側）、− は N 端向き（持ち上げ側）。
PATTERNS = {
    "#1": [(-12, 2.2), (10, 2.0), (-15, 1.5), (8, 2.5)],
    "#2": [(10, 2.0), (-13, 2.2), (9, 1.8), (-15, 1.5)],
    "#3": [(-10, 1.2), (10, 1.2), (-15, 0.9), (15, 0.9), (-8, 1.6), (8, 1.6)],
}
MARGIN = 600

def manual_phase(axes, duration):
    live = [a for a in axes if a.ready and not a.failed]
    for a in live:
        a.pat = PATTERNS.get(a.label, PATTERNS["#3"]); a.idx = -1; a.seg_until = 0; a.speed = 0; a.warned = len(a.lines)
    t_end = time.time() + duration
    while time.time() < t_end:
        t = time.time()
        for a in live:
            a.pump()
            # 守りが働いたら（WARN manual_）速度 0 のまま次の区間へ
            for line in a.lines[a.warned:]:
                if line.startswith("WARN manual_"):
                    a.speed = 0; a.seg_until = t + 0.8
            a.warned = len(a.lines)
            if t - a.last_q >= 0.25:
                a.last_q = t; a.s.write(b"?\n")
            p = a.pos(); hi = a.stroke() - MARGIN
            if p is not None and a.speed != 0:
                if (p < MARGIN and a.speed > 0) or (p > hi and a.speed < 0):
                    out(f"[{a.label}] 端の手前 pos={p} で折り返し")
                    a.speed = -a.speed if abs(a.speed) >= 10 else (-12 if a.speed > 0 else 10)
                    a.send("S 0"); time.sleep(0.25)
                    a.send(f"S {a.speed}"); a.seg_until = t + 1.5
                    continue
            if t >= a.seg_until:
                a.idx = (a.idx + 1) % len(a.pat)
                sp, dur = a.pat[a.idx]
                # 端に近いときは離れる向きにする
                if p is not None:
                    if p < MARGIN and sp > 0: sp = -abs(sp) if abs(sp) >= 10 else -12
                    if p > hi and sp < 0: sp = abs(sp)
                if a.speed != 0 and (a.speed > 0) != (sp > 0):
                    a.send("S 0"); time.sleep(0.25)
                a.speed = sp; a.send(f"S {sp}"); a.seg_until = t + dur
        time.sleep(0.05)
    for a in live:
        a.speed = 0; a.send("S 0")
    time.sleep(1.0)
    for a in live: a.pump()

def main():
    axes = [Axis(*spec.split("=", 1)) for spec in sys.argv[2:]]
    out(f"# wiggle start axes={[a.label + '@' + a.port for a in axes]}")
    time.sleep(1.5)
    for a in axes: a.pump(); prepare(a)
    for a in axes:
        if a.failed: out(f"[{a.label}] SKIP: {a.failed}")
    mode = os.environ.get("WIGGLE_MODE", "full")
    auto_all(axes, "HOME", "home", 40, only=lambda a: a.enc.get("origin") != "1")
    auto_all(axes, "UPPER", "upper", 60)
    if mode in ("full", "neutral"):
        auto_all(axes, "NEUTRAL", "neutral", 40)
    if mode != "neutral":
        secs = int(os.environ.get("WIGGLE_MANUAL_S", "25"))
        out(f"# 手動往復 {secs} s")
        manual_phase(axes, secs)
    if mode == "full":
        auto_all(axes, "UPPER", "upper", 60)
        auto_all(axes, "NEUTRAL", "neutral", 40)
    elif mode == "manual":
        auto_all(axes, "HOME", "home", 60)
    for a in axes:
        stat, enc = a.query()
        out(f"[{a.label}] final pos={enc.get('pos')} R={stat.get('R')} B={stat.get('B')} S={stat.get('S')} auto={enc.get('auto')}/{enc.get('phase')}/{enc.get('reason')}"
            + (f"  FAILED: {a.failed}" if a.failed else ""))
    out("# wiggle end; holding ports open (write quit to wiggle_cmd.txt)")
    while True:
        for a in axes: a.pump()
        if os.path.exists("wiggle_cmd.txt"):
            os.remove("wiggle_cmd.txt"); out("# quit"); break
        time.sleep(0.2)

main()
