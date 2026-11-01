"""3 系統を保護付きの自動移動で動かす。
usage: drive_axes.py <logfile> <mode:seq|par> <label=port> [<label=port> ...]
各軸: ? で校正確認 → R 1 / B 1（必要なら）→ LIVE 待ち → HOME → NEUTRAL → UPPER → NEUTRAL → HOME → NEUTRAL
seq: 1 軸ずつ。par: 全軸同時（各軸は上の列を独立に進める）。失敗/拒否が出た軸はそこで止める（X を送る）。
"""
import serial, time, sys, re, datetime

log = open(sys.argv[1], "a", buffering=1)
def now(): return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
def out(msg):
    line = f"{now()} {msg}"
    log.write(line + "\n"); print(line, flush=True)

class Axis:
    def __init__(self, label, port):
        self.label, self.port = label, port
        s = serial.Serial(); s.port = port; s.baudrate = 115200; s.timeout = 0
        s.dtr = True; s.rts = True; s.open()
        self.s = s; self.buf = b""; self.lines = []
        self.stat = {}; self.enc = {}
        self.failed = None
        self.steps = []; self.waiting = None; self.deadline = 0; self.last_enc_print = 0
    def send(self, cmd):
        out(f"[{self.label}] > {cmd}")
        self.s.write((cmd + "\n").encode())
    def pump(self):
        chunk = self.s.read(4096)
        if chunk:
            self.buf += chunk
            while b"\n" in self.buf:
                raw, self.buf = self.buf.split(b"\n", 1)
                line = raw.decode("utf-8", "replace").rstrip()
                if not line: continue
                self.lines.append(line)
                self.parse(line)
                quiet = line.startswith("ENC ") or line.startswith("STAT ")
                if quiet:
                    if time.time() - self.last_enc_print >= 2.0 or not self.waiting:
                        self.last_enc_print = time.time()
                        log.write(f"{now()} [{self.label}] {line}\n")
                else:
                    out(f"[{self.label}] {line}")
    def parse(self, line):
        if line.startswith("STAT ") or line.startswith("OK R="):
            self.stat = dict(kv.split("=", 1) for kv in line.split()[1:] if "=" in kv)
        elif line.startswith("ENC "):
            self.enc = dict(kv.split("=", 1) for kv in line.split()[1:] if "=" in kv)
    def query(self, wait=1.0):
        self.send("?"); t0 = time.time()
        while time.time() - t0 < wait: self.pump(); time.sleep(0.02)
        return self.stat, self.enc

def wait_auto(ax, kind, timeout):
    """AUTO <kind> done|failed|cancelled または ERR auto_ を待つ。"""
    t0 = time.time(); seen = len(ax.lines)
    while time.time() - t0 < timeout:
        ax.pump()
        for line in ax.lines[seen:]:
            if line.startswith("ERR auto_"):
                return ("rejected", line)
            m = re.match(rf"AUTO {kind} (done|failed|cancelled) ?(\S*) pos=(-?\d+)", line)
            if m:
                return (m.group(1), line)
        seen = len(ax.lines)
        time.sleep(0.02)
    return ("timeout", "")

SEQ = [("HOME", "home", 40), ("NEUTRAL", "neutral", 40), ("UPPER", "upper", 60),
       ("NEUTRAL", "neutral", 40), ("HOME", "home", 60), ("NEUTRAL", "neutral", 40)]

def prepare(ax):
    stat, enc = ax.query()
    out(f"[{ax.label}] state motor={enc.get('motor')} cal={enc.get('cal')} stroke={enc.get('stroke')} "
        f"neutral={enc.get('neutral')} R={stat.get('R')} B={stat.get('B')} LIVE={stat.get('LIVE')} count={enc.get('count')}")
    if enc.get("cal") != "1":
        ax.failed = "not calibrated"; return False
    if enc.get("auto") not in (None, "idle") and enc.get("phase") in ("moving", "dwell"):
        ax.failed = "already moving"; return False
    if stat.get("S") not in (None, "0"):
        ax.send("S 0"); time.sleep(0.3)
    turned_on = False
    if stat.get("R") != "1":
        ax.send("R 1"); turned_on = True; time.sleep(0.3)
    if stat.get("B") != "1":
        ax.send("B 1"); turned_on = True; time.sleep(0.3)
    if turned_on:
        out(f"[{ax.label}] 電源/ブレーキを入れたので 6 s 待つ（24V 立ち上がり）")
        time.sleep(6)
    stat, enc = ax.query()
    if stat.get("LIVE") != "1":
        ax.failed = f"not live (R={stat.get('R')} B={stat.get('B')})"; return False
    return True

def run_step(ax, cmd, kind, timeout):
    ax.send(cmd)
    result, line = wait_auto(ax, kind, timeout)
    out(f"[{ax.label}] {cmd} -> {result} {line}")
    if result != "done":
        ax.failed = f"{cmd}: {result} {line}"
        ax.send("X")
        return False
    return True

def run_mode(axes, mode):
    out(f"# run mode={mode}")
    for ax in axes: ax.failed = None; ax.pump()
    ready = [ax for ax in axes if prepare(ax)]
    for ax in axes:
        if ax.failed: out(f"[{ax.label}] SKIP: {ax.failed}")
    if mode == "seq":
        for ax in ready:
            for cmd, kind, to in SEQ:
                if not run_step(ax, cmd, kind, to): break
                time.sleep(1.0)
    else:
        # 全軸同時: 各軸にステップ列を持たせ、完了した軸から次へ進める
        for ax in ready: ax.steps = list(SEQ); ax.waiting = None
        active = list(ready)
        while active:
            for ax in list(active):
                ax.pump()
                if ax.waiting is None:
                    if not ax.steps: active.remove(ax); continue
                    cmd, kind, to = ax.steps.pop(0)
                    ax.send(cmd); ax.waiting = (cmd, kind); ax.deadline = time.time() + to; ax.seen = len(ax.lines)
                    continue
                cmd, kind = ax.waiting
                res = None
                for line in ax.lines[ax.seen:]:
                    if line.startswith("ERR auto_"): res = ("rejected", line); break
                    m = re.match(rf"AUTO {kind} (done|failed|cancelled) ?(\S*) pos=(-?\d+)", line)
                    if m: res = (m.group(1), line); break
                ax.seen = len(ax.lines)
                if res is None and time.time() > ax.deadline: res = ("timeout", "")
                if res is not None:
                    out(f"[{ax.label}] {cmd} -> {res[0]} {res[1]}")
                    if res[0] != "done":
                        ax.failed = f"{cmd}: {res[0]} {res[1]}"; ax.send("X"); active.remove(ax)
                    else:
                        ax.waiting = None; ax.next_at = time.time() + 1.0
            time.sleep(0.02)
    time.sleep(1.0)
    for ax in axes:
        stat, enc = ax.query()
        out(f"[{ax.label}] final pos={enc.get('pos')} origin={enc.get('origin')} stroke={enc.get('stroke')} "
            f"R={stat.get('R')} B={stat.get('B')} LIVE={stat.get('LIVE')} auto={enc.get('auto')}/{enc.get('phase')}/{enc.get('reason')}"
            + (f"  FAILED: {ax.failed}" if ax.failed else ""))
    out(f"# run end mode={mode}")

def main():
    # ポートは開いたまま保持する（ホスト側で閉じた直後にボードが USB に応答しなくなる現象があるため）。
    # 追加の実行は drive_cmd.txt に seq / par / quit を書く。
    axes = []
    for spec in sys.argv[3:]:
        label, port = spec.split("=", 1)
        axes.append(Axis(label, port))
    out(f"# drive start axes={[a.label + '@' + a.port for a in axes]}")
    time.sleep(1.5)
    run_mode(axes, sys.argv[2])
    import os
    cmdfile = "drive_cmd.txt"
    out("# holding ports open; waiting for drive_cmd.txt (seq|par|quit)")
    while True:
        for ax in axes: ax.pump()
        if os.path.exists(cmdfile):
            cmd = open(cmdfile).read().strip(); os.remove(cmdfile)
            if cmd == "quit":
                out("# quit requested; closing ports"); break
            if cmd in ("seq", "par"):
                run_mode(axes, cmd)
            else:
                out(f"# unknown cmd {cmd!r}")
        time.sleep(0.2)

main()
