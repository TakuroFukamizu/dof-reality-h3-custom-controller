#!/usr/bin/env python3
"""Scripted encoder-tester session over USB.

Modes
  dry   SET + CONFIG readback + DISARMED baseline poll. No ARM. No relay. No PWM.
  idle  dry, then one ARM without RUN, wait for DISARMED/FAULT, collect TRACE.
  run   dry, then a ladder of ARM -> RUN -> DISARMED trials in one port open.

The SET values live in SETTINGS below and in docs/set_proposal.md. They are
written on every open because opening the port resets the chip (RAM only).

Refuses to continue when the board does not identify itself as encoder-tester
(STAT R=... is the old moter-controler firmware), when a BOOT line appears
after the first boot (reset or power drop during a trial), or when the CONFIG
readback differs from what was sent.

Never raises the duty cap mid-session. Never re-ARMs after FAULT.
Ctrl+C sends one STOP, not ESTOP.
"""

import argparse
import os
import re
import sys
import threading
import time
from datetime import datetime

import serial

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOG_DIR = os.path.join(ROOT, "logs")
RUNS_DIR = os.path.join(ROOT, "docs", "runs")

HEARTBEAT_S = 0.2
BOOT_WAIT_S = 15.0
REPLY_WAIT_S = 3.0

# Rationale per key: docs/set_proposal.md. Order is the SET order.
SETTINGS = [
    ("MAX_DUTY_PERCENT", 25),
    ("MAX_RUN_MS", 600),
    ("MAX_TRAVEL_COUNTS", 3000),
    ("POWER_SETTLE_MS", 1500),
    ("BRAKE_RELEASE_WAIT_MS", 300),
    ("BRAKE_APPLY_WAIT_MS", 300),
    ("STOP_SPEED_THRESHOLD", 20),
    ("STOP_CONFIRM_MS", 200),
    ("STOP_TIMEOUT_MS", 2000),
    ("READY_HOLD_MS", 3000),
    ("START_GRACE_MS", 200),
    ("MAX_COUNT_PER_S", 1000000),
    ("UI_TIMEOUT_MS", 1000),
    ("LEASE_MS", 2000),
    ("JOG_STALE_MS", 150),
    ("JOG_DUTY", 10),
    ("ENCODER_SIGN", 1),
    ("GLITCH_NS", 0),
]

ACTIVE = dict(SETTINGS)  # replaced in main() with the session's effective values

CSV_FIELDS = (
    "seq", "t_us", "trial_id", "axis", "state", "power_on_cmd", "power_gpio_level",
    "brake_release_cmd", "brake_gpio_level", "requested_command", "requested_unit",
    "applied_duty_pct", "pwm_raw", "pwm_full_scale", "pwm_hz", "dir_level",
    "count_total_x4", "count_trial_x4", "velocity_count_s", "a_level", "b_level",
    "elapsed_ms", "stop_reason", "log_drop_count",
)


class Abort(Exception):
    pass


def parse_csv(line):
    if not line or not line[0].isdigit():
        return None
    parts = line.split(",")
    if len(parts) != len(CSV_FIELDS):
        return None
    return dict(zip(CSV_FIELDS, parts))


def parse_stat(line):
    if not line.startswith("STAT "):
        return {}
    return dict(re.findall(r"([A-Za-z_]+)=([^\s]*)", line))


def parse_cfg(line):
    match = re.match(r"^CFG ([A-Z0-9_]+)=(-?\d+)\b", line)
    return (match.group(1), match.group(2)) if match else None


def parse_trial(spec):
    """'10:+' -> (10, 'positive', None); '15:-:400' -> (15, 'negative', 400). Third field overrides --duration-ms."""
    parts = spec.split(":")
    if len(parts) not in (2, 3) or parts[1] not in ("+", "-") or not parts[0].isdigit():
        raise argparse.ArgumentTypeError("trial must look like 10:+ or 15:-:400")
    duration = int(parts[2]) if len(parts) == 3 and parts[2].isdigit() else None
    return int(parts[0]), "positive" if parts[1] == "+" else "negative", duration


class Session:
    def __init__(self, port, mode, stamp, verbose):
        self.port = port
        self.mode = mode
        self.stamp = stamp
        self.verbose = verbose
        self.ser = None
        self.lock = threading.Lock()
        self.log = open(os.path.join(LOG_DIR, "session_%s_%s.log" % (mode, stamp)), "x", encoding="utf-8")
        self.csv = open(os.path.join(LOG_DIR, "session_%s_%s.csv" % (mode, stamp)), "x", encoding="utf-8")
        self.csv.write(",".join(CSV_FIELDS) + "\n")
        self.t0 = time.monotonic()
        self.boots = 0
        self.lines = []           # (pc_s, text) everything received
        self.rows = []            # parsed CSV rows
        self.events = []          # EVT lists
        self.trace = []           # TRACE lines
        self.trace_end = None
        self.stat = {}
        self.heartbeat_on = False
        self.hb_thread = None
        self.reader_stop = threading.Event()
        self.reader_thread = None
        self.pending = []         # lines not yet consumed by wait()
        self.pending_cv = threading.Condition()
        self.reset_seen = False
        self.gpio_errors = 0

    # ---- transport -------------------------------------------------------
    def note(self, text):
        line = "%8.3f  %s" % (time.monotonic() - self.t0, text)
        self.log.write(line + "\n")
        self.log.flush()
        if self.verbose:
            print(line, flush=True)

    def open(self):
        link = serial.Serial()
        link.port = self.port
        link.baudrate = 115200
        link.timeout = 0.1
        link.write_timeout = 1
        link.dtr = True
        link.rts = True
        link.open()
        self.ser = link
        self.reader_thread = threading.Thread(target=self._reader, daemon=True)
        self.reader_thread.start()
        self.note("# opened %s" % self.port)

    def close(self):
        self.stop_heartbeat()
        self.reader_stop.set()
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.log.close()
        self.csv.close()

    def _reader(self):
        while not self.reader_stop.is_set():
            try:
                raw = self.ser.readline()
            except Exception as exc:
                self.note("# READER %s" % exc)
                self.reset_seen = True
                return
            if not raw:
                continue
            text = raw.decode("utf-8", "replace").replace("\x00", "").strip()
            if not text:
                continue
            now = time.monotonic() - self.t0
            self.lines.append((now, text))
            self.log.write("%8.3f  %s\n" % (now, text))
            row = parse_csv(text)
            if row is not None:
                self.rows.append(row)
                self.csv.write(text + "\n")
            elif text.startswith("EVT,"):
                self.events.append(text.split(","))
            elif text.startswith("TRACE"):
                self.trace.append(text)
                if text.startswith("TRACE end"):
                    self.trace_end = text
            elif text.startswith("STAT "):
                self.stat.update(parse_stat(text))
            if text.startswith("BOOT state="):
                self.boots += 1
                if self.boots > 1:
                    self.reset_seen = True
            if "is not set as GPIO" in text:
                self.gpio_errors += 1
            if self.verbose and row is None and text != "OK heartbeat":
                print("%8.3f  %s" % (now, text), flush=True)
            with self.pending_cv:
                self.pending.append(text)
                self.pending_cv.notify_all()
        self.log.flush()

    def send(self, cmd):
        with self.lock:
            self.ser.write((cmd + "\n").encode("ascii"))
            self.ser.flush()
        if cmd != "HEARTBEAT":
            self.note(">>> " + cmd)

    def wait(self, pred, timeout, what):
        """Block until a received line satisfies pred. Returns the line."""
        deadline = time.monotonic() + timeout
        with self.pending_cv:
            while True:
                while self.pending:
                    line = self.pending.pop(0)
                    if pred(line):
                        return line
                if self.reset_seen:
                    raise Abort("board reset / USB drop while waiting for %s" % what)
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise Abort("timeout waiting for %s" % what)
                self.pending_cv.wait(min(remaining, 0.2))

    def drain(self):
        with self.pending_cv:
            self.pending.clear()

    def start_heartbeat(self):
        self.heartbeat_on = True

        def beat():
            while self.heartbeat_on:
                try:
                    self.send("HEARTBEAT")
                except Exception:
                    return
                time.sleep(HEARTBEAT_S)

        self.hb_thread = threading.Thread(target=beat, daemon=True)
        self.hb_thread.start()

    def stop_heartbeat(self):
        self.heartbeat_on = False

    # ---- protocol steps --------------------------------------------------
    def boot(self):
        """Wait for the encoder-tester banner. Refuse other firmware."""
        try:
            self.wait(lambda l: l == "HELP" or l.startswith("STAT R="), BOOT_WAIT_S, "boot banner")
        except Abort:
            self.note("# no banner; asking STATUS")
            self.send("STATUS")
            self.wait(lambda l: l.startswith("STAT "), REPLY_WAIT_S, "STATUS reply")
        if any(t.startswith("STAT R=") or t.startswith("OK R=") for _s, t in self.lines):
            raise Abort("this board answers like moter-controler (STAT R=...). "
                        "Not encoder-tester. Upload encoder-tester first; do not continue.")
        time.sleep(0.5)
        self.drain()

    def status(self):
        self.send("STATUS")
        self.wait(lambda l: l.startswith("STAT cal="), REPLY_WAIT_S, "STATUS block")
        return dict(self.stat)

    def config(self):
        cfg = {}
        self.drain()
        self.send("CONFIG")
        while True:
            line = self.wait(lambda l: l.startswith("CFG"), REPLY_WAIT_S, "CONFIG block")
            item = parse_cfg(line)
            if item:
                cfg[item[0]] = item[1]
            if line == "CFG -1 means UNCONFIGURED":
                return cfg

    def apply_settings(self, settings):
        before = self.config()
        for key, value in settings:
            self.drain()
            self.send("SET %s %d" % (key, value))
            reply = self.wait(lambda l: l.startswith("OK ") or l.startswith("ERR "), REPLY_WAIT_S, "SET " + key)
            if reply.startswith("ERR"):
                raise Abort("SET %s %d rejected: %s" % (key, value, reply))
        after = self.config()
        mismatch = [(k, v, after.get(k)) for k, v in settings if after.get(k) != str(v)]
        if mismatch:
            raise Abort("CONFIG readback differs: %s" % mismatch)
        return before, after

    def baseline(self, seconds):
        """Poll STATUS while DISARMED. Returns list of (pc_s, count_total, a, b, state)."""
        samples = []
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            st = self.status()
            samples.append((round(time.monotonic() - self.t0, 3), st.get("count_total"), st.get("a"),
                            st.get("b"), st.get("state")))
            time.sleep(0.5)
        return samples

    def arm_and_wait_ready(self):
        self.drain()
        self.send("ARM")
        reply = self.wait(lambda l: l.startswith("OK state=") or l.startswith("ERR "), REPLY_WAIT_S, "ARM reply")
        if reply.startswith("ERR"):
            raise Abort("ARM rejected: " + reply)
        # READY shows in the CSV stream (50 Hz). Power settle + brake wait + margin.
        settle = ACTIVE["POWER_SETTLE_MS"] + ACTIVE["BRAKE_RELEASE_WAIT_MS"]
        line = self.wait(lambda l: (parse_csv(l) or {}).get("state") in ("READY", "DISARMED", "FAULT", "POWER_OFF"),
                         settle / 1000.0 + 4.0, "READY row")
        row = parse_csv(line)
        if row["state"] != "READY":
            raise Abort("left the ready path: state=%s stop=%s" % (row["state"], row["stop_reason"]))
        return row

    def wait_disarmed(self, timeout):
        line = self.wait(lambda l: (parse_csv(l) or {}).get("state") in ("DISARMED", "FAULT"), timeout, "DISARMED row")
        return parse_csv(line)

    def wait_trace(self, timeout=10.0):
        try:
            self.wait(lambda l: l.startswith("TRACE end"), timeout, "TRACE end")
        except Abort as exc:
            self.note("# %s" % exc)


# ---- reporting -----------------------------------------------------------

def rows_between(rows, seq_lo, seq_hi):
    return [r for r in rows if seq_lo <= int(r["seq"]) <= seq_hi]


def summarize_trial(sess, trial_no, duty, direction, duration_ms, seq_start, ready_row, end_row):
    rows = rows_between(sess.rows, seq_start, int(end_row["seq"]))
    running = [r for r in rows if r["state"] == "RUNNING"]
    # trial_id in EVT keeps counting across ARM cycles while the chip is not reset, so match the
    # n-th run EVT of this session rather than the firmware's id.
    runs = [e for e in sess.events if len(e) >= 6 and e[2] == "run"]
    evt_run = runs[trial_no - 1] if len(runs) >= trial_no else None
    evt_off = None
    evt_done = None
    if evt_run is not None:
        idx = sess.events.index(evt_run)
        evt_off = next((e for e in sess.events[idx:] if e[2] == "pwm_off"), None)
        evt_done = next((e for e in sess.events[idx:] if e[2] == "trial_done"), None)
    counts = [int(r["count_total_x4"]) for r in rows]
    vel_run = [int(r["velocity_count_s"]) for r in running]
    out = {
        "trial": trial_no,
        "duty": duty,
        "direction": direction,
        "duration_ms": duration_ms,
        "dir_level": sorted({r["dir_level"] for r in running}),
        "pwm_raw": sorted({r["pwm_raw"] for r in running}),
        "count_ready": int(ready_row["count_total_x4"]),
        "count_end": int(end_row["count_total_x4"]),
        "count_min": min(counts) if counts else None,
        "count_max": max(counts) if counts else None,
        "run_start_count": int(evt_run[5]) if evt_run else None,
        "pwm_off_count": int(evt_off[4]) if evt_off else None,
        "drive_delta": int(evt_done[4]) if evt_done else None,
        "coast": int(evt_done[5]) if evt_done else None,
        "stop_delta": int(evt_done[6]) if evt_done else None,
        "drive_ms": (int(evt_off[1]) - int(evt_run[1])) / 1000.0 if (evt_run and evt_off) else None,
        "stop_reason": evt_off[3] if evt_off else end_row["stop_reason"],
        "vel_run_max_abs": max((abs(v) for v in vel_run), default=None),
        "running_rows": len(running),
        "final_state": end_row["state"],
        "a_levels": sorted({r["a_level"] for r in rows}),
        "b_levels": sorted({r["b_level"] for r in rows}),
    }
    return out


def write_markdown(sess, args, before_cfg, after_cfg, baseline, trials, notes):
    os.makedirs(RUNS_DIR, exist_ok=True)
    name = "session_%s_%s.md" % (sess.mode, sess.stamp)
    path = os.path.join(RUNS_DIR, name)
    L = []
    L.append("# session %s %s" % (sess.mode, sess.stamp))
    L.append("")
    L.append("ポート `%s`。生ログ `logs/session_%s_%s.log`、CSV `logs/session_%s_%s.csv`。" % (
        sess.port, sess.mode, sess.stamp, sess.mode, sess.stamp))
    L.append("")
    L.append("## 結果の境界")
    L.append("")
    L.append("- 分解能: UNKNOWN。外部の角度か回転数を別に測って `CAL ANGLE` / `CAL REVS` に入れるまで、count/rev は出さない。")
    L.append("- positive は DIR Low、negative は DIR High。機構がどちらへ動くかは、この記録だけでは分からない。")
    L.append("- GPIO 列はピンのレベルで、リレー接点が閉じた証拠ではない。a/b は digitalRead の 1 点。")
    L.append("- 起動バナーの回数: %d。2 以上なら途中でリセットがあり、その試験は無効。" % sess.boots)
    L.append("")
    L.append("## SET（送信値と読み返し）")
    L.append("")
    L.append("| キー | 開始時 | 送信 | 読み返し |")
    L.append("| --- | --- | --- | --- |")
    for key, value in SETTINGS:
        L.append("| %s | %s | %d | %s |" % (key, before_cfg.get(key, ""), value, after_cfg.get(key, "")))
    L.append("")
    L.append("## DISARMED 基準（%d 点）" % len(baseline))
    L.append("")
    if baseline:
        counts = [int(s[1]) for s in baseline if s[1] is not None]
        L.append("- count_total: 最初 %s、最後 %s、最小 %s、最大 %s" % (counts[0], counts[-1], min(counts), max(counts)))
        L.append("- a の値: %s、b の値: %s" % (sorted({s[2] for s in baseline}), sorted({s[3] for s in baseline})))
        L.append("")
        L.append("| PC s | count_total | a | b | state |")
        L.append("| --- | --- | --- | --- | --- |")
        for s in baseline:
            L.append("| %s | %s | %s | %s | %s |" % s)
    L.append("")
    if trials:
        L.append("## 試験")
        L.append("")
        cols = ["trial", "duty", "direction", "duration_ms", "dir_level", "pwm_raw", "count_ready", "run_start_count",
                "pwm_off_count", "drive_delta", "coast", "stop_delta", "count_end", "drive_ms", "vel_run_max_abs",
                "stop_reason", "final_state", "a_levels", "b_levels"]
        L.append("| " + " | ".join(cols) + " |")
        L.append("| " + " | ".join("---" for _ in cols) + " |")
        for t in trials:
            L.append("| " + " | ".join(str(t.get(c)) for c in cols) + " |")
        L.append("")
        L.append("drive_delta は EVT trial_done の pwm_off−start。coast は PWM 停止から BRAKE_APPLY 入りまで。stop_delta は stop−start。")
        L.append("")
    if notes:
        L.append("## メモ")
        L.append("")
        for n in notes:
            L.append("- " + n)
        L.append("")
    L.append("## EVT")
    L.append("")
    L.append("```text")
    L.extend(",".join(e) for e in sess.events)
    L.append("```")
    L.append("")
    if sess.trace:
        L.append("## TRACE（制御周期 5 ms。リレー指令の変わった周期は tick と post の 2 行）")
        L.append("")
        L.append("delta が 0 でない行、post 行、状態が変わった行だけを写す。全文は生ログ。")
        L.append("")
        L.append("```text")
        prev_state = None
        for line in sess.trace:
            if not line.startswith("TRACE,"):
                L.append(line)
                continue
            parts = line.split(",")
            if len(parts) < 14:
                continue
            if parts[9] != "0" or parts[13] == "post" or parts[2] != prev_state:
                L.append(line)
            prev_state = parts[2]
        L.append("```")
        L.append("")
    L.append("## CSV で count_total_x4 が変わった行")
    L.append("")
    L.append("| seq | t_us | state | power_gpio | brake_cmd | brake_gpio | pwm_raw | dir | count_total | count_trial | vel | a | b | stop |")
    L.append("| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |")
    prev = None
    for r in sess.rows:
        if prev is not None and r["count_total_x4"] != prev["count_total_x4"]:
            for rr in (prev, r):
                L.append("| %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s |" % (
                    rr["seq"], rr["t_us"], rr["state"], rr["power_gpio_level"], rr["brake_release_cmd"],
                    rr["brake_gpio_level"], rr["pwm_raw"], rr["dir_level"], rr["count_total_x4"],
                    rr["count_trial_x4"], rr["velocity_count_s"], rr["a_level"], rr["b_level"], rr["stop_reason"]))
        prev = r
    L.append("")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(L) + "\n")
    return path


# ---- step mode -----------------------------------------------------------

def do_one_run(sess, limits, n, duty, direction, duration_ms, trials, notes):
    seq_start = int(sess.rows[-1]["seq"]) if sess.rows else 0
    ready = sess.arm_and_wait_ready()
    sess.drain()
    sess.send("RUN duty_pct=%d direction=%s duration_ms=%d" % (duty, direction, duration_ms))
    reply = sess.wait(lambda l: l.startswith("OK state=") or l.startswith("ERR "), REPLY_WAIT_S, "RUN reply")
    if reply.startswith("ERR"):
        notes.append("step %d: RUN 拒否 %s" % (n, reply))
        end = sess.wait_disarmed(limits["READY_HOLD_MS"] / 1000.0 + 6.0)
        trials.append(summarize_trial(sess, n, duty, direction, duration_ms, seq_start, ready, end))
        return trials[-1]
    end = sess.wait_disarmed(duration_ms / 1000.0 + limits["STOP_TIMEOUT_MS"] / 1000.0 +
                             limits["BRAKE_APPLY_WAIT_MS"] / 1000.0 + 5.0)
    time.sleep(0.5)
    t = summarize_trial(sess, n, duty, direction, duration_ms, seq_start, ready, end)
    trials.append(t)
    return t


def run_step_mode(sess, args, limits, trials, notes):
    cmd_file = args.cmd_file
    if os.path.isfile(cmd_file):
        os.unlink(cmd_file)
    st = sess.status()
    ref_count = None
    n = 0
    sess.drain()
    sess.send("VISUAL CONFIG shaft=%s reference=tape_line direction=negative revolutions=1 max_drive_ms=%d max_travel_counts=%d" % (
        args.shaft, limits["MAX_RUN_MS"], limits["MAX_TRAVEL_COUNTS"]))
    try:
        notes.append("VISUAL CONFIG → " + sess.wait(lambda l: l.startswith("OK ") or l.startswith("ERR "), REPLY_WAIT_S, "visual config"))
    except Abort as exc:
        notes.append("VISUAL CONFIG: %s" % exc)
    print("STEP ready. count_total=%s. waiting for commands in %s" % (st.get("count_total"), cmd_file), flush=True)
    while True:
        if sess.reset_seen:
            raise Abort("reset during step mode")
        if not os.path.isfile(cmd_file):
            time.sleep(0.5)
            continue
        time.sleep(0.2)
        with open(cmd_file, encoding="utf-8") as fh:
            lines = [l.strip() for l in fh if l.strip()]
        os.unlink(cmd_file)
        for line in lines:
            parts = line.split()
            if parts[0] == "quit":
                print("STEP quit", flush=True)
                return
            if parts[0] == "ref":
                sess.drain()
                sess.send("VISUAL REFERENCE")
                try:
                    reply = sess.wait(lambda l: l.startswith("OK ") or l.startswith("ERR "), REPLY_WAIT_S, "visual reference")
                except Abort as exc:
                    reply = str(exc)
                st = sess.status()
                ref_count = int(st["count_total"])
                notes.append("REF count_total=%d, VISUAL REFERENCE → %s" % (ref_count, reply))
                print("STEP ref count_total=%d (%s)" % (ref_count, reply), flush=True)
                continue
            if parts[0] == "finish":
                sess.drain()
                sess.send("VISUAL FINISH")
                try:
                    reply = sess.wait(lambda l: l.startswith("OK ") or l.startswith("ERR "), REPLY_WAIT_S, "visual finish")
                except Abort as exc:
                    reply = str(exc)
                sess.drain()
                sess.send("VISUAL RESULT")
                try:
                    result = sess.wait(lambda l: l.startswith("OK visual") or l.startswith("ERR "), REPLY_WAIT_S, "visual result")
                except Abort as exc:
                    result = str(exc)
                st = sess.status()
                fin = int(st["count_total"])
                delta = (fin - ref_count) if ref_count is not None else None
                notes.append("FINISH count_total=%d, delta from REF=%s, VISUAL FINISH → %s, VISUAL RESULT → %s" % (fin, delta, reply, result))
                print("STEP finish count_total=%d delta=%s (%s | %s)" % (fin, delta, reply, result), flush=True)
                continue
            if parts[0] == "run" and len(parts) == 4:
                duty = int(parts[1])
                direction = "positive" if parts[2] == "+" else "negative"
                duration_ms = int(parts[3])
                if not (1 <= duty <= limits["MAX_DUTY_PERCENT"]) or not (limits["START_GRACE_MS"] < duration_ms <= limits["MAX_RUN_MS"]):
                    print("STEP rejected %s (cap %d, grace %d, max_run %d)" % (line, limits["MAX_DUTY_PERCENT"], limits["START_GRACE_MS"], limits["MAX_RUN_MS"]), flush=True)
                    continue
                n += 1
                t = do_one_run(sess, limits, n, duty, direction, duration_ms, trials, notes)
                since = (t["count_end"] - ref_count) if ref_count is not None else None
                print("STEP run %d duty=%d %s %dms: drive=%s coast=%s stop=%s brake=%s count_end=%s since_ref=%s final=%s stop_reason=%s" % (
                    n, duty, direction, duration_ms, t["drive_delta"], t["coast"], t["stop_delta"],
                    (t["count_end"] - t["count_ready"] - t["stop_delta"]) if t["stop_delta"] is not None else None,
                    t["count_end"], since, t["final_state"], t["stop_reason"]), flush=True)
                if t["final_state"] == "FAULT":
                    notes.append("step %d で FAULT。終了。" % n)
                    return
                continue
            print("STEP unknown command: %s" % line, flush=True)


# ---- main ----------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True)
    ap.add_argument("--mode", choices=("dry", "idle", "run", "step"), default="dry")
    ap.add_argument("--cmd-file", default=None,
                    help="step mode: poll this file for one command per line (run DUTY +|- MS | ref | finish | quit); "
                         "the file is deleted after it is read. The port stays open so count_total accumulates.")
    ap.add_argument("--shaft", default="output", help="step mode: VISUAL CONFIG shaft token (encoder|motor|gearbox|output|other)")
    ap.add_argument("--baseline-s", type=float, default=20.0, help="DISARMED STATUS poll before anything else")
    ap.add_argument("--trial", type=parse_trial, action="append", default=[],
                    help="run mode: duty:sign, e.g. --trial 10:+ --trial 10:- (in order)")
    ap.add_argument("--duration-ms", type=int, default=400, help="RUN duration (must be <= MAX_RUN_MS)")
    ap.add_argument("--glitch-ns", type=int, default=None, help="override GLITCH_NS for this session")
    ap.add_argument("--travel", type=int, default=None, help="override MAX_TRAVEL_COUNTS for this session")
    ap.add_argument("--start-grace-ms", type=int, default=None,
                    help="override START_GRACE_MS (no-pulse stop). Must stay shorter than every RUN duration.")
    ap.add_argument("--ready-hold-ms", type=int, default=None,
                    help="override READY_HOLD_MS (idle mode: how long power+brake-release stay on with PWM 0, for meter checks)")
    ap.add_argument("--settle-s", type=float, default=3.0, help="pause between trials after DISARMED")
    ap.add_argument("--cal-angle-file", default=None,
                    help="run mode: after the last trial keep the port open and wait (up to --cal-wait-s) for this file "
                         "to contain the externally measured angle in degrees; then send CAL ANGLE and record the reply")
    ap.add_argument("--cal-wait-s", type=float, default=900.0)
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    settings = list(SETTINGS)
    if args.glitch_ns is not None:
        settings = [(k, args.glitch_ns if k == "GLITCH_NS" else v) for k, v in settings]
    if args.travel is not None:
        settings = [(k, args.travel if k == "MAX_TRAVEL_COUNTS" else v) for k, v in settings]
    if args.start_grace_ms is not None:
        settings = [(k, args.start_grace_ms if k == "START_GRACE_MS" else v) for k, v in settings]
    if args.ready_hold_ms is not None:
        settings = [(k, args.ready_hold_ms if k == "READY_HOLD_MS" else v) for k, v in settings]
    limits = dict(settings)
    ACTIVE.clear()
    ACTIVE.update(limits)
    if args.mode == "step" and not args.cmd_file:
        ap.error("step mode needs --cmd-file")
    if args.mode == "run":
        if not args.trial:
            ap.error("run mode needs at least one --trial")
        for duty, _d, dur in args.trial:
            if duty < 1 or duty > limits["MAX_DUTY_PERCENT"]:
                ap.error("duty %d outside 1..MAX_DUTY_PERCENT=%d; the cap is not raised here" % (duty, limits["MAX_DUTY_PERCENT"]))
            dur = dur if dur is not None else args.duration_ms
            if dur < 1 or dur > limits["MAX_RUN_MS"]:
                ap.error("duration %d outside 1..MAX_RUN_MS=%d" % (dur, limits["MAX_RUN_MS"]))
            if limits["START_GRACE_MS"] >= dur:
                ap.error("START_GRACE_MS must be shorter than the RUN duration or the stall stop can never fire")

    os.makedirs(LOG_DIR, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    sess = Session(args.port, args.mode, stamp, args.verbose)
    notes = []
    trials = []
    before_cfg = after_cfg = {}
    baseline = []
    code = 0
    try:
        sess.open()
        sess.boot()
        st = sess.status()
        if st.get("state") != "DISARMED":
            raise Abort("not DISARMED at start: %s" % st.get("state"))
        if sess.gpio_errors:
            raise Abort("GPIO error lines seen at boot")
        before_cfg, after_cfg = sess.apply_settings(settings)
        print("SET ok; readback matches (%d keys)" % len(settings), flush=True)
        baseline = sess.baseline(args.baseline_s)
        counts = [int(s[1]) for s in baseline]
        print("baseline %ds: count %s..%s a=%s b=%s" % (
            args.baseline_s, counts[0], counts[-1], sorted({s[2] for s in baseline}), sorted({s[3] for s in baseline})), flush=True)
        if args.mode == "dry":
            notes.append("dry: ARM も RUN も送っていない。")
        else:
            sess.drain()
            sess.send("AUTHORITY USB")
            sess.wait(lambda l: l == "OK authority=usb", REPLY_WAIT_S, "AUTHORITY USB")
            sess.start_heartbeat()
            sess.send("STREAM ON")
            sess.wait(lambda l: l == "OK stream=on", REPLY_WAIT_S, "STREAM ON")
            if args.mode == "step":
                run_step_mode(sess, args, limits, trials, notes)
            elif args.mode == "idle":
                ready = sess.arm_and_wait_ready()
                print("READY at count %s; waiting for ready_timeout -> DISARMED" % ready["count_total_x4"], flush=True)
                end = sess.wait_disarmed(limits["READY_HOLD_MS"] / 1000.0 + limits["STOP_TIMEOUT_MS"] / 1000.0 + 5.0)
                print("end state %s stop=%s count %s" % (end["state"], end["stop_reason"], end["count_total_x4"]), flush=True)
                time.sleep(2.5)
                sess.wait_trace()
                notes.append("idle: RUN 無し。READY は ready_timeout で終わる。TRACE 行数 %d。" % len(sess.trace))
                trials.append({"trial": 0, "duty": 0, "direction": "idle", "duration_ms": 0,
                               "count_ready": int(ready["count_total_x4"]), "count_end": int(end["count_total_x4"]),
                               "stop_reason": end["stop_reason"], "final_state": end["state"],
                               "a_levels": sorted({r["a_level"] for r in sess.rows}),
                               "b_levels": sorted({r["b_level"] for r in sess.rows})})
            else:
                for n, (duty, direction, dur) in enumerate(args.trial, start=1):
                    duration_ms = dur if dur is not None else args.duration_ms
                    seq_start = int(sess.rows[-1]["seq"]) if sess.rows else 0
                    ready = sess.arm_and_wait_ready()
                    sess.drain()
                    cmd = "RUN duty_pct=%d direction=%s duration_ms=%d" % (duty, direction, duration_ms)
                    sess.send(cmd)
                    reply = sess.wait(lambda l: l.startswith("OK state=") or l.startswith("ERR "), REPLY_WAIT_S, "RUN reply")
                    if reply.startswith("ERR"):
                        notes.append("trial %d: RUN 拒否 %s。以後の試験は送らない。" % (n, reply))
                        end = sess.wait_disarmed(limits["READY_HOLD_MS"] / 1000.0 + 6.0)
                        trials.append(summarize_trial(sess, n, duty, direction, duration_ms, seq_start, ready, end))
                        break
                    end = sess.wait_disarmed(duration_ms / 1000.0 + limits["STOP_TIMEOUT_MS"] / 1000.0 +
                                             limits["BRAKE_APPLY_WAIT_MS"] / 1000.0 + 5.0)
                    time.sleep(0.5)
                    t = summarize_trial(sess, n, duty, direction, duration_ms, seq_start, ready, end)
                    trials.append(t)
                    print("trial %d duty=%d %s: drive_delta=%s coast=%s stop_delta=%s end=%s stop=%s vel_max=%s count_end=%s" % (
                        n, duty, direction, t["drive_delta"], t["coast"], t["stop_delta"], t["final_state"],
                        t["stop_reason"], t["vel_run_max_abs"], t["count_end"]), flush=True)
                    if end["state"] == "FAULT":
                        notes.append("trial %d で FAULT。再 ARM しない。" % n)
                        break
                    if sess.reset_seen:
                        notes.append("trial %d のあとにリセットを検出。以後は無効。" % n)
                        break
                    time.sleep(args.settle_s)
                time.sleep(2.5)
                sess.wait_trace(timeout=6.0)
                if args.cal_angle_file and trials and trials[-1].get("final_state") == "DISARMED":
                    print("waiting for angle in %s (up to %.0f s). Port stays open; the chip is not reset." % (
                        args.cal_angle_file, args.cal_wait_s), flush=True)
                    deadline = time.monotonic() + args.cal_wait_s
                    angle = None
                    while time.monotonic() < deadline:
                        if os.path.isfile(args.cal_angle_file):
                            try:
                                angle = float(open(args.cal_angle_file, encoding="utf-8").read().strip())
                                break
                            except ValueError:
                                angle = None
                        time.sleep(1.0)
                        if sess.reset_seen:
                            break
                    if angle is None:
                        notes.append("CAL ANGLE: 角度が届かなかった（%s）。校正は送っていない。" % args.cal_angle_file)
                    else:
                        sess.drain()
                        sess.send("CAL ANGLE %g" % angle)
                        reply = sess.wait(lambda l: l.startswith("OK cal") or l.startswith("ERR "), REPLY_WAIT_S, "CAL reply")
                        last = trials[-1]
                        notes.append("CAL ANGLE %g → %s（対象: trial %s、stop_delta %s。count/deg = |%s| / %g）" % (
                            angle, reply, last.get("trial"), last.get("stop_delta"), last.get("stop_delta"), angle))
                        print(reply, flush=True)
            sess.stop_heartbeat()
            time.sleep(0.3)
            final = sess.status()
            notes.append("最終 STATUS: state=%s stop=%s count_total=%s fault=%s" % (
                final.get("state"), final.get("stop"), final.get("count_total"), final.get("fault")))
    except Abort as exc:
        notes.append("ABORT: %s" % exc)
        print("ABORT: %s" % exc, file=sys.stderr, flush=True)
        code = 1
        try:
            if sess.mode != "dry" and sess.ser is not None:
                sess.send("STOP")
                time.sleep(3.0)
        except Exception:
            pass
    except KeyboardInterrupt:
        notes.append("Ctrl+C: STOP を 1 回送った。")
        try:
            sess.send("STOP")
            time.sleep(3.0)
        except Exception:
            pass
        code = 130
    finally:
        sess.stop_heartbeat()
        if sess.reset_seen or sess.boots > 1:
            notes.append("リセットまたは USB 断を検出した。この記録の試験は無効として扱う。")
        try:
            path = write_markdown(sess, args, before_cfg, after_cfg, baseline, trials, notes)
            print("markdown: %s" % os.path.relpath(path, os.getcwd()), flush=True)
        finally:
            sess.close()
    return code


if __name__ == "__main__":
    sys.exit(main())
