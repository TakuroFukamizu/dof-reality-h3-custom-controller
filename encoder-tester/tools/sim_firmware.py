#!/usr/bin/env python3
"""Minimal encoder-tester protocol simulator on a pty, for testing tools/session.py offline.

It mimics the text protocol only (banner, STATUS, CONFIG, SET, AUTHORITY, STREAM, ARM, RUN, EVT,
CSV rows, TRACE). Counts are synthetic: a RUN moves +/- `--counts-per-run` over the run and the
brake switch adds `--brake-jump`. It is not the firmware and proves nothing about hardware.

Usage:  python tools/sim_firmware.py            # prints the pty path, keep it running
        python tools/session.py --port /dev/ttysNNN --mode run --trial 10:+ --trial 10:-
"""

import argparse
import os
import pty
import sys
import threading
import time

KEYS = ["MAX_DUTY_PERCENT", "MAX_RUN_MS", "MAX_TRAVEL_COUNTS", "POWER_SETTLE_MS", "BRAKE_RELEASE_WAIT_MS",
        "BRAKE_APPLY_WAIT_MS", "STOP_SPEED_THRESHOLD", "STOP_CONFIRM_MS", "STOP_TIMEOUT_MS", "READY_HOLD_MS",
        "START_GRACE_MS", "MAX_COUNT_PER_S", "UI_TIMEOUT_MS", "LEASE_MS", "JOG_STALE_MS", "JOG_DUTY"]


class Sim:
    def __init__(self, fd, counts_per_run, brake_jump, old_firmware=False, reset_after_arm=False):
        self.fd = fd
        self.cfg = {k: -1 for k in KEYS}
        self.cfg["ENCODER_SIGN"] = 1
        self.cfg["GLITCH_NS"] = 0
        self.state = "DISARMED"
        self.stop = ""
        self.count = 0
        self.trial = 0
        self.seq = 0
        self.usb = False
        self.stream = False
        self.power = 0
        self.brake = 0
        self.duty = 0
        self.dir_level = 0
        self.lock = threading.Lock()
        self.counts_per_run = counts_per_run
        self.brake_jump = brake_jump
        self.old_firmware = old_firmware
        self.reset_after_arm = reset_after_arm
        self.t0 = time.monotonic()
        self.trace = []
        self.trace_sealed = False
        self.trace_out = False
        self.lease_until = 0

    def us(self):
        return int((time.monotonic() - self.t0) * 1e6)

    def say(self, text):
        os.write(self.fd, (text + "\n").encode())

    def banner(self):
        if self.old_firmware:
            self.say("pwm 20000 Hz requested, timer 20000 Hz, 10-bit")
            self.say("STAT R=0 B=0 S=0 OUT=0 DIR=0 PWR_PIN=0 BRK_PIN=0 LIVE=0")
            return
        for line in ("BOOT state=DISARMED", "BOOT ARM locked until SET", "BOOT outputs off",
                     "BOOT CURRENT MONITOR NOT CONNECTED", "BOOT not a physical e-stop",
                     "BOOT opening USB resets this chip", "HELP"):
            self.say(line)

    def limits_ok(self):
        return all(self.cfg[k] >= 0 for k in KEYS if k != "JOG_DUTY")

    def status(self):
        pwm = self.duty * 1023 // 100
        self.say("STAT state=%s stop=%s power_on_cmd=%d power_gpio=%d brake_release_cmd=%d brake_gpio=%d requested=0 "
                 "applied_duty_pct=%d pwm_raw=%d pwm_full=1023 pwm_hz=20000 dir_level=%d" % (
                     self.state, self.stop, self.power, self.power, self.brake, self.brake, self.duty, pwm, self.dir_level))
        self.say("STAT count_total=%d count_display=%d count_trial=0 count_per_s=0 a=0 b=1 stable=1 trial=%d rem_ms=0 "
                 "ready_ms=0 travel=0 max_travel=%d" % (self.count, self.count, self.trial, self.cfg["MAX_TRAVEL_COUNTS"]))
        lock = "" if self.limits_ok() else "limits_unset"
        self.say("STAT lock=%s fault= reject= visual=VISUAL_IDLE visual_valid=0 authority=%s stream=%d drops=0" % (
            lock, "usb" if self.usb else "local", 1 if self.stream else 0))
        self.say("STAT missing=%s" % ",".join(k for k in KEYS if self.cfg[k] < 0 and k != "JOG_DUTY"))
        self.say("STAT ppr=UNKNOWN")
        self.say("STAT cal=PPR UNKNOWN angle UNKNOWN")

    def config(self):
        for k in KEYS:
            self.say("CFG %s=%d unit" % (k, self.cfg[k]))
        self.say("CFG ENCODER_SIGN=%d sign" % self.cfg["ENCODER_SIGN"])
        self.say("CFG GLITCH_NS=%d ns" % self.cfg["GLITCH_NS"])
        self.say("CFG AUTHORITY=%s LOCK=%s" % ("usb" if self.usb else "local", "" if self.limits_ok() else "limits_unset"))
        self.say("CFG MISSING=")
        self.say("CFG PINS power=2 brake=13 pwm=9 dir=8 enc_a=17 enc_b=18")
        self.say("CFG WARN not_physical_estop current_monitor_absent g2_may_feed_all_axes one_cores3_only")
        self.say("CFG -1 means UNCONFIGURED")

    def csv_row(self, elapsed=0):
        if not self.stream:
            return
        self.seq += 1
        pwm = self.duty * 1023 // 100
        self.say("%d,%d,%d,1,%s,%d,%d,%d,%d,0,none,%d,%d,1023,20000,%d,%d,0,0,0,1,%d,%s,0" % (
            self.seq, self.us(), self.trial, self.state, self.power, self.power, self.brake, self.brake,
            self.duty, pwm, self.dir_level, self.count, elapsed, self.stop))

    def trace_row(self, mark="tick"):
        self.trace.append("TRACE,%d,%s,%d,%d,%d,%d,%d,%d,%d,0,0,1,%s" % (
            self.us(), self.state, self.power, self.brake, self.power, self.brake, self.duty * 1023 // 100,
            self.count, self.count, mark))

    def sleep_rows(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            time.sleep(0.02)
            self.csv_row()

    def arm_sequence(self, run):
        """run = None for idle, else (duty, direction, duration_ms)."""
        self.stop = ""
        self.state = "POWER_WAIT"
        self.power = 1
        self.say("EVT,%d,power_on_cmd,1" % self.us())
        self.trace_row()
        self.trace_row("post")
        self.count += 2
        self.sleep_rows(self.cfg["POWER_SETTLE_MS"] / 1000.0)
        if self.reset_after_arm:
            time.sleep(0.3)
            self.banner()
            self.state = "DISARMED"
            self.power = 0
            return
        self.state = "BRAKE_WAIT"
        self.brake = 1
        self.say("EVT,%d,brake_release_cmd,1" % self.us())
        self.trace_row("post")
        self.sleep_rows(self.cfg["BRAKE_RELEASE_WAIT_MS"] / 1000.0)
        self.state = "READY"
        if run is None:
            self.sleep_rows(self.cfg["READY_HOLD_MS"] / 1000.0)
            self.stop = "ready_timeout"
        else:
            # wait for RUN to be requested by the host
            deadline = time.monotonic() + self.cfg["READY_HOLD_MS"] / 1000.0
            while self.pending_run is None and time.monotonic() < deadline:
                time.sleep(0.02)
                self.csv_row()
            if self.pending_run is None:
                self.stop = "ready_timeout"
            else:
                duty, direction, duration = self.pending_run
                self.pending_run = None
                self.trial += 1
                self.duty = duty
                self.dir_level = 0 if direction == "positive" else 1
                self.state = "RUNNING"
                start = self.count
                self.say("OK state=RUNNING")
                self.say("EVT,%d,run,%d,%d,%d" % (self.us(), self.trial, duty if direction == "positive" else -duty, start))
                steps = max(1, duration // 20)
                per = (self.counts_per_run if direction == "positive" else -self.counts_per_run) // steps
                for i in range(steps):
                    time.sleep(0.02)
                    self.count += per
                    self.csv_row(elapsed=(i + 1) * 20)
                self.duty = 0
                self.stop = "duration"
                self.say("EVT,%d,pwm_off,duration,%d" % (self.us(), self.count))
                self.state = "STOPPING"
                off = self.count
                self.sleep_rows(self.cfg["STOP_CONFIRM_MS"] / 1000.0 + 0.12)
                self.state = "BRAKE_APPLY"
                self.brake = 0
                self.say("EVT,%d,trial_done,%d,%d,%d,%d" % (self.us(), self.trial, off - start, self.count - off, self.count - start))
                self.say("EVT,%d,brake_release_cmd,0" % self.us())
                self.trace_row()
                self.count += self.brake_jump
                self.trace_row("post")
                self.sleep_rows(self.cfg["BRAKE_APPLY_WAIT_MS"] / 1000.0)
                self.state = "POWER_OFF"
                self.power = 0
                self.say("EVT,%d,power_on_cmd,0" % self.us())
                self.csv_row()
                self.state = "DISARMED"
                self.csv_row()
                return
        # idle / ready_timeout path
        self.state = "STOPPING"
        self.sleep_rows(self.cfg["STOP_CONFIRM_MS"] / 1000.0 + 0.12)
        self.state = "BRAKE_APPLY"
        self.brake = 0
        self.say("EVT,%d,brake_release_cmd,0" % self.us())
        self.trace_row()
        self.count += self.brake_jump
        self.trace_row("post")
        self.sleep_rows(self.cfg["BRAKE_APPLY_WAIT_MS"] / 1000.0)
        self.state = "POWER_OFF"
        self.power = 0
        self.say("EVT,%d,power_on_cmd,0" % self.us())
        self.csv_row()
        self.state = "DISARMED"
        self.csv_row()

    def flush_trace(self):
        if self.trace_out or not self.trace:
            return
        self.trace_out = True
        self.say("TRACE note a_level and b_level are single digitalRead samples, not a pulse or noise waveform")
        self.say("TRACE t_us,state,power_on_cmd,brake_release_cmd,power_gpio,brake_gpio,pwm_raw,pcnt_raw,count_total,delta,a_level,b_level,mark")
        for line in self.trace:
            self.say(line)
        self.say("TRACE end n=%d overflow=0" % len(self.trace))

    pending_run = None

    def handle(self, line):
        if self.old_firmware:
            self.say("ERR unknown" if line != "STATUS" else "STAT R=0 B=0 S=0 OUT=0 DIR=0 PWR_PIN=0 BRK_PIN=0 LIVE=0")
            return
        if line == "HEARTBEAT":
            self.say("OK heartbeat")
            return
        if line == "STATUS":
            self.status()
        elif line == "CONFIG":
            self.config()
        elif line.startswith("SET "):
            _s, key, value = line.split(" ", 2)
            if self.state != "DISARMED":
                self.say("ERR not_disarmed")
            elif key in self.cfg:
                self.cfg[key] = int(value)
                self.say("OK set" if key not in ("ENCODER_SIGN", "GLITCH_NS") else "OK " + key.lower())
            else:
                self.say("ERR unknown_key")
        elif line == "AUTHORITY USB":
            self.usb = True
            self.say("OK authority=usb")
        elif line == "STREAM ON":
            self.stream = True
            self.say("seq,t_us,trial_id,axis,state,power_on_cmd,power_gpio_level,brake_release_cmd,brake_gpio_level,"
                     "requested_command,requested_unit,applied_duty_pct,pwm_raw,pwm_full_scale,pwm_hz,dir_level,"
                     "count_total_x4,count_trial_x4,velocity_count_s,a_level,b_level,elapsed_ms,stop_reason,log_drop_count")
            self.say("OK stream=on")
        elif line == "ARM":
            if self.state != "DISARMED":
                self.say("ERR not_disarmed")
                return
            if not self.limits_ok():
                self.say("ERR config")
                return
            self.say("OK state=POWER_WAIT")
            threading.Thread(target=self.arm_sequence, args=(self.want_run,), daemon=True).start()
        elif line.startswith("RUN "):
            if self.state != "READY":
                self.say("ERR not_ready")
                return
            kv = dict(p.split("=") for p in line[4:].split())
            self.pending_run = (int(kv["duty_pct"]), kv["direction"], int(kv["duration_ms"]))
        elif line == "STOP":
            self.say("OK state=%s" % self.state)
        else:
            self.say("ERR unknown")

    want_run = None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--counts-per-run", type=int, default=400)
    ap.add_argument("--brake-jump", type=int, default=-260)
    ap.add_argument("--old-firmware", action="store_true", help="answer like moter-controler")
    ap.add_argument("--reset-after-arm", action="store_true", help="emit a second BOOT banner during POWER_WAIT")
    ap.add_argument("--expect-run", action="store_true", help="ARM waits for a RUN instead of idling to ready_timeout")
    args = ap.parse_args()
    master, slave = pty.openpty()
    print(os.ttyname(slave), flush=True)
    sim = Sim(master, args.counts_per_run, args.brake_jump, args.old_firmware, args.reset_after_arm)
    sim.want_run = True if args.expect_run else None
    time.sleep(1.0)
    sim.banner()
    buf = b""
    idle_since = None
    while True:
        try:
            data = os.read(master, 256)
        except OSError:
            break
        if not data:
            break
        buf += data
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            text = line.decode(errors="replace").strip()
            if text:
                sim.handle(text)
                if sim.state == "DISARMED" and sim.trace and sim.trial >= 0:
                    if idle_since is None:
                        idle_since = time.monotonic()
                    elif time.monotonic() - idle_since > 2.0:
                        sim.flush_trace()


if __name__ == "__main__":
    sys.exit(main())
