"""End-to-end scenario: h3-control-server (sim backend) + patched h3game bridge.py.

Run by control-server/scripts/e2e_sim.sh. Starts the server and the bridge
itself (own process groups, killed in `finally`), plays the browser (30 Hz
POSTs like the patched index.html) and the operator (admin API), and writes
timeline.jsonl + summary.md to --out. Exit code 0 only if every check passed.

Stdlib only (the bridge needs websockets, the driver does not). Python >= 3.9.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import platform
import re
import subprocess
import sys
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import _support as S  # noqa: E402

AXES = 3
POSE_TOL = 1e-9
COUNT_TOL = 1          # Rust round() is half-away-from-zero; allow 1 count
ACTUAL_TOL = 3         # sim plant settles within a few counts of the command


class ScenarioError(Exception):
    """A check failed; the remaining phases are skipped."""


# ------------------------------------------------------------------ model

def load_axes(config_path: str) -> List[Dict[str, Any]]:
    """[[axis]] tables of the TOML config (tomllib on 3.11+, tiny parser otherwise)."""
    try:
        import tomllib  # type: ignore
        with open(config_path, "rb") as f:
            cfg = tomllib.load(f)
        axes = cfg["axis"]
        timing, game, limits = cfg["timing"], cfg["game"], cfg["limits"]
    except ImportError:
        axes, timing, game, limits = _parse_toml_subset(config_path)
    axes = sorted(axes, key=lambda a: a["axis_id"])
    return [dict(a, _timing=timing, _game=game, _limits=limits) for a in axes]


def _parse_toml_subset(path: str):
    """Enough TOML for config/*.toml: [table], [[axis]], key = number|bool|"str"."""
    axes: List[Dict[str, Any]] = []
    tables: Dict[str, Dict[str, Any]] = {}
    cur: Dict[str, Any] = {}
    with open(path, encoding="utf-8") as f:
        for raw in f:
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            if line == "[[axis]]":
                cur = {}
                axes.append(cur)
            elif line.startswith("["):
                cur = tables.setdefault(line.strip("[]"), {})
            elif "=" in line:
                k, v = (x.strip() for x in line.split("=", 1))
                if v in ("true", "false"):
                    cur[k] = v == "true"
                elif v.startswith('"') or v.startswith("["):
                    cur[k] = v
                else:
                    cur[k] = float(v) if any(c in v for c in ".eE") else int(v)
    return axes, tables["timing"], tables["game"], tables["limits"]


def mix(pitch: float, roll: float, yaw: float, gain: float) -> List[float]:
    """compat::mix (h3_direct.py order of operations)."""
    g = gain * 511.0
    return [512.0 + g * (roll + pitch), 512.0 + g * (roll - pitch), 512.0 + g * yaw]


def saturate(v: List[float], lo: float, hi: float) -> Tuple[List[float], bool]:
    out = [min(max(x, lo), hi) for x in v]
    return out, out != v


def virtual_to_count(v: float, ax: Dict[str, Any]) -> float:
    c0, c512, c1023 = (float(ax["count_at_virtual_%d" % k]) for k in (0, 512, 1023))
    if v <= 512.0:
        return c0 + (c512 - c0) * (v / 512.0)
    return c512 + (c1023 - c512) * ((v - 512.0) / 511.0)


def rust_round(x: float) -> int:
    return int(math.copysign(math.floor(abs(x) + 0.5), x))


def expected(axes, pose: Dict[str, float], gain: float, max_gain: float):
    """(effective_gain, virtual targets, rounded count targets, saturated)."""
    eff = min(max(gain, 0.0), max_gain)
    virt, sat = saturate(mix(pose["pitch"], pose["roll"], pose["yaw"], eff), 0.0, 1023.0)
    counts = []
    for v, ax in zip(virt, axes):
        c = virtual_to_count(v, ax)
        c = min(max(c, ax["soft_min_count"]), ax["soft_max_count"])
        counts.append(rust_round(c))
    return eff, virt, counts, sat


# ------------------------------------------------------------------ io

class Timeline:
    """Small JSONL timeline: phase markers, checks, admin calls, sampled status."""

    def __init__(self, path: str, t0: float):
        self.t0 = t0
        self.lock = threading.Lock()
        self.f = open(path, "w", encoding="utf-8")

    def t(self) -> float:
        return round((time.monotonic() - self.t0) * 1000.0, 1)

    def write(self, kind: str, **kw) -> None:
        rec = {"t_ms": self.t(), "kind": kind}
        rec.update(kw)
        with self.lock:
            self.f.write(json.dumps(rec, separators=(",", ":"), sort_keys=False) + "\n")

    def close(self) -> None:
        with self.lock:
            self.f.close()


def compact(st: Dict[str, Any]) -> Dict[str, Any]:
    sess = st.get("session") or {}
    fault = st.get("fault")
    return {
        "rig": st.get("rig_state"),
        "src": st.get("active_source"),
        "fresh": sess.get("fresh"),
        "mcu": [a.get("mcu_state") for a in st["axes"]],
        "act": [a.get("actual_count") for a in st["axes"]],
        "tgt": st.get("target_counts"),
        "brk": [a.get("brake_release_cmd") for a in st["axes"]],
        "eg": st.get("effective_gain"),
        "lim": st.get("limits"),
        "fault": fault.get("kind") if isinstance(fault, dict) else None,
    }


class Sampler(threading.Thread):
    """Polls GET /v1/status every 10 ms; keeps the latest snapshot and every
    rig_state / mcu_state change with its time; writes the timeline at ~10 Hz
    and on every change."""

    PERIOD = 0.010
    WRITE_EVERY = 0.100

    def __init__(self, url: str, tl: Timeline):
        super().__init__(daemon=True)
        self.url = url
        self.tl = tl
        self.lock = threading.Lock()
        self.latest: Optional[Dict[str, Any]] = None
        self.latest_t = 0.0
        self.changes: List[Tuple[float, str, Tuple[str, ...]]] = []
        self.errors: Dict[str, int] = {}
        self.phase = "init"
        self._halt = threading.Event()

    def run(self) -> None:
        last_key = None
        last_write = 0.0
        while not self._halt.is_set():
            t = time.monotonic()
            try:
                code, st = S.http("GET", self.url + "/v1/status", timeout=1.0)
            except OSError as e:
                code, st = -1, str(e)
            if code != 200 or not isinstance(st, dict):
                k = str(code)
                self.errors[k] = self.errors.get(k, 0) + 1
            else:
                key = (st["rig_state"],) + tuple(a["mcu_state"] for a in st["axes"])
                with self.lock:
                    self.latest, self.latest_t = st, t
                    if key != last_key:
                        self.changes.append((t, st["rig_state"], key[1:]))
                changed = key != last_key
                if changed or t - last_write >= self.WRITE_EVERY:
                    self.tl.write("status", phase=self.phase, change=changed, **compact(st))
                    last_write = t
                last_key = key
            self._halt.wait(max(0.0, self.PERIOD - (time.monotonic() - t)))

    def get(self) -> Tuple[float, Dict[str, Any]]:
        deadline = time.monotonic() + 2.0
        while True:
            with self.lock:
                if self.latest is not None:
                    return self.latest_t, self.latest
            if time.monotonic() > deadline:
                raise ScenarioError("no status sample yet")
            time.sleep(0.005)

    def fresh_after(self, t: float, timeout: float = 2.0) -> Dict[str, Any]:
        """First snapshot polled after time t."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            lt, st = self.get()
            if lt > t:
                return st
            time.sleep(0.003)
        raise ScenarioError("status sampler stalled")

    def first_change(self, after: float, pred: Callable[[str, Tuple[str, ...]], bool]) -> Optional[float]:
        with self.lock:
            for t, rig, mcu in self.changes:
                if t >= after and pred(rig, mcu):
                    return t
        return None

    def stop(self) -> None:
        self._halt.set()
        self.join(3)


class Browser(threading.Thread):
    """The patched index.html as seen by the bridge: owner from GET /, POST /
    with {owner, pitch, roll, yaw, gain, frame_seq, frame_age_ms} at ~30 Hz.
    advance=False keeps POSTing the same frame_seq (frozen game loop)."""

    def __init__(self, url: str, owner: int, pose: Dict[str, float], gain: float):
        super().__init__(daemon=True)
        self.url = url
        self.lock = threading.Lock()
        self.owner = owner
        self.pose = dict(pose)
        self.gain = gain
        self.frame_seq = 0
        self.advance = True
        self.posts = 0
        self.post_errors = 0
        self.last_advancing_post = 0.0   # monotonic send time of the last new frame_seq
        self.rtt_ms: List[float] = []
        self._halt = threading.Event()

    def set(self, **kw) -> None:
        with self.lock:
            for k, v in kw.items():
                setattr(self, k, v)

    def run(self) -> None:
        nxt = time.monotonic()
        while not self._halt.is_set():
            with self.lock:
                if self.advance:
                    self.frame_seq += 1
                d = {"owner": self.owner, "gain": self.gain, "frame_seq": self.frame_seq, "frame_age_ms": 4.0}
                d.update(self.pose)
                adv = self.advance
            t = time.monotonic()
            try:
                code, _ = S.http("POST", self.url + "/", d, timeout=1.0)
                ok = code == 200
            except OSError:
                ok = False
            dt = (time.monotonic() - t) * 1000.0
            with self.lock:
                if ok:
                    self.posts += 1
                    if len(self.rtt_ms) < 100000:
                        self.rtt_ms.append(dt)
                    if adv:
                        self.last_advancing_post = t
                else:
                    self.post_errors += 1
            nxt += 1.0 / 30.0
            self._halt.wait(max(0.0, nxt - time.monotonic()))

    def stop(self) -> None:
        self._halt.set()
        self.join(3)


def owner_from_page(url: str) -> int:
    status, html = S.http("GET", url + "/")
    m = re.search(r"typeof (-?\d+)===", html if isinstance(html, str) else "")
    if status != 200 or not m:
        raise ScenarioError("owner not found in bridge page")
    return int(m.group(1))


def pct(xs: List[float], p: float) -> Optional[float]:
    if not xs:
        return None
    s = sorted(xs)
    k = max(0, min(len(s) - 1, int(math.ceil(p / 100.0 * len(s))) - 1))
    return s[k]


# ------------------------------------------------------------------ scenario

class Scenario:
    def __init__(self, args):
        self.args = args
        self.t0 = time.monotonic()
        os.makedirs(args.out, exist_ok=True)
        self.tl = Timeline(os.path.join(args.out, "timeline.jsonl"), self.t0)
        self.axes = load_axes(args.config)
        self.timing = self.axes[0]["_timing"]
        self.max_gain = float(self.axes[0]["_limits"]["max_gain"])
        self.neutral = [int(a["count_at_virtual_512"]) for a in self.axes]
        self.checks: List[Dict[str, Any]] = []
        self.metrics: Dict[str, Any] = {}
        self.poses: List[Dict[str, Any]] = []
        self.telemetry: Dict[str, Any] = {}
        self.procs: List[S.Proc] = []
        self.browser: Optional[Browser] = None
        self.sampler: Optional[Sampler] = None
        self.surl = ""
        self.burl = ""
        self.phase = "init"
        self.skipped: List[str] = []
        self.server_log_tail = ""
        self.bridge_log_tail = ""

    # ---- helpers

    def set_phase(self, name: str, note: str = "") -> None:
        self.phase = name
        if self.sampler:
            self.sampler.phase = name
        self.tl.write("phase", phase=name, note=note)
        print("[e2e] phase %s %s" % (name, note), flush=True)

    def check(self, name: str, ok: bool, detail: Any = None, fatal: bool = True) -> bool:
        rec = {"phase": self.phase, "name": name, "ok": bool(ok), "detail": detail}
        self.checks.append(rec)
        self.tl.write("check", **rec)
        print("[e2e]   %s %s %s" % ("PASS" if ok else "FAIL", name, "" if ok else detail), flush=True)
        if not ok and fatal:
            raise ScenarioError("%s: %s" % (name, detail))
        return ok

    def admin(self, path: str, token: Optional[str] = "default", body: Any = None):
        tok = self.args.token if token == "default" else token
        t = time.monotonic()
        code, resp = S.http("POST", self.surl + path, body=body, token=tok)
        ms = round((time.monotonic() - t) * 1000.0, 1)
        self.tl.write("admin", path=path, token=bool(tok), http=code, resp=resp, ms=ms)
        return code, resp, t

    def status(self) -> Dict[str, Any]:
        return self.sampler.get()[1]

    def wait(self, pred: Callable[[Dict[str, Any]], bool], timeout: float, what: str) -> Dict[str, Any]:
        deadline = time.monotonic() + timeout
        st = None
        while time.monotonic() < deadline:
            st = self.status()
            if pred(st):
                return st
            time.sleep(0.005)
        self.check("wait: " + what, False, compact(st) if st else None)
        raise AssertionError("unreachable")

    def wait_rig(self, state: str, timeout: float = 8.0) -> Dict[str, Any]:
        return self.wait(lambda s: s["rig_state"] == state, timeout, "rig_state " + state)

    def bstate(self) -> Dict[str, Any]:
        code, body = S.http("GET", self.burl + "/state")
        if code != 200 or not isinstance(body, dict):
            raise ScenarioError("bridge /state http %s" % code)
        return body

    def actual(self, st: Dict[str, Any]) -> List[int]:
        return [a["actual_count"] for a in st["axes"]]

    def observe_still(self, seconds: float, rig: str, mcu: str, name: str) -> List[int]:
        """Rig stays in `rig`, every axis in `mcu`, nothing moves, for `seconds`."""
        st0 = self.sampler.fresh_after(time.monotonic())
        ref = self.actual(st0)
        bad = None
        n = 0
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            st = self.status()
            n += 1
            mcus = [a["mcu_state"] for a in st["axes"]]
            moving = [a["velocity_count_s"] for a in st["axes"] if a["velocity_count_s"] != 0]
            if (st["rig_state"] != rig or any(m != mcu for m in mcus) or self.actual(st) != ref or moving
                    or st["armed"] or any(a["brake_release_cmd"] for a in st["axes"])):
                bad = compact(st)
                break
            time.sleep(0.01)
        self.check(name, bad is None, {"expected_rig": rig, "mcu": mcu, "held_counts": ref, "first_bad": bad,
                                       "samples": n, "seconds": seconds})
        return ref

    def track(self, label: str, pose: Dict[str, float], gain: float, timeout: float = 4.0) -> None:
        """Set a pose in the browser; check virtual_targets == mix(pose) at once and
        target/actual counts converge to calibration(mix(pose))."""
        eff, virt, counts, sat = expected(self.axes, pose, gain, self.max_gain)
        t_set = time.monotonic()
        self.browser.set(pose=dict(pose), gain=gain)

        def reflects(st):
            rp = st.get("requested_pose") or {}
            return (st.get("requested_gain") is not None and abs(st["requested_gain"] - gain) < POSE_TOL
                    and all(abs(rp.get(k, 1e9) - pose[k]) < POSE_TOL for k in ("pitch", "roll", "yaw")))
        st = self.wait(reflects, 2.0, "%s: server sees pose" % label)
        t_seen = time.monotonic()
        self.check("%s: still ACTIVE(game)" % label, st["rig_state"] == "ACTIVE" and st["active_source"] == "game",
                   compact(st))
        vt = st["virtual_targets"]
        self.check("%s: effective_gain" % label, abs(st["effective_gain"] - eff) < POSE_TOL,
                   {"effective_gain": st["effective_gain"], "expected": eff})
        self.check("%s: virtual_targets == mix(pose)" % label,
                   all(abs(a - b) < 1e-6 for a, b in zip(vt, virt)), {"got": vt, "expected": virt})
        if sat:
            self.check("%s: limits has virtual_saturated" % label, "virtual_saturated" in st["limits"], st["limits"])
        if gain > self.max_gain:
            self.check("%s: limits has gain_limited" % label, "gain_limited" in st["limits"], st["limits"])

        def converged(s):
            tg = s["target_counts"]
            return (all(abs(a - b) <= COUNT_TOL for a, b in zip(tg, counts))
                    and all(abs(a - b) <= ACTUAL_TOL for a, b in zip(self.actual(s), counts))
                    and all(a["velocity_count_s"] == 0 for a in s["axes"]))
        st = self.wait(converged, timeout, "%s: converge to %s" % (label, counts))
        t_conv = time.monotonic()
        exact = st["target_counts"] == counts
        rec = {"label": label, "pose": pose, "gain": gain, "effective_gain": eff,
               "virtual_expected": [round(x, 3) for x in virt], "virtual_status": [round(x, 3) for x in vt],
               "target_expected": counts, "target_status": st["target_counts"], "actual": self.actual(st),
               "target_exact": exact, "limits": st["limits"],
               "seen_ms": round((t_seen - t_set) * 1000, 1), "converge_ms": round((t_conv - t_set) * 1000, 1)}
        self.poses.append(rec)
        self.tl.write("pose", **rec)
        self.check("%s: target_counts == calibration(mix(pose))" % label,
                   all(abs(a - b) <= COUNT_TOL for a, b in zip(st["target_counts"], counts)),
                   {"got": st["target_counts"], "expected": counts, "exact": exact})

    def snapshot_telemetry(self, key: str) -> None:
        # Summary is refreshed every 10 ticks: let a refresh happen.
        time.sleep(0.15)
        self.telemetry[key] = self.status().get("telemetry")
        self.tl.write("telemetry", key=key, telemetry=self.telemetry[key])

    # ---- processes

    def start_processes(self) -> None:
        a = self.args
        sport, bport = S.free_port(), S.free_port()
        self.surl = "http://127.0.0.1:%d" % sport
        self.burl = "http://127.0.0.1:%d" % bport
        env = dict(os.environ, H3CS_ADMIN_TOKEN=a.token, RUST_LOG=os.environ.get("RUST_LOG", "info"))
        srv = S.Proc([a.server_bin, "--config", a.config, "--bind", "127.0.0.1:%d" % sport],
                     cwd=os.path.dirname(os.path.dirname(os.path.abspath(a.config))), env=env)
        self.procs.append(srv)
        self.note_pid(srv)
        self.server = srv
        t = time.monotonic()

        def server_ready():
            if not srv.alive():
                raise ScenarioError("server exited: %s" % srv.output()[-2000:])
            try:
                code, b = S.http("GET", self.surl + "/v1/status", timeout=0.5)
            except OSError:
                return False
            return code == 200 and isinstance(b, dict) and b

        first = S.wait_until(server_ready, timeout=10.0, msg="server /v1/status")
        self.tl.write("process", what="server", url=self.surl, first_rig_state=first.get("rig_state"))
        self.sampler = Sampler(self.surl, self.tl)
        self.sampler.phase = self.phase
        self.sampler.start()
        self.wait_rig("READY_TO_ARM", timeout=10.0)
        self.metrics["server_start_to_ready_ms"] = round((time.monotonic() - t) * 1000, 1)

        benv = {"H3_BACKEND": "server", "H3_SERVER_URL": self.surl}
        self.bridge = S.start_bridge(a.h3game_repo, bport, benv)
        self.procs.append(self.bridge)
        self.note_pid(self.bridge)
        self.tl.write("process", what="bridge", url=self.burl, backend="server")

    def note_pid(self, proc: S.Proc) -> None:
        # Backstop for e2e_sim.sh: it kills these process groups if we die.
        if self.args.pidfile:
            with open(self.args.pidfile, "a") as f:
                f.write("%d\n" % proc.p.pid)

    def stop_processes(self) -> None:
        # Kill the processes first; nothing below may prevent that.
        try:
            for th in (self.browser, self.sampler):
                if th is not None:
                    try:
                        th.stop()
                    except Exception as e:  # noqa: BLE001
                        print("[e2e] stopping %s: %r" % (type(th).__name__, e), flush=True)
        finally:
            for p in reversed(self.procs):
                try:
                    if p is getattr(self, "server", None):
                        self.server_log_tail = p.output()[-3000:]
                    if p is getattr(self, "bridge", None):
                        self.bridge_log_tail = p.output()[-2000:]
                except (OSError, ValueError):
                    pass
                p.kill()

    # ---- phases

    def run(self) -> bool:
        ok = False
        try:
            self.set_phase("connect", "server + bridge start; browser posts; nothing may move")
            self.start_processes()
            self.phase_connect()
            self.phase_arm_start()
            self.phase_poses()
            self.phase_gain_cap()
            self.snapshot_telemetry("after_tracking")
            self.phase_freeze()
            self.phase_no_auto_rearm()
            self.phase_reload_mid_ride()
            self.phase_api_stop()
            self.phase_center()
            self.phase_operator_fault()
            self.snapshot_telemetry("final")
            self.phase_shutdown()
            ok = all(c["ok"] for c in self.checks)
        except Exception as e:  # noqa: BLE001 - any error aborts the run, reported in the summary
            self.tl.write("abort", phase=self.phase, error="%s: %s" % (type(e).__name__, e))
            print("[e2e] ABORT in %s: %s: %s" % (self.phase, type(e).__name__, e), flush=True)
            if not isinstance(e, ScenarioError):
                self.checks.append({"phase": self.phase, "name": "unexpected error", "ok": False,
                                    "detail": "%s: %s" % (type(e).__name__, e)})
            try:
                if self.sampler and self.sampler.latest:
                    self.telemetry.setdefault("at_abort", self.status().get("telemetry"))
            except ScenarioError:
                pass
        finally:
            self.stop_processes()
            if self.sampler:
                self.metrics["status_poll_errors"] = dict(self.sampler.errors)
            self.tl.close()
            self.write_summary(ok)
        return ok

    def phase_connect(self) -> None:
        S.wait_until(lambda: self.bstate().get("server_reachable"), timeout=8.0, msg="bridge server_reachable")
        bs = self.bstate()
        self.check("bridge backend=server, not armed", bs.get("backend") == "server" and bs.get("armed") is False,
                   {k: bs.get(k) for k in ("backend", "rig_state", "armed", "connected", "port")})
        st = self.status()
        self.check("connect: all axes DISARMED, rig READY_TO_ARM",
                   st["rig_state"] == "READY_TO_ARM" and all(a["mcu_state"] == "DISARMED" for a in st["axes"]),
                   compact(st))
        pose = {"pitch": 0.4, "roll": 0.2, "yaw": -0.3}
        self.browser = Browser(self.burl, owner_from_page(self.burl), pose, 0.3)
        self.browser.start()
        st = self.wait(lambda s: (s["session"] or {}).get("fresh") and (s["session"]["last_frame_seq"] or 0) > 5,
                       5.0, "server receives fresh frames")
        self.check("freshness_mode frame_seq (require_frame_seq default)",
                   st["session"]["freshness_mode"] == "frame_seq", st["session"])
        self.observe_still(1.0, "READY_TO_ARM", "DISARMED", "connect + fresh poses: no auto arm/center for 1 s")
        code, r, _ = self.admin("/v1/start")
        self.check("START before ARM rejected (409)", code == 409, {"http": code, "resp": r})
        code, r, _ = self.admin("/v1/arm", token=None)
        self.check("ARM without token rejected (401)", code == 401, {"http": code, "resp": r})

    def phase_arm_start(self) -> None:
        self.set_phase("arm_start")
        before = self.actual(self.status())
        code, r, t = self.admin("/v1/arm")
        self.check("ARM accepted", code == 200, r)
        st = self.wait(lambda s: s["rig_state"] == "HOLDING" and all(a["mcu_state"] == "HOLDING" for a in s["axes"]),
                       5.0, "HOLDING")
        self.metrics["arm_to_holding_ms"] = round((time.monotonic() - t) * 1000, 1)
        self.check("HOLDING holds current position (no move to pose/neutral)",
                   st["target_counts"] == before and self.actual(st) == before,
                   {"before": before, "target": st["target_counts"], "actual": self.actual(st)})
        time.sleep(0.5)
        st = self.status()
        self.check("HOLDING 0.5 s with fresh poses: still not moving",
                   st["rig_state"] == "HOLDING" and self.actual(st) == before, compact(st))
        code, r, t = self.admin("/v1/start")
        self.check("START accepted", code == 200, r)
        st = self.wait(lambda s: s["rig_state"] == "ACTIVE", 3.0, "ACTIVE")
        self.metrics["start_to_active_ms"] = round((time.monotonic() - t) * 1000, 1)
        self.check("active_source game", st["active_source"] == "game", st["active_source"])

    def phase_poses(self) -> None:
        self.set_phase("poses", "single-axis and combined poses, gain 0.3")
        g = 0.3
        for label, p in [
            ("pitch+0.6", (0.6, 0.0, 0.0)), ("pitch-0.6", (-0.6, 0.0, 0.0)),
            ("roll+0.6", (0.0, 0.6, 0.0)), ("roll-0.6", (0.0, -0.6, 0.0)),
            ("yaw+0.6", (0.0, 0.0, 0.6)), ("yaw-0.6", (0.0, 0.0, -0.6)),
            ("combo(0.3,-0.2,0.4)", (0.3, -0.2, 0.4)), ("combo(-0.4,0.5,-0.5)", (-0.4, 0.5, -0.5)),
            ("neutral", (0.0, 0.0, 0.0)),
        ]:
            self.track(label, {"pitch": p[0], "roll": p[1], "yaw": p[2]}, g)
        # Largest mix at max_gain 0.5: 512 + 0.5*511*2 = 1023, so the legacy 0..1023
        # range cannot saturate while max_gain <= 0.5 (checked: no virtual_saturated).
        self.track("full(1,1,0) gain 0.35", {"pitch": 1.0, "roll": 1.0, "yaw": 0.0}, 0.35)
        st = self.status()
        self.check("max_gain 0.35: full pose does not saturate 0..1023", "virtual_saturated" not in st["limits"],
                   {"limits": st["limits"], "virtual_targets": st["virtual_targets"]})

    def phase_gain_cap(self) -> None:
        self.set_phase("gain_cap", "browser gain 0.6 > max_gain")
        self.track("gain 0.6 capped", {"pitch": 0.5, "roll": 0.0, "yaw": 0.5}, 0.6)
        bs = S.wait_until(lambda: (lambda b: b if b.get("effective_gain") is not None
                                   and abs(b["effective_gain"] - self.max_gain) < 1e-9 else None)(self.bstate()),
                          timeout=2.0, msg="bridge effective_gain")
        self.check("bridge /state effective_gain == max_gain", True,
                   {"gain": bs.get("gain"), "effective_gain": bs.get("effective_gain")})

    def phase_freeze(self) -> None:
        self.set_phase("freeze", "game loop frozen: same frame_seq keeps being POSTed")
        pose = {"pitch": 0.5, "roll": 0.3, "yaw": 0.4}
        self.track("pre-freeze pose", pose, 0.3)
        held = self.actual(self.status())
        posts0 = self.browser.posts
        self.browser.set(advance=False)
        time.sleep(0.05)  # let the in-flight POST finish so last_advancing_post is final
        t_last = self.browser.last_advancing_post
        st = self.wait(lambda s: s["rig_state"] in ("STOPPING", "BRAKED_DISARMED", "FAULT_LATCHED"), 3.0, "STOPPING")
        t_stop = self.sampler.first_change(t_last, lambda rig, mcu: rig == "STOPPING")
        self.check("freeze -> STOPPING (not FAULT)", t_stop is not None, compact(st))
        fresh_ms = float(self.timing["source_freshness_ms"])
        lat = round((t_stop - t_last) * 1000.0, 1)
        self.metrics["freeze_last_fresh_post_to_stopping_ms"] = lat
        self.check("STOPPING within source_freshness_ms (+ tick/POST slack)",
                   fresh_ms - 60 <= lat <= fresh_ms + 100, {"latency_ms": lat, "source_freshness_ms": fresh_ms})
        st = self.wait(lambda s: s["rig_state"] != "STOPPING" or all(a["mcu_state"] == "HOLDING" for a in s["axes"]),
                       float(self.timing["stop_settle_timeout_ms"]) / 1000 + 1, "axes HOLDING during STOPPING")
        t_hold = self.sampler.first_change(t_stop, lambda rig, mcu: rig == "STOPPING" and
                                           all(m == "HOLDING" for m in mcu))
        self.check("HOLD phase observed (rig STOPPING, all axes HOLDING)", t_hold is not None, compact(st))
        timeout = (float(self.timing["stop_hold_ms"]) + float(self.timing["stop_settle_timeout_ms"])) / 1000 + 2
        st = self.wait(lambda s: s["rig_state"] in ("BRAKED_DISARMED", "FAULT_LATCHED"), timeout, "BRAKED_DISARMED")
        self.check("then BRAKED_DISARMED", st["rig_state"] == "BRAKED_DISARMED", compact(st))
        t_brk = self.sampler.first_change(t_stop, lambda rig, mcu: rig == "BRAKED_DISARMED")
        self.metrics["stopping_to_axes_holding_ms"] = round((t_hold - t_stop) * 1000, 1)
        self.metrics["axes_holding_to_braked_ms"] = round((t_brk - t_hold) * 1000, 1)
        self.metrics["stopping_to_braked_ms"] = round((t_brk - t_stop) * 1000, 1)
        st = self.wait(lambda s: all(a["mcu_state"] == "DISARMED" and not a["brake_release_cmd"] for a in s["axes"]),
                       1.0, "axes DISARMED, brake applied")
        final = self.actual(st)
        self.check("no centering: braked where it was",
                   all(abs(a - b) <= 5 for a, b in zip(final, held)) and
                   any(abs(a - n) > 100 for a, n in zip(final, self.neutral)),
                   {"before_freeze": held, "braked": final, "neutral": self.neutral})
        self.check("bridge kept POSTing during freeze", self.browser.posts > posts0 + 20,
                   {"posts_during": self.browser.posts - posts0})
        bs = S.wait_until(lambda: (lambda b: b if b.get("rig_state") == "BRAKED_DISARMED" else None)(self.bstate()),
                          timeout=2.0, msg="bridge sees BRAKED_DISARMED")
        self.check("bridge /state: repeats counted, session not fresh",
                   bs["pose_stats"]["repeat"] > 0 and bs.get("session_fresh") is False,
                   {"pose_stats": bs.get("pose_stats"), "session_fresh": bs.get("session_fresh")})
        self.held_after_freeze = final

    def phase_no_auto_rearm(self) -> None:
        self.set_phase("no_auto_rearm", "game resumes, bridge reconnects: still braked until explicit ARM+START")
        self.browser.set(advance=True)
        self.wait(lambda s: (s["session"] or {}).get("fresh"), 2.0, "fresh again after unfreeze")
        held = self.observe_still(1.5, "BRAKED_DISARMED", "DISARMED", "game resumed: stays BRAKED_DISARMED 1.5 s")
        code, body = S.http("GET", self.burl + "/reconnect")
        self.tl.write("bridge", path="/reconnect", http=code)
        self.check("bridge /reconnect ok", code == 200, code)
        self.wait(lambda s: (s["session"] or {}).get("fresh"), 3.0, "fresh in new session")
        held2 = self.observe_still(1.0, "BRAKED_DISARMED", "DISARMED", "after reconnect: stays BRAKED_DISARMED 1 s")
        self.check("position unchanged across resume/reconnect", held == held2 == self.held_after_freeze,
                   {"freeze": self.held_after_freeze, "resume": held, "reconnect": held2})
        code, r, _ = self.admin("/v1/start")
        self.check("START without re-ARM rejected (409)", code == 409, {"http": code, "resp": r})
        code, r, _ = self.admin("/v1/arm")
        self.check("explicit re-ARM accepted", code == 200, r)
        st = self.wait(lambda s: s["rig_state"] == "HOLDING", 5.0, "HOLDING after re-ARM")
        self.check("re-ARM holds braked position (not neutral)",
                   st["target_counts"] == held2 and self.actual(st) == held2,
                   {"target": st["target_counts"], "actual": self.actual(st), "held": held2})
        code, r, _ = self.admin("/v1/start")
        self.check("START accepted after re-ARM", code == 200, r)
        self.wait_rig("ACTIVE", 3.0)
        self.track("after re-ARM", {"pitch": -0.3, "roll": 0.4, "yaw": 0.2}, 0.3)

    def phase_reload_mid_ride(self) -> None:
        self.set_phase("reload_mid_ride", "page reload = new owner = new session while ACTIVE")
        held = self.actual(self.status())
        prefix1 = self.status()["session"]["source_session_prefix"]
        owner = owner_from_page(self.burl)
        self.browser.set(owner=owner, frame_seq=0, pose={"pitch": 0.1, "roll": -0.1, "yaw": 0.0})
        st = self.wait(lambda s: s["rig_state"] in ("BRAKED_DISARMED", "FAULT_LATCHED"), 6.0,
                       "BRAKED_DISARMED after session change")
        self.check("session change mid-ride -> BRAKED_DISARMED", st["rig_state"] == "BRAKED_DISARMED", compact(st))
        self.check("stopped where it was (no centering)",
                   all(abs(a - b) <= 5 for a, b in zip(self.actual(st), held)),
                   {"before": held, "after": self.actual(st)})
        st = self.wait(lambda s: (s["session"] or {}).get("fresh"), 3.0, "new session fresh")
        self.check("new session prefix differs", st["session"]["source_session_prefix"] != prefix1,
                   {"old": prefix1, "new": st["session"]["source_session_prefix"]})
        self.observe_still(0.5, "BRAKED_DISARMED", "DISARMED", "new session: no automatic re-arm")
        for path, state in (("/v1/arm", "HOLDING"), ("/v1/start", "ACTIVE")):
            code, r, _ = self.admin(path)
            self.check("%s accepted (new session)" % path, code == 200, r)
            self.wait_rig(state, 5.0)
        self.track("new session tracks", {"pitch": 0.1, "roll": -0.1, "yaw": 0.0}, 0.3)

    def phase_api_stop(self) -> None:
        self.set_phase("api_stop", "POST /v1/stop without token while poses stay fresh")
        self.track("pre-stop pose", {"pitch": -0.5, "roll": -0.2, "yaw": 0.5}, 0.3)
        held = self.actual(self.status())
        code, r, t = self.admin("/v1/stop", token=None)
        self.check("stop without token accepted", code == 200, {"http": code, "resp": r})
        self.wait_rig("BRAKED_DISARMED", 6.0)
        self.metrics["api_stop_to_braked_ms"] = round((time.monotonic() - t) * 1000, 1)
        st = self.status()
        self.check("fresh poses kept arriving", (st["session"] or {}).get("fresh") is True, st["session"])
        held2 = self.observe_still(1.0, "BRAKED_DISARMED", "DISARMED", "late/fresh poses do not restart after STOP")
        self.check("stopped where it was", all(abs(a - b) <= 5 for a, b in zip(held2, held)),
                   {"before": held, "after": held2})

    def phase_center(self) -> None:
        self.set_phase("center", "explicit /v1/center only")
        code, r, _ = self.admin("/v1/arm")
        self.check("ARM accepted", code == 200, r)
        self.wait_rig("HOLDING", 5.0)
        code, r, t = self.admin("/v1/center")
        self.check("CENTER accepted", code == 200, r)
        st = self.wait(lambda s: s["rig_state"] == "ACTIVE", 2.0, "ACTIVE(center)")
        self.check("active_source center", st["active_source"] == "center", st["active_source"])
        st = self.wait(lambda s: s["rig_state"] == "HOLDING" and
                       all(abs(a - n) <= ACTUAL_TOL for a, n in zip(self.actual(s), self.neutral)), 10.0,
                       "HOLDING at neutral")
        self.metrics["center_ms"] = round((time.monotonic() - t) * 1000, 1)
        self.check("center ends HOLDING at neutral counts", True, {"actual": self.actual(st), "neutral": self.neutral})
        code, r, _ = self.admin("/v1/disarm")
        self.check("DISARM accepted", code == 200, r)
        self.wait(lambda s: s["rig_state"] == "BRAKED_DISARMED" and
                  all(a["mcu_state"] == "DISARMED" for a in s["axes"]), 5.0, "BRAKED_DISARMED after DISARM")

    def phase_operator_fault(self) -> None:
        self.set_phase("operator_fault", "POST /v1/fault while ACTIVE (sim fault injection has no HTTP route)")
        self.skipped.append("模擬軸の異常注入（stall / freeze_status / force_fault / reboot）: サーバーのバイナリに HTTP の"
                            "入口がないので e2e では行わない。Rust の tests/server.rs・supervisor.rs・sim_axis.rs で確認。"
                            "e2e の FAULT は管理 API の /v1/fault（operator）")
        for path, state in (("/v1/arm", "HOLDING"), ("/v1/start", "ACTIVE")):
            code, r, _ = self.admin(path)
            self.check("%s accepted" % path, code == 200, r)
            self.wait_rig(state, 5.0)
        self.track("pre-fault pose", {"pitch": 0.2, "roll": 0.4, "yaw": -0.4}, 0.3)
        code, r, t = self.admin("/v1/fault", body={"note": "e2e operator fault"})
        self.check("operator fault accepted", code == 200, r)
        st = self.wait(lambda s: s["rig_state"] == "FAULT_LATCHED" and
                       all(a["mcu_state"] == "FAULT" for a in s["axes"]), 2.0, "FAULT_LATCHED, all axes FAULT")
        self.metrics["fault_to_all_axes_fault_ms"] = round((time.monotonic() - t) * 1000, 1)
        f = st["fault"] or {}
        self.check("fault kind operator + note", f.get("kind") == "operator" and f.get("note") == "e2e operator fault",
                   f)
        self.check("all axes HOST_FAULT, brake applied",
                   all(a["fault_code"] == "HOST_FAULT" and not a["brake_release_cmd"] for a in st["axes"]),
                   [(a["fault_code"], a["brake_release_cmd"]) for a in st["axes"]])
        code, r, _ = self.admin("/v1/arm")
        self.check("ARM while latched rejected (409 fault_latched)",
                   code == 409 and isinstance(r, dict) and r.get("error") == "fault_latched", {"http": code, "resp": r})
        held = self.actual(self.status())
        resp = None
        for _ in range(20):
            code, resp, _ = self.admin("/v1/fault/reset")
            if code == 200 or (isinstance(resp, dict) and resp.get("error") != "reset_in_progress"):
                break
            time.sleep(0.1)
        self.check("fault reset accepted", code == 200, {"http": code, "resp": resp})
        st = self.wait(lambda s: s["rig_state"] in ("READY_TO_ARM", "BRAKED_DISARMED") and s["fault"] is None, 3.0,
                       "reclassified after reset")
        self.check("after reset: not armed, axes DISARMED/NONE",
                   not st["armed"] and all(a["mcu_state"] == "DISARMED" and a["fault_code"] == "NONE"
                                           for a in st["axes"]), compact(st))
        self.observe_still(1.0, st["rig_state"], "DISARMED", "after reset: no auto re-arm 1 s")
        self.check("fault_history has operator entry",
                   any((h.get("fault") or {}).get("kind") == "operator" for h in st.get("fault_history", [])),
                   st.get("fault_history"))
        self.check("held through fault (no centering)",
                   all(abs(a - b) <= 5 for a, b in zip(self.actual(self.status()), held)), held)

    def phase_shutdown(self) -> None:
        self.set_phase("shutdown", "SIGTERM server")
        self.browser.stop()
        self.sampler.stop()
        p = self.server.p
        p.send_signal(15)
        try:
            rc = p.wait(10)
        except subprocess.TimeoutExpired:
            rc = None
        self.check("server exits cleanly on SIGTERM", rc == 0, rc, fatal=False)

    # ---- report

    def write_summary(self, ok: bool) -> None:
        a = self.args
        n_ok = sum(1 for c in self.checks if c["ok"])
        lines = ["# e2e_sim 結果", ""]
        lines.append("- 結果: **%s**（チェック %d / %d 通過）" % ("PASS" if ok else "FAIL", n_ok, len(self.checks)))
        lines.append("- 実行: %s（ローカル時刻）、所要 %.1f s" % (time.strftime("%Y-%m-%d %H:%M:%S"),
                                                     time.monotonic() - self.t0))
        lines.append("- 構成: h3-control-server（`%s`、sim backend）＋ パッチ後の h3game `bridge.py`（`H3_BACKEND=server`）。"
                     "ブラウザーは driver が模擬（30 Hz POST、frame_age_ms = 4）" % os.path.relpath(a.config, a.repo_root))
        lines.append("- 環境: %s / Python %s" % (platform.platform(), platform.python_version()))
        for k, v in (("リポジトリ", a.repo_rev), ("h3game", a.h3game_rev)):
            if v:
                lines.append("- %s: `%s`" % (k, v))
        lines.append("- 模擬軸での結果。実機（CoreS3）・実ブラウザーでの値ではない。")
        lines.append("")
        if self.skipped:
            lines.append("## 省略")
            lines.append("")
            for s in self.skipped:
                lines.append("- " + s)
            lines.append("")
        lines.append("## 遷移・応答時間（driver 側で計測、status を 10 ms ごとに poll）")
        lines.append("")
        lines.append("| 項目 | ms |")
        lines.append("| --- | ---: |")
        for k, v in self.metrics.items():
            if not isinstance(v, dict):
                lines.append("| %s | %s |" % (k, v))
        if self.browser and self.browser.rtt_ms:
            r = self.browser.rtt_ms
            lines.append("| browser→bridge POST 往復 p50 / p95 / max（%d 件） | %.2f / %.2f / %.2f |"
                         % (len(r), pct(r, 50), pct(r, 95), max(r)))
        lines.append("")
        lines.append("`freeze_last_fresh_post_to_stopping_ms` は最後に frame_seq が進んだ POST の送信開始から、"
                     "status で STOPPING を初めて見るまで（poll 周期 10 ms 分の誤差を含む）。")
        lines.append("")
        lines.append("## サーバー telemetry（`GET /v1/status` の `telemetry`）")
        lines.append("")
        lines.append("p50 / p95 / p99 / max は直近 1024 サンプルの窓、`max_all` は起動以来、`count` は起動以来の件数。")
        lines.append("")
        for key, tel in self.telemetry.items():
            if not isinstance(tel, dict):
                continue
            lines.append("### %s" % key)
            lines.append("")
            lines.append("| 指標 | count | p50 | p95 | p99 | max | max_all |")
            lines.append("| --- | ---: | ---: | ---: | ---: | ---: | ---: |")
            for m in ("pose_rx_to_tick_ms", "tick_dt_ms", "status_age_ms", "frame_skew"):
                s = tel.get(m) or {}

                def f(x):
                    return "null" if x is None else ("%.3f" % x if isinstance(x, float) else str(x))
                lines.append("| %s | %s | %s | %s | %s | %s | %s |" % (
                    m, s.get("count"), f(s.get("p50")), f(s.get("p95")), f(s.get("p99")), f(s.get("max")),
                    f(s.get("max_all"))))
            lines.append("")
            lines.append("tick_count %s、tick_overruns %s、log_dropped %s、recorder_dropped %s" % (
                tel.get("tick_count"), tel.get("tick_overruns"), tel.get("log_dropped"), tel.get("recorder_dropped")))
            lines.append("")
        lines.append("frame_skew の単位はフレーム（ACTIVE 中だけ記録）。pose_rx_to_tick は WS 受信から制御 tick が"
                     "その Pose を取り込むまで（新しい seq ごとに 1 件）。")
        lines.append("")
        if self.poses:
            lines.append("## 姿勢ごとの追従（target は `target_counts`、actual は模擬軸の `actual_count`）")
            lines.append("")
            lines.append("| 姿勢 | gain → effective | 期待 count | target_counts | actual | 反映 ms | 収束 ms | limits |")
            lines.append("| --- | --- | --- | --- | --- | ---: | ---: | --- |")
            for p in self.poses:
                lines.append("| %s | %s → %s | %s | %s | %s | %s | %s | %s |" % (
                    p["label"], p["gain"], p["effective_gain"], p["target_expected"], p["target_status"], p["actual"],
                    p["seen_ms"], p["converge_ms"], ", ".join(p["limits"]) or "-"))
            lines.append("")
            lines.append("期待 count = 校正（3 点区分線形）∘ 飽和（0..1023）∘ 混合式（h3_direct.py と同じ）を "
                         "Rust の round（0.5 は 0 から遠い側）で丸めた値。許容差 target ±%d、actual ±%d。"
                         % (COUNT_TOL, ACTUAL_TOL))
            lines.append("")
        lines.append("## チェック一覧")
        lines.append("")
        lines.append("| フェーズ | チェック | 結果 |")
        lines.append("| --- | --- | --- |")
        for c in self.checks:
            res = "PASS" if c["ok"] else "**FAIL** `%s`" % json.dumps(c["detail"], ensure_ascii=False)[:300]
            lines.append("| %s | %s | %s |" % (c["phase"], c["name"].replace("|", "/"), res))
        lines.append("")
        if not ok:
            lines.append("## サーバーログ末尾（失敗時のみ）")
            lines.append("")
            lines.append("```text")
            lines.append(self.server_log_tail.strip()[-3000:])
            lines.append("```")
            lines.append("")
        lines.append("`timeline.jsonl`: フェーズ・チェック・管理 API 呼び出し・status の要約（約 10 Hz と状態変化時）。")
        with open(os.path.join(a.out, "summary.md"), "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--server-bin", required=True)
    ap.add_argument("--config", required=True)
    ap.add_argument("--h3game-repo", required=True, help="clone with the patches applied")
    ap.add_argument("--out", required=True)
    ap.add_argument("--repo-root", default=S.REPO)
    ap.add_argument("--repo-rev", default="")
    ap.add_argument("--h3game-rev", default="")
    ap.add_argument("--pidfile", default="", help="append spawned process-group ids here")
    args = ap.parse_args()
    args.token = os.environ.get("H3CS_ADMIN_TOKEN") or ""
    if not args.token:
        print("H3CS_ADMIN_TOKEN must be set", file=sys.stderr)
        return 2
    ok = Scenario(args).run()
    print("[e2e] %s -> %s" % ("PASS" if ok else "FAIL", args.out), flush=True)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
