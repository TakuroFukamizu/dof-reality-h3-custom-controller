"""Real h3-control-server (sim backend) + patched bridge.py; the browser is simulated.

Needs: control-server binary (cd control-server && cargo build), the h3game
source (H3GAME_SRC, or the cache clone _support makes) and websockets in this python. Skipped otherwise.
"""

from __future__ import annotations

import os
import re
import subprocess
import sys
import threading
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import _support as S  # noqa: E402

try:
    import websockets  # noqa: F401
    HAVE_WS = True
except ImportError:
    HAVE_WS = False

OLD_STATE_KEYS = {"pitch", "roll", "yaw", "connected", "port", "gain", "auto", "last_cmd_age"}
NEW_STATE_KEYS = {"backend", "rig_state", "armed", "calibration_valid", "fault", "server_url", "effective_gain"}

# config/sim.toml count_at_virtual_{0,512,1023} per axis.
SIM_CAL = [(-2000, 0, 2000), (-400, 0, 400), (-400, 0, 400)]


def expected_counts(pitch, roll, yaw, gain):
    """h3game-v1 mix (compat::mix) + 3-point calibration."""
    g = gain * 511.0
    virt = [512.0 + g * (roll + pitch), 512.0 + g * (roll - pitch), 512.0 + g * yaw]
    out = []
    for v, (c0, c512, c1023) in zip(virt, SIM_CAL):
        if v <= 512.0:
            out.append(c0 + (c512 - c0) * (v / 512.0))
        else:
            out.append(c512 + (c1023 - c512) * ((v - 512.0) / 511.0))
    return out


class Browser:
    """Posts the patched index.html's seat JSON to the bridge at ~30 Hz.

    Mirrors the page's hidden-tab rule: zeros with the last frame_seq POSTed
    while visible (sent_seq). This simulates the page, it does not test it;
    index.html itself is covered by test_browser.py.
    """

    def __init__(self, url, owner, pose, gain=0.3, send_frame_seq=True):
        self.url = url
        self.lock = threading.Lock()
        self.owner = owner
        self.pose = dict(pose)
        self.gain = gain
        self.frame_seq = 0       # rendered frames (page frameSeq)
        self.sent_seq = 0        # last frame_seq POSTed while visible (page sentSeq)
        self.advance = True      # False: game loop frozen, timer still posting
        self.hidden = False      # True: zeros, frame_seq not advancing (index.html behaviour)
        self.send_frame_seq = send_frame_seq  # False: page without the patch
        self.posts = 0
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()

    def set(self, **kw):
        with self.lock:
            if kw.get("hidden") and not self.hidden and self.advance:
                self.frame_seq += 1  # rAF finished a frame that no POST carried before the hide
            for k, v in kw.items():
                setattr(self, k, v)

    def _run(self):
        while not self._stop.is_set():
            with self.lock:
                if self.advance and not self.hidden:
                    self.frame_seq += 1  # two rAF frames per 30 Hz POST in reality; one is enough
                if not self.hidden:
                    self.sent_seq = self.frame_seq
                d = {"owner": self.owner, "gain": self.gain}
                for k in ("pitch", "roll", "yaw"):
                    d[k] = 0 if self.hidden else self.pose[k]
                if self.send_frame_seq:
                    d["frame_seq"] = self.sent_seq
                    d["frame_age_ms"] = 4.0
            try:
                S.http("POST", self.url + "/", d, timeout=1.0)
                self.posts += 1
            except OSError:
                pass
            self._stop.wait(1 / 30)

    def stop(self):
        self._stop.set()
        self._t.join(3)


def owner_from_page(url):
    """GET / like Chrome does; bridge substitutes __OWNER__ with a new owner number."""
    status, html = S.http("GET", url + "/")
    m = re.search(r"typeof (-?\d+)===", html if isinstance(html, str) else "")
    assert status == 200 and m, "owner not found in page"
    return int(m.group(1))


_skip = None
if not S.server_available():
    _skip = "h3-control-server binary not found at %s (cd control-server && cargo build)" % S.SERVER_BINARY
elif not S.src_available():
    _skip = S.src_skip_reason()
elif not S.patches():
    _skip = "no patch in %s" % S.PATCH_DIR
elif not HAVE_WS:
    _skip = "websockets not installed in this python"

REPO = None


def setUpModule():
    global REPO
    if _skip is None:
        REPO = S.make_clone(apply_patch=True)


def tearDownModule():
    if REPO:
        S.remove_clone(REPO)


@unittest.skipIf(_skip is not None, _skip or "")
class BridgeServerIntegration(unittest.TestCase):
    POSE = {"pitch": 0.3, "roll": 0.5, "yaw": 0.6}
    GAIN = 0.3

    def setUp(self):
        self.procs = []
        self.browsers = []
        self.sport = S.free_port()
        self.surl = "http://127.0.0.1:%d" % self.sport
        self.procs.append(S.start_server(self.sport))

    def tearDown(self):
        for b in self.browsers:
            b.stop()
        for p in reversed(self.procs):
            p.kill()

    # ------------------------------------------------------------ helpers

    def start_bridge(self, backend="server", extra=None):
        port = S.free_port()
        env = {"H3_BACKEND": backend, "H3_SERVER_URL": self.surl}
        env.update(extra or {})
        p = S.start_bridge(REPO, port, env)
        self.procs.append(p)
        self.bridge_proc = p
        return "http://127.0.0.1:%d" % port

    def browser(self, burl, pose=None, **kw):
        b = Browser(burl, owner_from_page(burl), pose or self.POSE, self.GAIN, **kw)
        self.browsers.append(b)
        return b

    def status(self):
        s, body = S.http("GET", self.surl + "/v1/status")
        self.assertEqual(s, 200, body)
        return body

    def wait_rig(self, state, timeout=8.0):
        return S.wait_until(lambda: (lambda st: st if st["rig_state"] == state else None)(self.status()),
                            timeout=timeout, msg="rig_state %s" % state)

    def admin(self, path):
        return S.http("POST", self.surl + path, token=S.ADMIN_TOKEN)

    def bstate(self, burl):
        s, body = S.http("GET", burl + "/state")
        self.assertEqual(s, 200)
        return body

    def wait_bridge_ready(self, burl):
        S.wait_until(lambda: self.bstate(burl).get("connected"), timeout=8.0, msg="bridge connected to server")

    def arm_and_start(self):
        s, r = self.admin("/v1/arm")
        self.assertEqual(s, 200, r)
        self.wait_rig("HOLDING")
        s, r = self.admin("/v1/start")
        self.assertEqual(s, 200, r)
        return self.wait_rig("ACTIVE")

    def counts(self, st=None):
        return [a["actual_count"] for a in (st or self.status())["axes"]]

    def wait_counts(self, expected, tol=3.0, timeout=5.0):
        def near():
            c = self.counts()
            return c if all(abs(a - e) <= tol for a, e in zip(c, expected)) else None
        return S.wait_until(near, timeout=timeout, msg="actual counts near %r" % (expected,))

    # ------------------------------------------------------------ tests

    def test_ride_follows_game_and_stops_without_centering(self):
        burl = self.start_bridge()
        self.wait_bridge_ready(burl)
        b = self.browser(burl)
        exp = expected_counts(self.POSE["pitch"], self.POSE["roll"], self.POSE["yaw"], self.GAIN)

        # Poses flow, but connecting / posting never arms or moves the rig.
        st = S.wait_until(lambda: (lambda s: s if (s["session"]["last_frame_seq"] or 0) > 5 else None)(self.status()),
                          msg="server receives frames")
        self.assertEqual(st["rig_state"], "READY_TO_ARM")
        self.assertTrue(st["session"]["fresh"])
        self.assertEqual(st["session"]["freshness_mode"], "frame_seq")
        self.assertEqual(self.counts(st), [0, 0, 0])

        self.arm_and_start()
        self.wait_counts(exp)

        bs = self.bstate(burl)
        self.assertTrue(OLD_STATE_KEYS <= set(bs), bs)
        self.assertTrue(NEW_STATE_KEYS <= set(bs), bs)
        self.assertEqual(bs["backend"], "server")
        self.assertEqual(bs["rig_state"], "ACTIVE")
        self.assertIs(bs["armed"], True)
        self.assertIs(bs["connected"], True)
        self.assertIs(bs["calibration_valid"], True)
        self.assertIsNone(bs["fault"])
        self.assertEqual(bs["server_url"], self.surl)
        self.assertAlmostEqual(bs["effective_gain"], self.GAIN)
        self.assertAlmostEqual(bs["pitch"], self.POSE["pitch"])  # commanded value
        self.assertIs(bs["session_fresh"], True)
        self.assertIsNone(bs["reject_reason"])

        # Browser gone; the operator's /state poll keeps running and must not keep it alive.
        b.stop()
        t0 = time.monotonic()
        seen = set()
        while time.monotonic() - t0 < 8.0:
            seen.add(self.bstate(burl)["rig_state"])
            if "BRAKED_DISARMED" in seen:
                break
            time.sleep(0.05)
        self.assertIn("STOPPING", seen)
        self.assertIn("BRAKED_DISARMED", seen)
        time.sleep(0.5)
        final = self.counts()
        for a, e in zip(final, exp):
            self.assertLess(abs(a - e), 25, (final, exp))  # held where it was
        self.assertGreater(final[0], 400)                  # not back at neutral (0)
        self.assertFalse(self.status()["armed"])

    def test_frozen_game_loop_stops_rig_although_posts_continue(self):
        burl = self.start_bridge()
        self.wait_bridge_ready(burl)
        b = self.browser(burl)
        exp = expected_counts(self.POSE["pitch"], self.POSE["roll"], self.POSE["yaw"], self.GAIN)
        S.wait_until(lambda: self.status()["session"]["fresh"], msg="fresh pose")
        self.arm_and_start()
        self.wait_counts(exp)
        # Timer keeps posting the same frame_seq; then the tab hides and posts zeros.
        b.set(advance=False)
        self.wait_rig("STOPPING", timeout=3.0)
        b.set(hidden=True)
        self.wait_rig("BRAKED_DISARMED")
        self.assertGreater(b.posts, 0)
        final = self.counts()
        for a, e in zip(final, exp):
            self.assertLess(abs(a - e), 25, (final, exp))  # zeros never reached the server
        # Bridge status comes over the WS (every 100 ms): may lag the server briefly.
        bs = S.wait_until(lambda: (lambda x: x if x["rig_state"] == "BRAKED_DISARMED" else None)(self.bstate(burl)),
                          msg="bridge sees BRAKED_DISARMED")
        self.assertGreater(bs["pose_stats"]["repeat"], 0)

    def test_hidden_tab_while_active_holds_position(self):
        # Game loop still running when the tab hides: the zeros must not reach the server.
        burl = self.start_bridge()
        self.wait_bridge_ready(burl)
        b = self.browser(burl)
        exp = expected_counts(self.POSE["pitch"], self.POSE["roll"], self.POSE["yaw"], self.GAIN)
        S.wait_until(lambda: self.status()["session"]["fresh"], msg="fresh pose")
        self.arm_and_start()
        self.wait_counts(exp)
        held = [a["target_count"] for a in self.status()["axes"]]
        b.set(hidden=True)
        drift = 0
        t0 = time.monotonic()
        while time.monotonic() - t0 < 3.0:
            st = self.status()
            if st["rig_state"] != "ACTIVE":
                break
            drift = max(drift, max(abs(a["target_count"] - h) for a, h in zip(st["axes"], held)))
            time.sleep(0.02)
        self.assertLessEqual(drift, 3, "target moved toward center while ACTIVE")
        st = self.wait_rig("BRAKED_DISARMED")
        for a, e in zip(self.counts(st), exp):
            self.assertLess(abs(a - e), 25, (self.counts(st), exp))
        self.assertEqual(st["session"]["last_frame_seq"], b.sent_seq)  # nothing newer was accepted
        bs = S.wait_until(lambda: (lambda x: x if x["pose_stats"]["repeat"] > 0 else None)(self.bstate(burl)),
                          msg="hidden POSTs counted as repeat")
        self.assertEqual(bs["frame_seq"], b.sent_seq)

    def test_page_reload_takes_new_session_and_needs_operator_restart(self):
        burl = self.start_bridge()
        self.wait_bridge_ready(burl)
        b = self.browser(burl)
        S.wait_until(lambda: self.status()["session"]["fresh"], msg="fresh pose")
        prefix1 = self.status()["session"]["source_session_prefix"]
        self.arm_and_start()
        self.wait_counts(expected_counts(self.POSE["pitch"], self.POSE["roll"], self.POSE["yaw"], self.GAIN))

        # Reload: new owner, frame_seq starts over, different pose.
        pose2 = {"pitch": -0.2, "roll": 0.1, "yaw": -0.5}
        new_owner = owner_from_page(burl)
        b.set(owner=new_owner, frame_seq=0, sent_seq=0, pose=pose2)
        self.wait_rig("BRAKED_DISARMED")
        st = S.wait_until(lambda: (lambda s: s if s["session"]["fresh"] else None)(self.status()),
                          msg="new session fresh")
        self.assertNotEqual(st["session"]["source_session_prefix"], prefix1)
        self.assertEqual(st["rig_state"], "BRAKED_DISARMED")  # no automatic re-arm
        self.assertEqual(self.bstate(burl)["pose_stats"]["new_page"], 1)

        self.arm_and_start()
        self.wait_counts(expected_counts(pose2["pitch"], pose2["roll"], pose2["yaw"], self.GAIN))

    def test_second_bridge_launch_leaves_the_ride_alone_and_lost_session_shows(self):
        burl = self.start_bridge()
        self.wait_bridge_ready(burl)
        self.browser(burl)
        S.wait_until(lambda: self.status()["session"]["fresh"], msg="fresh pose")
        prefix = self.status()["session"]["source_session_prefix"]
        self.arm_and_start()

        # The operator launches the bridge again: the port is taken, so it must
        # exit without touching the server session.
        h3 = os.path.join(REPO, "h3game")
        env = dict(os.environ, H3_BACKEND="server", H3_SERVER_URL=self.surl, H3_PORT=burl.rsplit(":", 1)[1],
                   PYTHONDONTWRITEBYTECODE="1")
        env.pop("H3_DRY", None)
        r = subprocess.run([sys.executable, "-u", os.path.join(h3, "bridge.py")], cwd=h3, env=env,
                           capture_output=True, text=True, timeout=20)
        self.assertNotEqual(r.returncode, 0, r.stdout + r.stderr)
        time.sleep(0.6)
        st = self.status()
        self.assertEqual(st["rig_state"], "ACTIVE")
        self.assertEqual(st["session"]["source_session_prefix"], prefix)
        self.assertTrue(st["session"]["fresh"])
        bs = self.bstate(burl)
        self.assertIs(bs["connected"], True)
        self.assertIs(bs["session_lost"], False)

        # Someone else does take the session: the ride stops and the page is
        # told (connected false, session_lost true), not "接続中".
        s, _ = S.http("POST", self.surl + "/v1/session")
        self.assertEqual(s, 200)
        self.wait_rig("BRAKED_DISARMED")
        bs = S.wait_until(lambda: (lambda x: x if x["session_lost"] else None)(self.bstate(burl)),
                          msg="bridge sees the lost session")
        self.assertIs(bs["connected"], False)
        self.assertIn("SESSION_LOST", bs["port"])
        self.assertIs(bs["server_reachable"], True)

    def test_page_without_frame_seq_is_rejected_and_cannot_start(self):
        burl = self.start_bridge()
        self.wait_bridge_ready(burl)
        self.browser(burl, send_frame_seq=False)
        S.wait_until(lambda: self.bstate(burl).get("reject_reason") == "missing_frame_seq",
                     msg="missing_frame_seq visible in bridge /state")
        self.assertFalse(self.status()["session"]["fresh"])
        s, r = self.admin("/v1/arm")
        self.assertEqual(s, 200, r)
        self.wait_rig("HOLDING")
        s, r = self.admin("/v1/start")
        self.assertEqual(s, 409, r)
        self.assertEqual(r["error"], "no_fresh_pose")

    def test_dry_bridge_sends_nothing_to_server(self):
        for extra, backend in (({}, "dry"), ({"H3_DRY": "1"}, "server")):
            burl = self.start_bridge(backend, extra)
            b = self.browser(burl)
            time.sleep(1.5)
            b.stop()
            st = self.status()
            self.assertIsNone(st["session"]["source_session_prefix"], (backend, extra))
            self.assertEqual(st["session"]["last_seq"], 0)
            self.assertEqual(st["rig_state"], "READY_TO_ARM")
            self.assertGreater(b.posts, 20)
            bs = self.bstate(burl)
            self.assertEqual(bs["backend"], "dry")
            self.assertFalse(bs["connected"])
            self.bridge_proc.kill()
            self.procs.remove(self.bridge_proc)


if __name__ == "__main__":
    unittest.main()
