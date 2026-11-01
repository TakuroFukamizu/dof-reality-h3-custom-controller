"""MotionClient against the real h3-control-server binary (sim backend).

Build first: cd control-server && cargo build. Skipped if the binary is missing
(set H3CS_BINARY to use another path).
"""

from __future__ import annotations

import json
import os
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import h3_motion_client as hmc  # noqa: E402
from h3_motion_client import MotionClient  # noqa: E402

REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
SERVER_DIR = os.path.join(REPO, "control-server")
BINARY = os.environ.get("H3CS_BINARY") or os.path.join(SERVER_DIR, "target", "debug", "h3-control-server")
SIM_CONFIG = os.path.join(SERVER_DIR, "config", "sim.toml")
TOKEN = "test-admin-token"
FRESHNESS_S = 0.250  # sim.toml timing.source_freshness_ms


def free_port() -> int:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def wait_until(pred, timeout=5.0, step=0.01, msg="condition"):
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        last = pred()
        if last:
            return last
        time.sleep(step)
    raise AssertionError("timed out waiting for %s (last %r)" % (msg, last))


class Streamer:
    """Submits poses at ~30 Hz from a thread, like the bridge would."""

    def __init__(self, client, advance_frame=True, gain=0.3):
        self.client = client
        self.advance_frame = advance_frame
        self.gain = gain
        self.frame = 0
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()

    def _run(self):
        while not self._stop.is_set():
            if self.advance_frame:
                self.frame += 1
            self.client.submit_pose(0.2, -0.1, 0.0, self.gain, frame_seq=self.frame, source_age_ms=1.0)
            self._stop.wait(1 / 30)

    def stop(self):
        self._stop.set()
        self._t.join(2)


@unittest.skipUnless(os.path.isfile(BINARY) and os.access(BINARY, os.X_OK),
                     "h3-control-server binary not found at %s (cd control-server && cargo build)" % BINARY)
class AgainstServerTests(unittest.TestCase):
    def setUp(self):
        self.port = free_port()
        self.url = "http://127.0.0.1:%d" % self.port
        self.log = tempfile.TemporaryFile()
        env = dict(os.environ, H3CS_ADMIN_TOKEN=TOKEN, RUST_LOG="warn")
        self.proc = subprocess.Popen(
            [BINARY, "--config", SIM_CONFIG, "--bind", "127.0.0.1:%d" % self.port],
            cwd=SERVER_DIR, env=env, stdout=self.log, stderr=subprocess.STDOUT)
        self.clients = []
        self.streamers = []
        try:
            wait_until(lambda: self._state() == "READY_TO_ARM", timeout=10.0, msg="server READY_TO_ARM")
        except Exception:
            self.tearDown()
            raise

    def tearDown(self):
        for s in self.streamers:
            s.stop()
        for c in self.clients:
            c.close()
        if self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(5)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait(5)
        self.log.close()

    # ------------------------------------------------------------ helpers

    def _state(self):
        try:
            return hmc.fetch_status(self.url, 0.5).get("rig_state")
        except Exception:  # noqa: BLE001
            if self.proc.poll() is not None:
                self.log.seek(0)
                raise AssertionError("server exited: %s" % self.log.read().decode(errors="replace"))
            return None

    def admin(self, path):
        req = urllib.request.Request(self.url + path, data=b"", method="POST",
                                     headers={"Authorization": "Bearer " + TOKEN})
        try:
            with hmc._HTTP.open(req, timeout=2) as r:
                return r.status, json.loads(r.read())
        except urllib.error.HTTPError as e:
            with e:
                return e.code, json.loads(e.read())

    def client(self):
        c = MotionClient(self.url)
        self.clients.append(c)
        c.start()
        wait_until(lambda: c.connected, msg="client connected")
        return c

    def ack_after(self, c, seq_before):
        """Wait for an ack newer than seq_before."""
        return wait_until(lambda: c.last_ack if (c.last_ack or {}).get("seq", 0) > seq_before else None,
                          msg="ack")

    def seq(self, c):
        return (c.last_ack or {}).get("seq") or 0

    # ------------------------------------------------------------ tests

    def test_session_and_pose_are_acked_without_arming(self):
        c = self.client()
        info = c.session_info
        self.assertEqual(info["protocol"], 1)
        self.assertEqual(info["max_gain"], 0.5)
        self.assertEqual(info["source_freshness_ms"], 250)
        self.assertTrue(info["require_frame_seq"])
        self.assertTrue(c.submit_pose(0.2, -0.1, 0.0, 0.3, frame_seq=1, source_age_ms=2.0))
        ack = self.ack_after(c, 0)
        self.assertTrue(ack["accepted"], ack)
        self.assertIsNone(ack["reason"])
        self.assertAlmostEqual(ack["effective_gain"], 0.3)
        self.assertEqual(ack["limits"], [])
        self.assertTrue(ack["fresh"])
        st = wait_until(lambda: c.status() if (c.status() or {}).get("session", {}).get("last_seq") else None,
                        msg="status with the pose")
        self.assertEqual(st["session"]["source_session_prefix"], c.session_id[:8])
        self.assertNotIn("type", st)
        # Connecting and sending poses never arms or moves the rig.
        time.sleep(0.3)
        st = hmc.fetch_status(self.url)
        self.assertEqual(st["rig_state"], "READY_TO_ARM")
        self.assertFalse(st["armed"])

    def test_gain_above_max_is_capped_and_reported(self):
        c = self.client()
        c.submit_pose(0.0, 0.0, 0.0, 0.9, frame_seq=1)
        ack = self.ack_after(c, 0)
        self.assertTrue(ack["accepted"], ack)
        self.assertAlmostEqual(ack["effective_gain"], 0.5)
        self.assertIn("gain_limited", ack["limits"])
        # Gain-only message: same cap, freshness not extended.
        s = self.seq(c)
        c.submit_gain(0.7)
        ack = self.ack_after(c, s)
        self.assertTrue(ack["accepted"], ack)
        self.assertAlmostEqual(ack["effective_gain"], 0.5)
        self.assertIn("gain_limited", ack["limits"])

    def test_limits_change_the_gain_ceiling_and_are_clamped_to_the_firmware(self):
        c = self.client()
        self.assertTrue(c.submit_limits(max_gain=0.8, max_vel_count_s=7000))
        ack = wait_until(lambda: c.last_limits_ack, msg="limits ack")
        self.assertTrue(ack["accepted"], ack)
        self.assertEqual(ack["effective_limits"]["max_gain"], 0.8)
        self.assertEqual(ack["effective_limits"]["max_vel_count_s"], 6000, "firmware absolute ceiling")
        self.assertEqual(ack["limits"], ["ceiling_clamped"])
        self.assertFalse(ack["fresh"], "a limits message never extends freshness")
        # The new ceiling applies to the next pose.
        s = self.seq(c)
        c.submit_pose(0.0, 0.0, 0.0, 0.7, frame_seq=1)
        ack = self.ack_after(c, s)
        self.assertAlmostEqual(ack["effective_gain"], 0.7)
        self.assertEqual(ack["limits"], [])
        # Visible in status and GET /v1/limits; a new session resets to the config.
        st = wait_until(lambda: (lambda v: v if v and v["motion_limits"]["effective"]["max_gain"] == 0.8 else None)(c.status()),
                        msg="status with the new limits")
        self.assertEqual(st["motion_limits"]["requested"]["max_vel_count_s"], 7000)
        self.assertEqual(st["motion_limits"]["limited"], ["max_vel_count_s"])
        with hmc._HTTP.open(self.url + "/v1/limits", timeout=2) as r:
            v = json.loads(r.read())
        self.assertEqual(v["effective"]["max_gain"], 0.8)
        c2 = self.client()
        self.assertEqual(c2.session_info["max_gain"], 0.5)
        with hmc._HTTP.open(self.url + "/v1/limits", timeout=2) as r:
            v = json.loads(r.read())
        self.assertEqual(v["effective"], v["defaults"])

    def test_largest_frame_seq_the_client_allows_is_accepted(self):
        # MotionClient's upper bound must match the server's u64 parse.
        c = self.client()
        self.assertTrue(c.submit_pose(0.0, 0.0, 0.0, 0.3, frame_seq=hmc.MAX_FRAME_SEQ))
        ack = self.ack_after(c, 0)
        self.assertTrue(ack["accepted"], ack)
        self.assertEqual(hmc.fetch_status(self.url)["session"]["last_frame_seq"], hmc.MAX_FRAME_SEQ)
        self.assertFalse(c.submit_pose(0.0, 0.0, 0.0, 0.3, frame_seq=hmc.MAX_FRAME_SEQ + 1))

    def test_missing_frame_seq_is_rejected_by_default(self):
        c = self.client()
        c.submit_pose(0.0, 0.0, 0.0, 0.3)
        ack = self.ack_after(c, 0)
        self.assertFalse(ack["accepted"])
        self.assertEqual(ack["reason"], "missing_frame_seq")

    def test_resending_same_frame_seq_does_not_keep_freshness(self):
        c = self.client()
        c.submit_pose(0.1, 0.0, 0.0, 0.3, frame_seq=5)
        first = self.ack_after(c, 0)
        self.assertTrue(first["accepted"] and first["fresh"], first)
        t0 = time.monotonic()
        stream = Streamer(c, advance_frame=False)
        stream.frame = 5
        self.streamers.append(stream)
        time.sleep(FRESHNESS_S + 0.25)
        # Still resending frame 5 (new seq each time): accepted but not fresh.
        s = self.seq(c)
        ack = self.ack_after(c, s)
        self.assertTrue(ack["accepted"], ack)
        self.assertFalse(ack["fresh"], ack)
        st = hmc.fetch_status(self.url)
        self.assertFalse(st["session"]["fresh"], st["session"])
        self.assertEqual(st["session"]["last_frame_seq"], 5)
        self.assertGreater(st["session"]["last_seq"], first["seq"])
        self.assertGreater(time.monotonic() - t0, FRESHNESS_S)
        # Advancing frame_seq makes it fresh again.
        stream.advance_frame = True
        wait_until(lambda: hmc.fetch_status(self.url)["session"]["fresh"], msg="fresh after frame advances")

    def test_stop_without_token(self):
        c = self.client()
        self.assertTrue(c.stop())
        self.assertFalse(MotionClient("http://127.0.0.1:%d" % free_port()).stop())

    def test_reconnect_mid_ride_stops_rig_until_operator_restarts(self):
        # Stage-B contract for the bridge: a new session while ACTIVE always
        # ends in BRAKED_DISARMED; the client never re-arms by itself.
        c = self.client()
        stream = Streamer(c)
        self.streamers.append(stream)
        wait_until(lambda: hmc.fetch_status(self.url)["session"]["fresh"], msg="fresh pose")
        self.assertEqual(self.admin("/v1/arm")[0], 200)
        wait_until(lambda: self._state() == "HOLDING", msg="HOLDING")
        self.assertEqual(self.admin("/v1/start")[0], 200)
        wait_until(lambda: self._state() == "ACTIVE", msg="ACTIVE")
        old = c.session_id
        c.reconnect()
        seen = []

        def braked():
            s = self._state()
            if not seen or seen[-1] != s:
                seen.append(s)
            return s == "BRAKED_DISARMED"
        wait_until(braked, timeout=6.0, step=0.005, msg="BRAKED_DISARMED")
        self.assertIn("STOPPING", seen)
        self.assertNotEqual(c.session_id, old)
        # Poses of the new session are accepted, yet nothing drives.
        s = self.seq(c)
        ack = self.ack_after(c, s)
        self.assertTrue(ack["accepted"], ack)
        time.sleep(0.4)
        self.assertEqual(self._state(), "BRAKED_DISARMED")


if __name__ == "__main__":
    unittest.main()
