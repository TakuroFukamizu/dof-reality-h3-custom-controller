"""MotionClient against an in-process fake server.

REST paths (POST /v1/session, /v1/stop, GET /v1/status) are served by a stdlib
HTTP server; WS /v1/motion by websockets.sync.server on a second port (older
websockets reject non-GET requests, so one port cannot serve both). The client
is pointed at the WS port by overriding ``_motion_url``. Every WS message is
recorded with its arrival time.
"""

from __future__ import annotations

import contextlib
import http.server
import io
import json
import os
import socket
import sys
import threading
import time
import unittest
import uuid

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from websockets.sync.server import serve  # noqa: E402

import h3_motion_client as hmc  # noqa: E402
from h3_motion_client import MotionClient  # noqa: E402


def free_port() -> int:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def wait_until(pred, timeout=3.0, step=0.01, msg="condition"):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        v = pred()
        if v:
            return v
        time.sleep(step)
    raise AssertionError("timed out waiting for " + msg)


class _FakeClient(MotionClient):
    ws_base = ""

    def _motion_url(self, session):
        return "%s/v1/motion?session=%s" % (self.ws_base, session)


def client_for(fake, **kw):
    c = _FakeClient(fake.url, **kw)
    c.ws_base = fake.ws_base
    return c


class FakeServer:
    def __init__(self, freshness_ms=10000, status_period=0.05):
        self.freshness_ms = freshness_ms
        self.status_period = status_period
        self.send_status = True
        self.ack_reason = None  # e.g. "stale_session" to reject everything
        self.stop_status = 200
        self.lock = threading.Lock()
        self.sessions = []  # issued ids, in order
        self.paths = []  # every request path (HTTP and WS)
        self.messages = []  # (monotonic, ws_query_session, dict)
        self.conns = []  # (conn, query_session)
        self.session_gate = threading.Event()
        self.session_gate.set()
        self.ws_server = serve(self._handler, "127.0.0.1", 0)
        self.ws_base = "ws://127.0.0.1:%d" % self.ws_server.socket.getsockname()[1]
        fake = self

        class Handler(http.server.BaseHTTPRequestHandler):
            def do_GET(self):
                fake._http(self)

            def do_POST(self):
                fake._http(self)

            def log_message(self, *args):
                pass

        self.http_server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.http_server.daemon_threads = True
        self.url = "http://127.0.0.1:%d" % self.http_server.server_address[1]
        self._threads = [threading.Thread(target=self.ws_server.serve_forever, daemon=True),
                         threading.Thread(target=self.http_server.serve_forever, daemon=True)]
        for t in self._threads:
            t.start()

    def close(self):
        self.drop_all()
        self.ws_server.shutdown()
        self.http_server.shutdown()
        self.http_server.server_close()
        for t in self._threads:
            t.join(3)

    def _http(self, h):
        path = h.path.split("?", 1)[0]
        with self.lock:
            self.paths.append(path)
        status, body = self._route(h.command, path)
        data = json.dumps(body).encode()
        h.send_response(status)
        h.send_header("Content-Type", "application/json")
        h.send_header("Content-Length", str(len(data)))
        h.end_headers()
        h.wfile.write(data)

    def _route(self, method, path):
        if method == "POST" and path == "/v1/session":
            if not self.session_gate.is_set():
                return 503, {"ok": False, "error": "control_loop_unavailable"}
            sid = uuid.uuid4().hex
            with self.lock:
                self.sessions.append(sid)
            return 200, {"source_session": sid, "protocol": 1, "max_gain": 0.35,
                         "source_freshness_ms": self.freshness_ms, "require_frame_seq": True}
        if method == "POST" and path == "/v1/stop":
            if self.stop_status == 200:
                return 200, {"ok": True, "rig_state": "STOPPING"}
            return self.stop_status, {"ok": False, "error": "control_loop_unavailable"}
        if method == "GET" and path == "/v1/status":
            return 200, {"rig_state": "READY_TO_ARM", "armed": False, "session": {}}
        return 404, {"ok": False, "error": "not_found"}

    def _handler(self, conn):
        path = conn.request.path
        q = path.split("session=", 1)
        qsession = q[1] if len(q) == 2 else None
        with self.lock:
            self.paths.append(path.split("?", 1)[0])
            self.conns.append((conn, qsession))
        try:
            while True:
                try:
                    raw = conn.recv(timeout=self.status_period)
                except TimeoutError:
                    if self.send_status:
                        conn.send(json.dumps({"type": "status", "rig_state": "HOLDING", "armed": True,
                                              "session": {"source_session_prefix": (qsession or "")[:8],
                                                          "fresh": False}}))
                    continue
                msg = json.loads(raw)
                with self.lock:
                    self.messages.append((time.monotonic(), qsession, msg))
                if self.ack_reason is None:
                    ack = {"type": "ack", "seq": msg.get("seq"), "accepted": True, "reason": None,
                           "effective_gain": min(msg.get("gain", 0.0), 0.35), "limits": [], "fresh": True}
                else:
                    ack = {"type": "ack", "seq": msg.get("seq"), "accepted": False, "reason": self.ack_reason,
                           "effective_gain": None, "limits": [], "fresh": False}
                conn.send(json.dumps(ack))
        except Exception:  # noqa: BLE001 - connection closed
            pass

    def drop_all(self):
        with self.lock:
            conns = list(self.conns)
            self.conns.clear()
        for c, _ in conns:
            try:
                c.close()
            except Exception:  # noqa: BLE001
                pass

    def poses(self):
        with self.lock:
            return [(t, q, m) for t, q, m in self.messages if m.get("type") == "pose"]

    def all_messages(self):
        with self.lock:
            return list(self.messages)


class FakeServerTests(unittest.TestCase):
    def setUp(self):
        self.fake = None
        self.client = None

    def tearDown(self):
        if self.client is not None:
            self.client.close()
        if self.fake is not None:
            self.fake.close()

    def start(self, **fake_kw):
        self.fake = FakeServer(**fake_kw)
        self.client = client_for(self.fake, connect_timeout=1.0)
        self.client.start()
        return self.fake, self.client

    def connected(self):
        wait_until(lambda: self.client.connected, msg="client connected")

    # ------------------------------------------------------------ submit

    def test_submit_is_non_blocking_while_server_hangs(self):
        # A listener that never accepts: the HTTP session request hangs until
        # connect_timeout. submit_pose must not notice.
        hole = socket.socket()
        hole.bind(("127.0.0.1", 0))
        hole.listen(0)
        try:
            self.client = MotionClient("http://127.0.0.1:%d" % hole.getsockname()[1], connect_timeout=1.0)
            self.client.start()
            time.sleep(0.05)  # worker is inside the blocking request now
            worst = 0.0
            for i in range(500):
                t0 = time.perf_counter()
                self.assertTrue(self.client.submit_pose(0.1, 0.2, 0.0, 0.3, frame_seq=i, source_age_ms=1.0))
                worst = max(worst, time.perf_counter() - t0)
            self.assertLess(worst, 0.005, "slowest submit_pose %.2f ms" % (worst * 1000))
            self.assertFalse(self.client.connected)
            self.assertIsNone(self.client.status())
        finally:
            self.client.close()
            self.client = None
            hole.close()

    def test_submit_is_non_blocking_when_nothing_listens(self):
        self.client = MotionClient("http://127.0.0.1:%d" % free_port())
        self.client.start()
        worst = 0.0
        for i in range(500):
            t0 = time.perf_counter()
            self.client.submit_pose(0.0, 0.0, 0.0, 0.3, frame_seq=i)
            self.client.submit_gain(0.2)
            worst = max(worst, time.perf_counter() - t0)
        self.assertLess(worst, 0.005)

    def test_invalid_values_are_not_stored(self):
        self.client = MotionClient("http://127.0.0.1:%d" % free_port())
        self.assertFalse(self.client.submit_pose(float("nan"), 0, 0, 0.3, frame_seq=1))
        self.assertFalse(self.client.submit_pose(0, 0, 0, float("inf"), frame_seq=1))
        self.assertFalse(self.client.submit_pose(0, 0, 0, 0.3, frame_seq=1, source_age_ms=-1))
        self.assertFalse(self.client.submit_pose(0, 0, 0, 0.3, frame_seq=1.5))
        self.assertFalse(self.client.submit_pose(0, 0, 0, 0.3, frame_seq=True))
        self.assertFalse(self.client.submit_gain(float("nan")))
        self.assertTrue(self.client.submit_pose(0, 0, 0, 0.3, frame_seq=1))

    def test_huge_ints_return_false_instead_of_raising(self):
        # float(10**400) raises OverflowError; callers (bridge handler threads) must get False.
        self.client = MotionClient("http://127.0.0.1:%d" % free_port())
        huge = 10**400
        for args, kw in (((huge, 0, 0, 0.3), {}), ((0, huge, 0, 0.3), {}), ((0, 0, huge, 0.3), {}),
                         ((0, 0, 0, huge), {}), ((0, 0, 0, 0.3), {"source_age_ms": huge})):
            self.assertFalse(self.client.submit_pose(*args, frame_seq=1, **kw), (args, kw))
        self.assertFalse(self.client.submit_gain(huge))
        self.assertIsNone(self.client._pose)
        self.assertIsNone(self.client._gain)

    def test_frame_seq_must_fit_the_servers_u64(self):
        self.client = MotionClient("http://127.0.0.1:%d" % free_port())
        self.assertEqual(hmc.MAX_FRAME_SEQ, 2**64 - 1)
        self.assertFalse(self.client.submit_pose(0, 0, 0, 0.3, frame_seq=2**64))
        self.assertFalse(self.client.submit_pose(0, 0, 0, 0.3, frame_seq=-1))
        self.assertIsNone(self.client._pose)
        self.assertTrue(self.client.submit_pose(0, 0, 0, 0.3, frame_seq=2**64 - 1))

    def test_latest_slot_sends_only_the_newest_pose(self):
        fake, client = self.start()
        self.connected()
        for i in range(100):
            client.submit_pose(i / 100.0, 0.0, 0.0, 0.3, frame_seq=i)
        wait_until(lambda: fake.poses() and fake.poses()[-1][2]["frame_seq"] == 99, msg="final pose")
        time.sleep(0.1)
        poses = [m for _, _, m in fake.poses()]
        self.assertLess(len(poses), 50, "latest-value slot, got %d messages" % len(poses))
        self.assertEqual(poses[-1]["pitch"], 0.99)
        self.assertEqual(poses[-1]["frame_seq"], 99)
        seqs = [m["seq"] for m in poses]
        self.assertTrue(all(a < b for a, b in zip(seqs, seqs[1:])), seqs)
        frames = [m["frame_seq"] for m in poses]
        self.assertTrue(all(a < b for a, b in zip(frames, frames[1:])), frames)
        m = poses[-1]
        self.assertEqual(m["type"], "pose")
        self.assertEqual(m["protocol"], 1)
        self.assertEqual(m["source_session"], fake.sessions[-1])

    def test_no_resend_when_idle(self):
        fake, client = self.start()
        self.connected()
        for i in range(5):
            client.submit_pose(0.1, 0.0, 0.0, 0.3, frame_seq=i)
            time.sleep(0.01)
        wait_until(lambda: fake.poses() and fake.poses()[-1][2]["frame_seq"] == 4, msg="last pose")
        n = len(fake.all_messages())
        time.sleep(0.6)  # status keeps arriving from the server meanwhile
        self.assertEqual(len(fake.all_messages()), n, "client sent something while idle")

    def test_frame_seq_none_is_sent_without_frame_seq(self):
        fake, client = self.start()
        self.connected()
        client.submit_pose(0.0, 0.0, 0.0, 0.3)
        m = wait_until(lambda: fake.poses(), msg="pose")[-1][2]
        self.assertNotIn("frame_seq", m)  # never invented by the client

    # ------------------------------------------------------------ source_age

    def test_source_age_includes_time_waiting_in_slot(self):
        fake = self.fake = FakeServer(freshness_ms=10000)
        fake.session_gate.clear()  # session refused: client is held off
        client = self.client = client_for(fake)
        client.start()
        t_submit = time.monotonic()
        client.submit_pose(0.2, 0.0, 0.0, 0.3, frame_seq=7, source_age_ms=10.0)
        time.sleep(0.3)
        t_release = time.monotonic()
        fake.session_gate.set()
        t_recv, _, m = wait_until(lambda: fake.poses(), timeout=4.0, msg="held pose")[0]
        lo = 10.0 + (t_release - t_submit) * 1000.0
        hi = 10.0 + (t_recv - t_submit) * 1000.0
        self.assertGreaterEqual(m["source_age_ms"], lo - 1.0)
        self.assertLessEqual(m["source_age_ms"], hi + 1.0)
        self.assertEqual(m["frame_seq"], 7)

    def test_source_age_while_connected_is_submitted_age_plus_small_delay(self):
        fake, client = self.start()
        self.connected()
        client.submit_pose(0.0, 0.0, 0.0, 0.3, frame_seq=1, source_age_ms=7.5)
        m = wait_until(lambda: fake.poses(), msg="pose")[-1][2]
        self.assertGreaterEqual(m["source_age_ms"], 7.5)
        self.assertLess(m["source_age_ms"], 7.5 + 50.0)

    # ------------------------------------------------------------ reconnect

    def test_reconnect_after_drop_uses_a_new_session(self):
        fake, client = self.start()
        self.connected()
        first = fake.sessions[-1]
        self.assertEqual(client.session_id, first)
        fake.drop_all()
        wait_until(lambda: len(fake.sessions) >= 2 and client.connected and client.session_id != first,
                   timeout=4.0, msg="reconnect")
        second = fake.sessions[-1]
        self.assertNotEqual(first, second)
        client.submit_pose(0.1, 0.0, 0.0, 0.3, frame_seq=1)
        _, q, m = wait_until(lambda: fake.poses(), msg="pose")[-1]
        self.assertEqual(m["source_session"], second)
        self.assertEqual(q, second)  # ?session= pinned on the WS URL
        # Transport only: no admin route was touched.
        self.assertTrue(set(fake.paths) <= {"/v1/session", "/v1/motion"}, fake.paths)

    def test_explicit_reconnect_new_session_and_no_resend(self):
        fake, client = self.start()
        self.connected()
        client.submit_pose(0.1, 0.0, 0.0, 0.3, frame_seq=1)
        wait_until(lambda: fake.poses(), msg="pose")
        first = client.session_id
        client.reconnect()
        wait_until(lambda: client.connected and client.session_id not in (None, first), msg="reconnect")
        time.sleep(0.3)
        self.assertEqual(len(fake.poses()), 1, "reconnect must not resend the last pose")
        seq1 = fake.poses()[0][2]["seq"]
        client.submit_pose(0.2, 0.0, 0.0, 0.3, frame_seq=2)
        m = wait_until(lambda: len(fake.poses()) == 2 and fake.poses(), msg="pose 2")[-1][2]
        self.assertEqual(m["source_session"], client.session_id)
        self.assertGreater(m["seq"], seq1)
        self.assertTrue(set(fake.paths) <= {"/v1/session", "/v1/motion"}, fake.paths)

    def test_silent_server_triggers_reconnect(self):
        fake = self.fake = FakeServer()
        fake.send_status = False
        client = self.client = client_for(fake)
        client.RECV_STALL_S = 0.3
        client.start()
        wait_until(lambda: len(fake.sessions) >= 2, timeout=3.0, msg="reconnect after stall")

    def test_stale_pose_is_dropped_at_reconnect(self):
        fake, client = self.start(freshness_ms=100)
        self.connected()
        fake.session_gate.clear()
        fake.drop_all()
        wait_until(lambda: not client.connected, msg="disconnect")
        client.submit_pose(0.5, 0.0, 0.0, 0.3, frame_seq=1, source_age_ms=0.0)
        time.sleep(0.25)  # > freshness 100 ms
        n_sessions = len(fake.sessions)
        fake.session_gate.set()
        wait_until(lambda: client.connected and len(fake.sessions) > n_sessions, timeout=4.0, msg="reconnect")
        time.sleep(0.3)
        self.assertEqual(fake.poses(), [], "stale pose was sent after reconnect")
        client.submit_pose(0.6, 0.0, 0.0, 0.3, frame_seq=2)
        m = wait_until(lambda: fake.poses(), msg="fresh pose")[-1][2]
        self.assertEqual(m["frame_seq"], 2)

    def test_pending_gain_is_merged_into_pending_pose(self):
        fake = self.fake = FakeServer()
        fake.session_gate.clear()
        client = self.client = client_for(fake)
        client.start()
        client.submit_pose(0.1, 0.0, 0.0, 0.3, frame_seq=3)
        client.submit_gain(0.2)
        fake.session_gate.set()
        wait_until(lambda: fake.poses(), timeout=4.0, msg="pose")
        time.sleep(0.1)
        msgs = [m for _, _, m in fake.all_messages()]
        self.assertEqual([m["type"] for m in msgs], ["pose"])
        self.assertEqual(msgs[0]["gain"], 0.2)

    def test_gain_only_message(self):
        fake, client = self.start()
        self.connected()
        client.submit_gain(0.25)
        m = wait_until(lambda: fake.all_messages(), msg="gain")[-1][2]
        self.assertEqual(m["type"], "gain")
        self.assertEqual(m["gain"], 0.25)
        self.assertEqual(m["source_session"], client.session_id)
        self.assertNotIn("frame_seq", m)

    def test_stale_session_ack_stops_sending(self):
        fake, client = self.start()
        self.connected()
        fake.ack_reason = "stale_session"
        client.submit_pose(0.1, 0.0, 0.0, 0.3, frame_seq=1)
        wait_until(lambda: client.session_lost, msg="session_lost")
        n = len(fake.poses())
        client.submit_pose(0.2, 0.0, 0.0, 0.3, frame_seq=2)
        time.sleep(0.2)
        self.assertEqual(len(fake.poses()), n)
        # reconnect() takes a new session and sends again.
        fake.ack_reason = None
        client.reconnect()
        wait_until(lambda: client.connected and not client.session_lost, msg="retake")
        client.submit_pose(0.3, 0.0, 0.0, 0.3, frame_seq=3)
        wait_until(lambda: len(fake.poses()) == n + 1, msg="pose after retake")

    # ------------------------------------------------------------ status / ack

    def test_status_and_ack_parsing(self):
        seen = []
        fake = self.fake = FakeServer()
        client = self.client = client_for(fake, on_status=seen.append)
        client.STATUS_MAX_AGE_S = 0.3
        client.start()
        self.connected()
        st = wait_until(client.status, msg="status")
        self.assertEqual(st["rig_state"], "HOLDING")
        self.assertNotIn("type", st)
        self.assertEqual(st["session"]["source_session_prefix"], client.session_id[:8])
        self.assertTrue(seen and seen[-1]["rig_state"] == "HOLDING")
        self.assertIsNone(client.last_ack)
        client.submit_pose(0.1, 0.0, 0.0, 0.9, frame_seq=1)
        ack = wait_until(lambda: client.last_ack, msg="ack")
        self.assertTrue(ack["accepted"])
        self.assertEqual(ack["effective_gain"], 0.35)
        self.assertEqual(ack["seq"], fake.poses()[-1][2]["seq"])
        self.assertEqual(client.session_info["source_freshness_ms"], 10000)
        fake.send_status = False
        time.sleep(0.5)
        self.assertIsNone(client.status(), "status older than STATUS_MAX_AGE_S must read as unknown")

    def test_status_unknown_before_connect(self):
        self.client = MotionClient("http://127.0.0.1:%d" % free_port())
        self.assertIsNone(self.client.status())
        self.assertIsNone(self.client.last_ack)
        self.assertFalse(self.client.connected)

    # ------------------------------------------------------------ stop / CLI

    def test_stop_returns_false_quickly_when_server_down(self):
        c = MotionClient("http://127.0.0.1:%d" % free_port())
        t0 = time.monotonic()
        self.assertFalse(c.stop())
        self.assertLess(time.monotonic() - t0, 0.5)

    def test_stop_result_follows_server_answer(self):
        fake = self.fake = FakeServer()
        c = MotionClient(fake.url)
        self.assertTrue(c.stop())
        fake.stop_status = 503
        self.assertFalse(c.stop())
        self.assertIn("/v1/stop", fake.paths)

    def test_cli(self):
        fake = self.fake = FakeServer()
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            self.assertEqual(hmc.main(["status", "--url", fake.url]), 0)
            self.assertEqual(hmc.main(["stop", "--url", fake.url]), 0)
        self.assertIn("READY_TO_ARM", out.getvalue())
        down = "http://127.0.0.1:%d" % free_port()
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(hmc.main(["stop", "--url", down]), 1)
            self.assertEqual(hmc.main(["status", "--url", down]), 1)

    def test_ws_url(self):
        self.assertEqual(hmc._ws_url("http://127.0.0.1:8892", "ab"), "ws://127.0.0.1:8892/v1/motion?session=ab")
        self.assertEqual(hmc._ws_url("https://h:1/x/", "ab"), "wss://h:1/x/v1/motion?session=ab")


if __name__ == "__main__":
    unittest.main()
