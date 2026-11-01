"""The h3game patch: applies with git am, legacy unchanged, server never touches serial.

Needs a clone of TakuroFukamizu/20261004_shi3z_hackathon: H3GAME_SRC, or the
cache clone at the pinned commit that _support makes (network on first use).
Skipped if neither is available. Runs bridge.py as a subprocess.
"""

from __future__ import annotations

import os
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import _support as S  # noqa: E402

OLD_STATE_KEYS = {"pitch", "roll", "yaw", "connected", "port", "gain", "auto"}
NEW_STATE_KEYS = {"backend", "rig_state", "armed", "calibration_valid", "fault", "server_url", "effective_gain"}


def lan_address():
    """A non-loopback IPv4 address of this machine, or None (no packet is sent)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("192.0.2.1", 9))  # TEST-NET-1: only picks the outgoing interface
        ip = s.getsockname()[0]
    except OSError:
        return None
    finally:
        s.close()
    return None if ip.startswith("127.") or ip == "0.0.0.0" else ip


class CountingServer:
    """Stand-in control server that only counts the requests it gets."""

    def __init__(self):
        self.requests = []
        outer = self

        class Handler(BaseHTTPRequestHandler):
            def _any(self):
                outer.requests.append((self.command, self.path))
                self.send_response(503)
                self.send_header("Content-Length", "0")
                self.end_headers()

            do_GET = do_POST = _any

            def log_message(self, *a):
                pass

        self.httpd = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.url = "http://127.0.0.1:%d" % self.httpd.server_address[1]
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()

    def close(self):
        self.httpd.shutdown()
        self.httpd.server_close()


def read(path):
    with open(path, encoding="utf-8") as f:
        return f.read()


def _skip_reason():
    if not S.src_available():
        return S.src_skip_reason()
    if not S.patches():
        return "no patch in %s" % S.PATCH_DIR
    return None


@unittest.skipIf(_skip_reason() is not None, _skip_reason() or "")
class PatchTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.orig = S.make_clone(apply_patch=False)
        cls.repo = S.make_clone(apply_patch=True)

    @classmethod
    def tearDownClass(cls):
        S.remove_clone(cls.orig)
        S.remove_clone(cls.repo)

    def setUp(self):
        self.procs = []
        self.tmp = tempfile.mkdtemp(prefix="h3stub-")
        self.marker = os.path.join(self.tmp, "imported.txt")
        self.stubs = os.path.join(self.tmp, "stubs")
        os.makedirs(self.stubs)

    def tearDown(self):
        for p in self.procs:
            p.kill()
        import shutil
        shutil.rmtree(self.tmp, ignore_errors=True)

    def imported(self):
        return read(self.marker).split() if os.path.exists(self.marker) else []

    def bridge(self, repo, env):
        port = S.free_port()
        p = S.start_bridge(repo, port, env, pythonpath=self.stubs)
        self.procs.append(p)
        return p, "http://127.0.0.1:%d" % port

    # ------------------------------------------------------------ patch itself

    def test_source_blobs_match_and_patch_touches_only_expected_files(self):
        for path, sha in S.EXPECTED_BLOBS.items():
            self.assertEqual(S.git(self.orig, "rev-parse", "HEAD:" + path).strip(), sha, path)
        changed = set(S.git(self.repo, "diff", "--name-only", "HEAD~1", "HEAD").split())
        self.assertEqual(changed, {"h3game/bridge.py", "h3game/web/index.html",
                                   "h3game/h3_server_backend.py", "h3game/h3_motion_client.py"})
        # h3_direct.py stays for legacy; scenes and game.js untouched.
        self.assertEqual(S.git(self.repo, "rev-parse", "HEAD:h3game/h3_direct.py").strip(),
                         S.EXPECTED_BLOBS["h3game/h3_direct.py"])

    def test_copies_match_canonical_sources(self):
        h3 = os.path.join(self.repo, "h3game")
        self.assertEqual(read(os.path.join(h3, "h3_server_backend.py")), read(S.CANONICAL_BACKEND))
        copy = read(os.path.join(h3, "h3_motion_client.py")).split("\n")
        self.assertTrue(copy[0].startswith("# Copy of clients/python/h3_motion_client.py"))
        self.assertEqual("\n".join(copy[2:]), read(S.CANONICAL_CLIENT))

    def test_index_html_sends_frame_seq_and_age(self):
        html = read(os.path.join(self.repo, "h3game", "web", "index.html"))
        self.assertIn("frame_age_ms:", html)
        self.assertIn("if(!document.hidden){frameSeq++;", html)
        # Hidden: zeros as before, but with the last frame_seq POSTed while visible
        # (rAF runs ahead of the POST; zeros with a new frame_seq would be a fresh frame).
        self.assertIn("const h=document.hidden;if(!h){sentSeq=frameSeq;sentAt=frameAt}", html)
        self.assertIn("frame_seq:sentSeq", html)
        self.assertNotIn("frame_seq:frameSeq", html)
        self.assertIn("o[a]=h?0:", html)

    def test_index_html_shows_server_rig_state_and_lost_session(self):
        html = read(os.path.join(self.repo, "h3game", "web", "index.html"))
        # The play HUD and the setup card show rig_state, a lost session, a
        # fault and a capped gain in server mode (legacy pages have no backend key).
        self.assertIn("function srvNote(){if(st.backend!=='server')return'';", html)
        self.assertIn("st.session_lost?", html)
        self.assertIn("st.effective_gain", html)
        # The server's axis-assignment note (/state hud_note: 系統の競合 etc.).
        self.assertIn("(st.hud_note?' '+st.hud_note:'')", html)
        self.assertIn("'● H3 未接続')+srvNote()+", html)

    # ------------------------------------------------------------ second launch / LAN

    def test_second_bridge_on_a_busy_port_never_touches_the_server(self):
        # Operator starts the launcher twice: the second bridge must fail to
        # bind BEFORE it takes the server session (that would stop the ride
        # and lock out the bridge that is running).
        srv = CountingServer()
        busy = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            busy.bind(("0.0.0.0", 0))
            busy.listen(1)
            port = busy.getsockname()[1]
            env = dict(os.environ, H3_BACKEND="server", H3_SERVER_URL=srv.url, H3_PORT=str(port),
                       PYTHONDONTWRITEBYTECODE="1", PYTHONPATH=self.stubs)
            env.pop("H3_DRY", None)
            h3 = os.path.join(self.repo, "h3game")
            r = subprocess.run([sys.executable, "-u", os.path.join(h3, "bridge.py")], cwd=h3, env=env,
                               capture_output=True, text=True, timeout=20)
            time.sleep(0.5)
            self.assertNotEqual(r.returncode, 0, r.stdout + r.stderr)
            self.assertIn("Address already in use", r.stderr)
            self.assertEqual(srv.requests, [], "a bridge that could not bind contacted the server")
            self.assertNotIn("制御サーバー:", r.stdout)
        finally:
            busy.close()
            srv.close()

    def test_lan_peers_cannot_reconnect_or_claim(self):
        lan = lan_address()
        if lan is None:
            self.skipTest("no non-loopback IPv4 address on this machine")
        p, url = self.bridge(self.repo, {"H3_BACKEND": "dry"})
        port = url.rsplit(":", 1)[1]
        lan_url = "http://%s:%s" % (lan, port)
        before = S.http("GET", url + "/claim")[1]["owner"]
        self.assertEqual(S.http("GET", lan_url + "/claim")[1], {"ignored": True})
        self.assertEqual(S.http("GET", lan_url + "/reconnect")[1], {"ignored": True})
        # Neither took effect: the owner moved only for the loopback claim, and
        # connect() ran only at start-up (each run prints the H3_DRY line).
        self.assertEqual(S.http("GET", url + "/claim")[1]["owner"], before + 1)
        self.assertEqual(p.output().count("H3_DRY"), 1, p.output())
        S.http("GET", url + "/reconnect")
        S.wait_until(lambda: p.output().count("H3_DRY") == 2, timeout=2.0, msg="loopback /reconnect runs connect()")
        # The LAN may still look (state) but not drive.
        self.assertEqual(S.http("GET", lan_url + "/state")[0], 200)
        self.assertEqual(S.http("POST", lan_url + "/", {"owner": before + 1, "pitch": 0.5})[1], {"ignored": True})

    # ------------------------------------------------------------ imports

    def test_importing_bridge_imports_neither_serial_nor_websockets(self):
        S.write_stub(self.stubs, "serial", self.marker)
        S.write_stub(self.stubs, "websockets", self.marker)
        h3 = os.path.join(self.repo, "h3game")
        for backend in ("", "legacy", "server", "dry"):
            env = dict(os.environ, PYTHONPATH=self.stubs, H3_BACKEND=backend, PYTHONDONTWRITEBYTECODE="1")
            subprocess.run([sys.executable, "-c", "import bridge"], cwd=h3, env=env, check=True,
                           capture_output=True, timeout=20)
        self.assertEqual(self.imported(), [])

    def test_invalid_backend_value_exits(self):
        h3 = os.path.join(self.repo, "h3game")
        env = dict(os.environ, H3_BACKEND="sever", H3_PORT=str(S.free_port()))
        r = subprocess.run([sys.executable, "bridge.py"], cwd=h3, env=env, capture_output=True, text=True,
                           timeout=20)
        self.assertNotEqual(r.returncode, 0)
        self.assertIn("H3_BACKEND", r.stderr)

    # ------------------------------------------------------------ legacy

    def test_legacy_default_imports_h3_direct_lazily_and_matches_original(self):
        S.write_stub(self.stubs, "serial", self.marker)
        S.write_stub(self.stubs, "websockets", self.marker)
        pose = {"owner": 1, "pitch": 0.5, "roll": -0.25, "yaw": 0.125, "gain": 0.3,
                "frame_seq": 12, "frame_age_ms": 3.0}
        results = {}
        for name, repo in (("orig", self.orig), ("patched", self.repo)):
            p, url = self.bridge(repo, {})
            # connect() at startup: h3_direct → serial (stub raises) → "未接続".
            S.wait_until(lambda: "serial" in self.imported(), msg="%s: serial import attempt" % name)
            st0 = S.http("GET", url + "/state")[1]
            post = S.http("POST", url + "/", pose)[1]
            results[name] = (st0, post, p, url)
            self.assertNotIn("websockets", self.imported(), name)  # legacy needs no new dependency
            if name == "orig":
                p.kill()  # its watcher would keep re-importing serial
                self.procs.remove(p)
            os.remove(self.marker)
        st_o, post_o, _, _ = results["orig"]
        st_p, post_p, proc, url = results["patched"]
        self.assertEqual(st_p, st_o)
        self.assertEqual(post_p, post_o)
        self.assertEqual(set(st_p), set(st_o))
        self.assertFalse(NEW_STATE_KEYS & set(st_p), st_p)
        self.assertIn("H3 未接続(空回りモード)", proc.output())
        # The legacy 1 s fade to center still runs.
        S.wait_until(lambda: abs(S.http("GET", url + "/state")[1]["pitch"]) < 0.4, timeout=4.0,
                     msg="legacy fade to center")

    # ------------------------------------------------------------ server

    def test_server_mode_never_imports_serial_and_does_not_block(self):
        S.write_stub(self.stubs, "serial", self.marker)
        dead = "http://127.0.0.1:%d" % S.free_port()  # nothing listens here
        p, url = self.bridge(self.repo, {"H3_BACKEND": "server", "H3_SERVER_URL": dead})
        slowest = 0.0
        for i in range(1, 31):
            t0 = time.monotonic()
            s, r = S.http("POST", url + "/", {"owner": 1, "pitch": 0.2, "roll": 0.0, "yaw": 0.0, "gain": 0.3,
                                              "frame_seq": i, "frame_age_ms": 1.0})
            slowest = max(slowest, time.monotonic() - t0)
            self.assertEqual(s, 200)
            time.sleep(1 / 30)
        self.assertLess(slowest, 0.25, "POST waited on the unreachable server")
        # Bad numbers (JSON int beyond float, frame_seq beyond u64) are answered and counted
        # as invalid; neither raises in the handler nor blocks the next frame.
        for extra in ({"frame_seq": 31, "frame_age_ms": 10**400}, {"frame_seq": 2**64, "frame_age_ms": 1.0},
                      {"frame_seq": 31, "frame_age_ms": 1.0}):
            d = {"owner": 1, "pitch": 0.2, "roll": 0.0, "yaw": 0.0, "gain": 0.3}
            d.update(extra)
            self.assertEqual(S.http("POST", url + "/", d)[0], 200, extra)
        S.http("POST", url + "/", {"owner": 1, "auto": True})
        S.http("GET", url + "/reconnect")
        time.sleep(3.5)  # past one watcher() period
        st = S.http("GET", url + "/state")[1]
        self.assertEqual(self.imported(), [], p.output())
        self.assertTrue(OLD_STATE_KEYS <= set(st), st)
        self.assertTrue(NEW_STATE_KEYS <= set(st), st)
        self.assertEqual(st["backend"], "server")
        self.assertEqual(st["server_url"], dead)
        self.assertFalse(st["connected"])
        self.assertIsNone(st["rig_state"])
        self.assertFalse(st["auto"])
        self.assertEqual(st["auto_ignored"], 1)
        self.assertEqual(st["pose_stats"]["invalid"], 2, st["pose_stats"])
        self.assertEqual(st["pose_stats"]["sent"], 31, st["pose_stats"])
        self.assertEqual(st["frame_seq"], 31)
        self.assertNotIn("backend_errors", st)
        # Commanded values, not faded toward center (no sender in server mode).
        self.assertAlmostEqual(st["pitch"], 0.2)
        self.assertIn("制御サーバー:", p.output())
        self.assertTrue(p.alive())

    def test_dry_modes_import_nothing(self):
        S.write_stub(self.stubs, "serial", self.marker)
        S.write_stub(self.stubs, "websockets", self.marker)
        for env in ({"H3_BACKEND": "dry"}, {"H3_BACKEND": "server", "H3_DRY": "1"}):
            p, url = self.bridge(self.repo, env)
            S.http("POST", url + "/", {"owner": 1, "pitch": 0.2, "roll": 0, "yaw": 0, "gain": 0.3, "frame_seq": 1})
            time.sleep(3.5)
            st = S.http("GET", url + "/state")[1]
            self.assertEqual(st["backend"], "dry", env)
            self.assertFalse(st["connected"])
            self.assertIn("H3_DRY", p.output())
            self.assertEqual(self.imported(), [], env)


if __name__ == "__main__":
    unittest.main()
