"""Unit tests of h3_server_backend with a fake MotionClient (no network)."""

from __future__ import annotations

import json
import os
import sys
import threading
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import h3_server_backend as hsb  # noqa: E402
from h3_server_backend import ServerBackend  # noqa: E402


class FakeClient:
    """Records calls; mirrors MotionClient's public surface used by the backend."""

    def __init__(self, url):
        self.url = url
        self.calls = []
        self.connected = False
        self.session_id = None
        self.session_lost = False
        self.last_ack = None
        self._status = None
        self.reject_pose = False

    def start(self):
        self.calls.append(("start",))

    def close(self):
        self.calls.append(("close",))

    def reconnect(self):
        self.calls.append(("reconnect",))

    def submit_pose(self, pitch, roll, yaw, gain, frame_seq=None, source_age_ms=0.0):
        self.calls.append(("pose", pitch, roll, yaw, gain, frame_seq, source_age_ms))
        return not self.reject_pose

    def submit_gain(self, gain):
        self.calls.append(("gain", gain))
        return True

    def status(self):
        return self._status

    # test helpers
    def kinds(self):
        return [c[0] for c in self.calls]

    def poses(self):
        return [c for c in self.calls if c[0] == "pose"]


def body(frame_seq, pitch=0.1, roll=0.2, yaw=0.3, gain=0.3, age=4.0, **extra):
    d = {"owner": 1, "pitch": pitch, "roll": roll, "yaw": yaw, "gain": gain}
    if frame_seq is not None:
        d["frame_seq"] = frame_seq
    if age is not None:
        d["frame_age_ms"] = age
    d.update(extra)
    return d


STATE = {"pitch": 0.0, "roll": 0.0, "yaw": 0.0, "gain": 0.35}


class SubmitTests(unittest.TestCase):
    def setUp(self):
        self.b = ServerBackend("http://127.0.0.1:9/", client_factory=FakeClient)
        self.c = self.b.client

    def test_url_normalised_and_no_network_on_construct(self):
        self.assertEqual(self.b.url, "http://127.0.0.1:9")
        self.assertEqual(self.c.url, "http://127.0.0.1:9")
        self.assertEqual(self.c.calls, [])

    def test_lifecycle_delegates_and_never_arms(self):
        self.b.start()
        self.b.reconnect()
        self.b.close()
        self.assertEqual(self.c.kinds(), ["start", "reconnect", "close"])
        self.assertFalse(hasattr(self.b, "arm") or hasattr(self.b, "start_ride"))

    def test_pose_with_frame_seq_is_forwarded_with_age(self):
        self.assertEqual(self.b.submit(1, body(1, age=7.5), STATE), hsb.SENT)
        self.assertEqual(self.c.poses(), [("pose", 0.1, 0.2, 0.3, 0.3, 1, 7.5)])

    def test_missing_age_means_zero(self):
        self.b.submit(1, body(1, age=None), STATE)
        self.assertEqual(self.c.poses()[-1][6], 0.0)

    def test_repeated_frame_seq_is_not_forwarded(self):
        self.b.submit(1, body(5), STATE)
        # Hidden tab: zeros with the same frame_seq; frozen loop: same values.
        self.assertEqual(self.b.submit(1, body(5, 0.0, 0.0, 0.0), STATE), hsb.REPEAT)
        self.assertEqual(self.b.submit(1, body(5), STATE), hsb.REPEAT)
        self.assertEqual(len(self.c.poses()), 1)
        self.assertEqual(self.b.stats[hsb.REPEAT], 2)

    def test_same_frame_with_new_gain_is_gain_only(self):
        self.b.submit(1, body(5, gain=0.3), STATE)
        self.assertEqual(self.b.submit(1, body(5, gain=0.25), STATE), hsb.GAIN_ONLY)
        self.assertEqual(self.c.calls[-1], ("gain", 0.25))
        self.assertEqual(len(self.c.poses()), 1)
        # Same gain again: nothing new.
        self.assertEqual(self.b.submit(1, body(5, gain=0.25), STATE), hsb.REPEAT)

    def test_gain_only_post_without_pose_keys(self):
        self.assertEqual(self.b.submit(1, {"owner": 1, "gain": 0.2}, STATE), hsb.GAIN_ONLY)
        self.assertEqual(self.c.calls, [("gain", 0.2)])
        self.assertEqual(self.b.submit(1, {"owner": 1}, STATE), hsb.NOTHING)

    def test_frame_seq_backwards_same_owner_is_dropped(self):
        self.b.submit(1, body(10), STATE)
        self.assertEqual(self.b.submit(1, body(9), STATE), hsb.OLD_FRAME)
        self.assertEqual(len(self.c.poses()), 1)
        self.assertEqual(self.b.submit(1, body(11), STATE), hsb.SENT)

    def test_new_owner_reconnects_then_accepts_restarted_frame_seq(self):
        self.b.submit(1, body(500), STATE)
        self.assertEqual(self.b.submit(2, body(1), STATE), hsb.SENT)
        kinds = self.c.kinds()
        self.assertEqual(kinds, ["pose", "reconnect", "pose"])
        self.assertEqual(self.c.poses()[-1][5], 1)
        self.assertEqual(self.b.stats["new_page"], 1)

    def test_first_owner_does_not_reconnect(self):
        self.b.submit(7, body(1), STATE)
        self.assertNotIn("reconnect", self.c.kinds())

    def test_page_without_frame_seq_is_forwarded_every_time(self):
        # Old page: server decides (rejects unless allow_missing_frame_seq).
        self.assertEqual(self.b.submit(1, body(None), STATE), hsb.SENT)
        self.assertEqual(self.b.submit(1, body(None), STATE), hsb.SENT)
        self.assertEqual([p[5] for p in self.c.poses()], [None, None])

    def test_invalid_values_are_dropped(self):
        bad = [
            body(1, pitch=float("nan")),
            body(1, roll=float("inf")),
            body(1, pitch="abc"),
            body(1, gain=None),
            body(1.5),
            body(-1),
            body(True),
            body("3"),
            body(1, age=-1.0),
            body(1, age=float("nan")),
            body(1, age="x"),
            body(2**64),                 # beyond the server's u64
            body(1, pitch=10**400),      # JSON int too large for float (OverflowError)
            body(1, gain=10**400),
            body(1, age=10**400),
        ]
        for d in bad:
            self.assertEqual(self.b.submit(1, d, STATE), hsb.INVALID, d)
        self.assertEqual(self.c.poses(), [])
        self.assertEqual(self.b.stats[hsb.INVALID], len(bad))

    def test_too_large_frame_seq_does_not_block_later_frames(self):
        # Stored, it would turn every normal frame into old_frame until a reload.
        self.assertEqual(self.b.submit(1, body(2**64), STATE), hsb.INVALID)
        self.assertEqual(self.b.submit(1, body(6), STATE), hsb.SENT)
        self.assertEqual(self.b.submit(1, body(7), STATE), hsb.SENT)
        self.assertEqual(self.b.submit(1, body(2**64 - 1), STATE), hsb.SENT)  # largest u64 is valid

    def test_overflowing_json_from_post_is_invalid_not_raised(self):
        raw = '{"owner":1,"pitch":0.1,"roll":0,"yaw":0,"gain":0.3,"frame_seq":5,"frame_age_ms":1%s}' % ("0" * 400)
        self.assertEqual(self.b.submit(1, json.loads(raw), STATE), hsb.INVALID)
        self.assertEqual(self.b.submit(1, body(5), STATE), hsb.SENT)

    def test_client_refusal_counts_invalid_and_keeps_frame(self):
        self.c.reject_pose = True
        self.assertEqual(self.b.submit(1, body(3), STATE), hsb.INVALID)
        self.c.reject_pose = False
        self.assertEqual(self.b.submit(1, body(3), STATE), hsb.SENT)

    def test_values_are_passed_raw_not_clamped(self):
        # Range checks and limit reporting belong to the server.
        self.b.submit(1, body(1, pitch=1.5, gain=0.9), STATE)
        self.assertEqual(self.c.poses()[-1][1], 1.5)
        self.assertEqual(self.c.poses()[-1][4], 0.9)

    def test_missing_axis_uses_last_sent_value_and_fallback_gain(self):
        self.b.submit(1, {"owner": 1, "pitch": 0.4, "roll": 0.5, "yaw": 0.6, "frame_seq": 1}, STATE)
        self.assertEqual(self.c.poses()[-1][1:5], (0.4, 0.5, 0.6, 0.35))
        self.b.submit(1, {"owner": 1, "pitch": -0.1, "frame_seq": 2}, STATE)
        self.assertEqual(self.c.poses()[-1][1:5], (-0.1, 0.5, 0.6, 0.35))

    def test_submit_never_blocks_on_client_network(self):
        # The fake returns at once; the backend itself must not sleep or poll.
        t0 = time.monotonic()
        for i in range(1, 2001):
            self.b.submit(1, body(i), STATE)
        self.assertLess(time.monotonic() - t0, 1.0)

    def test_concurrent_submits_keep_frame_order(self):
        # ThreadingHTTPServer: POSTs can overlap; forwarded frame_seq never goes back.
        def worker(start):
            for i in range(start, 3000, 4):
                self.b.submit(1, body(i), STATE)

        ts = [threading.Thread(target=worker, args=(s,)) for s in range(1, 5)]
        for t in ts:
            t.start()
        for t in ts:
            t.join()
        seqs = [p[5] for p in self.c.poses()]
        self.assertEqual(seqs, sorted(seqs))
        self.assertEqual(len(seqs), len(set(seqs)))


def snapshot(rig_state="READY_TO_ARM", prefix="abcdef12", fresh=True, **kw):
    st = {
        "rig_state": rig_state, "armed": False, "calibration_valid": True, "fault": None, "blockers": [],
        "effective_gain": 0.3,
        "session": {"source_session_prefix": prefix, "fresh": fresh},
    }
    st.update(kw)
    return st


class StateTests(unittest.TestCase):
    NEW_KEYS = ("backend", "rig_state", "armed", "calibration_valid", "fault", "server_url", "effective_gain")

    def setUp(self):
        self.b = ServerBackend("http://127.0.0.1:8892", client_factory=FakeClient)
        self.c = self.b.client

    def test_unreachable_reports_unknown_not_false(self):
        s = self.b.state()
        for k in self.NEW_KEYS + ("connected", "port"):
            self.assertIn(k, s)
        self.assertEqual(s["backend"], "server")
        self.assertEqual(s["server_url"], "http://127.0.0.1:8892")
        self.assertFalse(s["connected"])
        self.assertFalse(s["server_reachable"])
        self.assertIsNone(s["rig_state"])
        self.assertIsNone(s["armed"])
        self.assertIsNone(s["calibration_valid"])
        self.assertIsNone(s["effective_gain"])

    def test_status_without_ws_is_not_reachable(self):
        self.c._status = snapshot()
        self.c.connected = False
        self.assertFalse(self.b.state()["server_reachable"])
        self.assertIsNone(self.b.state()["rig_state"])

    def test_connected_requires_rig_not_offline(self):
        self.c.connected = True
        self.c._status = snapshot("OFFLINE")
        s = self.b.state()
        self.assertTrue(s["server_reachable"])
        self.assertFalse(s["connected"])
        self.assertEqual(s["rig_state"], "OFFLINE")
        self.c._status = snapshot("BRAKED_DISARMED")
        s = self.b.state()
        self.assertTrue(s["connected"])
        self.assertEqual(s["port"], "http://127.0.0.1:8892 BRAKED_DISARMED")

    def test_lost_session_is_not_connected(self):
        # Another client (e.g. a second bridge) took the session: poses from
        # this bridge are dropped, so the page must not show "connected".
        self.c.connected = True
        self.c._status = snapshot("BRAKED_DISARMED")
        self.c.session_lost = True
        s = self.b.state()
        self.assertTrue(s["server_reachable"])
        self.assertIs(s["session_lost"], True)
        self.assertFalse(s["connected"])
        self.assertEqual(s["port"], "http://127.0.0.1:8892 BRAKED_DISARMED SESSION_LOST")
        self.c.session_lost = False
        self.assertTrue(self.b.state()["connected"])

    def test_fields_copied_from_status(self):
        self.c.connected = True
        fault = {"kind": "axis_fault", "axis": 1, "code": "FOLLOWING_ERROR"}
        self.c._status = snapshot("FAULT_LATCHED", armed=False, calibration_valid=True, fault=fault,
                                  effective_gain=0.35)
        s = self.b.state()
        self.assertEqual(s["fault"], fault)
        self.assertEqual(s["effective_gain"], 0.35)
        self.assertIs(s["armed"], False)

    def test_session_fresh_only_for_own_session(self):
        self.c.connected = True
        self.c._status = snapshot(prefix="abcdef12", fresh=True)
        self.c.session_id = "abcdef12" + "0" * 24
        self.assertIs(self.b.state()["session_fresh"], True)
        self.c.session_id = "99999999" + "0" * 24
        self.assertIsNone(self.b.state()["session_fresh"])

    def test_reject_reason_and_stats(self):
        self.c.last_ack = {"type": "ack", "accepted": False, "reason": "missing_frame_seq"}
        self.b.submit(1, body(None), STATE)
        s = self.b.state()
        self.assertEqual(s["reject_reason"], "missing_frame_seq")
        self.assertEqual(s["pose_stats"][hsb.SENT], 1)
        self.assertIsNone(s["frame_seq"])
        self.b.submit(1, body(4), STATE)
        self.assertEqual(self.b.state()["frame_seq"], 4)

    # ------------------------------------------------------------ 系統の割当 (DESIGN.md §10)

    BLOCKERS = ["axis_conflict: axis1 claimed by /dev/a(5131000000000001), /dev/b(5131000000000002)",
                "axis_device_mismatch: axis2 device 5131000000000003, config expects 5131000000000009",
                "reference_stale: axis3 re-reference required after axis change"]

    def test_axis_conflict_and_hud_note_from_status(self):
        self.c.connected = True
        self.c._status = snapshot("OFFLINE", blockers=list(self.BLOCKERS), axis_conflict=[1])
        s = self.b.state()
        self.assertEqual(s["axis_conflict"], [1])
        self.assertEqual(s["hud_note"], "系統の競合(軸1) / 個体の不一致 / 再基準化が必要")
        self.assertFalse(s["connected"])  # conflicted axis has no primary → OFFLINE
        self.assertEqual(s["blockers"], self.BLOCKERS)  # raw codes still available to the page

    def test_hud_note_adds_reject_and_axes(self):
        self.c.connected = True
        self.c._status = snapshot("OFFLINE", blockers=[self.BLOCKERS[0]], axis_conflict=[1, 3])
        self.c.last_ack = {"type": "ack", "seq": 5, "accepted": False, "reason": "axis_conflict"}
        s = self.b.state()
        self.assertEqual(s["reject_reason"], "axis_conflict")
        self.assertEqual(s["hud_note"], "系統の競合(軸1,3、入力拒否)")
        # Blocker gone but the reject still says conflict (status lags the ack): still shown once.
        self.c._status = snapshot("READY_TO_ARM", blockers=[], axis_conflict=[])
        self.assertEqual(self.b.state()["hud_note"], "系統の競合(入力拒否)")
        # Accepted again: nothing to say.
        self.c.last_ack = {"type": "ack", "seq": 6, "accepted": True, "reason": None}
        self.assertIsNone(self.b.state()["hud_note"])
        self.assertEqual(self.b.state()["axis_conflict"], [])

    def test_hud_note_ignores_other_blockers_and_is_none_when_unreachable(self):
        self.c.connected = True
        self.c._status = snapshot("READY_TO_ARM", blockers=["calibration_invalid: axis2", "not_connected: axis3"],
                                  axis_conflict=[])
        s = self.b.state()
        self.assertIsNone(s["hud_note"])
        self.assertEqual(s["axis_conflict"], [])
        self.c.connected = False
        s = self.b.state()
        self.assertIsNone(s["hud_note"])
        self.assertIsNone(s["axis_conflict"])
        # Old server without §10 fields.
        self.c.connected = True
        self.c._status = snapshot("READY_TO_ARM")
        s = self.b.state()
        self.assertIsNone(s["hud_note"])
        self.assertIsNone(s["axis_conflict"])

    def test_hud_note_tolerates_malformed_input(self):
        self.assertIsNone(hsb.hud_note(None))
        self.assertIsNone(hsb.hud_note("axis_conflict: x"))  # not a list
        self.assertIsNone(hsb.hud_note([None, 3, "", ":", "unknown_code: y"]))
        self.assertEqual(hsb.hud_note(["axis_conflict"], axis_conflict="1"), "系統の競合")  # no colon, bad axes
        self.assertEqual(hsb.hud_note(["axis_conflict:axis1", "axis_conflict: again"], [True, 2, "x", 2]),
                         "系統の競合(軸2)")
        self.assertEqual(hsb.hud_note([], reject_reason="old_frame"), None)
        self.assertEqual(hsb.blocker_code("reference_stale: axis3 ..."), "reference_stale")
        self.assertIsNone(hsb.blocker_code(5))


class DefaultFactoryTests(unittest.TestCase):
    def test_default_factory_is_motion_client(self):
        try:
            import websockets  # noqa: F401
        except ImportError:
            self.skipTest("websockets not installed")
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), "python"))
        try:
            b = ServerBackend("http://127.0.0.1:9")
        finally:
            sys.path.pop(0)
        self.assertEqual(type(b.client).__name__, "MotionClient")
        self.assertFalse(b.client.connected)  # constructing does not connect
        # Real client: huge values come back as invalid, never as an exception.
        self.assertEqual(b.submit(1, body(2**64), STATE), hsb.INVALID)
        self.assertEqual(b.submit(1, body(1, age=10**400), STATE), hsb.INVALID)
        self.assertEqual(b.submit(1, body(1), STATE), hsb.SENT)
        b.close()


if __name__ == "__main__":
    unittest.main()
