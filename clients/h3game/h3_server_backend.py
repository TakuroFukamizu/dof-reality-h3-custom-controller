"""h3-control-server backend for h3game bridge.py (H3_BACKEND=server).

Canonical source: clients/h3game/h3_server_backend.py in
dof-reality-h3-custom-controller (copied into h3game/ by the patch).

bridge.py keeps its HTTP API, owner check and /state; in server mode it hands
each accepted POST to ``ServerBackend.submit`` and merges ``state()`` into
/state. Rules (DESIGN.md §0, §8; architecture.md §4):

- Never opens serial, never imports h3_direct. Never arms, starts or centers:
  connecting only gets a session and reads status (operator uses the admin API).
- ``submit`` only fills the MotionClient's latest-value slot: it never waits
  on the network, so the bridge's HTTP handler never blocks on the server.
- Freshness comes from the page's ``frame_seq``. A POST that repeats the last
  frame_seq (frozen game loop, hidden tab sending zeros) is not forwarded as a
  pose; only a changed gain goes out as a gain-only message. Nothing is resent
  to keep the server alive, so it stops the rig when the game stops.
- A POST from a new owner (page reload / another tab) restarts frame_seq, so
  the backend reconnects with a new session. If the rig was ACTIVE the server
  stops it; the operator must ARM + START again (intended).
- A page without frame_seq is forwarded as is; the server rejects it
  (``missing_frame_seq``) unless ``allow_missing_frame_seq`` is set. The
  reason shows in /state as ``reject_reason``.
- /state ``connected`` is False once another client took the session
  (``session_lost``): this bridge's poses are dropped until /reconnect, so the
  page must not say the seat is connected.
- 系統 (axis) assignment problems (DESIGN.md §10) are shown to the player in
  Japanese through ``hud_note`` (built from the status ``blockers`` codes and
  the WS reject reason): 系統の競合 (``axis_conflict``), 個体の不一致
  (``axis_device_mismatch``), 再基準化が必要 (``reference_stale``). The bridge
  does nothing about them (operator fixes the selection on the CoreS3); the
  server already refuses ARM / START / poses meanwhile.

Python >= 3.9, stdlib + websockets>=13 (through h3_motion_client).
"""

from __future__ import annotations

import logging
import math
import numbers
import threading
from typing import Any, Callable, Dict, List, Optional

DEFAULT_URL = "http://127.0.0.1:8892"
AXES = ("pitch", "roll", "yaw")

# Blocker codes (the text before ':' in StatusSnapshot.blockers[], DESIGN.md
# §7.2 / §10.2) and WS ack reject reasons that the player should see in
# Japanese. Other codes stay operator-facing (status / check-config).
HUD_TEXT = {
    "axis_conflict": "系統の競合",          # 2 台の CoreS3 が同じ系統を選択している
    "axis_device_mismatch": "個体の不一致",  # 設定の device_id と選択された個体が違う
    "reference_stale": "再基準化が必要",     # 同じ boot のまま系統が変わった個体
}
REJECT_SUFFIX = "入力拒否"  # the WS ack said axis_conflict: our poses are being refused right now


def blocker_code(b: Any) -> Optional[str]:
    """``"<code>: <detail>"`` → ``"<code>"``; None for anything that is not a string."""
    if not isinstance(b, str):
        return None
    return b.split(":", 1)[0].strip() or None


def hud_note(blockers: Any, axis_conflict: Any = None, reject_reason: Any = None) -> Optional[str]:
    """Japanese one-liner for the game HUD, or None when there is nothing to say.

    ``blockers`` / ``axis_conflict`` are the StatusSnapshot fields (any shape
    tolerated), ``reject_reason`` the reason of the last rejected WS ack. Each
    phrase appears once, e.g. ``系統の競合(軸1、入力拒否) / 再基準化が必要``.
    """
    parts: List[str] = []
    seen = set()
    codes = [blocker_code(b) for b in blockers] if isinstance(blockers, list) else []
    rejected = reject_reason == "axis_conflict"
    if rejected and "axis_conflict" not in codes:
        codes.append("axis_conflict")  # the reject can arrive before (or without) the blocker
    axes = [a for a in axis_conflict if isinstance(a, int) and not isinstance(a, bool)] \
        if isinstance(axis_conflict, list) else []
    for code in codes:
        if code is None or code not in HUD_TEXT or code in seen:
            continue
        seen.add(code)
        text = HUD_TEXT[code]
        if code == "axis_conflict":
            detail = []
            if axes:
                detail.append("軸%s" % ",".join(str(a) for a in sorted(set(axes))))
            if rejected:
                detail.append(REJECT_SUFFIX)
            if detail:
                text += "(%s)" % "、".join(detail)
        parts.append(text)
    return " / ".join(parts) if parts else None

# Return values of submit() (also the keys of ServerBackend.stats).
SENT = "sent"            # pose put in the client slot
GAIN_ONLY = "gain_only"  # same frame_seq, gain changed → gain-only message
REPEAT = "repeat"        # same frame_seq, nothing new → not forwarded
OLD_FRAME = "old_frame"  # frame_seq went backwards for the same owner → dropped
INVALID = "invalid"      # non-numeric / non-finite / bad frame_seq or age → dropped
NOTHING = "nothing"      # no pose keys, no gain

# The server parses frame_seq as u64: a larger value would be stored here,
# rejected there (bad_message) and then block every later frame as old_frame.
MAX_FRAME_SEQ = 2**64 - 1


def _num(v: Any) -> Optional[float]:
    """float(v) like bridge.py does, but None for anything non-finite or unparsable."""
    try:
        f = float(v)
    except (TypeError, ValueError, OverflowError):  # OverflowError: JSON int beyond float
        return None
    return f if math.isfinite(f) else None


def _frame_seq(v: Any) -> Optional[int]:
    if isinstance(v, bool) or not isinstance(v, numbers.Integral) or not 0 <= v <= MAX_FRAME_SEQ:
        raise ValueError(v)
    return int(v)


class ServerBackend:
    """Bridge-side adapter around MotionClient (see module docstring)."""

    def __init__(
        self,
        url: str = DEFAULT_URL,
        client_factory: Optional[Callable[[str], Any]] = None,
        logger: Optional[logging.Logger] = None,
    ) -> None:
        if client_factory is None:
            from h3_motion_client import MotionClient  # needs websockets

            client_factory = MotionClient
        self.url = url.rstrip("/")
        self.log = logger or logging.getLogger("h3_server_backend")
        self.client = client_factory(self.url)
        self._lock = threading.Lock()
        self._owner: Optional[int] = None
        self._frame_seq: Optional[int] = None
        self._pose = {k: 0.0 for k in AXES}
        self._gain: Optional[float] = None
        self.stats: Dict[str, int] = {k: 0 for k in (SENT, GAIN_ONLY, REPEAT, OLD_FRAME, INVALID)}
        self.stats["new_page"] = 0

    # ------------------------------------------------------------ lifecycle

    def start(self) -> None:
        """Start the client thread; returns at once (no ARM / START / center)."""
        self.client.start()

    def reconnect(self) -> None:
        """Re-establish the transport with a new session. Never re-arms."""
        self.client.reconnect()

    def close(self) -> None:
        """Close the connection. Does not stop the rig: without new poses the
        server's freshness expires and it stops by itself."""
        self.client.close()

    # ------------------------------------------------------------ input

    def submit(self, owner: int, d: Dict[str, Any], fallback: Optional[Dict[str, Any]] = None) -> str:
        """Hand one accepted POST body ``d`` to the client; never blocks.

        ``owner`` is the owner number that passed the bridge's check.
        ``fallback`` supplies values for keys missing from ``d`` (bridge state).
        """
        fallback = fallback or {}
        with self._lock:
            if self._owner is not None and owner != self._owner:
                # New page: its frame_seq starts over, the old session's would reject it.
                self._frame_seq = None
                self.stats["new_page"] += 1
                self.log.info("new page (owner %s -> %s): reconnecting with a new session", self._owner, owner)
                self.client.reconnect()
            self._owner = owner
            return self._submit_locked(d, fallback)

    def _submit_locked(self, d: Dict[str, Any], fallback: Dict[str, Any]) -> str:
        has_pose = any(k in d for k in AXES)
        raw_gain = d["gain"] if "gain" in d else (self._gain if self._gain is not None else fallback.get("gain"))
        gain = _num(raw_gain)
        if not has_pose and "frame_seq" not in d:
            if "gain" not in d:
                return NOTHING
            return self._gain_only(gain)

        pose = {}
        for k in AXES:
            pose[k] = _num(d[k]) if k in d else self._pose[k]
        try:
            fs = None if d.get("frame_seq") is None else _frame_seq(d.get("frame_seq"))
        except ValueError:
            return self._count(INVALID)
        age = 0.0 if d.get("frame_age_ms") is None else _num(d.get("frame_age_ms"))
        if gain is None or age is None or age < 0 or any(v is None for v in pose.values()):
            return self._count(INVALID)

        if fs is not None and self._frame_seq is not None:
            if fs < self._frame_seq:
                return self._count(OLD_FRAME)
            if fs == self._frame_seq:
                # No new game frame: never a pose (would look alive); gain may still change.
                if gain != self._gain:
                    return self._gain_only(gain)
                return self._count(REPEAT)

        if not self.client.submit_pose(pose["pitch"], pose["roll"], pose["yaw"], gain,
                                       frame_seq=fs, source_age_ms=age):
            return self._count(INVALID)
        self._pose = pose
        self._gain = gain
        if fs is not None:
            self._frame_seq = fs
        return self._count(SENT)

    def _gain_only(self, gain: Optional[float]) -> str:
        if gain is None or not self.client.submit_gain(gain):
            return self._count(INVALID)
        self._gain = gain
        return self._count(GAIN_ONLY)

    def _count(self, result: str) -> str:
        self.stats[result] += 1
        return result

    # ------------------------------------------------------------ status

    def state(self) -> Dict[str, Any]:
        """Keys merged into bridge /state. Unknown values are None (not 0/False)."""
        c = self.client
        st = c.status()  # None when no status for 2 s
        reachable = bool(c.connected) and st is not None
        st = st if reachable else None
        rig = st.get("rig_state") if st else None
        sess = (st or {}).get("session") or {}
        sid = c.session_id
        mine = bool(sid) and sess.get("source_session_prefix") == sid[:8]
        ack = c.last_ack or {}
        with self._lock:
            frame_seq = self._frame_seq
            stats = dict(self.stats)
        lost = bool(c.session_lost)
        reject = ack.get("reason") if ack.get("accepted") is False else None
        conflict = st.get("axis_conflict") if st else None
        return {
            "backend": "server",
            "server_url": self.url,
            # Existing keys, server meaning: reachable, every axis identified and
            # this bridge still owns the session (otherwise its poses go nowhere).
            "connected": reachable and rig not in (None, "OFFLINE") and not lost,
            "port": "%s %s%s" % (self.url, rig, " SESSION_LOST" if lost else "") if reachable else self.url,
            "server_reachable": reachable,
            "rig_state": rig,
            "armed": st.get("armed") if st else None,
            "calibration_valid": st.get("calibration_valid") if st else None,
            "fault": st.get("fault") if st else None,
            "blockers": st.get("blockers") if st else None,
            "effective_gain": st.get("effective_gain") if st else None,
            "session_fresh": sess.get("fresh") if mine else None,
            "session_lost": lost,
            "reject_reason": ack.get("reason"),
            # Axes claimed by 2+ CoreS3 units (same 系統 selected), DESIGN.md §10.
            "axis_conflict": conflict if isinstance(conflict, list) else None,
            # Japanese text for the page (系統の競合 / 個体の不一致 / 再基準化が必要);
            # None when the server is unreachable or there is nothing to show.
            "hud_note": hud_note(st.get("blockers"), conflict, reject) if st else None,
            "frame_seq": frame_seq,
            "pose_stats": stats,
        }
