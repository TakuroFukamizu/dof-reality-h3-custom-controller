"""Game-side client for h3-control-server (DESIGN.md §7, §8).

Usage from a game / bridge:

    client = MotionClient("http://127.0.0.1:8892")
    client.start()
    client.submit_pose(pitch, roll, yaw, gain, frame_seq=n, source_age_ms=age)
    ...
    client.close()

Contract (what this client does and never does):

- ``submit_pose`` / ``submit_gain`` only store the value in a single
  latest-value slot and return; they never touch the network.
- A background thread gets a session (POST /v1/session), opens
  WS /v1/motion and sends the slot only when something new was submitted.
  Nothing is resent to keep the server alive: if the game stops submitting,
  the server's freshness runs out and it stops the rig.
- ``source_age_ms`` sent = submitted age + time spent waiting in the slot.
- On any transport error it reconnects with backoff and a NEW session.
  Connecting never arms, starts or centers the rig (admin API, operator only).
  A new session while the rig is ACTIVE makes the server stop the rig
  (drive is bound to the session current at START); ARM + START again.
- ``frame_seq`` is passed through as given. The server's default config
  requires it (``require_frame_seq``); it is never invented here.
- No admin token handling: arm / start / disarm belong to the operator CLI.
- An ``axis_conflict`` reject (two CoreS3 units selected the same 系統,
  DESIGN.md §10) is an operator problem, not a transport problem: the client
  keeps sending (the server accepts again as soon as the selection is fixed),
  never reconnects because of it, and logs it once per episode. It is exposed
  as ``axis_conflict`` / ``last_reject``.

Dependencies: Python >= 3.9, stdlib + ``websockets>=13`` (sync client).

CLI for operators: ``python3 h3_motion_client.py status|stop [--url URL]``.
"""

from __future__ import annotations

import argparse
import inspect
import json
import logging
import math
import numbers
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from typing import Any, Callable, Dict, List, Optional

try:
    from websockets.exceptions import ConnectionClosed
    from websockets.sync.client import connect as _ws_connect
except ImportError:  # pragma: no cover - CLI status/stop work without it
    _ws_connect = None  # type: ignore[assignment]

    class ConnectionClosed(Exception):  # type: ignore[no-redef]
        pass


__all__ = ["MotionClient", "fetch_status", "request_stop", "DEFAULT_URL", "PROTOCOL_VERSION"]

DEFAULT_URL = "http://127.0.0.1:8892"
PROTOCOL_VERSION = 1

# Fallback if the session response lacks it (server default).
_DEFAULT_FRESHNESS_MS = 250.0
_BACKOFF_MIN_S = 0.2
_BACKOFF_MAX_S = 2.0
# A connection must stay up this long before the backoff resets, so a server
# that accepts and drops at once does not get a new session every 0.2 s.
_BACKOFF_RESET_AFTER_S = 1.0

# Bypass *_proxy env vars: the server is local.
_HTTP = urllib.request.build_opener(urllib.request.ProxyHandler({}))


# ---------------------------------------------------------------- HTTP helpers


def _url(base_url: str, path: str) -> str:
    return base_url.rstrip("/") + path


def _http_json(method: str, url: str, timeout: float) -> Any:
    """Send a request (POST with an empty body); return (status, parsed JSON or None)."""
    data = b"" if method == "POST" else None
    req = urllib.request.Request(url, data=data, method=method, headers={"Accept": "application/json"})
    try:
        with _HTTP.open(req, timeout=timeout) as resp:
            status, body = resp.status, resp.read()
    except urllib.error.HTTPError as e:
        with e:
            status, body = e.code, e.read()
    try:
        parsed = json.loads(body.decode("utf-8")) if body else None
    except ValueError:
        parsed = None
    return status, parsed


def fetch_status(base_url: str = DEFAULT_URL, timeout: float = 1.0) -> dict:
    """GET /v1/status. Raises on transport errors and non-200 answers."""
    status, body = _http_json("GET", _url(base_url, "/v1/status"), timeout)
    if status != 200 or not isinstance(body, dict):
        err = body.get("error") if isinstance(body, dict) else None
        raise RuntimeError("GET /v1/status: HTTP %s %s" % (status, err or ""))
    return body


def request_stop(base_url: str = DEFAULT_URL, timeout: float = 1.5) -> bool:
    """POST /v1/stop (no token). True only for 200 {"ok": true}; never raises.

    False includes 503 ``control_loop_unavailable``: the stop may still run
    when the loop catches up, but that is not confirmed.
    """
    try:
        status, body = _http_json("POST", _url(base_url, "/v1/stop"), timeout)
    except Exception:  # noqa: BLE001 - stop must never raise
        return False
    return status == 200 and isinstance(body, dict) and body.get("ok") is True


def _ws_url(base_url: str, session: str) -> str:
    u = urllib.parse.urlsplit(base_url)
    scheme = {"http": "ws", "https": "wss", "ws": "ws", "wss": "wss"}.get(u.scheme, "ws")
    path = u.path.rstrip("/") + "/v1/motion"
    query = urllib.parse.urlencode({"session": session})
    return urllib.parse.urlunsplit((scheme, u.netloc, path, query, ""))


def _connect_kwargs(timeout: float) -> Dict[str, Any]:
    kw: Dict[str, Any] = {"open_timeout": timeout, "close_timeout": 0.5, "compression": None}
    if _ws_connect is not None:
        try:
            # websockets >= 15 reads *_proxy env vars by default; the server is local.
            if "proxy" in inspect.signature(_ws_connect).parameters:
                kw["proxy"] = None
        except (TypeError, ValueError):  # pragma: no cover
            pass
    return kw


def _is_num(x: Any) -> bool:
    return isinstance(x, numbers.Real) and not isinstance(x, bool)


# Server parses frame_seq as u64; anything larger gets bad_message.
MAX_FRAME_SEQ = 2**64 - 1


def _finite(x: Any) -> bool:
    """Real, not bool, and finite as a float (huge ints count as invalid, not raise)."""
    if not _is_num(x):
        return False
    try:
        return math.isfinite(float(x))
    except (OverflowError, TypeError, ValueError):
        return False


# ---------------------------------------------------------------- client


class _Pending:
    __slots__ = ("pitch", "roll", "yaw", "gain", "frame_seq", "source_age_ms", "submitted_at")

    def __init__(self, pitch, roll, yaw, gain, frame_seq, source_age_ms, submitted_at):
        self.pitch = pitch
        self.roll = roll
        self.yaw = yaw
        self.gain = gain
        self.frame_seq = frame_seq
        self.source_age_ms = source_age_ms
        self.submitted_at = submitted_at

    def age_ms(self, now: float) -> float:
        return self.source_age_ms + max(0.0, now - self.submitted_at) * 1000.0


class MotionClient:
    """Latest-value pose sender for WS /v1/motion (see module docstring)."""

    #: status() returns None when the last status message is older than this.
    STATUS_MAX_AGE_S = 2.0
    #: No message for this long → reconnect. Status arrives every
    #: server.status_period_ms (default 100), so this only catches a wedged
    #: server. Kept generous: a reconnect while ACTIVE stops the rig.
    RECV_STALL_S = 10.0

    def __init__(
        self,
        base_url: str = DEFAULT_URL,
        *,
        on_status: Optional[Callable[[dict], None]] = None,
        connect_timeout: float = 1.0,
        logger: Optional[logging.Logger] = None,
    ) -> None:
        if _ws_connect is None:
            raise RuntimeError("MotionClient needs the 'websockets' package (>= 13)")
        self.base_url = base_url.rstrip("/")
        self.on_status = on_status
        self.connect_timeout = float(connect_timeout)
        self.log = logger or logging.getLogger("h3_motion_client")

        self._cond = threading.Condition()
        # Everything below is guarded by self._cond. Never do I/O while holding it.
        self._pose: Optional[_Pending] = None
        self._gain: Optional[float] = None  # pending gain-only update
        self._limits: Optional[Dict[str, float]] = None  # pending limits patch (merged until sent)
        self._limits_seqs: set = set()  # seqs of limits messages awaiting their ack
        self._last_limits_ack: Optional[dict] = None
        self._closing = False
        self._reconnect_req = False
        self._conn_failed = False  # set by the reader when its connection ends
        self._ws = None
        self._conn_id = 0
        self._connected = False
        self._session: Optional[str] = None
        self._session_info: Optional[dict] = None
        self._session_lost = False
        self._seq = 0
        self._last_ack: Optional[dict] = None
        self._last_reject: Optional[dict] = None  # last ack with accepted=False
        self._axis_conflict = False  # last ack was rejected with axis_conflict
        self._status: Optional[dict] = None
        self._status_at = 0.0
        self._warned_no_frame_seq = False
        self._fail_logged = False  # worker thread only

        self._thread: Optional[threading.Thread] = None

    # ------------------------------------------------------------ public API

    def start(self) -> None:
        """Start the background thread (idempotent). Returns immediately."""
        with self._cond:
            if self._thread is not None:
                return
            self._closing = False
            t = threading.Thread(target=self._run, name="h3-motion-client", daemon=True)
            self._thread = t
        t.start()

    def close(self, timeout: float = 3.0) -> None:
        """Stop the thread and close the connection. Does NOT stop the rig:
        without new poses the server's freshness expires and it stops itself.
        Call stop() first for an immediate stop."""
        with self._cond:
            self._closing = True
            t = self._thread
            ws = self._ws
            self._cond.notify_all()
        if t is None:
            return
        t.join(timeout)
        if t.is_alive() and ws is not None:
            # Worker stuck in a blocking send: break the socket.
            try:
                ws.socket.shutdown(2)
            except Exception:  # noqa: BLE001
                pass
            t.join(1.0)
        with self._cond:
            self._thread = None

    def submit_pose(
        self,
        pitch: float,
        roll: float,
        yaw: float,
        gain: float,
        frame_seq: Optional[int] = None,
        source_age_ms: float = 0.0,
    ) -> bool:
        """Store the latest pose; never blocks on the network.

        ``frame_seq`` must advance on every completed game update (same
        frame_seq resent never extends freshness). ``source_age_ms`` is how
        old the pose already is (ms since the game produced it).
        Returns False (and stores nothing) for non-finite / invalid input,
        including frame_seq outside 0..MAX_FRAME_SEQ (the server's u64).
        """
        now = time.monotonic()
        vals = (pitch, roll, yaw, gain, source_age_ms)
        if not all(_finite(v) for v in vals) or source_age_ms < 0:
            self.log.warning("submit_pose: invalid value dropped %r", vals)
            return False
        if frame_seq is not None and (not isinstance(frame_seq, numbers.Integral) or isinstance(frame_seq, bool)
                                      or not 0 <= frame_seq <= MAX_FRAME_SEQ):
            self.log.warning("submit_pose: invalid frame_seq dropped %r", frame_seq)
            return False
        p = _Pending(float(pitch), float(roll), float(yaw), float(gain),
                     None if frame_seq is None else int(frame_seq), float(source_age_ms), now)
        with self._cond:
            self._pose = p
            self._gain = None  # the pose carries the newer gain
            self._cond.notify_all()
        return True

    def submit_limits(
        self,
        *,
        max_gain: Optional[float] = None,
        max_vel_count_s: Optional[float] = None,
        max_acc_count_s2: Optional[float] = None,
        duty_max_percent: Optional[float] = None,
    ) -> bool:
        """Change the server's shared motion limits (``{"type":"limits"}``).

        Only the given fields change. Values above the ceilings (firmware
        absolutes, ``overspeed / 1.5``, gain 1.0) are clamped by the server
        and reported in the ack (``effective_limits``, ``limits`` has
        ``ceiling_clamped``). Never extends freshness. Several calls before
        the next send are merged (last value per field wins). Returns False
        (and stores nothing) for a non-finite value or when no field is given.
        The server resets the limits to its config on every new session, so
        a client that wants its values must resend them after ``reconnect()``
        (watch ``session_id``).
        """
        fields = {
            "max_gain": max_gain,
            "max_vel_count_s": max_vel_count_s,
            "max_acc_count_s2": max_acc_count_s2,
            "duty_max_percent": duty_max_percent,
        }
        patch = {k: v for k, v in fields.items() if v is not None}
        if not patch:
            return False
        if not all(_finite(v) for v in patch.values()):
            self.log.warning("submit_limits: invalid value dropped %r", patch)
            return False
        with self._cond:
            merged = dict(self._limits or {})
            merged.update({k: float(v) for k, v in patch.items()})
            self._limits = merged
            self._cond.notify_all()
        return True

    @property
    def last_limits_ack(self) -> Optional[dict]:
        """Last ack of a ``limits`` message (accepted or rejected); None before any."""
        with self._cond:
            return self._last_limits_ack

    def submit_gain(self, gain: float) -> bool:
        """Gain-only update (never extends pose freshness on the server)."""
        if not _finite(gain):
            self.log.warning("submit_gain: invalid gain dropped %r", gain)
            return False
        with self._cond:
            if self._pose is not None:
                self._pose.gain = float(gain)  # unsent pose: just carry the new gain
            else:
                self._gain = float(gain)
            self._cond.notify_all()
        return True

    def status(self) -> Optional[dict]:
        """Latest status snapshot from the WS (GET /v1/status shape), or None
        if none yet or older than STATUS_MAX_AGE_S. Treat it as read-only."""
        with self._cond:
            if self._status is None or time.monotonic() - self._status_at > self.STATUS_MAX_AGE_S:
                return None
            return self._status

    @property
    def connected(self) -> bool:
        """Session issued and WS open (transport only; says nothing about ARM)."""
        with self._cond:
            return self._connected

    @property
    def session_lost(self) -> bool:
        """True after a ``stale_session`` ack: another client took the session.
        Poses are not sent any more; call reconnect() to take it back."""
        with self._cond:
            return self._session_lost

    @property
    def session_id(self) -> Optional[str]:
        """Full id of our current session (status shows only its first 8 chars)."""
        with self._cond:
            return self._session

    @property
    def session_info(self) -> Optional[dict]:
        """Last POST /v1/session response (max_gain, source_freshness_ms, ...)."""
        with self._cond:
            return None if self._session_info is None else dict(self._session_info)

    @property
    def last_ack(self) -> Optional[dict]:
        with self._cond:
            return self._last_ack

    @property
    def last_reject(self) -> Optional[dict]:
        """Last ack with ``accepted: false`` (its ``reason`` is the code, e.g.
        ``axis_conflict``), or None if nothing was rejected yet. Not cleared by
        a later accepted ack: compare with ``last_ack`` to see which is newer."""
        with self._cond:
            return self._last_reject

    @property
    def axis_conflict(self) -> bool:
        """True while the server rejects our input with ``axis_conflict``
        (two units selected the same 系統; DESIGN.md §10). Cleared by the next
        ack that is accepted or rejected for another reason. Poses are still
        sent meanwhile; the operator fixes the selection on the CoreS3."""
        with self._cond:
            return self._axis_conflict

    def reconnect(self) -> None:
        """Drop the connection and reconnect now with a new session.
        Transport only: never arms or starts. If the rig is ACTIVE, the new
        session makes the server stop it (operator must ARM + START again)."""
        with self._cond:
            self._reconnect_req = True
            self._cond.notify_all()

    def stop(self, timeout: float = 1.5) -> bool:
        """POST /v1/stop. True only if the server confirmed; never raises."""
        return request_stop(self.base_url, timeout)

    # ------------------------------------------------------------ worker

    def _run(self) -> None:
        backoff = _BACKOFF_MIN_S
        while True:
            with self._cond:
                if self._closing:
                    return
                self._reconnect_req = False
            up_since = self._connect_and_serve()
            with self._cond:
                if self._closing:
                    return
                explicit = self._reconnect_req
            if explicit or (up_since is not None and time.monotonic() - up_since >= _BACKOFF_RESET_AFTER_S):
                backoff = _BACKOFF_MIN_S
            if explicit:
                continue
            with self._cond:
                # Interruptible by close() / reconnect().
                self._cond.wait_for(lambda: self._closing or self._reconnect_req, timeout=backoff)
            backoff = min(backoff * 2.0, _BACKOFF_MAX_S)

    def _motion_url(self, session: str) -> str:
        # Separate method so tests can point the WS at another port.
        return _ws_url(self.base_url, session)

    def _connect_and_serve(self) -> Optional[float]:
        """One connection. Returns the time it came up, or None if it never did."""
        up_since: Optional[float] = None
        try:
            status, info = _http_json("POST", _url(self.base_url, "/v1/session"), self.connect_timeout)
            if status != 200 or not isinstance(info, dict) or not isinstance(info.get("source_session"), str):
                raise RuntimeError("POST /v1/session: HTTP %s %r" % (status, info))
            if info.get("protocol", PROTOCOL_VERSION) != PROTOCOL_VERSION:
                raise RuntimeError("server protocol %r != %d" % (info.get("protocol"), PROTOCOL_VERSION))
            session = info["source_session"]
            # Context-manager form works on websockets 13..17+ (17.1 warns otherwise).
            with _ws_connect(self._motion_url(session), **_connect_kwargs(self.connect_timeout)) as ws:
                up_since = time.monotonic()
                self._fail_logged = False
                self._serve(ws, session, info)
        except Exception as e:  # noqa: BLE001
            if up_since is None:
                # Warn once per outage, then stay quiet while retrying.
                (self.log.debug if self._fail_logged else self.log.warning)("connect failed: %s", e)
                self._fail_logged = True
            else:
                self.log.warning("connection error: %s", e)
        return up_since

    def _serve(self, ws, session: str, info: dict) -> None:
        freshness_ms = info.get("source_freshness_ms")
        if not _is_num(freshness_ms):
            freshness_ms = _DEFAULT_FRESHNESS_MS
        dropped_age = None
        with self._cond:
            self._conn_id += 1
            conn_id = self._conn_id
            self._ws = ws
            self._session = session
            self._session_info = info
            self._session_lost = False
            self._conn_failed = False
            self._connected = True
            self._axis_conflict = False  # a still-present conflict is logged again on this connection
            # A pose that waited through the outage and is already stale is
            # useless to the new session: drop it instead of sending.
            if self._pose is not None:
                age = self._pose.age_ms(time.monotonic())
                if age >= freshness_ms:
                    self._pose = None
                    dropped_age = age
            closing = self._closing
        self.log.info("connected, session %s...", session[:8])
        if dropped_age is not None:
            self.log.info("dropped pose stale at reconnect (%.0f ms old)", dropped_age)

        reader = threading.Thread(target=self._read_loop, args=(ws, conn_id),
                                  name="h3-motion-client-rx", daemon=True)
        reader.start()
        try:
            if not closing:
                self._send_loop(ws, session, info)
        except Exception as e:  # noqa: BLE001
            self.log.warning("connection error: %s", e)
        finally:
            with self._cond:
                self._connected = False
                self._ws = None
            try:
                ws.close()
            except Exception:  # noqa: BLE001
                pass
            reader.join(2.0)
            self.log.info("disconnected")

    def _send_loop(self, ws, session: str, info: dict) -> None:
        require_frame_seq = info.get("require_frame_seq") is not False
        while True:
            with self._cond:
                self._cond.wait_for(lambda: (self._closing or self._reconnect_req or self._conn_failed
                                             or self._pose is not None or self._gain is not None
                                             or self._limits is not None))
                if self._closing or self._reconnect_req or self._conn_failed:
                    return
                if self._session_lost:
                    # Not our session any more: drop instead of sending into rejection.
                    self._pose = None
                    self._gain = None
                    self._limits = None
                    continue
                pose, gain, limits = self._pose, self._gain, self._limits
                self._pose = None
                self._gain = None
                self._limits = None
                msgs: List[str] = []
                now = time.monotonic()
                if limits is not None:
                    # Before the pose: the new gain ceiling applies to it.
                    self._seq += 1
                    m_l: Dict[str, Any] = {"type": "limits", "protocol": PROTOCOL_VERSION,
                                           "source_session": session, "seq": self._seq}
                    m_l.update(limits)
                    self._limits_seqs.add(self._seq)
                    if len(self._limits_seqs) > 64:
                        self._limits_seqs = set(sorted(self._limits_seqs)[-64:])
                    msgs.append(json.dumps(m_l, allow_nan=False))
                if pose is not None:
                    if pose.frame_seq is None and require_frame_seq and not self._warned_no_frame_seq:
                        self._warned_no_frame_seq = True
                        self.log.warning("server requires frame_seq; poses without it are rejected")
                    self._seq += 1
                    m: Dict[str, Any] = {
                        "type": "pose", "protocol": PROTOCOL_VERSION, "source_session": session,
                        "seq": self._seq, "source_age_ms": pose.age_ms(now),
                        "pitch": pose.pitch, "roll": pose.roll, "yaw": pose.yaw, "gain": pose.gain,
                    }
                    if pose.frame_seq is not None:
                        m["frame_seq"] = pose.frame_seq
                    msgs.append(json.dumps(m, allow_nan=False))
                if gain is not None:
                    self._seq += 1
                    msgs.append(json.dumps({"type": "gain", "protocol": PROTOCOL_VERSION,
                                            "source_session": session, "seq": self._seq, "gain": gain},
                                           allow_nan=False))
            for text in msgs:  # outside the lock
                ws.send(text)

    def _read_loop(self, ws, conn_id: int) -> None:
        try:
            while True:
                try:
                    raw = ws.recv(timeout=self.RECV_STALL_S)
                except TimeoutError:
                    self.log.warning("no message for %.1f s; reconnecting", self.RECV_STALL_S)
                    return
                self._handle_message(raw)
        except ConnectionClosed:
            pass
        except Exception as e:  # noqa: BLE001
            self.log.debug("reader ended: %s", e)
        finally:
            with self._cond:
                if self._conn_id == conn_id:
                    self._conn_failed = True
                    self._cond.notify_all()

    def _limits_seq_pending(self, ack: dict) -> bool:
        """True if this (rejected) ack answers a limits message we sent (guarded by _cond)."""
        seq = ack.get("seq")
        if seq in self._limits_seqs:
            self._limits_seqs.discard(seq)
            return True
        return False

    def _handle_message(self, raw: Any) -> None:
        try:
            msg = json.loads(raw)
        except (TypeError, ValueError):
            self.log.debug("ignoring non-JSON message")
            return
        if not isinstance(msg, dict):
            return
        kind = msg.get("type")
        if kind == "ack":
            reason = msg.get("reason")
            rejected = msg.get("accepted") is False
            with self._cond:
                self._last_ack = msg
                if "effective_limits" in msg:
                    self._limits_seqs.discard(msg.get("seq"))
                    self._last_limits_ack = msg
                elif rejected and self._limits_seq_pending(msg):
                    self._last_limits_ack = msg
                if rejected:
                    self._last_reject = msg
                lost = rejected and reason == "stale_session" and not self._session_lost
                if lost:
                    self._session_lost = True
                # Edge-triggered: one warning per conflict episode, not per rejected pose.
                conflict = rejected and reason == "axis_conflict"
                conflict_began = conflict and not self._axis_conflict
                conflict_ended = self._axis_conflict and not conflict
                self._axis_conflict = conflict
            if lost:
                self.log.warning("session taken over by another client; not sending poses (reconnect() to retake)")
            elif conflict_began:
                self.log.warning("axis_conflict: two CoreS3 units selected the same axis; server rejects our input "
                                 "until the operator fixes the selection (still sending, not reconnecting)")
            elif conflict_ended:
                self.log.info("axis_conflict cleared")
            elif rejected:
                self.log.debug("pose rejected: %s", reason)
        elif kind == "status":
            msg.pop("type", None)
            with self._cond:
                self._status = msg
                self._status_at = time.monotonic()
            cb = self.on_status
            if cb is not None:
                try:
                    cb(msg)
                except Exception:  # noqa: BLE001
                    self.log.exception("on_status callback failed")


# ---------------------------------------------------------------- CLI


def _claimants(a: dict) -> str:
    """``port(device_id), port(device_id)`` from axes[].claimants (DESIGN.md §10.2); ``-`` if none."""
    items = a.get("claimants")
    if not isinstance(items, list) or not items:
        return "-"
    out = []
    for c in items:
        if isinstance(c, dict):
            out.append("%s(%s)" % (c.get("port"), c.get("device_id")))
        else:
            out.append(str(c))
    return ", ".join(out)


def _summary(st: dict) -> str:
    sess = st.get("session") or {}
    conflict = st.get("axis_conflict")
    if not isinstance(conflict, list):
        conflict = []
    lines = [
        "rig_state:         %s" % st.get("rig_state"),
        "armed:             %s" % st.get("armed"),
        "active_source:     %s" % st.get("active_source"),
        "calibration_valid: %s" % st.get("calibration_valid"),
        "fault:             %s" % json.dumps(st.get("fault"), ensure_ascii=False),
        "blockers:          %s" % json.dumps(st.get("blockers"), ensure_ascii=False),
        # Axes claimed by two or more CoreS3 units (same 系統 selected on both):
        # the server refuses ARM / START / poses until the operator fixes it.
        "axis_conflict:     %s" % (json.dumps(conflict) if conflict else "none"),
        "session:           %s fresh=%s age_ms=%s last_seq=%s last_frame_seq=%s" % (
            sess.get("source_session_prefix"), sess.get("fresh"), sess.get("source_age_ms"),
            sess.get("last_seq"), sess.get("last_frame_seq")),
        "effective_gain:    %s limits=%s" % (st.get("effective_gain"), st.get("limits")),
    ]
    for a in st.get("axes") or []:
        if not isinstance(a, dict):
            continue
        lines.append("axis %s: connected=%s mcu_state=%s fault_code=%s actual_count=%s target_count=%s" % (
            a.get("axis_id"), a.get("connected"), a.get("mcu_state"), a.get("fault_code"),
            a.get("actual_count"), a.get("target_count")))
        # Assignment (DESIGN.md §10): which unit drives this axis and why.
        # assignment: "selected" (MCU's 系統 selection) / "config" (device_id only) / None.
        lines.append("        device_id=%s expected=%s assignment=%s%s claimants=%s" % (
            a.get("device_id"), a.get("expected_device_id"), a.get("assignment"),
            " CONFLICT" if a.get("axis_id") in conflict else "", _claimants(a)))
        if a.get("reference_stale"):
            lines.append("        reference_stale: re-reference required (axis changed on the same boot)")
        if a.get("calibration_stale"):
            lines.append("        calibration_stale: recalibrated; export the result and restart the server with it before ARM")
    return "\n".join(lines)


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="h3-control-server status / stop (no admin token)")
    ap.add_argument("command", choices=["status", "stop"])
    ap.add_argument("--url", default=DEFAULT_URL, help="server base URL (default %(default)s)")
    ap.add_argument("--json", action="store_true", help="status: print the full JSON")
    ap.add_argument("--timeout", type=float, default=1.5)
    args = ap.parse_args(argv)
    if args.command == "stop":
        ok = request_stop(args.url, args.timeout)
        print("stop: confirmed" if ok else "stop: NOT confirmed (server down, 503 or refused); check status")
        return 0 if ok else 1
    try:
        st = fetch_status(args.url, args.timeout)
    except Exception as e:  # noqa: BLE001
        print("status: %s" % e, file=sys.stderr)
        return 1
    print(json.dumps(st, indent=2, ensure_ascii=False) if args.json else _summary(st))
    return 0


if __name__ == "__main__":
    sys.exit(main())
