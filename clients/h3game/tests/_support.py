"""Shared helpers for the h3game adapter tests (stdlib only)."""

from __future__ import annotations

import glob
import json
import os
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request
from typing import Any, Dict, List, Optional, Tuple

HERE = os.path.dirname(os.path.abspath(__file__))
H3GAME_DIR = os.path.dirname(HERE)                      # clients/h3game
REPO = os.path.abspath(os.path.join(H3GAME_DIR, "..", ".."))
PATCH_DIR = os.path.join(H3GAME_DIR, "patches")
CANONICAL_CLIENT = os.path.join(REPO, "clients", "python", "h3_motion_client.py")
CANONICAL_BACKEND = os.path.join(H3GAME_DIR, "h3_server_backend.py")

SERVER_DIR = os.path.join(REPO, "control-server")
SERVER_BINARY = os.environ.get("H3CS_BINARY") or os.path.join(SERVER_DIR, "target", "debug", "h3-control-server")
SIM_CONFIG = os.path.join(SERVER_DIR, "config", "sim.toml")
ADMIN_TOKEN = "test-admin-token"

# h3game source (TakuroFukamizu/20261004_shi3z_hackathon). H3GAME_SRC points at
# an existing clone; otherwise a cache clone is made on first use under
# control-server/target/ (gitignored; `cargo clean` removes it) and checked out
# at H3GAME_PIN, the commit whose blobs the patch was made against.
H3GAME_URL = "https://github.com/TakuroFukamizu/20261004_shi3z_hackathon.git"
H3GAME_PIN = "649ab66"
H3GAME_CACHE = os.path.join(SERVER_DIR, "target", "h3game-src")
H3GAME_SRC = os.environ.get("H3GAME_SRC") or H3GAME_CACHE
CLONE_TIMEOUT_S = 120

# Blob SHAs the patch was written against (architecture.md §1).
EXPECTED_BLOBS = {
    "h3game/bridge.py": "807b4d9c45e6ef04550a64e34ceaf89a01599c18",
    "h3game/web/index.html": "55827600269b6b840b43f864718d19d7ac4b68ca",
    "h3game/h3_direct.py": "06bc7f0bdda2c5ebff5f9d323074b582629582a0",
}

# Local servers only: never go through *_proxy.
HTTP = urllib.request.build_opener(urllib.request.ProxyHandler({}))


def patches() -> List[str]:
    return sorted(glob.glob(os.path.join(PATCH_DIR, "*.patch")))


_src_reason: Optional[List[Optional[str]]] = None  # cached [reason]


def src_skip_reason() -> Optional[str]:
    """None when H3GAME_SRC is usable, else why not (used as the skip message).
    Without H3GAME_SRC this clones H3GAME_URL into H3GAME_CACHE once (network)."""
    global _src_reason
    if _src_reason is None:
        _src_reason = [_resolve_src()]
    return _src_reason[0]


def src_available() -> bool:
    return src_skip_reason() is None


def _is_clone(path: str) -> bool:
    return os.path.isdir(os.path.join(path, ".git")) and os.path.isfile(os.path.join(path, "h3game", "bridge.py"))


def _git_quiet(cwd: str, *args: str, timeout: float = 30) -> subprocess.CompletedProcess:
    env = dict(os.environ, GIT_TERMINAL_PROMPT="0")
    return subprocess.run(["git"] + list(args), cwd=cwd, env=env, capture_output=True, text=True, timeout=timeout)


def _resolve_src() -> Optional[str]:
    if os.environ.get("H3GAME_SRC"):
        if _is_clone(H3GAME_SRC):
            return None
        return "H3GAME_SRC is not a clone of h3game: %s" % H3GAME_SRC
    if shutil.which("git") is None:
        return "git not found (needed for the h3game clone)"
    try:
        if not _is_clone(H3GAME_CACHE):
            parent = os.path.dirname(H3GAME_CACHE)
            os.makedirs(parent, exist_ok=True)
            tmp = tempfile.mkdtemp(prefix="h3game-src-", dir=parent)
            try:
                r = _git_quiet(parent, "clone", "-q", H3GAME_URL, os.path.join(tmp, "repo"), timeout=CLONE_TIMEOUT_S)
                if r.returncode != 0:
                    return ("H3GAME_SRC not set and cloning %s failed (offline?): %s. "
                            "Set H3GAME_SRC to an h3game clone at %s." % (H3GAME_URL, r.stderr.strip()[:200], H3GAME_PIN))
                if os.path.isdir(H3GAME_CACHE):
                    shutil.rmtree(H3GAME_CACHE)
                os.rename(os.path.join(tmp, "repo"), H3GAME_CACHE)
            finally:
                shutil.rmtree(tmp, ignore_errors=True)
        head = _git_quiet(H3GAME_CACHE, "rev-parse", "HEAD").stdout.strip()
        if not head.startswith(H3GAME_PIN):
            r = _git_quiet(H3GAME_CACHE, "checkout", "-q", "-B", "h3cs-pinned", H3GAME_PIN)
            if r.returncode != 0:
                _git_quiet(H3GAME_CACHE, "fetch", "-q", "origin", timeout=CLONE_TIMEOUT_S)
                r = _git_quiet(H3GAME_CACHE, "checkout", "-q", "-B", "h3cs-pinned", H3GAME_PIN)
            if r.returncode != 0:
                return "cannot check out %s in %s: %s" % (H3GAME_PIN, H3GAME_CACHE, r.stderr.strip()[:200])
    except (OSError, subprocess.TimeoutExpired) as e:
        return "H3GAME_SRC not set and preparing %s failed: %r" % (H3GAME_CACHE, e)
    return None if _is_clone(H3GAME_CACHE) else "h3game cache clone is incomplete: %s" % H3GAME_CACHE


def server_available() -> bool:
    return os.path.isfile(SERVER_BINARY) and os.access(SERVER_BINARY, os.X_OK)


def git(cwd: str, *args: str) -> str:
    cmd = ["git", "-c", "user.name=h3game-adapter-test", "-c", "user.email=test@example.invalid",
           "-c", "commit.gpgsign=false"] + list(args)
    return subprocess.run(cmd, cwd=cwd, check=True, capture_output=True, text=True).stdout


def make_clone(apply_patch: bool = True) -> str:
    """Fresh temp clone of H3GAME_SRC (optionally with the patches applied via git am).
    Caller removes it with shutil.rmtree."""
    tmp = tempfile.mkdtemp(prefix="h3game-test-")
    dest = os.path.join(tmp, "repo")
    subprocess.run(["git", "clone", "-q", H3GAME_SRC, dest], check=True, capture_output=True)
    if apply_patch:
        git(dest, "am", "-q", *patches())
    return dest


def remove_clone(path: str) -> None:
    shutil.rmtree(os.path.dirname(path), ignore_errors=True)


def free_port() -> int:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def wait_until(pred, timeout: float = 5.0, step: float = 0.02, msg: str = "condition"):
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        last = pred()
        if last:
            return last
        time.sleep(step)
    raise AssertionError("timed out waiting for %s (last %r)" % (msg, last))


def http(method: str, url: str, body: Optional[Any] = None, token: Optional[str] = None,
         timeout: float = 2.0) -> Tuple[int, Any]:
    """Return (status, parsed JSON or raw text)."""
    data = None
    if body is not None:
        data = body if isinstance(body, bytes) else json.dumps(body).encode()
    elif method == "POST":
        data = b""
    headers = {}
    if token:
        headers["Authorization"] = "Bearer " + token
    req = urllib.request.Request(url, data=data, method=method, headers=headers)
    try:
        with HTTP.open(req, timeout=timeout) as r:
            status, raw = r.status, r.read()
    except urllib.error.HTTPError as e:
        with e:
            status, raw = e.code, e.read()
    try:
        return status, json.loads(raw.decode("utf-8"))
    except ValueError:
        return status, raw.decode("utf-8", "replace")


class Proc:
    """Subprocess in its own process group (bridge.py spawns joy_reader.py)."""

    def __init__(self, args: List[str], cwd: str, env: Dict[str, str]):
        self.log = tempfile.TemporaryFile()
        self.p = subprocess.Popen(args, cwd=cwd, env=env, stdout=self.log, stderr=subprocess.STDOUT,
                                  start_new_session=True)

    def alive(self) -> bool:
        return self.p.poll() is None

    def output(self) -> str:
        self.log.seek(0)
        return self.log.read().decode("utf-8", "replace")

    def kill(self) -> None:
        if self.p.poll() is None:
            try:
                os.killpg(self.p.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                self.p.wait(5)
            except subprocess.TimeoutExpired:
                os.killpg(self.p.pid, signal.SIGKILL)
                self.p.wait(5)
        else:
            # Parent gone: still reap any child left in the group.
            try:
                os.killpg(self.p.pid, signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass
        self.log.close()


def start_bridge(repo: str, port: int, extra_env: Dict[str, str], pythonpath: Optional[str] = None) -> Proc:
    env = dict(os.environ)
    for k in ("H3_BACKEND", "H3_DRY", "H3_SERVER_URL"):
        env.pop(k, None)
    env.update(extra_env)
    env["H3_PORT"] = str(port)
    env["PYTHONDONTWRITEBYTECODE"] = "1"
    env["PYTHONUNBUFFERED"] = "1"
    if pythonpath:
        env["PYTHONPATH"] = pythonpath
    else:
        env.pop("PYTHONPATH", None)
    h3 = os.path.join(repo, "h3game")
    proc = Proc([sys.executable, "-u", os.path.join(h3, "bridge.py")], cwd=h3, env=env)
    url = "http://127.0.0.1:%d" % port

    def up():
        if not proc.alive():
            raise AssertionError("bridge exited: %s" % proc.output())
        try:
            return http("GET", url + "/state", timeout=0.5)[0] == 200
        except OSError:
            return False

    try:
        wait_until(up, timeout=10.0, msg="bridge /state")
    except Exception:
        proc.kill()
        raise
    return proc


def start_server(port: int) -> Proc:
    env = dict(os.environ, H3CS_ADMIN_TOKEN=ADMIN_TOKEN, RUST_LOG="warn")
    proc = Proc([SERVER_BINARY, "--config", SIM_CONFIG, "--bind", "127.0.0.1:%d" % port], cwd=SERVER_DIR, env=env)
    url = "http://127.0.0.1:%d" % port

    def ready():
        if not proc.alive():
            raise AssertionError("server exited: %s" % proc.output())
        try:
            s, b = http("GET", url + "/v1/status", timeout=0.5)
        except OSError:
            return False
        return s == 200 and b.get("rig_state") == "READY_TO_ARM"

    try:
        wait_until(ready, timeout=10.0, msg="server READY_TO_ARM")
    except Exception:
        proc.kill()
        raise
    return proc


def write_stub(dirpath: str, name: str, marker: str) -> None:
    """Package `name` that appends to `marker` and raises when imported."""
    pkg = os.path.join(dirpath, name)
    os.makedirs(pkg, exist_ok=True)
    with open(os.path.join(pkg, "__init__.py"), "w") as f:
        f.write("with open(%r, 'a') as _f:\n    _f.write(%r)\n" % (marker, name + "\n"))
        f.write("raise ImportError('stub %s: import is not allowed in this test')\n" % name)
