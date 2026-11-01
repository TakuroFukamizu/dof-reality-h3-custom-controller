"""Runs browser_check.mjs (headless Chromium via Node Playwright) against a patched bridge.

Best effort: skipped unless node, the playwright package and a Chromium are found.
PLAYWRIGHT_MODULE / CHROMIUM_PATH override the lookup. With the server binary
the bridge runs in server mode against the sim server, otherwise in dry mode.
In server mode it also ARMs and STARTs the sim rig (ADMIN_TOKEN) and checks
that hiding the tab while ACTIVE does not move it toward center.
"""

from __future__ import annotations

import glob
import json
import os
import shutil
import subprocess
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import _support as S  # noqa: E402

SCRIPT = os.path.join(HERE, "browser_check.mjs")


def _playwright_module():
    if os.environ.get("PLAYWRIGHT_MODULE"):
        return os.environ["PLAYWRIGHT_MODULE"]
    node = shutil.which("node")
    if not node:
        return None
    r = subprocess.run([node, "-e", "console.log(require.resolve('playwright'))"], capture_output=True, text=True)
    if r.returncode == 0:
        return os.path.dirname(r.stdout.strip())
    # npx cache (e.g. from @playwright/mcp)
    found = sorted(glob.glob(os.path.expanduser("~/.npm/_npx/*/node_modules/playwright/package.json")))
    return os.path.dirname(found[-1]) if found else None


def _chromium():
    if os.environ.get("CHROMIUM_PATH"):
        return os.environ["CHROMIUM_PATH"]
    pats = ["~/Library/Caches/ms-playwright/chromium_headless_shell-*/chrome-headless-shell-*/chrome-headless-shell",
            "~/.cache/ms-playwright/chromium_headless_shell-*/chrome-headless-shell-*/chrome-headless-shell"]
    found = []
    for p in pats:
        found += glob.glob(os.path.expanduser(p))
    return sorted(found)[-1] if found else None  # None: let playwright pick its own


PW = _playwright_module()
_skip = None
if not shutil.which("node"):
    _skip = "node not found"
elif not PW:
    _skip = "playwright (Node) not found; set PLAYWRIGHT_MODULE"
elif not S.src_available():
    _skip = S.src_skip_reason()
elif not S.patches():
    _skip = "no patch in %s" % S.PATCH_DIR


@unittest.skipIf(_skip is not None, _skip or "")
class BrowserCheck(unittest.TestCase):
    def test_page_sends_frame_seq_and_holds_it_when_hidden(self):
        # Covers the hidden-tab race: zeros must never go out with a new frame_seq.
        repo = S.make_clone(apply_patch=True)
        procs = []
        try:
            env = {}
            server_url = None
            if S.server_available():
                sport = S.free_port()
                procs.append(S.start_server(sport))
                server_url = "http://127.0.0.1:%d" % sport
                env = {"H3_BACKEND": "server", "H3_SERVER_URL": server_url}
            else:
                env = {"H3_BACKEND": "dry"}
            bport = S.free_port()
            procs.append(S.start_bridge(repo, bport, env))
            run_env = dict(os.environ, BRIDGE_URL="http://127.0.0.1:%d" % bport, PLAYWRIGHT_MODULE=PW)
            if server_url:
                run_env["SERVER_URL"] = server_url
                run_env["ADMIN_TOKEN"] = S.ADMIN_TOKEN  # enables the hide-while-ACTIVE check
            chrome = _chromium()
            if chrome:
                run_env["CHROMIUM_PATH"] = chrome
            r = subprocess.run(["node", SCRIPT], env=run_env, capture_output=True, text=True, timeout=90)
            line = (r.stdout.strip().splitlines() or [""])[-1]
            sys.stderr.write("\nbrowser_check: %s\n" % line)
            if r.returncode != 0 and not line.startswith("{"):
                self.fail("browser_check.mjs failed to run: %s" % (r.stderr[-2000:] or r.stdout[-2000:]))
            res = json.loads(line)
            self.assertEqual(res["failures"], [], res)
            self.assertEqual(r.returncode, 0, res)
        finally:
            for p in reversed(procs):
                p.kill()
            S.remove_clone(repo)


if __name__ == "__main__":
    unittest.main()
