#!/usr/bin/env bash
# End-to-end check on the sim backend: h3-control-server + patched h3game bridge.
#
#   control-server/scripts/e2e_sim.sh [--out DIR]
#
# Builds the server, clones h3game (H3GAME_SRC) into a temp dir and applies
# clients/h3game/patches/*.patch with git am, then runs
# clients/h3game/tests/e2e_driver.py, which starts the server (config/sim.toml,
# free loopback port) and the bridge (H3_BACKEND=server), plays browser and
# operator, and writes summary.md + timeline.jsonl.
#
# Default output: docs/control-server/runs/e2e_sim_<YYYYmmdd-HHMMSS>/
# Env: H3GAME_SRC (an h3game clone; default: a cache clone of
#      github.com/TakuroFukamizu/20261004_shi3z_hackathon at commit 649ab66 in
#      control-server/target/h3game-src, made on first use, needs network),
#      PYTHON (needs websockets>=13 for the bridge),
#      H3CS_ADMIN_TOKEN (default: random per run).
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVER_DIR="$(cd "$HERE/.." && pwd)"
REPO="$(cd "$SERVER_DIR/.." && pwd)"
H3GAME_URL="https://github.com/TakuroFukamizu/20261004_shi3z_hackathon.git"
H3GAME_PIN="649ab66"   # the commit whose blobs the patch was made against
H3GAME_CACHE="$SERVER_DIR/target/h3game-src"
PYTHON="${PYTHON:-$REPO/clients/python/.venv/bin/python}"
[ -x "$PYTHON" ] || PYTHON="$(command -v python3)"

OUT=""
while [ $# -gt 0 ]; do
  case "$1" in
    --out) OUT="$2"; shift 2 ;;
    -h|--help) sed -n '2,18p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done
[ -n "$OUT" ] || OUT="$REPO/docs/control-server/runs/e2e_sim_$(date +%Y%m%d-%H%M%S)"

WORK="$(mktemp -d "${TMPDIR:-/tmp}/h3-e2e.XXXXXX")"
DRIVER_PID=""
cleanup() {
  local rc=$?
  # The driver kills its own process groups; this is the backstop if it was killed.
  if [ -n "$DRIVER_PID" ] && kill -0 "$DRIVER_PID" 2>/dev/null; then
    kill -TERM "$DRIVER_PID" 2>/dev/null || true
    wait "$DRIVER_PID" 2>/dev/null || true
  fi
  if [ -f "$WORK/pids" ]; then
    while read -r pg; do
      kill -TERM -- "-$pg" 2>/dev/null || true
    done < "$WORK/pids"
    sleep 0.5
    while read -r pg; do
      kill -KILL -- "-$pg" 2>/dev/null || true
    done < "$WORK/pids"
  fi
  rm -rf "$WORK"
  exit "$rc"
}
trap cleanup EXIT
trap 'exit 130' INT TERM

echo "[e2e_sim] python: $PYTHON ($("$PYTHON" -c 'import sys; print(sys.version.split()[0])'))"
"$PYTHON" -c 'import websockets' 2>/dev/null || { echo "websockets is not installed for $PYTHON" >&2; exit 2; }

# h3game source: H3GAME_SRC, else the pinned cache clone (cloned once).
if [ -z "${H3GAME_SRC:-}" ]; then
  H3GAME_SRC="$H3GAME_CACHE"
  if [ ! -f "$H3GAME_SRC/h3game/bridge.py" ]; then
    echo "[e2e_sim] H3GAME_SRC not set: cloning $H3GAME_URL -> $H3GAME_SRC"
    mkdir -p "$(dirname "$H3GAME_SRC")"
    rm -rf "$H3GAME_SRC.tmp"
    GIT_TERMINAL_PROMPT=0 git clone -q "$H3GAME_URL" "$H3GAME_SRC.tmp" || {
      rm -rf "$H3GAME_SRC.tmp"
      echo "cannot clone $H3GAME_URL (offline?). Set H3GAME_SRC to an h3game clone at $H3GAME_PIN." >&2; exit 2; }
    rm -rf "$H3GAME_SRC" && mv "$H3GAME_SRC.tmp" "$H3GAME_SRC"
  fi
  case "$(git -C "$H3GAME_SRC" rev-parse HEAD)" in
    "$H3GAME_PIN"*) ;;
    *) git -C "$H3GAME_SRC" checkout -q -B h3cs-pinned "$H3GAME_PIN" 2>/dev/null \
         || { GIT_TERMINAL_PROMPT=0 git -C "$H3GAME_SRC" fetch -q origin \
              && git -C "$H3GAME_SRC" checkout -q -B h3cs-pinned "$H3GAME_PIN"; } \
         || { echo "cannot check out $H3GAME_PIN in $H3GAME_SRC" >&2; exit 2; } ;;
  esac
fi
[ -f "$H3GAME_SRC/h3game/bridge.py" ] && [ -e "$H3GAME_SRC/.git" ] \
  || { echo "H3GAME_SRC is not an h3game clone: $H3GAME_SRC" >&2; exit 2; }

echo "[e2e_sim] cargo build"
(cd "$SERVER_DIR" && cargo build --quiet)
BIN="$SERVER_DIR/target/debug/h3-control-server"

echo "[e2e_sim] h3game clone + git am ($H3GAME_SRC)"
git clone -q "$H3GAME_SRC" "$WORK/repo"
git -C "$WORK/repo" -c user.name=e2e-sim -c user.email=e2e@example.invalid -c commit.gpgsign=false \
  am -q "$REPO"/clients/h3game/patches/*.patch
H3GAME_REV="$(git -C "$H3GAME_SRC" rev-parse --short HEAD) + $(ls "$REPO"/clients/h3game/patches/*.patch | xargs -n1 basename | tr '\n' ' ')"
REPO_REV="$(git -C "$REPO" rev-parse --short HEAD 2>/dev/null || echo unknown) (worktree; uncommitted changes not identified)"

export H3CS_ADMIN_TOKEN="${H3CS_ADMIN_TOKEN:-e2e-$(od -An -N8 -tx1 /dev/urandom | tr -d ' \n')}"
mkdir -p "$OUT"
echo "[e2e_sim] driver -> $OUT"
set +e
"$PYTHON" -u "$REPO/clients/h3game/tests/e2e_driver.py" \
  --server-bin "$BIN" --config "$SERVER_DIR/config/sim.toml" --h3game-repo "$WORK/repo" \
  --out "$OUT" --repo-root "$REPO" --repo-rev "$REPO_REV" --h3game-rev "$H3GAME_REV" --pidfile "$WORK/pids" &
DRIVER_PID=$!
wait "$DRIVER_PID"
RC=$?
DRIVER_PID=""
set -e
echo "[e2e_sim] exit $RC; summary: $OUT/summary.md"
exit "$RC"
