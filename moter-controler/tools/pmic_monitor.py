#!/usr/bin/env python3
"""Issue #1 の調査用。CoreS3（env:m5stack-cores3-pmiclog）のシリアル出力を、時刻つきでファイルに記録する。

- ボードには何も送らない（読むだけ）。
- ポートは DTR=1 / RTS=1 のまま開く。pyserial の既定（開いた後に DTR を先に下げる）では、
  一瞬 DTR=0 / RTS=1 になり、ESP32-S3 の USB-Serial-JTAG がチップをリセットする。リセットは主電源リレーを落とす。
- 開く前に、ポートの USB シリアル番号（= チップの MAC）で CoreS3 であることを確かめる。別の ESP32-S3 が同じポート名で出ることがある。
- USB が切れたら、戻るまで待って記録を続ける。電源キーで起こした後の PMICBOOT 行（前回の電源断要因）も同じファイルに残る。
- 並行して macOS の USB ログ（切断の前にケーブル検出 CC が落ちたか）を別ファイルに記録する。

使い方:
  ~/.platformio/penv/bin/python tools/pmic_monitor.py --port /dev/cu.usbmodem1101
"""

import argparse
import datetime as dt
import os
import re
import subprocess
import sys
import time

import serial

# 手元の CoreS3。48:27 は 10/26 に追加した個体、30:ED は 10/25 まで使っていた個体（Issue #1 を再現した個体）。
CORES3_MACS = ["48:27:E2:66:B0:68", "30:ED:A0:0B:20:40"]
ESPRESSIF_VID = 0x303A
MAC_LOG_PREDICATE = (
    'process == "kernel" AND NOT eventMessage CONTAINS "transaction error" AND '
    '(eventMessage CONTAINS "0x303a" OR eventMessage CONTAINS "CC] active")'
)


def now_text():
    return dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def port_serials():
    """ioreg から /dev/cu.* と USB シリアル番号（= チップの MAC）の対応を作る。"""
    out = subprocess.run(
        ["ioreg", "-p", "IOService", "-l", "-w0", "-r", "-c", "IOUSBHostDevice"], capture_output=True, text=True
    ).stdout
    mapping = {}
    serial_no = None
    for line in out.splitlines():
        m = re.search(r'"USB Serial Number" = "([^"]+)"', line)
        if m:
            serial_no = m.group(1).upper()
            continue
        m = re.search(r'"IOCalloutDevice" = "([^"]+)"', line)
        if m and serial_no:
            mapping[m.group(1)] = serial_no
    return mapping


def check_identity(path, macs):
    """path が手元の CoreS3 なら None、そうでなければ理由を返す。ポートは開かない。"""
    serial_no = port_serials().get(path)
    if serial_no is None:
        return f"{path} の USB シリアル番号が分からない"
    if serial_no not in macs:
        return f"{path} は CoreS3 ではない（シリアル番号 {serial_no}）"
    return None


def open_port(path):
    port = serial.Serial()
    port.port = path
    port.baudrate = 115200
    port.timeout = 0.2
    port.dtr = True
    port.rts = True
    port.open()
    return port


class Recorder:
    def __init__(self, path):
        self.file = open(path, "a", buffering=1, encoding="utf-8")

    def write(self, text, echo=True):
        line = f"{now_text()} {text}"
        self.file.write(line + "\n")
        if echo:
            print(line, flush=True)


def wait_for_port(path, macs, rec):
    rec.write(f"# WAIT port={path}")
    while True:
        if os.path.exists(path):
            reason = check_identity(path, macs)
            if reason is None:
                return
            rec.write(f"# SKIP {reason}")
            time.sleep(5)
        time.sleep(0.5)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", required=True, help="例: /dev/cu.usbmodem1101")
    parser.add_argument("--out", default=os.path.join(os.path.dirname(__file__), "..", "logs"))
    parser.add_argument("--mac", action="append", default=None, help="CoreS3 の USB シリアル番号（MAC）。既定は CORES3_MACS")
    parser.add_argument("--no-maclog", action="store_true", help="macOS の USB ログを記録しない")
    args = parser.parse_args()

    os.makedirs(args.out, exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    serial_path = os.path.join(args.out, f"pmic_{stamp}.log")
    maclog_path = os.path.join(args.out, f"pmic_{stamp}_maclog.log")

    macs = [m.upper() for m in (args.mac or CORES3_MACS)]
    port_path = args.port
    reason = check_identity(port_path, macs)
    if reason is not None:
        print(f"中止: {reason}", file=sys.stderr)
        return 2

    maclog = None
    if not args.no_maclog:
        maclog_file = open(maclog_path, "a", encoding="utf-8")
        maclog = subprocess.Popen(
            ["/usr/bin/log", "stream", "--style", "compact", "--predicate", MAC_LOG_PREDICATE],
            stdout=maclog_file,
            stderr=subprocess.STDOUT,
        )

    rec = Recorder(serial_path)
    rec.write(f"# START port={port_path} mac={port_serials().get(port_path)} serial_log={serial_path} maclog={maclog_path}")
    first_open = True
    try:
        while True:
            try:
                port = open_port(port_path)
            except (serial.SerialException, OSError) as exc:
                rec.write(f"# OPEN_FAILED {exc}")
                wait_for_port(port_path, macs, rec)
                continue
            opened_at = time.monotonic()
            rec.write("# OPEN")
            buf = b""
            try:
                while True:
                    chunk = port.read(4096)
                    if not chunk:
                        continue
                    buf += chunk
                    while b"\n" in buf:
                        raw, buf = buf.split(b"\n", 1)
                        text = raw.decode(errors="replace").rstrip("\r")
                        # PMIC 行は毎秒出るので、画面には 10 秒に 1 行だけ出す（ファイルには全部残す）
                        echo = not text.startswith("PMIC up=") or text.split("up=")[1].split(" ")[0].endswith("0")
                        rec.write(text, echo=echo)
                        if first_open and text.startswith("BOOT ") and time.monotonic() - opened_at < 2.0:
                            rec.write("# WARNING ポートを開いた直後に BOOT 行が来た。開いたことでボードがリセットされた可能性がある")
                first_open = False
            except (serial.SerialException, OSError) as exc:
                rec.write(f"# DISCONNECT {exc}")
                try:
                    port.close()
                except Exception:
                    pass
                first_open = False
                wait_for_port(port_path, macs, rec)
    except KeyboardInterrupt:
        rec.write("# STOP")
    finally:
        if maclog is not None:
            maclog.terminate()
    return 0


if __name__ == "__main__":
    sys.exit(main())
