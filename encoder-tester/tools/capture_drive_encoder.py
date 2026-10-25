#!/usr/bin/env python3
"""Read the encoder-tester USB text log.

Opening the serial port resets the CoreS3. This script sends STOP on Ctrl+C.
It does not send ESTOP. The MCU still stops on its own deadline if STOP never arrives.
"""

import argparse
import os
import signal
import sys
import threading
import time
from datetime import datetime

import serial
from serial.tools import list_ports


def unique_file(path):
    flags = os.O_CREAT | os.O_EXCL | os.O_WRONLY
    fd = os.open(path, flags, 0o644)
    return os.fdopen(fd, "w", encoding="utf-8", newline="\n")


def list_serial_ports():
    found = list(list_ports.comports())
    if not found:
        print("no serial ports")
        return
    for port in found:
        print("%s\t%s" % (port.device, port.description))


def parse_args():
    parser = argparse.ArgumentParser(description="Capture encoder-tester USB text. Opening the port resets the board.")
    parser.add_argument("--list", action="store_true", help="list serial ports and exit")
    parser.add_argument("--port", help="serial device, for example /dev/cu.usbmodem101 or COM3")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--out", help="CSV path. Refuses to overwrite. Default is a timestamped file under logs/")
    parser.add_argument("--heartbeat-ms", type=int, default=200, help="HEARTBEAT interval. LEASE_MS must be longer.")
    parser.add_argument("--no-heartbeat", action="store_true")
    parser.add_argument("--no-stream", action="store_true", help="do not send STREAM ON after the port opens")
    return parser.parse_args()


def default_out():
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    directory = os.path.join(os.getcwd(), "logs")
    os.makedirs(directory, exist_ok=True)
    return os.path.join(directory, "capture_%s.csv" % stamp)


def sidecar_path(csv_path):
    base, ext = os.path.splitext(csv_path)
    if ext.lower() == ".csv":
        return base + ".log"
    return csv_path + ".log"


def main():
    args = parse_args()
    if args.list:
        list_serial_ports()
        return 0
    if not args.port:
        print("pass --port, or --list", file=sys.stderr)
        return 2
    if args.heartbeat_ms < 1:
        print("heartbeat-ms must be >= 1", file=sys.stderr)
        return 2

    csv_path = args.out or default_out()
    log_path = sidecar_path(csv_path)
    if os.path.exists(csv_path) or os.path.exists(log_path):
        print("refusing to overwrite %s" % csv_path, file=sys.stderr)
        return 2

    print("Opening the USB port resets the CoreS3 (USB_UART_CHIP_RESET).", file=sys.stderr)
    print("LEASE_MS must be longer than the %d ms heartbeat." % args.heartbeat_ms, file=sys.stderr)
    print("csv %s" % csv_path, file=sys.stderr)
    print("log %s" % log_path, file=sys.stderr)

    csv_file = unique_file(csv_path)
    log_file = unique_file(log_path)
    try:
        port = serial.Serial(args.port, args.baud, timeout=0.2)
    except Exception as exc:
        csv_file.close()
        log_file.close()
        os.unlink(csv_path)
        os.unlink(log_path)
        print("could not open %s: %s" % (args.port, exc), file=sys.stderr)
        return 2
    write_lock = threading.Lock()
    stop = threading.Event()

    def send(text):
        data = (text.strip() + "\n").encode("utf-8")
        with write_lock:
            port.write(data)
            port.flush()

    def handle_sigint(signum, frame):
        del signum, frame
        try:
            port.write(b"STOP\n")
            port.flush()
        except Exception:
            pass
        os._exit(0)

    signal.signal(signal.SIGINT, handle_sigint)

    def heartbeat():
        while not stop.is_set():
            try:
                send("HEARTBEAT")
            except Exception:
                return
            stop.wait(args.heartbeat_ms / 1000.0)

    def stdin_loop():
        while not stop.is_set():
            line = sys.stdin.readline()
            if line == "":
                return
            if line.strip() == "":
                continue
            try:
                send(line)
            except Exception as exc:
                print("write failed: %s" % exc, file=sys.stderr)
                return

    threads = [threading.Thread(target=stdin_loop, daemon=True)]
    if not args.no_heartbeat:
        threads.append(threading.Thread(target=heartbeat, daemon=True))
    for thread in threads:
        thread.start()

    if not args.no_stream:
        time.sleep(1.5)
        try:
            send("STREAM ON")
        except Exception as exc:
            print("STREAM ON failed: %s" % exc, file=sys.stderr)

    try:
        while True:
            raw = port.readline()
            if not raw:
                continue
            text = raw.decode("utf-8", "replace").replace("\x00", "").strip()
            if text == "":
                continue
            print(text)
            sys.stdout.flush()
            if text.startswith("seq,") or text[0].isdigit():
                csv_file.write(text + "\n")
                csv_file.flush()
            else:
                log_file.write(text + "\n")
                log_file.flush()
    finally:
        stop.set()
        csv_file.close()
        log_file.close()
        port.close()


if __name__ == "__main__":
    sys.exit(main())
