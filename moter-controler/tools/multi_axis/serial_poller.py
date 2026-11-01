import serial, time, glob, sys, datetime
out = open(sys.argv[1], "a", buffering=1)
def log(line):
    out.write(f"{datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]} {line}\n")
log("# poller start (sends ? every 500 ms)")
while True:
    ports = glob.glob(sys.argv[2] if len(sys.argv) > 2 else '/dev/cu.usbmodem*')
    if not ports:
        time.sleep(0.5); continue
    try:
        s = serial.Serial(); s.port = ports[0]; s.baudrate = 115200; s.timeout = 0.1
        s.dtr = True; s.rts = True; s.open()
        log(f"# opened {ports[0]}")
        buf = b""; next_q = 0
        while True:
            if time.time() >= next_q:
                s.write(b"?\n"); next_q = time.time() + 0.5
            chunk = s.read(4096)
            if chunk:
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    log(line.decode("utf-8", "replace").rstrip())
    except Exception as e:
        log(f"# port error: {e}"); time.sleep(1)
