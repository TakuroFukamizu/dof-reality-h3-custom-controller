import serial, time, sys, datetime
def now(): return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
log = open("shutdown_log.txt", "a", buffering=1)
axes = []
for spec in sys.argv[1:]:
    label, port = spec.split("=", 1)
    s = serial.Serial(); s.port = port; s.baudrate = 115200; s.timeout = 0
    s.dtr = True; s.rts = True; s.open(); axes.append((label, s))
time.sleep(1.5)
def send(cmd):
    for label, s in axes:
        s.write((cmd + "\n").encode()); log.write(f"{now()} [{label}] > {cmd}\n")
def drain(wait):
    t0 = time.time(); bufs = {l: b"" for l, _ in axes}
    while time.time() - t0 < wait:
        for label, s in axes:
            c = s.read(4096)
            if c: bufs[label] += c
        time.sleep(0.02)
    outl = {}
    for label, _ in axes:
        for line in bufs[label].decode("utf-8", "replace").splitlines():
            if line.strip():
                log.write(f"{now()} [{label}] {line}\n")
                if line.startswith(("STAT", "OK R=")): outl[label] = line
    return outl
drain(0.5)
send("B 0"); r1 = drain(0.8)
send("R 0"); r2 = drain(0.8)
send("?"); r3 = drain(1.2)
for label, _ in axes:
    print(label, "after B 0:", r1.get(label, "(no reply)"))
    print(label, "after R 0:", r2.get(label, "(no reply)"))
    print(label, "status   :", r3.get(label, "(no reply)"))
for _, s in axes: s.close()
