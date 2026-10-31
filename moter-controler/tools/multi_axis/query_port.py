import serial, time, sys
s = serial.Serial(); s.port = sys.argv[1]; s.baudrate = 115200; s.timeout = 0.2
s.dtr = True; s.rts = True; s.open(); time.sleep(1.5); s.reset_input_buffer()
for c in sys.argv[2:]: s.write((c+"\n").encode()); time.sleep(0.7)
buf=b""; t0=time.time()
while time.time()-t0 < 2.5: buf += s.read(4096)
s.close(); print(buf.decode('utf-8','replace'))
