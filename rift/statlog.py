#!/usr/bin/env python3
# Log band and receiver stats every PERIOD seconds: statlog.py SECONDS [PERIOD]
import re, sys, time, serial
from con import PORTS
total, period = float(sys.argv[1]), float(sys.argv[2]) if len(sys.argv) > 2 else 5
ports = {k: serial.Serial(v, 115200, timeout=0.1) for k, v in PORTS.items()}
def ask(k):
	s = ports[k]
	s.read(100000)
	s.write(b"stats\r\n")
	time.sleep(0.4)
	return s.read(100000).decode(errors="replace")
ask("band"); ask("rx")
t0 = time.time()
while time.time() - t0 < total:
	time.sleep(period)
	b = re.search(r"(\d+) written, (\d+) acked, (\d+) failed, (\d+) flushed", ask("band"))
	r = re.search(r"id 0: (\d+) packets, ([\d.]+)/s, (\d+) lost \(([\d.]+)%\), rssi (-?\d+)", ask("rx"))
	print(f"{time.time() - t0:6.1f} s  band {'/'.join(b.groups()) if b else '?'}  "
		  f"rx {r.group(2) + '/s, lost ' + r.group(3) + ', rssi ' + r.group(5) if r else '?'}", flush=True)
