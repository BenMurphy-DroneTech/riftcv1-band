#!/usr/bin/env python3
# Send console commands (USB serial) to the band or receiver: con.py band|rx "cmd" ["cmd" ...]
# A command "wait N" sleeps N seconds while printing what arrives.
import sys, time, serial
PORTS = {
	"band": "/dev/serial/by-id/usb-Seeed_Studio_SlimeNRF_Tracker_XIAO_Sense_405A67C5CDC65501-if00",
	"rx": "/dev/serial/by-id/usb-Seeed_Studio_SlimeNRF_Receiver_XIAO_47C7544E4180A6C7-if00",
}

def main():
	s = serial.Serial(PORTS[sys.argv[1]], 115200, timeout=0.2)
	time.sleep(0.3)
	s.read(100000)
	for c in sys.argv[2:]:
		if c.startswith("wait "):
			end = time.time() + float(c.split()[1])
			while time.time() < end:
				sys.stdout.write(s.read(10000).decode(errors="replace"))
			continue
		s.write((c + "\r\n").encode())
		time.sleep(0.8)
		sys.stdout.write(s.read(100000).decode(errors="replace"))
	sys.stdout.flush()

if __name__ == "__main__":
	main()
