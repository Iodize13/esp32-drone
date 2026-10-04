"""Read the COM port without resetting the board. Usage: readser.py SECONDS"""
import serial, time, sys
s = serial.Serial()
s.port = '/dev/ttyACM0'; s.baudrate = 115200; s.timeout = 0.5
s.dtr = False; s.rts = False
s.open()
end = time.time() + float(sys.argv[1])
while time.time() < end:
    d = s.read(512)
    if d:
        sys.stdout.write(d.decode(errors='replace')); sys.stdout.flush()
        if b'press BOOT to repeat' in d or b'ABORTED' in d or b'motors off' in d:
            break
