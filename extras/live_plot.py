#!/usr/bin/env python3
"""Live plot for TEST_PID: angle vs setpoint, P/I/D terms, motor duty.

Keys typed in the plot window go straight to the drone (same keys as the
serial monitor): p/P i/I d/D t/T l r c 0 x. BOOT on the board still
starts and stops the motors.

    python3 extras/live_plot.py [/dev/ttyACM0]

Close any other serial monitor first - only one program can hold the port.
"""

import re
import sys
import threading
import time
from collections import deque

import matplotlib.pyplot as plt
import serial
from matplotlib.animation import FuncAnimation

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
BAUD = 115200
WINDOW_S = 15.0
KEYS = set("pPiIdDtTlrc0x")

NUM = r"([+-]?\d+(?:\.\d+)?)"
LOG_RE = re.compile(
    rf"^([RP])\s+{NUM}\s+sp\s+{NUM}\s+rate\s+{NUM}\s+P\s+{NUM}\s+I\s+{NUM}"
    rf"\s+D\s+{NUM}\s+M\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)"
)
GAIN_RE = re.compile(
    rf"kp\s+{NUM}\s+ki\s+{NUM}\s+kd\s+{NUM}\s+thr\s+(\d+)\s+sp\s+{NUM}\s+PID\s+(\w+)"
)

data = {k: deque() for k in ("t", "ang", "sp", "p", "i", "d", "m1", "m2", "m3", "m4")}
events = deque()                    # (time, label) for gain / mode changes
state = {"gains": "waiting for data - press BOOT on the board", "axis": "R"}
lock = threading.Lock()
t0 = time.monotonic()

ser = serial.Serial()
ser.port = PORT
ser.baudrate = BAUD
ser.timeout = 0.1
ser.dtr = False                     # do not reset the board on open
ser.rts = False
ser.open()


def send(ch):
    ser.write(ch.encode())
    print(f"> sent '{ch}'", flush=True)


def reader():
    buf = b""
    while ser.is_open:
        try:
            buf += ser.read(256)
        except (TypeError, serial.SerialException):
            break                   # port closed on exit
        while b"\n" in buf:
            raw, buf = buf.split(b"\n", 1)
            line = raw.decode(errors="replace").strip("\r ")
            now = time.monotonic() - t0
            m = LOG_RE.match(line)
            g = GAIN_RE.search(line)
            with lock:
                if m:
                    state["axis"] = m.group(1)
                    vals = [float(x) for x in m.groups()[1:]]
                    for key, v in zip(("ang", "sp", None, "p", "i", "d",
                                       "m1", "m2", "m3", "m4"), vals):
                        if key:
                            data[key].append(v)
                    data["t"].append(now)
                    while data["t"] and data["t"][0] < now - WINDOW_S:
                        for q in data.values():
                            q.popleft()
                elif g:
                    print(line, flush=True)     # board confirmed the key
                    kp, ki, kd, thr, sp, on = g.groups()
                    state["gains"] = (f"kp {float(kp):.1f}   ki {float(ki):.1f}   "
                                      f"kd {float(kd):.2f}   throttle {thr}   PID {on}")
                    events.append((now, f"kp {float(kp):.1f} kd {float(kd):.2f}"
                                        + ("" if on == "ON" else "\nPID OFF")))
                elif line:
                    print(line)


def stdin_keys():
    """Keys typed in the terminal are forwarded too (no Enter needed)."""
    import atexit
    import termios
    import tty
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    atexit.register(termios.tcsetattr, fd, termios.TCSADRAIN, old)
    tty.setcbreak(fd)
    while True:
        ch = sys.stdin.read(1)
        if ch in KEYS:
            send(ch)


threading.Thread(target=reader, daemon=True).start()
if sys.stdin.isatty():
    threading.Thread(target=stdin_keys, daemon=True).start()

# free the letter keys matplotlib uses for its own shortcuts
for name in list(plt.rcParams):
    if name.startswith("keymap."):
        plt.rcParams[name] = []

fig, (ax_a, ax_pid, ax_m) = plt.subplots(
    3, 1, sharex=True, figsize=(11, 8), gridspec_kw={"height_ratios": [3, 1.4, 1.4]})
line_sp, = ax_a.plot([], [], "--", color="tab:gray", lw=2, label="setpoint (command)")
line_ang, = ax_a.plot([], [], color="tab:blue", lw=2, label="measured angle")
ax_a.set_ylabel("angle (deg)")
ax_a.set_ylim(-35, 35)
ax_a.axhline(0, color="black", lw=0.5)
ax_a.legend(loc="upper left")

line_p, = ax_pid.plot([], [], label="P")
line_i, = ax_pid.plot([], [], label="I")
line_d, = ax_pid.plot([], [], label="D")
ax_pid.set_ylabel("correction\n(per-mille)")
ax_pid.legend(loc="upper left", ncol=3)

line_left, = ax_m.plot([], [], color="tab:green", label="left motors M3/M4")
line_right, = ax_m.plot([], [], color="tab:red", label="right motors M1/M2")
ax_m.set_ylabel("duty\n(per-mille)")
ax_m.set_xlabel("time (s)")
ax_m.set_ylim(0, 1000)
ax_m.legend(loc="upper left", ncol=2)

title = fig.suptitle("", fontsize=13)
fig.text(0.01, 0.005, "keys: P/p kp  D/d kd  I/i ki  T/t throttle  "
         "l/r/c lean left/right/centre  0 PID on/off  x stop", fontsize=9)
markers = []


def on_key(event):
    if event.key and len(event.key) == 1 and event.key in KEYS:
        send(event.key)


fig.canvas.mpl_connect("key_press_event", on_key)


def update(_frame):
    with lock:
        t = list(data["t"])
        cols = {k: list(v) for k, v in data.items()}
        evs = [e for e in events if t and e[0] >= t[0]]
        title.set_text(state["gains"])
    if not t:
        return []
    line_ang.set_data(t, cols["ang"])
    line_sp.set_data(t, cols["sp"])
    line_p.set_data(t, cols["p"])
    line_i.set_data(t, cols["i"])
    line_d.set_data(t, cols["d"])
    line_left.set_data(t, [(a + b) / 2 for a, b in zip(cols["m3"], cols["m4"])])
    line_right.set_data(t, [(a + b) / 2 for a, b in zip(cols["m1"], cols["m2"])])
    ax_a.set_xlim(max(t[-1] - WINDOW_S, 0), max(t[-1], WINDOW_S))
    ax_a.set_ylabel("roll (deg)" if state["axis"] == "R" else "pitch (deg)")
    span = max([abs(v) for v in cols["p"] + cols["i"] + cols["d"]] + [20])
    ax_pid.set_ylim(-span * 1.1, span * 1.1)

    for mk in markers:
        mk.remove()
    markers.clear()
    for et, label in evs:
        markers.append(ax_a.axvline(et, color="tab:orange", lw=1))
        markers.append(ax_a.text(et, 32, label, fontsize=8, va="top",
                                 color="tab:orange"))
    return []


anim = FuncAnimation(fig, update, interval=50, cache_frame_data=False)
plt.tight_layout(rect=(0, 0.02, 1, 0.97))
plt.show()
ser.close()
