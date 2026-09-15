#!/usr/bin/env python3
"""
cartpole.py -- host-side console for the ESP32 cart-pole.

    python3 cartpole.py

Then, at the >>> prompt:

    data       live dashboard of everything the board is streaming
    control    drive the cart by hand: A / D keys (W / S for the screws)
    auto       swing-up, then balance, with the dashboard up
    stop       emergency stop (also: spacebar or ESC while a window is open)

Everything streamed is logged to ./logs/ as CSV, always, without being asked.

Requires: pip install pyserial matplotlib
"""

import argparse
import csv
import os
import select
import sys
import termios
import threading
import time
import tty
from collections import deque
from datetime import datetime

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial missing.  pip3 install pyserial")

# Telemetry field order must match emitTelemetry() in the firmware.
FIELDS = ["t_ms", "mode", "theta", "theta_dot", "x", "v",
          "accel", "energy", "z1", "z2", "agc", "loop_us"]
MODES = ["IDLE", "MANUAL", "SWINGUP", "BALANCE", "FAULT"]
WINDOW_S = 10.0          # seconds of history kept for the plots


# --------------------------------------------------------------------------
# serial link
# --------------------------------------------------------------------------
class Link:
    """Owns the port. A reader thread parses lines into ring buffers and a CSV."""

    def __init__(self, port=None, baud=921600, logdir="logs"):
        self.port_name = port or self.autodetect()
        if not self.port_name:
            sys.exit("No ESP32 found. Plug it in, or pass --port /dev/cu.xxx")
        self.ser = serial.Serial(self.port_name, baud, timeout=0.1)
        self.lock = threading.Lock()
        self.buf = {k: deque(maxlen=20000) for k in FIELDS}
        self.t0 = None
        self.params = {}
        self.mode = "?"
        self.rx_count = 0
        self.rx_rate = 0.0
        self.alive = True

        os.makedirs(logdir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.logpath = os.path.join(logdir, f"run_{stamp}.csv")
        self._logfile = open(self.logpath, "w", newline="")
        self._csv = csv.writer(self._logfile)
        self._csv.writerow(FIELDS)

        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()
        time.sleep(0.3)

    @staticmethod
    def autodetect():
        hints = ("usbserial", "SLAB", "wchusbserial", "usbmodem", "UART")
        for p in list_ports.comports():
            if any(h.lower() in p.device.lower() or h.lower() in (p.description or "").lower()
                   for h in hints):
                return p.device
        return None

    def _reader(self):
        t_last, n_last = time.time(), 0
        while self.alive:
            try:
                raw = self.ser.readline()
            except Exception:
                break
            if not raw:
                continue
            line = raw.decode("utf-8", "replace").strip()
            if not line:
                continue

            if line[0] == "T":
                parts = line.split()
                if len(parts) != len(FIELDS) + 1:
                    continue
                try:
                    vals = [float(p) for p in parts[1:]]
                except ValueError:
                    continue
                with self.lock:
                    if self.t0 is None:
                        self.t0 = vals[0]
                    for k, val in zip(FIELDS, vals):
                        self.buf[k].append(val)
                    m = int(vals[1])
                    self.mode = MODES[m] if 0 <= m < len(MODES) else "?"
                    self.rx_count += 1
                self._csv.writerow(vals)
            elif line[0] == "=":
                parts = line.split()
                if len(parts) == 3:
                    with self.lock:
                        self.params[parts[1]] = float(parts[2])
                    print(f"  {parts[1]} = {parts[2]}")
            elif line[0] in "#!":
                print(f"  {line}")

            now = time.time()
            if now - t_last > 1.0:
                with self.lock:
                    self.rx_rate = (self.rx_count - n_last) / (now - t_last)
                    n_last = self.rx_count
                t_last = now

    def send(self, cmd):
        try:
            self.ser.write((cmd + "\n").encode())
        except Exception as e:
            print(f"  ! write failed: {e}")

    def recent(self, seconds=WINDOW_S):
        """Return (t_rel, {field: list}) for the last `seconds` of samples."""
        with self.lock:
            if not self.buf["t_ms"]:
                return [], {}
            t = list(self.buf["t_ms"])
            cut = t[-1] - seconds * 1000.0
            i = 0
            for i in range(len(t) - 1, -1, -1):
                if t[i] < cut:
                    i += 1
                    break
            out = {k: list(self.buf[k])[i:] for k in FIELDS}
        tt = [(v - out["t_ms"][-1]) / 1000.0 for v in out["t_ms"]]
        return tt, out

    def close(self):
        self.alive = False
        try:
            self.send("stop")
            time.sleep(0.1)
            self.ser.close()
        except Exception:
            pass
        self._logfile.close()


# --------------------------------------------------------------------------
# keyboard: raw terminal reader, used alongside the plot window
# --------------------------------------------------------------------------
class RawKeys:
    """Non-blocking single-char reads from the terminal, restored on exit."""

    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.saved = None

    def __enter__(self):
        self.saved = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, *a):
        if self.saved:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)

    def poll(self):
        keys = []
        while select.select([sys.stdin], [], [], 0)[0]:
            keys.append(sys.stdin.read(1))
        return keys


# --------------------------------------------------------------------------
# live dashboard
# --------------------------------------------------------------------------
def dashboard(link, interactive=False, title=""):
    """
    Blocking live plot. `interactive` enables A/D cart driving and W/S screws.

    Hold-to-move works through the firmware's 250 ms watchdog: terminal key
    repeat (or matplotlib repeat) keeps resending `v`, and the instant the
    repeat stops the board coasts to zero on its own. That means a crashed or
    disconnected host cannot leave the cart driving into the end of the rail.
    """
    try:
        import matplotlib
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
    except ImportError:
        return text_dashboard(link, interactive)

    jog_v = link.params.get("vman", 0.02)
    jog_z = link.params.get("zvmax", 2.0)
    held = {"key": None, "t": 0.0}
    quit_flag = {"q": False}

    def press(k):
        if k in ("a", "d", "w", "s"):
            held["key"], held["t"] = k, time.time()
        elif k == " " or k == "escape" or k == "\x1b":
            link.send("stop")
            held["key"] = None
            print("\r  STOP")
        elif k == "q":
            quit_flag["q"] = True

    def release(k):
        if k == held["key"]:
            held["key"] = None

    fig, ax = plt.subplots(5, 1, figsize=(10, 9), sharex=True)
    fig.canvas.manager.set_window_title("cart-pole" + (f" -- {title}" if title else ""))

    if interactive:
        fig.canvas.mpl_connect("key_press_event", lambda e: press(e.key))
        fig.canvas.mpl_connect("key_release_event", lambda e: release(e.key))

    lines = {}
    lines["theta"], = ax[0].plot([], [], lw=1.4)
    ax[0].set_ylabel("theta (rad)")
    ax[0].axhline(0, color="k", lw=0.6)
    ax[0].axhspan(-0.30, 0.30, color="tab:green", alpha=0.10)
    ax[0].set_ylim(-3.3, 3.3)

    lines["theta_dot"], = ax[1].plot([], [], lw=1.2, color="tab:orange")
    ax[1].set_ylabel("theta_dot\n(rad/s)")
    ax[1].axhline(0, color="k", lw=0.6)

    lines["x"], = ax[2].plot([], [], lw=1.4, color="tab:blue", label="x (m)")
    lines["v"], = ax[2].plot([], [], lw=1.0, color="tab:cyan", alpha=0.8, label="v (m/s)")
    ax[2].set_ylabel("cart")
    ax[2].legend(loc="upper left", fontsize=8)
    rail = link.params.get("rail", 0.22)
    ax[2].axhline(rail, color="r", lw=0.8, ls="--")
    ax[2].axhline(-rail, color="r", lw=0.8, ls="--")

    lines["accel"], = ax[3].plot([], [], lw=1.0, color="tab:red")
    ax[3].set_ylabel("accel cmd\n(m/s^2)")
    ax[3].axhline(0, color="k", lw=0.6)

    lines["energy"], = ax[4].plot([], [], lw=1.4, color="tab:purple")
    ax[4].set_ylabel("energy / E_up")
    ax[4].axhline(1.0, color="g", lw=0.8, ls="--")
    ax[4].axhline(-1.0, color="k", lw=0.6)
    ax[4].set_ylim(-1.6, 2.2)
    ax[4].set_xlabel("seconds before now")

    status = fig.text(0.01, 0.985, "", fontsize=9, va="top", family="monospace")

    def update(_):
        # keep the jog alive while a key is held down
        if interactive and held["key"]:
            if time.time() - held["t"] > 0.6:
                held["key"] = None
            else:
                k = held["key"]
                if k == "a":
                    link.send(f"v {-jog_v:.3f}")
                elif k == "d":
                    link.send(f"v {jog_v:.3f}")
                elif k == "w":
                    link.send(f"zv {jog_z:.1f}")
                elif k == "s":
                    link.send(f"zv {-jog_z:.1f}")

        t, d = link.recent()
        if not t:
            return list(lines.values()) + [status]
        for key, ln in lines.items():
            ln.set_data(t, d[key])
        for a in ax:
            a.set_xlim(-WINDOW_S, 0.4)
        for a, keys in ((ax[1], ["theta_dot"]), (ax[2], ["x", "v"]), (ax[3], ["accel"])):
            vals = [y for k in keys for y in d[k]] or [0, 1]
            lo, hi = min(vals), max(vals)
            pad = max(0.1, (hi - lo) * 0.15)
            a.set_ylim(lo - pad, hi + pad)

        status.set_text(
            f"mode {link.mode:<8} theta {d['theta'][-1]:+.3f}  x {d['x'][-1]:+.4f}m  "
            f"v {d['v'][-1]:+.3f}m/s  z {d['z1'][-1]:.1f}mm  "
            f"agc {int(d['agc'][-1])}  loop {int(d['loop_us'][-1])}us  "
            f"rx {link.rx_rate:.0f}Hz"
            + ("   [A/D cart  W/S screws  SPACE stop  Q quit]" if interactive else "   [Q quit]")
        )
        return list(lines.values()) + [status]

    anim = FuncAnimation(fig, update, interval=40, blit=False, cache_frame_data=False)

    # Terminal keys work too, so you don't have to chase window focus.
    with RawKeys() as kb:
        def pump():
            while plt.fignum_exists(fig.number) and not quit_flag["q"]:
                for k in kb.poll():
                    press(k)
                time.sleep(0.02)
            quit_flag["q"] = True
            try:
                plt.close(fig)
            except Exception:
                pass

        threading.Thread(target=pump, daemon=True).start()
        try:
            plt.show()
        except KeyboardInterrupt:
            pass
        quit_flag["q"] = True

    link.send("stop")
    del anim


def text_dashboard(link, interactive=False):
    """Fallback when matplotlib isn't installed: one refreshing status line."""
    print("  (matplotlib not found -- text mode.  Ctrl-C to exit)")
    jog_v = 0.15
    with RawKeys() as kb:
        try:
            while True:
                for k in kb.poll():
                    if k == "a" and interactive:
                        link.send(f"v {-jog_v}")
                    elif k == "d" and interactive:
                        link.send(f"v {jog_v}")
                    elif k in (" ", "\x1b"):
                        link.send("stop")
                    elif k == "q":
                        raise KeyboardInterrupt
                t, d = link.recent(1.0)
                if t:
                    print(f"\r  {link.mode:<8} th{d['theta'][-1]:+.3f} "
                          f"x{d['x'][-1]:+.4f} v{d['v'][-1]:+.3f} "
                          f"E{d['energy'][-1]:+.2f} rx{link.rx_rate:.0f}Hz   ", end="")
                time.sleep(0.05)
        except KeyboardInterrupt:
            print()
    link.send("stop")


# --------------------------------------------------------------------------
# REPL
# --------------------------------------------------------------------------
HELP = """
  data        live dashboard (read-only)
  control     drive the cart:  A / D  cart left-right,  W / S  screws up-down
              SPACE = stop,  Q = back to prompt
  auto        swing-up then balance, dashboard opens
  bal         balance only -- hold the pole upright first
  stop        emergency stop (velocities to zero, motors stay energized)

  slow        crawl speeds -- the boot default, for bring-up and wiring checks.
              Will NOT balance: the cart can't accelerate under a falling pole.
  fast        full speeds -- what 'bal' and 'auto' actually need. Keep clear.

  off / on    de-energize / energize all three drivers.
              WARNING: the SFU1605 screws back-drive. 'off' with the rail
              raised will let it sink. Support it first.

  home        call the cart's current position x = 0
  zhome       call the current rail height z = 0
  zero        re-zero the encoder (pendulum hanging and still)
  mag         AS5600 magnet health (status + AGC)
  stat        one-shot state dump
  params      list every live-tunable parameter
  set k v     change one, e.g.  set leff 0.185   /   set pw 9
  get k       read one back
  rate hz     telemetry rate (default 100, try 250 for fast captures)
  log         path of the CSV being written right now
  q / quit    exit
"""


def repl(link):
    print(f"  connected: {link.port_name}")
    print(f"  logging:   {link.logpath}")
    link.send("params")
    time.sleep(0.4)
    print("  'help' for commands.\n")

    while True:
        try:
            raw = input(">>> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not raw:
            continue
        word = raw.split()[0]

        if word in ("q", "quit", "exit"):
            break
        elif word == "help":
            print(HELP)
        elif word == "log":
            print(f"  {link.logpath}")
        elif word == "data":
            dashboard(link, interactive=False, title="data")
        elif word == "control":
            link.send("manual")
            time.sleep(0.1)
            dashboard(link, interactive=True, title="manual control")
        elif word == "auto":
            link.send("auto")
            time.sleep(0.1)
            dashboard(link, interactive=True, title="swing-up + balance")
        elif word == "bal":
            link.send("bal")
            time.sleep(0.1)
            dashboard(link, interactive=True, title="balance")
        else:
            link.send(raw)          # everything else goes straight to the board
            time.sleep(0.15)


def main():
    ap = argparse.ArgumentParser(description="ESP32 cart-pole console")
    ap.add_argument("--port", help="serial device, e.g. /dev/cu.usbserial-0001")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--rate", type=int, default=100, help="telemetry Hz")
    ap.add_argument("--logdir", default="logs")
    args = ap.parse_args()

    if not args.port:
        found = [p.device for p in list_ports.comports()]
        if found:
            print("  ports seen: " + ", ".join(found))

    link = Link(args.port, args.baud, args.logdir)
    link.send(f"rate {args.rate}")
    try:
        repl(link)
    finally:
        print("  stopping.")
        link.close()


if __name__ == "__main__":
    main()
