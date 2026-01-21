#!/usr/bin/env python3
"""scribit_jog_cli.py

Terminal (curses) test program for Scribit
- Controls left/right cable length to change position on the wall
- Rotates the carousel for pen select/up/down
- Runs an HTTP server at TCP port 80 for GCODE download

Design goals
- Stateless / safe-anywhere: jogs in *motor/cable space* (dL, dR) so it does NOT
  require knowing current XY.
- No temp files: serves predictable endpoints /g/<CMD>.gcode from an in-memory cache.
- Strict firmware behavior support:
  - URL sent to robot must NOT include ':port' (so HTTP must bind on port 80).
  - MQTT 'print' payload MUST be "URL;<suffix>" (2 tokens separated by ';').
    The suffix is executed when the job stops; default is a harmless "G4 P0".

Controls
- Arrow keys / WASD : Up=both in, Down=both out, Left=leftish, Right=rightish
- Q / E             : left in/out
- Z / C             : right in/out
- J / K             : rotate pen carousel CCW/CW by 'step' degrees (relative Z)
- 1 / 2 / 3 / 4     : pen 1/2/3/4 DOWN (slot Z then G101)
- ! / @ / # / $     : pen 1/2/3/4 UP   (slot Z only)
- H                 : G77 (home Z-cylinder / carousel using hall sensor)
- [ / ]             : step down/up
- - / =             : feed down/up
- x                 : Reset N (stop)
- ESC / Ctrl+G      : quit

Notes
- 'step' is used as:
  - mm for cable jog (X/Y)
  - degrees for carousel rotation jog (Z)

Requirements:
  sudo apt install mosquitto-clients

Run (needs port 80 -> root):
  sudo python3 scribit_jog_cli.py \
    --robot-id 30aea4d9fc5c \
    --mqtt-host 192.168.240.2 \
    --host-ip 192.168.240.2 \
    --http-port 80
"""

from __future__ import annotations

import argparse
import subprocess
import threading
import time
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse
from typing import Optional

# ------------------------------------------------------------
# Motor-space jog mapping
# ------------------------------------------------------------
# Scribit open-firmware convention (from existing tools):
#   X = dL
#   Y = -dR
# So for desired right delta +dR, emit Y = -dR.
MOTOR_CMDS: dict[str, tuple[int, int]] = {
    "BOTH_IN":  (-1, -1),   # up-ish
    "BOTH_OUT": (+1, +1),   # down-ish
    "L_IN":     (-1,  0),
    "L_OUT":    (+1,  0),
    "R_IN":     ( 0, -1),
    "R_OUT":    ( 0, +1),

    # handy combos for horizontal-ish nudges:
    "LEFTISH":  (-1, +1),   # L in + R out
    "RIGHTISH": (+1, -1),   # L out + R in
}

# Carousel rotation (Z axis is used as carousel angle in this firmware)
CAROUSEL_CMDS = {"CAR_CCW", "CAR_CW"}

# Pen slot Z positions (from open firmware g101.h)
PEN_SLOT_Z = {
    1: 89,
    2: 161,
    3: 233,
    4: 305,
}
#PEN_SLOT_Z = {
#        1: 47,
#        2: 119,
#        3: 191,
#        4: 263,
#        }

# Pen commands we serve as separate endpoints
PEN_CMDS = {
    "P1_UP", "P2_UP", "P3_UP", "P4_UP",
    "P1_DOWN", "P2_DOWN", "P3_DOWN", "P4_DOWN",
}

SPECIAL_CMDS = {"G77"} | CAROUSEL_CMDS | PEN_CMDS


@dataclass(frozen=True)
class Key:
    step: float
    feed: int


class Cache:
    def __init__(self):
        self._lock = threading.Lock()
        self._cache: dict[tuple[Key, str], str] = {}

    def get_gcode(self, key: Key, cmd: str) -> str:
        k = (key, cmd)
        with self._lock:
            if k in self._cache:
                return self._cache[k]

            gcode = self._build_gcode(key, cmd)
            self._cache[k] = gcode
            return gcode

    def _build_gcode(self, key: Key, cmd: str) -> str:
        # 1) Motor/cable jogs
        if cmd in MOTOR_CMDS:
            sx, sy = MOTOR_CMDS[cmd]
            dL = sx * key.step
            dR = sy * key.step
            lines = [
                "G21",  # mm
                "G91",  # relative
                "M17",  # motors on
                f"G1 X{dL:.3f} Y{-dR:.3f} F{int(key.feed)}",
            ]
            return "\n".join(lines) + "\n"

        # 2) Carousel rotate (relative Z, degrees)
        if cmd in CAROUSEL_CMDS:
            dz = key.step if cmd == "CAR_CCW" else -key.step
            lines = [
                "G21",
                "G91",  # relative
                "M17",
                f"G1 Z{dz:.3f} F{int(key.feed)}",
            ]
            return "\n".join(lines) + "\n"
        # 3) Pen up/down is built dynamically (needs current carousel Z)
        #    so it is not handled by the cache.

        # 4) G77 home
        if cmd == "G77":
            lines = [
                "G21",
                "G90",
                "M17",
                "G77",
                "G92 Z-56",
            ]
            return "\n".join(lines) + "\n"

        raise ValueError(f"unknown cmd {cmd}")


CACHE = Cache()

# Current settings used by endpoints
CUR_LOCK = threading.Lock()
CURRENT = {"step": 2.0, "feed": 900}


def mosq_pub(host: str, port: int, user: str, pw: str, topic: str, msg: str) -> None:
    cmd = [
        "mosquitto_pub",
        "-h", host,
        "-p", str(port),
        "-u", user,
        "-P", pw,
        "-t", topic,
        "-m", msg,
    ]
    subprocess.run(cmd, check=True)


class App:
    def __init__(
        self,
        robot_id: str,
        mqtt_host: str,
        mqtt_port: int,
        mqtt_user: str,
        mqtt_pass: str,
        host_ip: str,
        http_port: int,
        suffix: str,
    ):
        self.robot_id = robot_id
        self.mqtt_host = mqtt_host
        self.mqtt_port = mqtt_port
        self.mqtt_user = mqtt_user
        self.mqtt_pass = mqtt_pass
        self.host_ip = host_ip
        self.http_port = http_port

        # Firmware requires the second ';token'. Default is harmless (doesn't release motors).
        self.suffix = suffix

        self.last = ""
        self.last_lock = threading.Lock()

        # Track carousel Z (degrees). None means unknown (not homed yet).
        self._z_lock = threading.Lock()
        self._z: Optional[float] = None

        # One-time nudge (non-fatal)
        try:
            self.ensure_idle()
            time.sleep(0.02)
        except Exception:
            pass

    def topic(self, suffix: str) -> str:
        return f"tin/{self.robot_id}/{suffix}"

    def ensure_idle(self) -> None:
        # Safe to spam; helps when firmware is in 'boot'/'waiting' state.
        mosq_pub(self.mqtt_host, self.mqtt_port, self.mqtt_user, self.mqtt_pass,
                 self.topic("status"), "{}")

    def reset_n(self) -> None:
        mosq_pub(self.mqtt_host, self.mqtt_port, self.mqtt_user, self.mqtt_pass,
                 self.topic("reset"), "N")
        with self.last_lock:
            self.last = "RESET N"


    # ----------------------------
    # Carousel/Z tracking helpers
    # ----------------------------
    def _z_get(self) -> Optional[float]:
        with self._z_lock:
            return self._z

    def _z_set(self, z: Optional[float]) -> None:
        with self._z_lock:
            self._z = z

    @staticmethod
    def _ccw_only_target(current_z: Optional[float], slot_z: float) -> float:
        """Return an absolute target that moves CCW-only by adding +360 as needed."""
        if current_z is None:
            # Best effort; user should press H (G77) once after boot.
            return float(slot_z)
        target = float(slot_z)
        while target < current_z:
            target += 360.0
        return target

    def build_pen_gcode(self, pen: int, is_down: bool, fz: int = 2000) -> str:
        slot = PEN_SLOT_Z[pen]
        cur = self._z_get()
        target = self._ccw_only_target(cur, slot)
        lines = [
            "G21",
            "G90",  # absolute
            "M17",
            f"G1 Z{target:.3f} F{int(fz)}",
        ]
        if is_down:
            lines += ["G101", "G101", "G101"]
        self._z_set(target)
        return "\n".join(lines) + "\n"

    def build_g77_gcode(self) -> str:
        lines = [
            "G21",
            "G90",
            "M17",
            "G77",
            "G92 Z-56",
        ]
        self._z_set(-56.0)
        return "\n".join(lines) + "\n"

    def publish_cmd(self, cmd: str) -> None:
        # Prepare HTTP endpoint content.
        with CUR_LOCK:
            key = Key(step=float(CURRENT["step"]), feed=int(CURRENT["feed"]))

        # Pen select + G77 are dynamic because we must enforce CCW-only moves.
        if cmd in PEN_CMDS:
            pen = int(cmd[1])
            is_down = cmd.endswith("DOWN")
            gcode = self.build_pen_gcode(pen=pen, is_down=is_down, fz=2000)
            Handler.set_dynamic_gcode(cmd, gcode)
        elif cmd == "G77":
            gcode = self.build_g77_gcode()
            Handler.set_dynamic_gcode(cmd, gcode)
        else:
            _ = CACHE.get_gcode(key, cmd)
            # Best-effort Z tracking for manual carousel jog.
            if cmd in CAROUSEL_CMDS:
                dz = key.step if cmd == "CAR_CCW" else -key.step
                cur = self._z_get()
                if cur is not None:
                    self._z_set(cur + dz)

        self.ensure_idle()
        time.sleep(0.02)

        # URL must not include port
        url = f"http://{self.host_ip}/g/{cmd}.gcode"
        payload = f"{url};{self.suffix}"

        # Stop then print
        self.reset_n()
        time.sleep(0.08)

        mosq_pub(self.mqtt_host, self.mqtt_port, self.mqtt_user, self.mqtt_pass,
                 self.topic("print"), payload)

        with self.last_lock:
            self.last = f"{cmd}  step={key.step:g}  feed={key.feed}  payload='{payload}'"


class Handler(BaseHTTPRequestHandler):
    app: App = None  # type: ignore
    _dyn_lock = threading.Lock()
    _dyn_gcode: dict[str, str] = {}

    @classmethod
    def set_dynamic_gcode(cls, cmd: str, gcode: str) -> None:
        with cls._dyn_lock:
            cls._dyn_gcode[cmd] = gcode

    def log_message(self, fmt: str, *args) -> None:
        return

    def _send(self, code: int, body: bytes, ctype: str) -> None:
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        u = urlparse(self.path)
        p = u.path

        if p == "/health":
            return self._send(200, b"ok\n", "text/plain; charset=utf-8")

        if p.startswith("/g/") and p.endswith(".gcode"):
            cmd = p.split("/")[-1].replace(".gcode", "")
            try:
                with Handler._dyn_lock:
                    if cmd in Handler._dyn_gcode:
                        gcode = Handler._dyn_gcode[cmd]
                        return self._send(200, gcode.encode("utf-8"), "text/plain; charset=utf-8")

                with CUR_LOCK:
                    key = Key(step=float(CURRENT["step"]), feed=int(CURRENT["feed"]))
                gcode = CACHE.get_gcode(key, cmd)
                return self._send(200, gcode.encode("utf-8"), "text/plain; charset=utf-8")
            except Exception as e:
                return self._send(400, f"bad request: {e}\n".encode("utf-8"),
                                  "text/plain; charset=utf-8")

        return self._send(404, b"not found\n", "text/plain; charset=utf-8")


def run_curses(app: App, step0: float, feed0: int):
    import curses

    step_choices = [0.5, 1.0, 2.0, 5.0, 10.0, 30.0, 50.0, 72.0, 90.0, 100.0, 200.0]
    feed_choices = [600, 900, 1200, 1800, 2400, 3000]

    def nearest(val, choices):
        return min(choices, key=lambda c: abs(c - val))

    step = nearest(step0, step_choices)
    feed = nearest(feed0, feed_choices)

    def apply_settings():
        with CUR_LOCK:
            CURRENT["step"] = step
            CURRENT["feed"] = feed

    apply_settings()

    help_lines = [
        "Arrow keys / WASD : Up=both in, Down=both out, Left=leftish, Right=rightish",
        "Q / E             : left in/out",
        "Z / C             : right in/out",
        "J / K             : carousel CCW/CW by step (relative Z)",
        "1 2 3 4           : pen 1-4 DOWN (slot Z + G101)",
        "! @ # $           : pen 1-4 UP   (slot Z)",
        "H                 : G77 (home Z-cylinder / carousel)",
        "[ / ]             : step down/up",
        "- / =             : feed down/up",
        "x                 : Reset N (stop)",
        "ESC / Ctrl+G      : quit",
        "",
        "NOTE: URL must NOT include ':port'. HTTP must bind on port 80.",
        "NOTE: MQTT print payload must be 'URL;<suffix>' (suffix default is 'G4 P0').",
    ]

    def draw(stdscr):
        stdscr.clear()
        stdscr.addstr(0, 0, "Scribit Jog CLI (motor-space + carousel/pen helpers)")
        stdscr.addstr(2, 0, f"HTTP health:  http://{app.host_ip}/health   (server binds :{app.http_port})")
        stdscr.addstr(3, 0, f"MQTT broker:  {app.mqtt_host}:{app.mqtt_port}   robot_id={app.robot_id}")
        stdscr.addstr(4, 0, f"Suffix:       ;{app.suffix}")

        stdscr.addstr(6, 0, f"step={step:g}   feed={feed}   (step is mm for cables, degrees for carousel)")
        with app.last_lock:
            last = app.last
        stdscr.addstr(7, 0, f"last: {last}")

        y = 9
        for line in help_lines:
            stdscr.addstr(y, 0, line)
            y += 1
        stdscr.refresh()

    def publish(cmd: str):
        app.publish_cmd(cmd)

    def step_down():
        nonlocal step
        i = step_choices.index(step)
        if i > 0:
            step = step_choices[i - 1]
            apply_settings()

    def step_up():
        nonlocal step
        i = step_choices.index(step)
        if i < len(step_choices) - 1:
            step = step_choices[i + 1]
            apply_settings()

    def feed_down():
        nonlocal feed
        i = feed_choices.index(feed)
        if i > 0:
            feed = feed_choices[i - 1]
            apply_settings()

    def feed_up():
        nonlocal feed
        i = feed_choices.index(feed)
        if i < len(feed_choices) - 1:
            feed = feed_choices[i + 1]
            apply_settings()

    def loop(stdscr):
        stdscr.nodelay(False)
        stdscr.keypad(True)
        curses.curs_set(0)

        while True:
            draw(stdscr)
            ch = stdscr.getch()

            # arrows
            if ch == curses.KEY_UP:
                publish("BOTH_IN")
            elif ch == curses.KEY_DOWN:
                publish("BOTH_OUT")
            elif ch == curses.KEY_LEFT:
                publish("LEFTISH")
            elif ch == curses.KEY_RIGHT:
                publish("RIGHTISH")

            # WASD
            elif ch in (ord('w'), ord('W')):
                publish("BOTH_IN")
            elif ch in (ord('s'), ord('S')):
                publish("BOTH_OUT")
            elif ch in (ord('a'), ord('A')):
                publish("LEFTISH")
            elif ch in (ord('d'), ord('D')):
                publish("RIGHTISH")

            # individual cable motors
            elif ch in (ord('q'), ord('Q')):
                publish("L_IN")
            elif ch in (ord('e'), ord('E')):
                publish("L_OUT")
            elif ch in (ord('z'), ord('Z')):
                publish("R_IN")
            elif ch in (ord('c'), ord('C')):
                publish("R_OUT")

            # carousel rotate
            elif ch in (ord('j'), ord('J')):
                publish("CAR_CCW")
            elif ch in (ord('k'), ord('K')):
                publish("CAR_CW")

            # pen up/down
            elif ch == ord('1'):
                publish("P1_DOWN")
            elif ch == ord('2'):
                publish("P2_DOWN")
            elif ch == ord('3'):
                publish("P3_DOWN")
            elif ch == ord('4'):
                publish("P4_DOWN")
            elif ch == ord('!'):
                publish("P1_UP")
            elif ch == ord('@'):
                publish("P2_UP")
            elif ch == ord('#'):
                publish("P3_UP")
            elif ch == ord('$'):
                publish("P4_UP")

            # G77 home
            elif ch == ord('H'):
                publish("G77")

            # step/feed
            elif ch == ord('['):
                step_down()
            elif ch == ord(']'):
                step_up()
            elif ch == ord('-'):
                feed_down()
            elif ch == ord('='):
                feed_up()

            # stop
            elif ch in (ord('x'), ord('X')):
                app.reset_n()

            # quit nicely
            elif ch in (27, 7):
                return

    curses.wrapper(loop)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot-id", required=True)
    ap.add_argument("--mqtt-host", required=True)
    ap.add_argument("--mqtt-port", type=int, default=1883)
    ap.add_argument("--mqtt-user", default="scribit")
    ap.add_argument("--mqtt-pass", default="scribit")

    ap.add_argument("--host-ip", required=True, help="PC IP that robot can reach for HTTP (no port in URL!)")
    ap.add_argument("--http-port", type=int, default=80, help="HTTP bind port (MUST be 80 for current firmware)")
    ap.add_argument("--suffix", default="G4 P0", help="Required suffix appended after URL (default 'G4 P0')")

    ap.add_argument("--step", type=float, default=2.0)
    ap.add_argument("--feed", type=int, default=900)

    args = ap.parse_args()

    if args.http_port != 80:
        print("ERROR: Your firmware rejects URLs with ':port'. Use --http-port 80 and run with sudo.")
        raise SystemExit(2)

    app = App(
        robot_id=args.robot_id,
        mqtt_host=args.mqtt_host,
        mqtt_port=args.mqtt_port,
        mqtt_user=args.mqtt_user,
        mqtt_pass=args.mqtt_pass,
        host_ip=args.host_ip,
        http_port=args.http_port,
        suffix=args.suffix,
    )

    Handler.app = app
    httpd = ThreadingHTTPServer(("0.0.0.0", args.http_port), Handler)
    th = threading.Thread(target=httpd.serve_forever, daemon=True)
    th.start()

    print(f"[scribit_jog_cli] HTTP listening on 0.0.0.0:{args.http_port} (robot fetches via http://{args.host_ip}/...)")
    print(f"[scribit_jog_cli] MQTT broker {args.mqtt_host}:{args.mqtt_port}  robot_id={args.robot_id}  suffix=;{args.suffix}")

    try:
        run_curses(app, step0=args.step, feed0=args.feed)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

