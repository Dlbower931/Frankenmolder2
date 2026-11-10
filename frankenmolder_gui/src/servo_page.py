#!/usr/bin/env python3
"""
Frankenmolder - AC servo test page.

Drives the XP200 servo through the ESP32's pulse/direction generator
(CAN 0x230-0x232, see esp32_firmware/ac_servo.h): pick a pulse frequency and
direction, RUN / STOP / E-STOP, or make an exact-length position move.

The page only draws and reports taps. The GUI node passes in:
    send_speed(signed_hz, accel)     -> /servo/target_hz + /servo/accel
    send_cmd(cmd)                    -> /servo/command  ("RUN", "STOP", "ESTOP", "ZERO")
    send_move(signed_pulses)         -> /servo/move_pulses
    ask_number(current, lo, hi, apply)   opens the number pad, calls apply(value)
and feeds set_data() a dict whose "servo" entry looks like blank_servo().

Standalone preview with simulated drive:  python3 servo_page.py [--size 800x480]
"""

import time

from hmi import C, FACE, LAMP, HmiCanvas, _valid

MAX_HZ = 200000          # firmware limit (ESP32 + FastAccelStepper)
MAX_ACCEL = 2000000
GEARBOX = 30.0           # screw gearbox ratio
PRESETS = (1000, 5000, 10000, 30000, 60000, 100000)
GEAR_OPTIONS = ((10000, "Pr029 = 1", "10000 /rev"), (2000, "Pr029 = 5", "2000 /rev"))


def blank_servo():
    return {"link": False, "state": None, "actual_hz": None, "position": None,
            "target_echo": None, "flags": []}


def fmt_hz(hz):
    hz = abs(hz)
    return f"{hz / 1000:.1f}k" if hz >= 1000 else f"{hz:.0f}"


class ServoTestPage(HmiCanvas):

    def __init__(self, parent, send_speed=None, send_cmd=None, send_move=None, ask_number=None,
                 tick_ms=150, **kw):
        noop = lambda *a: None
        self.send_speed = send_speed or noop
        self.send_cmd = send_cmd or noop
        self.send_move = send_move or noop
        self.ask_number = ask_number or noop
        # operator settings (live on the page, pushed to the ESP32 when changed)
        self.target_hz = 5000
        self.direction = 1
        self.accel = 20000
        self.move_pulses = 10000
        self.pulses_per_rev = 10000
        self.servo = blank_servo()
        super().__init__(parent, tick_ms=tick_ms, **kw)

    # ------------------------------------------------------------ data
    def set_data(self, data):
        self.servo = data.get("servo", blank_servo())

    # ------------------------------------------------------------ actions
    def _push_speed(self):
        self.send_speed(self.direction * self.target_hz, self.accel)

    def _set_target(self, hz):
        self.target_hz = int(max(0, min(MAX_HZ, hz)))
        self.log(f"TARGET {self.target_hz} Hz", "cmd")
        self._push_speed()

    def _set_dir(self, d):
        self.direction = d
        self.log("DIR FWD" if d > 0 else "DIR REV", "cmd")
        self._push_speed()

    def _set_accel(self, a):
        self.accel = int(max(100, min(MAX_ACCEL, a)))
        self.log(f"ACCEL {self.accel} Hz/s", "cmd")
        self._push_speed()

    def _set_pulses(self, p):
        self.move_pulses = int(max(1, p))

    def _cmd(self, cmd):
        if cmd == "RUN":
            self._push_speed()          # make sure the ESP32 has the current settings
        self.log(cmd if cmd != "ESTOP" else "E-STOP", "alarm" if cmd == "ESTOP" else "cmd")
        self.send_cmd(cmd)

    def _move(self, sign):
        self._push_speed()
        self.log(f"MOVE {'+' if sign > 0 else '-'}{self.move_pulses}", "cmd")
        self.send_move(sign * self.move_pulses)

    def _set_gear(self, ppr):
        self.pulses_per_rev = ppr

    # ------------------------------------------------------------ helpers
    def motor_rpm(self, hz):
        return abs(hz) * 60.0 / self.pulses_per_rev

    def _flag(self, name):
        return name in (self.servo.get("flags") or [])

    # ------------------------------------------------------------ drawing
    def render(self):
        sv = self.servo
        self.begin()
        self.background()
        link = sv.get("link")
        self.title_bar("SERVO TEST",
                       "ESP32 SERVO LINK OK" if link else "NO SERVO STATUS FROM ESP32",
                       C["ok"] if link else C["alarm"])
        self.section(46, "PULSE COMMAND")
        self._command_panel()
        self._drive_panel()
        self._actual_panel(sv)
        self.section(362, "POSITION / SETUP")
        self._move_panel()
        self._setup_panel()
        self.event_panel(708, 372, 988, 590, "COMMANDS")

    def _command_panel(self):
        x0, y0, x1, y1 = 12, 56, 420, 352
        self.panel(x0, y0, x1, y1, "FREQUENCY")
        self.value_button(x0 + 8, y0 + 26, x0 + 240, y0 + 78, "TARGET", self.target_hz, "Hz",
                          fmt="{:.0f}",
                          on_press=lambda: self.ask_number(self.target_hz, 0, MAX_HZ,
                                                           self._set_target))
        self.lamp_button(x0 + 246, y0 + 24, x0 + 324, y0 + 80, "FWD", FACE["blue"],
                         lamp_on=self.direction > 0, lamp_color=LAMP["blue"],
                         on_press=lambda: self._set_dir(1), size=12)
        self.lamp_button(x0 + 328, y0 + 24, x1 - 8, y0 + 80, "REV", FACE["blue"],
                         lamp_on=self.direction < 0, lamp_color=LAMP["blue"],
                         on_press=lambda: self._set_dir(-1), size=12)
        # presets
        self.text(x0 + 10, y0 + 92, "PRESETS", 8, bold=True, anchor="w", fill=C["dim"])
        bw = (x1 - x0 - 16) / len(PRESETS)
        for i, hz in enumerate(PRESETS):
            bx = x0 + 8 + i * bw
            self.lamp_button(bx, y0 + 100, bx + bw - 4, y0 + 152, fmt_hz(hz), FACE["neutral"],
                             lamp_on=self.target_hz == hz, lamp_color=LAMP["amber"],
                             on_press=lambda hz=hz: self._set_target(hz), size=11)
        # acceleration
        self.value_button(x0 + 8, y0 + 162, x0 + 240, y0 + 210, "ACCEL", self.accel, "Hz/s",
                          fmt="{:.0f}", size=17,
                          on_press=lambda: self.ask_number(self.accel, 100, MAX_ACCEL,
                                                           self._set_accel))
        ramp_s = self.target_hz / self.accel if self.accel else 0
        self.text(x0 + 252, y0 + 176, "RAMP 0 → TARGET", 8, bold=True, anchor="w", fill=C["dim"])
        self.text(x0 + 252, y0 + 196, f"{ramp_s:.2f} s", 16, mono=True, bold=True, anchor="w",
                  fill=C["value"])
        # what that frequency means at the motor and screw
        mrpm = self.motor_rpm(self.target_hz)
        self.readout(x0, x0 + 200, y0 + 236, "MOTOR", mrpm, "rpm", size=15)
        self.readout(x0 + 200, x1, y0 + 236, "SCREW", mrpm / GEARBOX, "rpm", "{:.2f}", size=15)
        warn = mrpm > 3000
        self.text(x0 + 10, y0 + 264,
                  "OVER 3000 MOTOR RPM - check drive limit (Pr075)" if warn else
                  f"at {self.pulses_per_rev} pulses/rev, {GEARBOX:.0f}:1 gearbox",
                  8, anchor="w", fill=C["warn"] if warn else C["dim"])
        self.text(x0 + 10, y0 + 280, "Changes apply live while running", 8, anchor="w",
                  fill=C["dim"])

    def _drive_panel(self):
        x0, y0, x1, y1 = 428, 56, 700, 352
        st = self.servo.get("state")
        moving = st in ("RUNNING", "MOVING", "STOPPING")
        self.panel(x0, y0, x1, y1, "DRIVE", chip=st or "---",
                   chip_color=C["ok"] if moving else C["alarm"] if st == "FAULT" else C["off"])
        mid = (x0 + x1) / 2
        self.lamp_button(x0 + 8, y0 + 26, mid - 4, y0 + 176, "RUN", FACE["start"],
                         lamp_on=st == "RUNNING", lamp_color=LAMP["green"],
                         on_press=lambda: self._cmd("RUN"), size=17)
        self.lamp_button(mid + 4, y0 + 26, x1 - 8, y0 + 176, "STOP", FACE["stop"],
                         lamp_on=not moving, lamp_color=LAMP["red"],
                         on_press=lambda: self._cmd("STOP"), size=17)
        self.lamp_button(x0 + 8, y0 + 184, x1 - 8, y1 - 34, "E-STOP", "#d1242f",
                         lamp_on=st == "FAULT", lamp_color=LAMP["amber"],
                         on_press=lambda: self._cmd("ESTOP"), size=17)
        self.text(mid, y1 - 18, "STOP ramps down · E-STOP cuts pulses instantly", 8,
                  fill=C["dim"])

    def _actual_panel(self, sv):
        x0, y0, x1, y1 = 708, 56, 988, 352
        self.panel(x0, y0, x1, y1, "ACTUAL")
        hz = sv.get("actual_hz")
        self.gauge(x0 + 72, y0 + 92, 50, abs(hz) / 1000 if _valid(hz) else None, MAX_HZ / 1000,
                   "kHz", "{:.1f}", setp=self.target_hz / 1000)
        pos = sv.get("position")
        self.readout(x0 + 136, x1, y0 + 44, "HZ", abs(hz) if _valid(hz) else None, "", "{:.0f}",
                     size=15)
        self.readout(x0 + 136, x1, y0 + 72, "DIR",
                     ("FWD" if hz >= 0 else "REV") if _valid(hz) and abs(hz) > 0 else None, "",
                     size=13)
        self.readout(x0 + 136, x1, y0 + 100, "MOTOR", self.motor_rpm(hz) if _valid(hz) else None,
                     "rpm", size=13)
        self.readout(x0, x1, y0 + 170, "POSITION", pos, "pls", "{:d}", size=15)
        self.readout(x0, x1, y0 + 194, "MOTOR REVS",
                     pos / self.pulses_per_rev if _valid(pos) else None, "rev", "{:.3f}", size=13)
        echo = sv.get("target_echo")
        self.readout(x0, x1, y0 + 218, "ESP32 TARGET", echo, "Hz", "{:.0f}", size=13,
                     color=C["setp"])
        cy = y0 + 240
        w = (x1 - x0) / 4
        alarm_wired = self._flag("ALARM_INPUT")
        self.indicator(x0 + w * 0.5, cy, "AT SPEED", self._flag("AT_SPEED"), LAMP["green"], w=48)
        self.indicator(x0 + w * 1.5, cy, "LINK", bool(sv.get("link")), LAMP["green"], w=48)
        self.indicator(x0 + w * 2.5, cy, "WATCHDOG", self._flag("WATCHDOG"), LAMP["amber"], w=48)
        self.indicator(x0 + w * 3.5, cy, "ALARM" if alarm_wired else "ALM N/C",
                       self._flag("ALARM"), LAMP["red"], w=48)

    def _move_panel(self):
        x0, y0, x1, y1 = 12, 372, 420, 590
        st = self.servo.get("state")
        self.panel(x0, y0, x1, y1, "POSITION MOVE")
        self.value_button(x0 + 8, y0 + 26, x0 + 240, y0 + 78, "PULSES", self.move_pulses, "pls",
                          fmt="{:.0f}",
                          on_press=lambda: self.ask_number(self.move_pulses, 1, 10000000,
                                                           self._set_pulses))
        revs = self.move_pulses / self.pulses_per_rev
        self.text(x0 + 252, y0 + 40, "MOTOR REVS", 8, bold=True, anchor="w", fill=C["dim"])
        self.text(x0 + 252, y0 + 62, f"{revs:.3f}", 16, mono=True, bold=True, anchor="w",
                  fill=C["value"])
        self.text(x0 + 252, y0 + 80, f"screw {revs / GEARBOX:.4f}", 8, mono=True, anchor="w",
                  fill=C["dim"])
        self.lamp_button(x0 + 8, y0 + 90, x0 + 136, y1 - 26, "MOVE\n+", FACE["blue"],
                         lamp_on=st == "MOVING", lamp_color=LAMP["blue"],
                         on_press=lambda: self._move(1), size=14)
        self.lamp_button(x0 + 140, y0 + 90, x0 + 268, y1 - 26, "MOVE\n−", FACE["blue"],
                         lamp_on=st == "MOVING", lamp_color=LAMP["blue"],
                         on_press=lambda: self._move(-1), size=14)
        self.lamp_button(x0 + 272, y0 + 90, x1 - 8, y1 - 26, "ZERO\nPOS", FACE["neutral"],
                         lamp_on=self.servo.get("position") == 0, lamp_color=LAMP["white"],
                         on_press=lambda: self._cmd("ZERO"), size=13)
        self.text(x0 + 10, y1 - 12, "Moves use the target frequency + accel, from standstill",
                  8, anchor="w", fill=C["dim"])

    def _setup_panel(self):
        x0, y0, x1, y1 = 428, 372, 700, 590
        self.panel(x0, y0, x1, y1, "ELECTRONIC GEAR (DISPLAY)")
        mid = (x0 + x1) / 2
        for i, (ppr, a, b) in enumerate(GEAR_OPTIONS):
            bx0 = x0 + 8 if i == 0 else mid + 4
            bx1 = mid - 4 if i == 0 else x1 - 8
            self.lamp_button(bx0, y0 + 26, bx1, y0 + 120, f"{a}\n{b}", FACE["neutral"],
                             lamp_on=self.pulses_per_rev == ppr, lamp_color=LAMP["amber"],
                             on_press=lambda ppr=ppr: self._set_gear(ppr), size=11)
        lines = ("Only changes the RPM maths on this page.",
                 "Set Pr029/Pr030 on the drive to match.",
                 "Limits: ESP32 200 kHz · drive 500 kHz (diff).",
                 "Drive is always enabled (SON wired on).")
        for i, t in enumerate(lines):
            self.text(x0 + 10, y0 + 140 + i * 17, t, 8, anchor="w",
                      fill=C["warn"] if i == 3 else C["dim"])


# ---------------------------------------------------------------- demo
class DemoServo:
    """Fake ESP32 servo controller so the page can be previewed."""

    def __init__(self):
        self.hz = 0.0
        self.target = 0.0
        self.accel = 20000.0
        self.mode = "IDLE"
        self.pos = 0.0
        self.move_left = 0
        self.last = time.time()

    def speed(self, hz, accel):
        self.target, self.accel = hz, accel

    def cmd(self, c):
        if c == "RUN":
            self.mode = "RUNNING"
        elif c == "STOP":
            self.mode = "STOPPING"
        elif c == "ESTOP":
            self.mode, self.hz = "IDLE", 0.0
        elif c == "ZERO" and self.hz == 0:
            self.pos = 0

    def move(self, pulses):
        if self.hz == 0:
            self.mode, self.move_left = "MOVING", pulses

    def step(self):
        now = time.time()
        dt, self.last = now - self.last, now
        goal = self.target if self.mode == "RUNNING" else 0.0
        if self.mode == "MOVING":
            goal = abs(self.target) * (1 if self.move_left > 0 else -1)
        dv = self.accel * dt
        self.hz += max(-dv, min(dv, goal - self.hz))
        self.pos += self.hz * dt
        if self.mode == "MOVING":
            self.move_left -= self.hz * dt
            if abs(self.move_left) < abs(self.hz * dt) + 1:
                self.mode, self.hz, self.move_left = "IDLE", 0.0, 0
        if self.mode == "STOPPING" and self.hz == 0:
            self.mode = "IDLE"
        state = self.mode if (self.hz != 0 or self.mode == "MOVING") else "IDLE"
        at_speed = self.mode == "RUNNING" and abs(self.hz - self.target) < max(1, abs(self.target) * 0.01)
        return {"servo": {"link": True, "state": state, "actual_hz": self.hz,
                          "position": int(self.pos), "target_echo": self.target,
                          "flags": ["AT_SPEED"] if at_speed else []}}


def main():
    import argparse
    import tkinter as tk

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--size", default="1024x600")
    args = ap.parse_args()
    root = tk.Tk()
    root.title("Frankenmolder - Servo test")
    root.geometry(args.size + "+40+60")
    sim = DemoServo()
    page = ServoTestPage(root, send_speed=sim.speed, send_cmd=sim.cmd, send_move=sim.move)
    page.pack(fill="both", expand=True)

    def pump():
        page.set_data(sim.step())
        root.after(100, pump)

    root.bind("<Escape>", lambda e: root.destroy())
    pump()
    root.mainloop()


if __name__ == "__main__":
    main()
