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

Standalone preview with simulated drive:  python3 servo_page.py [--size 1024x600]
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
    if hz >= 1000:
        return f"{hz // 1000:.0f}k" if hz % 1000 == 0 else f"{hz / 1000:.1f}k"
    return f"{hz:.0f}"


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
    # Virtual space is hmi.VW x VH (1000x525 = the 7" screen's canvas at 0.8 scale).
    TOP_Y = (44, 316)
    BOT_Y = (324, 520)

    def render(self):
        sv = self.servo
        self.begin()
        self.background()
        link = sv.get("link")
        self.title_bar("SERVO LINK OK" if link else "NO SERVO STATUS FROM ESP32",
                       C["ok"] if link else C["alarm"])
        self._command_panel()
        self._drive_panel()
        self._actual_panel(sv)
        self._move_panel()
        self._setup_panel()
        self.event_panel(680, self.BOT_Y[0], 992, self.BOT_Y[1], "COMMANDS")

    def _command_panel(self):
        x0, x1 = 8, 416
        y0, y1 = self.TOP_Y
        mrpm = self.motor_rpm(self.target_hz)
        warn = mrpm > 3000
        self.panel(x0, y0, x1, y1, "FREQUENCY", chip="OVER 3000 RPM" if warn else None,
                   chip_color=C["warn"])
        self.value_button(x0 + 8, y0 + 34, x0 + 236, y0 + 94, "TARGET", self.target_hz, "Hz",
                          fmt="{:.0f}",
                          on_press=lambda: self.ask_number(self.target_hz, 0, MAX_HZ,
                                                           self._set_target))
        self.lamp_button(x0 + 244, y0 + 32, x0 + 324, y0 + 96, "FWD", FACE["blue"],
                         lamp_on=self.direction > 0, lamp_color=LAMP["blue"],
                         on_press=lambda: self._set_dir(1), size=16)
        self.lamp_button(x0 + 328, y0 + 32, x1 - 8, y0 + 96, "REV", FACE["blue"],
                         lamp_on=self.direction < 0, lamp_color=LAMP["blue"],
                         on_press=lambda: self._set_dir(-1), size=16)
        # presets
        bw = (x1 - x0 - 16) / len(PRESETS)
        for i, hz in enumerate(PRESETS):
            bx = x0 + 8 + i * bw
            self.lamp_button(bx, y0 + 104, bx + bw - 4, y0 + 160, fmt_hz(hz), FACE["neutral"],
                             lamp_on=self.target_hz == hz, lamp_color=LAMP["amber"],
                             on_press=lambda hz=hz: self._set_target(hz), size=15)
        # acceleration
        self.value_button(x0 + 8, y0 + 168, x0 + 236, y0 + 226, "ACCEL", self.accel, "Hz/s",
                          fmt="{:.0f}", size=22,
                          on_press=lambda: self.ask_number(self.accel, 100, MAX_ACCEL,
                                                           self._set_accel))
        ramp_s = self.target_hz / self.accel if self.accel else 0
        self.cell(x0 + 252, y0 + 180, "RAMP TIME", ramp_s, "s", "{:.2f}")
        # what that frequency means at the motor and screw
        self.cell(x0 + 12, y0 + 238, "MOTOR", mrpm, "rpm", "{:.0f}",
                  color=C["warn"] if warn else None)
        self.cell(x0 + 180, y0 + 238, "SCREW", mrpm / GEARBOX, "rpm", "{:.2f}")

    def _drive_panel(self):
        x0, x1 = 424, 672
        y0, y1 = self.TOP_Y
        st = self.servo.get("state")
        moving = st in ("RUNNING", "MOVING", "STOPPING")
        self.panel(x0, y0, x1, y1, "DRIVE", chip=st or "---",
                   chip_color=C["ok"] if moving else C["alarm"] if st == "FAULT" else C["off"])
        mid = (x0 + x1) / 2
        self.lamp_button(x0 + 8, y0 + 34, mid - 4, y0 + 150, "RUN", FACE["start"],
                         lamp_on=st == "RUNNING", lamp_color=LAMP["green"],
                         on_press=lambda: self._cmd("RUN"), size=22)
        self.lamp_button(mid + 4, y0 + 34, x1 - 8, y0 + 150, "STOP", FACE["stop"],
                         lamp_on=not moving, lamp_color=LAMP["red"],
                         on_press=lambda: self._cmd("STOP"), size=22)
        self.lamp_button(x0 + 8, y0 + 158, x1 - 8, y1 - 30, "E-STOP", "#d1242f",
                         lamp_on=st == "FAULT", lamp_color=LAMP["amber"],
                         on_press=lambda: self._cmd("ESTOP"), size=22)
        self.text(mid, y1 - 14, "STOP ramps · E-STOP is instant", 13, fill=C["dim"])

    def _actual_panel(self, sv):
        x0, x1 = 680, 992
        y0, y1 = self.TOP_Y
        self.panel(x0, y0, x1, y1, "ACTUAL")
        hz = sv.get("actual_hz")
        self.gauge(x0 + 66, y0 + 84, 44, abs(hz) / 1000 if _valid(hz) else None, MAX_HZ / 1000,
                   "kHz", "{:.1f}", setp=self.target_hz / 1000)
        pos = sv.get("position")
        self.readout(x0 + 124, x1, y0 + 50, "HZ", abs(hz) if _valid(hz) else None, "",
                     "{:.0f}", size=20)
        self.readout(x0 + 124, x1, y0 + 80, "DIR",
                     ("FWD" if hz >= 0 else "REV") if _valid(hz) and abs(hz) > 0 else None, "",
                     size=20)
        self.readout(x0 + 124, x1, y0 + 110, "MOTOR",
                     self.motor_rpm(hz) if _valid(hz) else None, "rpm", "{:.0f}", size=20)
        self.readout(x0, x1, y0 + 152, "POSITION", pos, "pls", "{:d}", size=20)
        self.readout(x0, x1, y0 + 180, "REVS", pos / self.pulses_per_rev if _valid(pos) else None,
                     "rev", "{:.3f}", size=20)
        self.readout(x0, x1, y0 + 208, "ESP32 TGT", sv.get("target_echo"), "Hz", "{:.0f}",
                     size=20, color=C["setp"])
        cy = y0 + 232
        w = (x1 - x0) / 5
        alarm_wired = self._flag("ALARM_INPUT")
        self.indicator(x0 + w * 0.5, cy, "SON", self._flag("SON"), LAMP["green"], w=44)
        self.indicator(x0 + w * 1.5, cy, "AT SPD", self._flag("AT_SPEED"), LAMP["green"], w=44)
        self.indicator(x0 + w * 2.5, cy, "LINK", bool(sv.get("link")), LAMP["green"], w=44)
        self.indicator(x0 + w * 3.5, cy, "WDOG", self._flag("WATCHDOG"), LAMP["amber"], w=44)
        self.indicator(x0 + w * 4.5, cy, "ALARM" if alarm_wired else "ALM N/C",
                       self._flag("ALARM"), LAMP["red"], w=44)

    def _move_panel(self):
        x0, x1 = 8, 416
        y0, y1 = self.BOT_Y
        st = self.servo.get("state")
        self.panel(x0, y0, x1, y1, "POSITION MOVE", chip="FROM STANDSTILL", chip_color=C["off"])
        self.value_button(x0 + 8, y0 + 34, x0 + 236, y0 + 94, "PULSES", self.move_pulses, "pls",
                          fmt="{:.0f}",
                          on_press=lambda: self.ask_number(self.move_pulses, 1, 10000000,
                                                           self._set_pulses))
        revs = self.move_pulses / self.pulses_per_rev
        self.cell(x0 + 252, y0 + 44, "MOTOR REVS", revs, "", "{:.3f}")
        self.text(x0 + 252, y0 + 88, f"screw {revs / GEARBOX:.4f}", 13, mono=True, anchor="w",
                  fill=C["dim"])
        self.lamp_button(x0 + 8, y0 + 102, x0 + 136, y1 - 6, "MOVE +", FACE["blue"],
                         lamp_on=st == "MOVING", lamp_color=LAMP["blue"],
                         on_press=lambda: self._move(1))
        self.lamp_button(x0 + 140, y0 + 102, x0 + 268, y1 - 6, "MOVE −", FACE["blue"],
                         lamp_on=st == "MOVING", lamp_color=LAMP["blue"],
                         on_press=lambda: self._move(-1))
        self.lamp_button(x0 + 272, y0 + 102, x1 - 8, y1 - 6, "ZERO", FACE["neutral"],
                         lamp_on=self.servo.get("position") == 0, lamp_color=LAMP["white"],
                         on_press=lambda: self._cmd("ZERO"))

    def _setup_panel(self):
        x0, x1 = 424, 672
        y0, y1 = self.BOT_Y
        self.panel(x0, y0, x1, y1, "PULSES / REV")
        mid = (x0 + x1) / 2
        for i, (ppr, a, b) in enumerate(GEAR_OPTIONS):
            bx0 = x0 + 8 if i == 0 else mid + 4
            bx1 = mid - 4 if i == 0 else x1 - 8
            self.lamp_button(bx0, y0 + 34, bx1, y0 + 116, f"{b.split()[0]}\n{a}", FACE["neutral"],
                             lamp_on=self.pulses_per_rev == ppr, lamp_color=LAMP["amber"],
                             on_press=lambda ppr=ppr: self._set_gear(ppr), size=15)
        lines = ("Display maths only - set",
                 "Pr029/Pr030 on the drive.",
                 "Max 200 kHz (24 V inputs)",
                 "SON: on at RUN/MOVE, off at STOP")
        for i, t in enumerate(lines):
            self.text(x0 + 10, y0 + 132 + i * 16, t, 13, anchor="w",
                      fill=C["warn"] if i == 3 else C["dim"])


# ---------------------------------------------------------------- demo
class DemoServo:
    """Fake ESP32 servo controller so the page can be previewed."""

    def __init__(self):
        self.hz = 0.0
        self.target = 0.0
        self.accel = 20000.0
        self.mode = "IDLE"
        self.son = False
        self.pos = 0.0
        self.move_left = 0
        self.last = time.time()

    def speed(self, hz, accel):
        self.target, self.accel = hz, accel

    def cmd(self, c):
        if c == "RUN":
            self.mode, self.son = "RUNNING", True
        elif c == "STOP":
            self.mode = "STOPPING"
        elif c == "ESTOP":
            self.mode, self.hz, self.son = "IDLE", 0.0, False
        elif c == "ZERO" and self.hz == 0:
            self.pos = 0

    def move(self, pulses):
        if self.hz == 0:
            self.mode, self.move_left, self.son = "MOVING", pulses, True

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
            self.mode, self.son = "IDLE", False
        state = self.mode if (self.hz != 0 or self.mode == "MOVING") else "IDLE"
        at_speed = self.mode == "RUNNING" and abs(self.hz - self.target) < max(1, abs(self.target) * 0.01)
        return {"servo": {"link": True, "state": state, "actual_hz": self.hz,
                          "position": int(self.pos), "target_echo": self.target,
                          "flags": (["AT_SPEED"] if at_speed else []) + (["SON"] if self.son else [])}}


def main():
    import argparse
    import tkinter as tk

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--size", default="800x420", help="800x420 = 7in screen minus tabs/status")
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
