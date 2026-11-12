#!/usr/bin/env python3
"""
Frankenmolder - Extruder control page: extruder barrel heaters Z1-Z3 + screw drive.
(Z4, the injection barrel heater, lives on the Injection tab - injection_page.py.)

The screw drive is the AC servo on the ESP32 (see esp32_firmware/ac_servo.h); the CAN
bridge turns screw rpm + direction into the servo's pulse frequency.

Same look as the process overview, with lamp-style push buttons: the lamp in a
button is lit while the thing it controls is in that state (e.g. HEAT lit while
the zone is heating/holding, OFF lit while it is off).

The page only draws and reports taps; ROS publishing lives in extruder_gui_node.py,
which passes callbacks in and feeds set_data() the same dict as the dashboard.

Standalone preview with simulated data:
    python3 extruder_page.py [--size 1024x600]
"""

import time
from collections import deque

from hmi import (C, EXTRUDER_ZONES, FACE, LAMP, STATE_COLORS, TEMP_MAX, HmiCanvas, _valid,
                 temp_color)
from process_dashboard import blank_data

ZONE_TITLES = {"zone1": "Z1 FEED", "zone2": "Z2 TRANSITION",
               "zone3": "Z3 METERING", "zone4": "Z4 INJECTION BARREL"}


class ExtruderPage(HmiCanvas):
    HIST_EVERY_S = 1.0
    ZONE_IDS = EXTRUDER_ZONES      # zones shown on this page
    HEAT_LABEL = "EXTRUDER HEATERS"

    def __init__(self, parent, on_setpoint_tap=None, on_zone_cmd=None, on_all_cmd=None,
                 on_rpm_tap=None, on_motor_cmd=None, on_direction_cmd=None, tick_ms=250, **kw):
        """
        on_setpoint_tap(zone_id)    open the number pad for a zone setpoint
        on_zone_cmd(zone_id, cmd)   cmd is "ON" or "OFF"
        on_all_cmd(cmd)             same, for every zone on this page (Z1-Z3)
        on_rpm_tap()                open the number pad for screw RPM
        on_motor_cmd(cmd)           "ON" (start) or "OFF" (stop)
        on_direction_cmd(dir)       "FWD" or "REV" (only offered while the screw is stopped);
                                    returns False if it could not be sent
        """
        noop = lambda *a: None
        self.on_setpoint_tap = on_setpoint_tap or noop
        self.on_zone_cmd = on_zone_cmd or noop
        self.on_all_cmd = on_all_cmd or noop
        self.on_rpm_tap = on_rpm_tap or noop
        self.on_motor_cmd = on_motor_cmd or noop
        self.on_direction_cmd = on_direction_cmd or noop
        self.data = blank_data()
        self.hist = {z: deque(maxlen=120) for z in self.data["zones"]}
        self._last_hist = 0.0
        super().__init__(parent, tick_ms=tick_ms, **kw)

    # ------------------------------------------------------------ public API
    def set_data(self, data):
        self.data = data
        now = time.time()
        if now - self._last_hist >= self.HIST_EVERY_S:
            self._last_hist = now
            for z, zd in data["zones"].items():
                self.hist[z].append(zd.get("act"))

    # ------------------------------------------------------------ taps
    def _zone_cmd(self, zone_id, cmd):
        self.log(f"Z{zone_id[-1]} {'HEAT' if cmd == 'ON' else 'OFF'} sent", "cmd")
        self.on_zone_cmd(zone_id, cmd)

    def _all_cmd(self, cmd):
        self.log(f"ALL {'HEAT' if cmd == 'ON' else 'OFF'} sent", "cmd")
        self.on_all_cmd(cmd)

    def _motor_cmd(self, cmd):
        self.log(f"DRIVE {'START' if cmd == 'ON' else 'STOP'} sent", "cmd")
        self.on_motor_cmd(cmd)

    def _direction_cmd(self, d):
        if self._motor_moving():
            self.log("DIR LOCKED - STOP first", "alarm")
            return
        if self.on_direction_cmd(d) is not False:
            self.data["motor"]["direction"] = d   # show it now; next set_data confirms
            self.log(f"DIRECTION {d}", "cmd")

    def _motor_on(self):
        return str(self.data["motor"].get("state") or "").upper() in ("ON", "RUN", "RUNNING")

    def _motor_moving(self):
        """Pulses going out (running or still ramping down)."""
        return str(self.data["motor"].get("state") or "").upper() in (
            "ON", "RUN", "RUNNING", "MOVING", "STOPPING")

    # ------------------------------------------------------------ drawing
    # Virtual space is hmi.VW x VH (1000x525 = the 7" screen's canvas at 0.8 scale).
    CARD_Y = (44, 370)     # zone cards
    CTRL_Y = (378, 520)    # bottom control row

    def render(self):
        d = self.data
        self.begin()
        self.background()
        self.title_bar(*self.heaters_ready_text(d, self.ZONE_IDS, self.HEAT_LABEL))
        y0, y1 = self.CARD_Y
        n, gap = len(self.ZONE_IDS), 8
        w = (984 - gap * (n - 1)) / n
        for i, zone_id in enumerate(self.ZONE_IDS):
            x0 = 8 + i * (w + gap)
            self._zone_card(x0, y0, x0 + w, y1, zone_id, d["zones"][zone_id])
        self._all_zones(d)
        self._drive(d)
        self.event_panel(760, self.CTRL_Y[0], 992, self.CTRL_Y[1], "COMMANDS")

    def _zone_card(self, x0, y0, x1, y1, zone_id, z):
        st = str(z.get("state") or "OFF").upper()
        act, setp = z.get("act"), z.get("set")
        fault = st == "FAULT" or (isinstance(act, float) and not _valid(act))
        self.panel(x0, y0, x1, y1, ZONE_TITLES.get(zone_id, zone_id.upper()),
                   chip="HEAT" if st == "HEATING" else st,
                   chip_color=STATE_COLORS.get(st, C["warn"]))
        # actual temperature, big
        if fault:
            s, col = "FAULT", C["alarm"]
        elif _valid(act):
            s = f"{act:.1f}"
            col = (C["alarm"] if _valid(setp) and act > setp + 15 else
                   C["ok"] if st == "PID" else C["warn"] if st == "HEATING" else C["value"])
        else:
            s, col = "---", C["dim"]
        self.text(x0 + 10, y0 + 46, "ACTUAL", self.T_LABEL, bold=True, anchor="w", fill=C["dim"])
        self.text(x1 - 36, y0 + 72, s, 40, mono=True, bold=True, anchor="e", fill=col)
        self.text(x1 - 10, y0 + 64, "°C", 15, anchor="e", fill=C["dim"])
        self.hbar(x0 + 10, x1 - 10, y0 + 96, 8,
                  act / TEMP_MAX if _valid(act) else None, temp_color(act),
                  marker=setp / TEMP_MAX if _valid(setp) else None)
        # setpoint (tap to edit)
        self.value_button(x0 + 8, y0 + 112, x1 - 8, y0 + 172, "SETPOINT", setp, "°C",
                          on_press=lambda: self.on_setpoint_tap(zone_id))
        # trend
        self.text(x0 + 10, y0 + 186, "LAST 2 MIN", self.T_UNIT, bold=True, anchor="w",
                  fill=C["dim"])
        self.spark(x0 + 10, y0 + 196, x1 - 10, y0 + 240, self.hist[zone_id], setp)
        # push buttons (the lamp shows which state the zone is in)
        heating = st in ("HEATING", "PID", "HEAT")
        mid = (x0 + x1) / 2
        self.lamp_button(x0 + 8, y0 + 250, mid - 4, y1 - 6, "HEAT", FACE["start"],
                         lamp_on=heating, lamp_color=LAMP["green"],
                         on_press=lambda: self._zone_cmd(zone_id, "ON"))
        self.lamp_button(mid + 4, y0 + 250, x1 - 8, y1 - 6, "OFF", FACE["stop"],
                         lamp_on=not heating, lamp_color=LAMP["red"],
                         on_press=lambda: self._zone_cmd(zone_id, "OFF"))

    def _all_zones(self, d):
        x0, x1 = 8, 256
        y0, y1 = self.CTRL_Y
        zones = [d["zones"][z] for z in self.ZONE_IDS]
        ready = all(z.get("state") == "PID" for z in zones)
        any_on = any(z.get("state") in ("HEATING", "PID", "HEAT") for z in zones)
        self.panel(x0, y0, x1, y1, "EXTRUDER Z1-Z3", chip="READY" if ready else "COLD",
                   chip_color=C["ok"] if ready else C["off"])
        mid = (x0 + x1) / 2
        self.lamp_button(x0 + 8, y0 + 36, mid - 4, y1 - 6, "ALL\nHEAT", FACE["start"],
                         lamp_on=any_on, lamp_color=LAMP["green"],
                         on_press=lambda: self._all_cmd("ON"))
        self.lamp_button(mid + 4, y0 + 36, x1 - 8, y1 - 6, "ALL\nOFF", FACE["stop"],
                         lamp_on=not any_on, lamp_color=LAMP["red"],
                         on_press=lambda: self._all_cmd("OFF"))

    def _drive(self, d):
        m = d["motor"]
        st = str(m.get("state") or "").upper()
        on = self._motor_on()
        moving = self._motor_moving()
        link = m.get("link", True)
        flags = m.get("flags") or []
        direction = m.get("direction", "FWD")
        ready = all(z.get("state") == "PID" for z in d["zones"].values())
        if not link:
            chip, chip_col = "NO LINK", C["alarm"]
        elif st == "FAULT" or m.get("alarm_code"):
            code = m.get("alarm_code")
            chip, chip_col = (f"DRIVE ALM {code}" if code else "FAULT"), C["alarm"]
        elif on and not ready:   # turning before the barrel is at temperature
            chip, chip_col = "RUN COLD", C["warn"]
        elif on:
            chip, chip_col = "RUN", C["ok"]
        elif st == "STOPPING":
            chip, chip_col = "STOPPING", C["warn"]
        else:
            chip, chip_col = "STOP", C["off"]
        if "WATCHDOG" in flags and not moving:
            chip, chip_col = "CAN TIMEOUT", C["warn"]
        x0, x1 = 264, 752
        y0, y1 = self.CTRL_Y
        by0, by1 = y0 + 36, y1 - 6
        self.panel(x0, y0, x1, y1, "SCREW DRIVE", chip=chip, chip_color=chip_col)
        self.lamp_button(x0 + 8, by0, x0 + 96, by1, "START", FACE["start"],
                         lamp_on=on, lamp_color=LAMP["green"],
                         on_press=lambda: self._motor_cmd("ON"), size=18)
        self.lamp_button(x0 + 102, by0, x0 + 190, by1, "STOP", FACE["stop"],
                         lamp_on=not moving, lamp_color=LAMP["red"],
                         on_press=lambda: self._motor_cmd("OFF"), size=18)
        # direction: locked while the screw turns
        for i, dname in enumerate(("FWD", "REV")):
            bx = x0 + 198 + i * 68
            self.lamp_button(bx, by0, bx + 64, by1, dname, FACE["blue"],
                             lamp_on=direction == dname, lamp_color=LAMP["blue"],
                             enabled=not moving, size=15,
                             on_press=lambda dname=dname: self._direction_cmd(dname))
        self.value_button(x0 + 338, by0, x1 - 8, y0 + 94, "SET RPM", m.get("rpm_set"),
                          "rpm", size=22, on_press=self.on_rpm_tap)
        self.readout(x0 + 334, x1 - 4, y0 + 116, "ACT", m.get("rpm_act"), "rpm", size=20,
                     color=C["ok"] if on else C["value"])


def main():
    import argparse
    import tkinter as tk
    from process_dashboard import DemoSim

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--size", default="800x420", help="800x420 = 7in screen minus tabs/status")
    args = ap.parse_args()
    root = tk.Tk()
    root.title("Frankenmolder - Extruder")
    root.geometry(args.size + "+40+60")
    page = ExtruderPage(root, on_setpoint_tap=lambda z: page.log(f"tap {z} setpoint"),
                        on_rpm_tap=lambda: page.log("tap rpm"))
    page.pack(fill="both", expand=True)
    sim = DemoSim()

    def pump():
        page.set_data(sim.step())
        root.after(250, pump)

    root.bind("<Escape>", lambda e: root.destroy())
    pump()
    root.mainloop()


if __name__ == "__main__":
    main()
