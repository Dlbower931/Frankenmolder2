#!/usr/bin/env python3
"""
Frankenmolder - Extruder control page (heaters + screw drive).

Same look as the process overview, with lamp-style push buttons: the lamp in a
button is lit while the thing it controls is in that state (e.g. HEAT lit while
the zone is heating/holding, OFF lit while it is off).

The page only draws and reports taps; ROS publishing lives in extruder_gui_node.py,
which passes callbacks in and feeds set_data() the same dict as the dashboard.

Standalone preview with simulated data:
    python3 extruder_page.py [--size 800x480]
"""

import time
from collections import deque

from hmi import (C, FACE, LAMP, RPM_MAX, STATE_COLORS, TEMP_MAX, HmiCanvas, _valid,
                 temp_color)
from process_dashboard import blank_data

ZONE_TITLES = {"zone1": "ZONE 1  FEED", "zone2": "ZONE 2  TRANSITION",
               "zone3": "ZONE 3  METERING", "zone4": "ZONE 4  CHAMBER"}


class ExtruderPage(HmiCanvas):
    HIST_EVERY_S = 1.0

    def __init__(self, parent, on_setpoint_tap=None, on_zone_cmd=None, on_all_cmd=None,
                 on_rpm_tap=None, on_motor_cmd=None, tick_ms=250, **kw):
        """
        on_setpoint_tap(zone_id)    open the number pad for a zone setpoint
        on_zone_cmd(zone_id, cmd)   cmd is "ON" or "OFF"
        on_all_cmd(cmd)             same, for every zone
        on_rpm_tap()                open the number pad for screw RPM
        on_motor_cmd(cmd)           "ON" (start) or "OFF" (stop)
        """
        noop = lambda *a: None
        self.on_setpoint_tap = on_setpoint_tap or noop
        self.on_zone_cmd = on_zone_cmd or noop
        self.on_all_cmd = on_all_cmd or noop
        self.on_rpm_tap = on_rpm_tap or noop
        self.on_motor_cmd = on_motor_cmd or noop
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

    def _motor_on(self):
        return str(self.data["motor"].get("state", "")).upper() in ("ON", "RUN", "RUNNING")

    # ------------------------------------------------------------ drawing
    def render(self):
        d = self.data
        self.begin()
        self.background()
        self.title_bar("EXTRUDER", *self.heaters_ready_text(d))
        self.section(46, "BARREL HEATERS")
        for i, zone_id in enumerate(d["zones"]):
            x0 = 12 + i * 246
            self._zone_card(x0, 56, x0 + 238, 384, zone_id, d["zones"][zone_id])
        self.section(396, "CONTROLS")
        self._all_zones(d)
        self._drive(d)
        self.event_panel(762, 406, 988, 590, "COMMANDS")

    def _zone_card(self, x0, y0, x1, y1, zone_id, z):
        st = str(z.get("state") or "OFF").upper()
        act, setp = z.get("act"), z.get("set")
        fault = st == "FAULT" or (isinstance(act, float) and not _valid(act))
        self.panel(x0, y0, x1, y1, ZONE_TITLES.get(zone_id, zone_id.upper()), chip=st,
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
        self.text(x0 + 10, y0 + 32, "ACTUAL", 9, bold=True, anchor="w", fill=C["dim"])
        self.text(x1 - 40, y0 + 62, s, 34, mono=True, bold=True, anchor="e", fill=col)
        self.text(x1 - 12, y0 + 52, "°C", 12, anchor="e", fill=C["dim"])
        self.hbar(x0 + 10, x1 - 10, y0 + 92, 7,
                  act / TEMP_MAX if _valid(act) else None, temp_color(act),
                  marker=setp / TEMP_MAX if _valid(setp) else None)
        # setpoint (tap to edit)
        self.value_button(x0 + 8, y0 + 106, x1 - 8, y0 + 152, "SETPOINT", setp, "°C",
                          on_press=lambda: self.on_setpoint_tap(zone_id))
        # trend
        self.text(x0 + 10, y0 + 166, "TREND  2 MIN", 8, bold=True, anchor="w", fill=C["dim"])
        self.spark(x0 + 10, y0 + 174, x1 - 10, y0 + 216, self.hist[zone_id], setp)
        # pilot lamps
        cx, w = (x0 + x1) / 2, (x1 - x0) / 3
        self.indicator(cx - w, y0 + 230, "HEATING", st == "HEATING", LAMP["amber"], w=50)
        self.indicator(cx, y0 + 230, "AT TEMP", st == "PID", LAMP["green"], w=50)
        self.indicator(cx + w, y0 + 230, "FAULT", fault, LAMP["red"], w=50)
        # push buttons
        heating = st in ("HEATING", "PID", "HEAT")
        mid = (x0 + x1) / 2
        self.lamp_button(x0 + 8, y0 + 264, mid - 4, y1 - 8, "HEAT", FACE["start"],
                         lamp_on=heating, lamp_color=LAMP["green"],
                         on_press=lambda: self._zone_cmd(zone_id, "ON"))
        self.lamp_button(mid + 4, y0 + 264, x1 - 8, y1 - 8, "OFF", FACE["stop"],
                         lamp_on=not heating, lamp_color=LAMP["red"],
                         on_press=lambda: self._zone_cmd(zone_id, "OFF"))

    def _all_zones(self, d):
        x0, y0, x1, y1 = 12, 406, 236, 590
        zones = d["zones"].values()
        ready = all(z.get("state") == "PID" for z in zones)
        any_on = any(z.get("state") in ("HEATING", "PID", "HEAT") for z in zones)
        self.panel(x0, y0, x1, y1, "ALL ZONES")
        self.lamp_button(x0 + 8, y0 + 28, x0 + 108, y0 + 112, "ALL\nHEAT", FACE["start"],
                         lamp_on=any_on, lamp_color=LAMP["green"],
                         on_press=lambda: self._all_cmd("ON"), size=12)
        self.lamp_button(x0 + 116, y0 + 28, x1 - 8, y0 + 112, "ALL\nOFF", FACE["stop"],
                         lamp_on=not any_on, lamp_color=LAMP["red"],
                         on_press=lambda: self._all_cmd("OFF"), size=12)
        self.indicator((x0 + x1) / 2, y0 + 128, "HEAT READY", ready, LAMP["green"], w=120)

    def _drive(self, d):
        m = d["motor"]
        on = self._motor_on()
        ready = all(z.get("state") == "PID" for z in d["zones"].values())
        x0, y0, x1, y1 = 244, 406, 754, 590
        self.panel(x0, y0, x1, y1, "SCREW DRIVE", chip="RUN" if on else "STOP",
                   chip_color=C["ok"] if on else C["off"])
        self.lamp_button(x0 + 8, y0 + 28, x0 + 118, y1 - 8, "START", FACE["start"],
                         lamp_on=on, lamp_color=LAMP["green"],
                         on_press=lambda: self._motor_cmd("ON"), size=15)
        self.lamp_button(x0 + 126, y0 + 28, x0 + 236, y1 - 8, "STOP", FACE["stop"],
                         lamp_on=not on, lamp_color=LAMP["red"],
                         on_press=lambda: self._motor_cmd("OFF"), size=15)
        self.value_button(x0 + 246, y0 + 28, x0 + 386, y0 + 76, "SCREW RPM", m.get("rpm_set"),
                          "rpm", on_press=self.on_rpm_tap)
        self.indicator(x0 + 280, y0 + 98, "DRIVE ON", on, LAMP["green"], w=60)
        self.indicator(x0 + 352, y0 + 98, "COLD", on and not ready, LAMP["amber"], w=60)
        self.text(x0 + 316, y0 + 150, "COLD = drive running\nbefore heaters at temp", 8,
                  fill=C["dim"], justify="center")
        self.gauge(x1 - 58, y0 + 92, 40, m.get("rpm_act"), RPM_MAX, "RPM", "{:.1f}",
                   setp=m.get("rpm_set"))


def main():
    import argparse
    import tkinter as tk
    from process_dashboard import DemoSim

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--size", default="1024x600")
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
