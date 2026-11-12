#!/usr/bin/env python3
"""
Frankenmolder - Injection page: the injection barrel heater (Z4) and, once the plunger
position sensor is fitted, shot metering.

Reuses the Extruder page's zone card and taps; extruder_gui_node.py passes the same
callbacks (setpoint pad, HEAT/OFF) and feeds set_data() the dashboard dict.

Standalone preview with simulated data:  python3 injection_page.py [--size 1024x600]
"""

from extruder_page import ExtruderPage
from hmi import C, INJECTION_ZONES, _valid


class InjectionPage(ExtruderPage):
    ZONE_IDS = INJECTION_ZONES
    HEAT_LABEL = "INJ BARREL"
    CARD = (8, 44, 330, 370)

    def __init__(self, parent, on_setpoint_tap=None, on_zone_cmd=None, tick_ms=250, **kw):
        super().__init__(parent, on_setpoint_tap=on_setpoint_tap, on_zone_cmd=on_zone_cmd,
                         tick_ms=tick_ms, **kw)

    def render(self):
        d = self.data
        self.begin()
        self.background()
        self.title_bar(*self.heaters_ready_text(d, self.ZONE_IDS, self.HEAT_LABEL))
        x0, y0, x1, y1 = self.CARD
        self._zone_card(x0, y0, x1, y1, "zone4", d["zones"]["zone4"])
        self._shot_panel(d)
        self._melt_ready(d)
        self.event_panel(338, self.CTRL_Y[0], 992, self.CTRL_Y[1], "COMMANDS")

    def _shot_panel(self, d):
        inj = d.get("injection", {})
        wired = inj.get("wired")
        x0, y0, x1, y1 = 338, 44, 992, 370
        self.panel(x0, y0, x1, y1, "SHOT METERING", wired=wired)
        cy = y0 + 56
        self.cell(x0 + 16, cy, "PLUNGER", inj.get("plunger") * 100 if _valid(inj.get("plunger"))
                  else None, "%", "{:.0f}", wired=wired)
        self.cell(x0 + 180, cy, "SHOT WT", inj.get("shot_weight"), "g", wired=wired)
        self.cell(x0 + 340, cy, "PRESSURE", inj.get("pressure"), "psi", "{:.0f}", wired=wired)
        self.cell(x0 + 500, cy, "SPEED", inj.get("speed"), "mm/s", "{:.0f}", wired=wired)
        if not wired:
            lines = ("Waiting for the plunger position sensor.",
                     "Shots will be set in grams here and metered by plunger",
                     "position: the screw fills the barrel until the plunger",
                     "reaches the shot size, then stops.")
            for i, t in enumerate(lines):
                self.text((x0 + x1) / 2, y0 + 170 + i * 26, t, 15,
                          fill=C["text"] if i == 0 else C["dim"], bold=i == 0)

    def _melt_ready(self, d):
        """Can melt go into the barrel? Needs the extruder zones and Z4 at temperature."""
        x0, y0, x1, y1 = 8, self.CTRL_Y[0], 330, self.CTRL_Y[1]
        zones = d["zones"]
        ext_ok = all(zones[z].get("state") == "PID" for z in ("zone1", "zone2", "zone3"))
        inj_ok = zones["zone4"].get("state") == "PID"
        ok = ext_ok and inj_ok
        self.panel(x0, y0, x1, y1, "READY TO FILL", chip="YES" if ok else "NO",
                   chip_color=C["ok"] if ok else C["off"])
        self.readout(x0, x1, y0 + 54, "Extruder Z1-Z3", "AT TEMP" if ext_ok else "NOT READY", "",
                     size=16, color=C["ok"] if ext_ok else C["warn"])
        self.readout(x0, x1, y0 + 86, "Injection barrel Z4", "AT TEMP" if inj_ok else "NOT READY",
                     "", size=16, color=C["ok"] if inj_ok else C["warn"])
        self.text((x0 + x1) / 2, y1 - 16, "Screw drive is on the Extruder tab", 13, fill=C["dim"])


def main():
    import argparse
    import tkinter as tk
    from process_dashboard import DemoSim

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--size", default="800x420", help="800x420 = 7in screen minus tabs/status")
    args = ap.parse_args()
    root = tk.Tk()
    root.title("Frankenmolder - Injection")
    root.geometry(args.size + "+40+60")
    page = InjectionPage(root, on_setpoint_tap=lambda z: page.log(f"tap {z} setpoint"))
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
