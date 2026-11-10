#!/usr/bin/env python3
"""
Frankenmolder - control-room style process overview.

Used as the Dashboard tab of extruder_gui_node.py, which calls set_data() with live
ROS data. It can also run on its own with simulated data (no ROS needed):

    python3 process_dashboard.py              # extruder live, future sections "NOT WIRED"
    python3 process_dashboard.py --full       # simulate the whole injection machine
    python3 process_dashboard.py --size 800x480

Keys (standalone only): F = toggle full-machine demo, F11 = fullscreen, Esc = quit.

Data contract: see blank_data(). Any value left as None is drawn as "---"; a float
NaN temperature is drawn as FAULT; any section with "wired": False is greyed out
with a NOT WIRED tag.
"""

import math
import random
import time
import tkinter as tk
from collections import deque

from hmi import (C, LAMP, PSI_MAX, RPM_MAX, STATE_COLORS, TEMP_MAX, VH, VW, HmiCanvas,
                 _valid, blend, temp_color)

CX = VW / 2  # injection chamber + mold centre line
CYCLE_STEPS = ("LOAD", "INJECT", "HOLD", "COOL", "EJECT")


def blank_data():
    """The data contract between ROS (or the demo) and the dashboard."""
    return {
        "zones": {f"zone{i}": {"set": None, "act": None, "state": "OFF"} for i in range(1, 5)},
        "motor": {"state": "OFF", "rpm_set": None, "rpm_act": None, "load": None},
        "hopper": {"wired": False, "level": None},
        "head": {"wired": False, "pressure": None},
        "injection": {"wired": False, "pressure": None, "shot_weight": None, "speed": None,
                      "plunger": None},  # plunger 0 = shot injected .. 1 = chamber loaded
        "cycle": {"wired": False, "step": None, "elapsed": None, "parts": None},
        "clamp": {"wired": False, "force": None, "cycle_time": None, "status": None,
                  "pressure": None, "gap": 0.0, "fill": 0.0},
        "mold": {"wired": False, "set": None, "act": None},
        "can_ok": True, "estop": False, "drive_alarm": False,
    }




class ControlRoomDashboard(HmiCanvas):
    HIST_EVERY_S = 1.0  # sparkline sample rate

    def __init__(self, parent, on_zone_click=None, on_motor_click=None, tick_ms=None, **kw):
        self.on_zone_click = on_zone_click
        self.on_motor_click = on_motor_click
        self.data = blank_data()
        self.hist = {z: deque(maxlen=120) for z in self.data["zones"]}
        self._last_hist = 0.0
        self._prev = None
        super().__init__(parent, tick_ms=tick_ms, **kw)

    # ------------------------------------------------------------ public API
    def set_data(self, data):
        self.data = data
        now = time.time()
        if now - self._last_hist >= self.HIST_EVERY_S:
            self._last_hist = now
            for z, zd in data["zones"].items():
                self.hist[z].append(zd.get("act"))
        self._log_changes(data)

    # ------------------------------------------------------------ internals
    def _log_changes(self, d):
        snap = {z: zd.get("state") for z, zd in d["zones"].items()}
        snap["motor"] = d["motor"].get("state")
        if self._prev is not None:
            for k, v in snap.items():
                if v != self._prev.get(k):
                    name = "DRIVE" if k == "motor" else "Z" + k[-1]
                    lvl = "alarm" if v == "FAULT" else ("ok" if v in ("PID", "ON") else "info")
                    self.log(f"{name} {self._prev.get(k)}>{v}", lvl)
        self._prev = snap

    def _screw_speed(self):
        """0..~1 animation speed: the screw only turns (and melt only flows) when it has RPM."""
        if not self._motor_on():
            return 0.0
        rpm = self.data["motor"].get("rpm_act")
        return max(0.0, rpm) / 25.0 if _valid(rpm) else 1.0

    def animate(self):
        self.phase += self._screw_speed()

    def _motor_on(self):
        return str(self.data["motor"].get("state", "")).upper() in ("ON", "RUN", "RUNNING")

    # ------------------------------------------------------------ the page
    def render(self):
        d = self.data
        self.begin()
        running = self._motor_on()
        self.background()

        self._title_bar(d)
        self.section(46, "EXTRUSION")
        self.section(301, "INJECTION")
        self.section(420, "CLAMP / MOLD")

        self._barrel(d, running)
        self._hopper(d)
        self._motor_panel(d)
        for i, (x0, title) in enumerate(((285, "ZONE 1  FEED"), (455, "ZONE 2  TRANS"),
                                         (625, "ZONE 3  METER"))):
            self._zone_panel(x0, 55, x0 + 150, 160, f"zone{i + 1}", title)
            self.bubble(x0 + 75, 177, f"TE{i + 1}")
        self._head(d, running)
        self._injection(d, running)
        self._cycle(d)
        self._mold(d)
        self._event_log()
        self._annunciator(d)

    def _title_bar(self, d):
        self.title_bar("PROCESS OVERVIEW", *self.heaters_ready_text(d))

    def _hopper(self, d):
        h = d["hopper"]
        x0, x1, top, mid, bot = 188, 262, 52, 104, 140
        tx0, tx1 = 214, 236
        self.poly(x0, top, x1, top, x1, mid, tx1, bot, tx0, bot, x0, mid,
                  fill=C["panel"], outline=C["steel_hi"], width=max(1, 1.5 * self.s))
        lvl = h.get("level") if h.get("wired") else None
        if _valid(lvl):
            y = bot - (bot - top) * max(0.0, min(1.0, lvl / 100))
            if y > mid:  # in the taper
                f = (y - mid) / (bot - mid)
                lx0, lx1 = x0 + (tx0 - x0) * f, x1 + (tx1 - x1) * f
                self.poly(lx0, y, lx1, y, tx1, bot, tx0, bot, fill="#6b4a2a", outline="")
            else:
                self.poly(x0, y, x1, y, x1, mid, tx1, bot, tx0, bot, x0, mid,
                          fill="#6b4a2a", outline="")
        self.rect(tx0, bot, tx1, 191, fill=C["steel"], outline=C["steel_hi"])
        self.text((x0 + x1) / 2, top + 14, "HOPPER", 9, bold=True, fill=C["text"])
        self.text((x0 + x1) / 2, top + 32, f"{lvl:.0f}%" if _valid(lvl) else "---", 13,
                  mono=True, bold=True, fill=C["value"] if _valid(lvl) else C["nowire"])

    def _barrel(self, d, running):
        bx0, bx1 = 185, 830
        # gearbox
        self.rect(158, 194, 185, 238, fill=C["steel"], outline=C["steel_hi"])
        self.text(171.5, 216, "30:1", 7, mono=True, fill=C["text"])
        # walls + bore
        self.rect(bx0, 198, bx1, 234, fill=C["steel"], outline=C["steel_hi"])
        self.rect(bx0, 205, bx1, 227, fill=C["bore"], outline="")
        # screw root grows feed -> meter (compression)
        self.poly(bx0, 213, 400, 213, 600, 210, bx1 - 4, 209,
                  bx1 - 4, 223, 600, 222, 400, 219, bx0, 219, fill="#3b4b5e", outline="")
        off = (self.phase * 2) % 14
        x = bx0 + 6 + off
        while x < bx1 - 12:
            self.line(x, 206, x + 7, 226, fill="#7f97b0", width=2)
            x += 14
        # band heaters (cut-away: shown above and below the barrel wall)
        for i, (x0, x1) in enumerate(((300, 420), (470, 590), (640, 760))):
            z = d["zones"][f"zone{i + 1}"]
            self._band_h(x0, x1, z.get("act"), z.get("state"))
            self.line((x0 + x1) / 2, 160, (x0 + x1) / 2, 189, fill=C["steel_hi"], dash=(3, 2))

    def _band_h(self, x0, x1, act, state):
        col = temp_color(act)
        glow = blend(C["bg"], col, 0.35)
        for y0, y1 in ((189, 199), (233, 243)):
            if state in ("HEATING", "PID"):
                self.rect(x0 - 3, y0 - 3, x1 + 3, y1 + 3, fill=glow, outline="")
            self.rect(x0, y0, x1, y1, fill=col, outline=C["steel_hi"])
            for sx in range(int(x0) + 8, int(x1), 10):
                self.line(sx, y0 + 1, sx, y1 - 1, fill=blend(col, "#000000", 0.35))

    def _band_v(self, x0, x1, y0, y1, act, state):
        col = temp_color(act)
        if state in ("HEATING", "PID"):
            self.rect(x0 - 3, y0 - 3, x1 + 3, y1 + 3, fill=blend(C["bg"], col, 0.35), outline="")
        self.rect(x0, y0, x1, y1, fill=col, outline=C["steel_hi"])
        for sy in range(int(y0) + 6, int(y1), 8):
            self.line(x0 + 1, sy, x1 - 1, sy, fill=blend(col, "#000000", 0.35))

    def _zone_panel(self, x0, y0, x1, y1, zid, title, spark=True, zd=None, wired=True):
        z = zd if zd is not None else self.data["zones"][zid]
        st = str(z.get("state") or "OFF").upper()
        act, setp = z.get("act"), z.get("set")
        cb = (lambda: self.on_zone_click(zid)) if (self.on_zone_click and zid) else None
        self.panel(x0, y0, x1, y1, title, chip=st, chip_color=STATE_COLORS.get(st, C["warn"]),
                   wired=wired, hot=cb)
        if _valid(act) and _valid(setp) and act > setp + 15:
            acol = C["alarm"]
        elif st == "PID":
            acol = C["ok"]
        elif st == "HEATING":
            acol = C["warn"]
        else:
            acol = C["value"]
        self.readout(x0, x1, y0 + 32, "SET", setp, "°C", size=14, color=C["setp"], wired=wired)
        self.readout(x0, x1, y0 + 53, "ACT", act, "°C", size=19, color=acol, wired=wired)
        self.hbar(x0 + 8, x1 - 8, y0 + 68, 5,
                  act / TEMP_MAX if wired and _valid(act) else None, temp_color(act),
                  marker=setp / TEMP_MAX if wired and _valid(setp) else None)
        if spark and zid:
            self.spark(x0 + 8, y0 + 79, x1 - 8, y1 - 6, self.hist[zid], setp)

    def _motor_panel(self, d):
        m = d["motor"]
        on = self._motor_on()
        x0, y0, x1, y1 = 12, 140, 152, 292
        self.panel(x0, y0, x1, y1, "SCREW DRIVE", chip="RUN" if on else "STOP",
                   chip_color=C["ok"] if on else C["off"],
                   hot=self.on_motor_click if self.on_motor_click else None)
        self.gauge(82, 206, 38, m.get("rpm_act"), RPM_MAX, "SCREW RPM", "{:.1f}",
                   setp=m.get("rpm_set"))
        self.readout(x0, x1, 258, "SET", m.get("rpm_set"), "rpm", size=13, color=C["setp"])
        load = m.get("load")
        self.readout(x0, x1, 275, "LOAD", load, "%", "{:.0f}", size=13)
        self.hbar(x0 + 8, x1 - 8, 284, 4, load / 100 if _valid(load) else None,
                  C["alarm"] if _valid(load) and load > 85 else C["accent"])

    def _head(self, d, running):
        h = d["head"]
        self.rect(830, 189, 852, 243, fill=C["steel"], outline=C["steel_hi"])
        self.line(852, 216, 868, 216, fill=C["steel_hi"], dash=(3, 2))
        self.panel(868, 140, 990, 274, "MELT", wired=h.get("wired"))
        self.gauge(929, 208, 38, h.get("pressure"), PSI_MAX, "PSI", "{:.0f}",
                   wired=h.get("wired"))
        self.text(929, 262, "HEAD PRESSURE  PT1", 8, fill=C["dim"])
        # melt path: head -> check valve -> side inlet of the injection chamber
        flowing = self._screw_speed() > 0.04
        self.pipe(841, 243, 841, 290, 590, 290, 590, 404, CX + 20, 404, flowing=flowing)
        self.poly(712, 280, 712, 300, 690, 290, fill=C["plastic"] if flowing else C["steel_hi"],
                  outline=C["text"])
        self.line(688, 279, 688, 301, fill=C["text"], width=2)
        self.text(700, 274, "CHECK", 7, mono=True, fill=C["dim"])

    def _injection(self, d, running):
        inj = d["injection"]
        w = inj.get("wired")
        x0, y0, x1, y1 = 290, 308, 438, 410
        plunger = inj.get("plunger")
        self.panel(x0, y0, x1, y1, "INJECTION", wired=w,
                   chip=f"SHOT {plunger * 100:.0f}%" if _valid(plunger) else None,
                   chip_color=C["plastic"])
        self.readout(x0, x1, 338, "PRESSURE", inj.get("pressure"), "psi", "{:.0f}", wired=w)
        self.readout(x0, x1, 362, "SHOT WT", inj.get("shot_weight"), "g", wired=w)
        self.readout(x0, x1, 386, "INJ SPEED", inj.get("speed"), "mm/s", "{:.0f}", wired=w)
        self.line(x1, 338, CX - 29, 338, fill=C["steel_hi"], dash=(3, 2))
        self.bubble(x1 + 17, 338, "PT2")
        self._chamber(d)
        self.line(CX + 29, 352, 625, 352, fill=C["steel_hi"], dash=(3, 2))
        self.bubble(CX + 56, 352, "TE4")
        self._zone_panel(625, 308, 775, 410, "zone4", "ZONE 4  CHAMBER")

    def _chamber(self, d):
        """Zone 4: vertical injection chamber, centred on the screen. Melt enters the
        side inlet below the plunger and pushes it up (loading); the ram drives it
        down to inject."""
        inj = d["injection"]
        z4 = d["zones"]["zone4"]
        w = inj.get("wired")
        p = inj.get("plunger") if w else None
        cx0, cx1, top, bot = CX - 20, CX + 20, 296, 414
        # ram / actuator above the chamber
        self.rect(CX - 28, 250, CX + 28, 284, fill=C["steel"], outline=C["steel_hi"])
        self.text(CX, 267, "RAM", 8, mono=True, bold=True, fill=C["text"])
        # chamber body (cut-away), nozzle taper
        self.rect(cx0, top, cx1, bot, fill=C["steel"], outline=C["steel_hi"])
        self.rect(cx0 + 6, top + 5, cx1 - 6, bot, fill=C["bore"], outline="")
        self.poly(cx0, bot, cx1, bot, CX + 6, 428, CX - 6, 428, fill=C["steel"],
                  outline=C["steel_hi"])
        # heater band, shown on both walls like the barrel
        for bx0, bx1 in ((CX - 29, CX - 21), (CX + 21, CX + 29)):
            self._band_v(bx0, bx1, 312, 392, z4.get("act"), z4.get("state"))
        # plunger: p=0 bottomed out (shot injected), p=1 fully loaded
        y_full, y_empty = top + 6, 388
        if _valid(p):
            py = y_empty - (y_empty - y_full) * max(0.0, min(1.0, p))
            self.rect(cx0 + 6, py + 8, cx1 - 6, bot, fill=C["plastic"], outline="")
            self.poly(cx0 + 6, bot, cx1 - 6, bot, CX + 3, 426, CX - 3, 426, fill=C["plastic"],
                      outline="")
            rod_col, head_col = C["steel_hi"], "#8aa2bb"
        else:
            py = (y_full + y_empty) / 2
            rod_col, head_col = C["nowire"], C["steel"]
        self.line(CX, 284, CX, py, fill=rod_col, width=7)
        self.rect(cx0 + 7, py, cx1 - 7, py + 8, fill=head_col, outline=C["steel_hi"])

    def _cycle(self, d):
        cy = d["cycle"]
        w = cy.get("wired")
        x0, y0, x1, y1 = 12, 308, 276, 410
        self.panel(x0, y0, x1, y1, "MACHINE CYCLE", chip="AUTO", chip_color=C["accent"], wired=w)
        step = cy.get("step") if w else None
        bw, gap = 44, 6
        for i, name in enumerate(CYCLE_STEPS):
            bx = x0 + 10 + i * (bw + gap)
            active = step == name
            fill = C["ok"] if active else C["well"]
            self.rect(bx, 333, bx + bw, 355, fill=fill,
                      outline=C["ok"] if active else (C["edge"] if w else C["nowire"]))
            self.text(bx + bw / 2, 344, name, 8, mono=True, bold=True,
                      fill="#0a0e13" if active else (C["text"] if w else C["nowire"]))
            if i < len(CYCLE_STEPS) - 1:
                self.poly(bx + bw + 1, 341, bx + bw + 1, 347, bx + bw + gap - 1, 344,
                          fill=C["dim"] if w else C["nowire"], outline="")
        self.readout(x0, x0 + 130, 377, "ELAPSED", cy.get("elapsed") if w else None, "s", wired=w)
        self.readout(x0 + 130, x1, 377, "PARTS", cy.get("parts") if w else None, "", "{:04d}",
                     wired=w)
        self.text(x0 + 8, 397, "Tap a zone or the drive to adjust", 8, anchor="w", fill=C["dim"])

    def block_arrow(self, x_tail, x_tip, y, color):
        sgn = 1 if x_tip > x_tail else -1
        xb = x_tip - sgn * 26
        self.poly(x_tail, y - 10, xb, y - 10, xb, y - 22, x_tip, y, xb, y + 22, xb, y + 10,
                  x_tail, y + 10, fill=color, outline="")

    def _mold(self, d):
        cl, mo = d["clamp"], d["mold"]
        w = cl.get("wired")
        gap = cl.get("gap", 0.0) if w else 0.0
        fill = cl.get("fill", 0.0) if w else 0.0
        # clamp panel
        x0, y0, x1, y1 = 12, 428, 272, 590
        self.panel(x0, y0, x1, y1, "CLAMP", wired=w)
        self.readout(x0, x1, 458, "CLAMP FORCE", cl.get("force"), "t", wired=w)
        self.readout(x0, x1, 488, "CYCLE TIME", cl.get("cycle_time"), "s", wired=w)
        status = cl.get("status")
        scol = C["ok"] if status == "CLOSED" else C["warn"]
        self.readout(x0, x1, 518, "STATUS", status, "", wired=w, color=scol)
        self.readout(x0, x1, 548, "PRESSURE", cl.get("pressure"), "psi", "{:.0f}", wired=w)
        self.text(x0 + 8, 575, "Clamp tonnage TBD", 8, anchor="w", fill=C["dim"])

        pl, half = 22, 85.5          # platen thickness, mold half width
        g = -gap                     # moving side slides left when opening
        # squeeze arrows: lit while the clamp is holding force
        force = cl.get("force")
        squeezing = w and _valid(force) and force > 0
        acol = C["accent"] if squeezing else (C["edge"] if w else C["nowire"])
        L = 78                       # both arrows the same length
        lt, rt = CX - half - pl - 4 + g, CX + half + pl + 4
        self.block_arrow(lt - L, lt, 512, acol)
        self.block_arrow(rt + L, rt, 512, acol)
        if squeezing:
            self.text(lt - L / 2, 478, f"{force:.1f} t", 10, mono=True, bold=True,
                      fill=C["accent"])
        # moving platen + half, fixed half + platen
        self.rect(CX - half - pl + g, 440, CX - half + g, 585, fill=C["steel"],
                  outline=C["steel_hi"])
        self.rect(CX - half + g, 452, CX + g, 573, fill="#33414f", outline=C["steel_hi"])
        self.rect(CX, 452, CX + half, 573, fill="#33414f", outline=C["steel_hi"])
        self.rect(CX + half, 440, CX + half + pl, 585, fill=C["steel"], outline=C["steel_hi"])
        # cavity: symmetric about the parting line and about its own centre (y=515)
        cav = ((32.5, 465, 480), (47.5, 480, 550), (32.5, 550, 565))
        top, bot = 465, 565
        fy = bot - (bot - top) * max(0.0, min(1.0, fill))
        for hw, cy0, cy1 in cav:
            for a, b, sh in ((CX - hw, CX, g), (CX, CX + hw, 0)):
                self.rect(a + sh, cy0, b + sh, cy1, fill=C["bore"], outline="")
                if fill > 0 and fy < cy1:
                    self.rect(a + sh, max(cy0, fy), b + sh, cy1, fill=C["plastic"], outline="")
        self.rect(CX - 3.5, 426, CX + 3.5, 465, fill=C["plastic"] if fill > 0 else C["bore"],
                  outline="")
        self.line(CX, 452, CX, 573, fill=C["dim"], dash=(2, 3))
        # mold temp
        self.line(CX + half + pl, 470, 700, 470, fill=C["steel_hi"], dash=(3, 2))
        self.bubble(CX + half + pl + 40, 470, "TE5")
        self._zone_panel(700, 428, 855, 512, None, "MOLD TEMP", spark=False, zd=mo,
                         wired=mo.get("wired"))

    def _event_log(self):
        self.event_panel(868, 286, 990, 512)

    def _annunciator(self, d):
        zones = d["zones"].values()
        blink = int(time.time() * 2) % 2 == 0
        tiles = [
            ("HEAT", "READY", all(z.get("state") == "PID" for z in zones), C["ok"]),
            ("DRIVE", "RUN", self._motor_on(), C["ok"]),
            ("CAN", "LINK", d.get("can_ok"), C["ok"]),
            ("MOLD", "CLOSED", d["clamp"].get("wired") and d["clamp"].get("status") == "CLOSED", C["ok"]),
            ("OVER", "TEMP", any(_valid(z.get("act")) and _valid(z.get("set")) and
                                 z["act"] > z["set"] + 15 for z in zones), C["alarm"]),
            ("TC", "FAULT", any(z.get("state") == "FAULT" or
                                (isinstance(z.get("act"), float) and math.isnan(z["act"]))
                                for z in zones), C["alarm"]),
            ("DRIVE", "ALARM", d.get("drive_alarm"), C["alarm"]),
            ("E-STOP", "", d.get("estop"), C["alarm"]),
        ]
        x0, y0, tw, th, gp = 700, 522, 69, 32, 4
        for i, (a, b, lit, col) in enumerate(tiles):
            r, c = divmod(i, 4)
            tx, ty = x0 + c * (tw + gp), y0 + r * (th + gp)
            on = bool(lit) and (col != C["alarm"] or blink)
            self.rect(tx, ty, tx + tw, ty + th, fill=col if on else C["well"],
                      outline=col if lit else C["edge"])
            fg = "#0a0e13" if on else (col if lit else C["dim"])
            if b:
                self.text(tx + tw / 2, ty + 10, a, 9, mono=True, bold=True, fill=fg)
                self.text(tx + tw / 2, ty + 22, b, 9, mono=True, bold=True, fill=fg)
            else:
                self.text(tx + tw / 2, ty + th / 2, a, 10, mono=True, bold=True, fill=fg)


# ---------------------------------------------------------------- demo data
class DemoSim:
    """Fake process data so the page can be judged without the machine."""

    def __init__(self, full=False):
        self.full = full
        self.t0 = self.last = time.time()
        self.sets = [200.0, 215.0, 225.0, 230.0]
        self.act = [22.0, 22.5, 21.8, 22.1]
        self.state = ["OFF"] * 4
        self.rpm = 0.0
        self.parts = 0
        self.cyc0 = None

    def step(self):
        now = time.time()
        dt, t = now - self.last, now - self.t0
        self.last = now
        for i in range(4):
            if self.state[i] == "OFF" and t > 1.5 + i * 0.8:
                self.state[i] = "HEATING"
            if self.state[i] == "HEATING":
                self.act[i] += (self.sets[i] + 80 - self.act[i]) * dt / 14
                if self.act[i] >= self.sets[i] - 3:
                    self.state[i] = "PID"
            elif self.state[i] == "PID":
                self.act[i] += (self.sets[i] - self.act[i]) * dt / 2.5 + random.gauss(0, 0.3)
            else:
                self.act[i] += (22 - self.act[i]) * dt / 60
        hot = all(s == "PID" for s in self.state)

        # Machine cycle (full demo only): the screw turns only while LOADing the chamber.
        CYCLE = (("LOAD", 4.0), ("INJECT", 5.2), ("HOLD", 6.7), ("COOL", 10.0), ("EJECT", 12.0))
        step, ct = None, 0.0
        if self.full and hot:
            if self.cyc0 is None:
                self.cyc0 = now
            n, ct = divmod(now - self.cyc0, CYCLE[-1][1])
            if int(n) != self.parts:
                self.parts = int(n)
            step = next(name for name, end in CYCLE if ct < end)

        rpm_set = 25.0 if hot and (step is None or step == "LOAD") else 0.0
        self.rpm += (rpm_set - self.rpm) * min(1.0, dt / 0.6)
        on = hot

        d = blank_data()
        for i in range(4):
            d["zones"][f"zone{i + 1}"] = {"set": self.sets[i], "act": self.act[i],
                                          "state": self.state[i]}
        turning = self.rpm > 0.5
        d["motor"] = {"state": "ON" if on else "OFF", "rpm_set": rpm_set,
                      "rpm_act": max(0.0, self.rpm + (random.gauss(0, 0.15) if turning else 0)),
                      "load": (38 + random.gauss(0, 2)) * self.rpm / 25 if on else 0.0}
        if not self.full:
            return d

        # ---- future sections (simulated) ----
        d["hopper"] = {"wired": True, "level": max(5.0, 82 - t * 0.2)}
        d["mold"] = {"wired": True, "set": 60.0, "act": 58.5 + random.gauss(0, 0.4),
                     "state": "PID"}
        CUSHION = 0.06  # plunger never fully bottoms out
        if step == "LOAD":       # melt pushes the plunger up
            plunger = CUSHION + (1 - CUSHION) * min(1.0, ct / 3.6)
        elif step == "INJECT":   # ram drives it down, mold fills
            f = (ct - 4.0) / 1.2
            plunger = 1 - (1 - CUSHION - 0.04) * f
        elif step is None:
            plunger = CUSHION
        else:                    # HOLD / COOL / EJECT: sitting on the cushion
            plunger = CUSHION + 0.04 * (1 - min(1.0, (ct - 5.2) / 1.5)) if step == "HOLD" else CUSHION
        fill = {"INJECT": (ct - 4.0) / 1.2, "HOLD": 1.0, "COOL": 1.0}.get(step, 0.0)
        gap = 40 * math.sin(math.pi * (ct - 10.0) / 2.0) if step == "EJECT" else 0.0
        head_p = 1700 + 250 * math.sin(t / 3) if step == "LOAD" else 400.0
        inj_p = {"INJECT": 3200, "HOLD": 1800}.get(step, 0)
        d["head"] = {"wired": True, "pressure": head_p + random.gauss(0, 40) if hot else 30.0}
        d["injection"] = {"wired": True, "pressure": max(0.0, inj_p + random.gauss(0, 30)),
                          "shot_weight": 18.4, "speed": 45.0 if step == "INJECT" else 0.0,
                          "plunger": plunger}
        d["cycle"] = {"wired": True, "step": step, "elapsed": ct, "parts": self.parts}
        d["clamp"] = {"wired": True, "force": 12.0 if step != "EJECT" else 0.0,
                      "cycle_time": CYCLE[-1][1], "status": "OPEN" if step == "EJECT" else "CLOSED",
                      "pressure": 1450.0 if step != "EJECT" else 0.0, "gap": gap, "fill": fill}
        return d


def main():
    import argparse
    import subprocess

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--full", action="store_true", help="simulate the whole injection machine")
    ap.add_argument("--size", default="1024x600", help="window size, e.g. 800x480")
    ap.add_argument("--fullscreen", action="store_true")
    ap.add_argument("--capture", help=argparse.SUPPRESS)  # dev: screenshot path (macOS)
    ap.add_argument("--capture-after", type=float, default=8.0, help=argparse.SUPPRESS)
    args = ap.parse_args()

    root = tk.Tk()
    root.title("Frankenmolder - Process Overview")
    root.geometry(args.size + "+40+60")
    root.configure(bg=C["bg"])
    if args.fullscreen:
        root.attributes("-fullscreen", True)

    dash = ControlRoomDashboard(root, on_zone_click=lambda z: dash.log(f"tap {z}"),
                                on_motor_click=lambda: dash.log("tap drive"))
    dash.pack(fill="both", expand=True)
    sim = DemoSim(full=args.full)

    def pump():
        dash.set_data(sim.step())
        root.after(200, pump)

    def toggle_full(_e=None):
        sim.full = not sim.full

    root.bind("<Escape>", lambda e: root.destroy())
    root.bind("<F11>", lambda e: root.attributes("-fullscreen",
                                                 not root.attributes("-fullscreen")))
    root.bind("f", toggle_full)
    root.bind("F", toggle_full)
    pump()

    if args.capture:
        def snap():
            root.update()
            x, y = dash.winfo_rootx(), dash.winfo_rooty()
            w, h = dash.winfo_width(), dash.winfo_height()
            subprocess.run(["screencapture", "-x", f"-R{x},{y},{w},{h}", args.capture])
            root.destroy()
        root.after(int(args.capture_after * 1000), snap)

    root.mainloop()


if __name__ == "__main__":
    main()
