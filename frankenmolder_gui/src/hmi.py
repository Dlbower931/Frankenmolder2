#!/usr/bin/env python3
"""
Shared look-and-feel for the Frankenmolder touchscreen pages.

HmiCanvas is a tk.Frame holding one Canvas that is drawn in a fixed virtual space
(VW x VH) and scaled to whatever size the window is. Pages subclass it and
implement render(). Touch targets are registered during render() and fire on
release, with a pressed look while the finger is down.
"""

import math
import time
import tkinter as tk
import tkinter.font as tkfont
from collections import deque

# Virtual drawing space; everything is scaled to fit the real window. 1000x525 matches
# the 7" 800x480 touchscreen once the tab bar and status bar are taken (800x420, scale 0.8),
# so a text size of 13 here is ~10 px on the screen - treat that as the minimum.
VW, VH = 1000, 525
TEMP_MAX = 350.0
RPM_MAX = 60.0
PSI_MAX = 5000.0

C = {
    "bg": "#0a0e13", "grid": "#111821", "panel": "#111821", "head": "#17212c",
    "edge": "#263345", "accent": "#3d8bd1", "well": "#0b1118",
    "dim": "#8494a8", "text": "#b9c6d4", "value": "#eaf2fa", "setp": "#4fc3f7",
    "steel": "#2c3846", "steel_hi": "#4a5d72", "bore": "#0d1218",
    "ok": "#3fb950", "warn": "#e3a008", "alarm": "#f85149", "off": "#36404c",
    "plastic": "#f0883e", "nowire": "#3a4757",
}
STATE_COLORS = {"OFF": C["off"], "HEATING": C["warn"], "HEAT": C["warn"],
                "PID": C["ok"], "FAULT": C["alarm"]}
MONO_FONTS = ("DejaVu Sans Mono", "Menlo", "Consolas", "Liberation Mono", "Courier")
SANS_FONTS = ("DejaVu Sans", "Helvetica Neue", "Helvetica", "Liberation Sans", "Arial")

# Button face colours (lamp-button style)
FACE = {"start": "#238636", "stop": "#b62324", "blue": "#1f5fbf", "neutral": "#2d3a4a"}
LAMP = {"green": "#7ee787", "red": "#ff7b72", "amber": "#f2cc60", "blue": "#79c0ff",
        "white": "#e6edf3"}


def _valid(v):
    return v is not None and not (isinstance(v, float) and math.isnan(v))


def _hex(rgb):
    return "#%02x%02x%02x" % tuple(max(0, min(255, int(c))) for c in rgb)


def _rgb(h):
    return tuple(int(h[i:i + 2], 16) for i in (1, 3, 5))


def blend(a, b, t):
    ra, rb = _rgb(a), _rgb(b)
    return _hex([ra[i] + (rb[i] - ra[i]) * t for i in range(3)])


def temp_color(t):
    """Steel -> dull red -> orange -> yellow-white as a heater gets hotter."""
    if not _valid(t):
        return C["steel"]
    stops = [(20, (44, 56, 70)), (110, (120, 40, 36)), (190, (214, 78, 28)),
             (250, (246, 148, 40)), (320, (255, 228, 150))]
    if t <= stops[0][0]:
        return _hex(stops[0][1])
    for (t0, c0), (t1, c1) in zip(stops, stops[1:]):
        if t <= t1:
            f = (t - t0) / (t1 - t0)
            return _hex([c0[i] + (c1[i] - c0[i]) * f for i in range(3)])
    return _hex(stops[-1][1])


class HmiCanvas(tk.Frame):
    TICK_MS = 100  # redraw rate
    VW, VH = VW, VH  # virtual size; a page may override to match its screen's aspect
    HDR = 28                                       # panel header height
    T_TITLE, T_CHIP, T_LABEL, T_UNIT = 15, 13, 14, 13

    def __init__(self, parent, tick_ms=None, **kw):
        super().__init__(parent, bg=C["bg"], **kw)
        if tick_ms:
            self.TICK_MS = tick_ms
        self.cv = tk.Canvas(self, bg=C["bg"], highlightthickness=0)
        self.cv.pack(fill="both", expand=True)
        fams = set(tkfont.families(self))
        self.mono = next((f for f in MONO_FONTS if f in fams), "Courier")
        self.sans = next((f for f in SANS_FONTS if f in fams), "Helvetica")
        self.s, self.ox, self.oy = 1.0, 0.0, 0.0
        self.phase = 0.0
        self.events = deque(maxlen=9)
        self._hot = []
        self._pressed = None

        self.cv.bind("<Configure>", self._on_resize)
        self.cv.bind("<ButtonPress-1>", self._on_press)
        self.cv.bind("<ButtonRelease-1>", self._on_release)
        self.after(self.TICK_MS, self._tick)

    # ------------------------------------------------------------ framework
    def render(self):
        raise NotImplementedError

    def animate(self):
        """Advance animation state once per tick (override)."""

    def begin(self):
        """Call at the top of render(): clears the canvas and touch targets."""
        self.cv.delete("all")
        self._hot = []

    def log(self, msg, level="info"):
        self.events.appendleft((time.strftime("%H:%M:%S"), msg, level))

    def _on_resize(self, ev):
        self.s = min(ev.width / self.VW, ev.height / self.VH)
        self.ox = (ev.width - self.VW * self.s) / 2
        self.oy = (ev.height - self.VH * self.s) / 2
        self.render()

    def _hit(self, ev):
        x, y = (ev.x - self.ox) / self.s, (ev.y - self.oy) / self.s
        for h in reversed(self._hot):  # last drawn = on top
            if h[0] <= x <= h[2] and h[1] <= y <= h[3]:
                return h
        return None

    def _on_press(self, ev):
        h = self._hit(ev)
        self._pressed = tuple(h[:4]) if h else None
        if h:
            self.render()

    def _on_release(self, ev):
        h, pressed = self._hit(ev), self._pressed
        self._pressed = None
        if h and tuple(h[:4]) == pressed:
            h[4]()
        if pressed:
            self.render()

    def is_pressed(self, x0, y0, x1, y1):
        return self._pressed == (x0, y0, x1, y1)

    def _tick(self):
        self.animate()
        self.render()
        self.after(self.TICK_MS, self._tick)

    # ------------------------------------------------------------ primitives
    def P(self, x, y):
        return self.ox + x * self.s, self.oy + y * self.s

    def _flat(self, pts):
        out = []
        for x, y in zip(pts[::2], pts[1::2]):
            out.extend(self.P(x, y))
        return out

    def rect(self, x0, y0, x1, y1, **kw):
        return self.cv.create_rectangle(*self.P(x0, y0), *self.P(x1, y1), **kw)

    def line(self, *pts, width=1, **kw):
        return self.cv.create_line(*self._flat(pts), width=max(1, width * self.s), **kw)

    def poly(self, *pts, **kw):
        return self.cv.create_polygon(*self._flat(pts), **kw)

    def oval(self, cx, cy, r, **kw):
        return self.cv.create_oval(*self.P(cx - r, cy - r), *self.P(cx + r, cy + r), **kw)

    def text(self, x, y, txt, size=11, mono=False, bold=False, **kw):
        font = (self.mono if mono else self.sans, -max(6, int(round(size * self.s))),
                "bold" if bold else "normal")
        return self.cv.create_text(*self.P(x, y), text=txt, font=font, **kw)

    def arc(self, cx, cy, r, start, extent, color, width):
        if abs(extent) < 0.5:
            return
        self.cv.create_arc(*self.P(cx - r, cy - r), *self.P(cx + r, cy + r), start=start,
                           extent=extent, style=tk.ARC, outline=color,
                           width=max(1, width * self.s))

    # ------------------------------------------------------------ widgets
    def chip(self, xr, yc, txt, color, fg="#0a0e13", size=9, h=7):
        w = len(txt) * size * 0.64 + 10
        self.rect(xr - w, yc - h, xr, yc + h, fill=color, outline="")
        self.text(xr - w / 2, yc, txt, size, mono=True, bold=True, fill=fg)

    def panel(self, x0, y0, x1, y1, title, chip=None, chip_color=None, wired=True, hot=None):
        """Framed panel with a header. chip=False suppresses the NOT WIRED chip (when the
        title leaves no room for it); hot makes the whole panel a touch target."""
        pressed = hot is not None and self.is_pressed(x0, y0, x1, y1)
        self.rect(x0, y0, x1, y1, fill="#1a2634" if pressed else C["panel"],
                  outline=C["accent"] if pressed else C["edge"])
        hy = y0 + self.HDR / 2
        self.rect(x0, y0, x1, y0 + self.HDR, fill=C["head"], outline=C["edge"])
        acc = C["accent"] if wired else C["nowire"]
        L = 10
        for cx, cy, dx, dy in ((x0, y0, 1, 1), (x1, y0, -1, 1), (x0, y1, 1, -1), (x1, y1, -1, -1)):
            self.line(cx, cy + dy * L, cx, cy, cx + dx * L, cy, fill=acc, width=2)
        self.text(x0 + 8, hy, title, self.T_TITLE, bold=True, anchor="w",
                  fill=C["text"] if wired else C["dim"])
        if not wired:
            if chip is not False:
                self.chip(x1 - 5, hy, "NOT WIRED", C["nowire"], fg=C["text"], size=12, h=10)
        elif chip:
            self.chip(x1 - 5, hy, chip, chip_color or C["dim"], size=self.T_CHIP, h=10)
        if hot:
            self._hot.append((x0, y0, x1, y1, hot))

    def _fmt(self, val, fmt, wired):
        """(text, colour or None) for a value; None colour means 'use the caller's'."""
        if not wired:
            return "---", C["nowire"]
        if not _valid(val):
            return ("FAULT", C["alarm"]) if isinstance(val, float) else ("---", C["dim"])
        return (val if isinstance(val, str) else fmt.format(val)), None

    def readout(self, x0, x1, y, label, val, unit, fmt="{:.1f}", size=22, color=None, wired=True):
        """Label on the left, big value right-aligned, unit at the far right."""
        self.text(x0 + 8, y, label, self.T_LABEL, anchor="w", fill=C["dim"])
        s, col = self._fmt(val, fmt, wired)
        self.text(x1 - 30, y, s, size, mono=True, bold=True, anchor="e",
                  fill=col or color or C["value"])
        self.text(x1 - 7, y + 1, unit, self.T_UNIT, anchor="e", fill=C["dim"])

    def cell(self, x, y, label, val, unit, fmt="{:.1f}", size=20, color=None, wired=True):
        """Stacked readout: small label above a big value (for narrow columns)."""
        self.text(x, y, label, self.T_LABEL, anchor="w", fill=C["dim"])
        s, col = self._fmt(val, fmt, wired)
        self.text(x, y + 22, s, size, mono=True, bold=True, anchor="w",
                  fill=col or color or C["value"])
        if unit:
            self.text(x + len(s) * size * 0.61 + 4, y + 24, unit, self.T_UNIT, anchor="w",
                      fill=C["dim"])

    def hbar(self, x0, x1, y, h, frac, color, marker=None):
        self.rect(x0, y, x1, y + h, fill=C["well"], outline="")
        if frac is not None and frac > 0:
            self.rect(x0, y, x0 + (x1 - x0) * min(1.0, frac), y + h, fill=color, outline="")
        if marker is not None:
            mx = x0 + (x1 - x0) * max(0.0, min(1.0, marker))
            self.poly(mx, y - 1, mx - 4, y - 6, mx + 4, y - 6, fill=C["setp"], outline="")

    def spark(self, x0, y0, x1, y1, hist, setp):
        self.rect(x0, y0, x1, y1, fill=C["well"], outline="")
        vals = [v for v in hist if _valid(v)]
        if len(vals) < 2:
            return
        ref = vals + ([setp] if _valid(setp) else [])
        lo, hi = min(ref), max(ref)
        pad = max(4.0, (hi - lo) * 0.12)
        lo, hi = lo - pad, hi + pad
        n, k = hist.maxlen, len(hist)
        pts = []
        for i, v in enumerate(hist):
            if _valid(v):
                pts += [x0 + (x1 - x0) * (i + n - k) / (n - 1), y1 - (y1 - y0) * (v - lo) / (hi - lo)]
        if _valid(setp):
            ys = y1 - (y1 - y0) * (setp - lo) / (hi - lo)
            self.line(x0, ys, x1, ys, fill=C["setp"], dash=(2, 3))
        if len(pts) >= 4:
            self.line(*pts, fill=C["ok"], width=1.3)

    def gauge(self, cx, cy, r, val, vmax, unit, fmt="{:.0f}", setp=None, wired=True,
              warn=0.75, alarm=0.9, vsize=22, usize=13):
        A = lambda f: 210 - 240 * f  # fraction -> Tk angle (deg, ccw from 3 o'clock)
        self.arc(cx, cy, r, A(1), 240, C["well"], 7)
        if wired:
            self.arc(cx, cy, r + 7, A(alarm), 240 * (alarm - warn), C["warn"], 2)
            self.arc(cx, cy, r + 7, A(1), 240 * (1 - alarm), C["alarm"], 2)
        for i in range(11):
            a = math.radians(A(i / 10))
            r0 = r - (11 if i % 5 == 0 else 8)
            self.line(cx + math.cos(a) * r0, cy - math.sin(a) * r0,
                      cx + math.cos(a) * (r - 5), cy - math.sin(a) * (r - 5),
                      fill=C["dim"] if wired else C["nowire"])
        if wired and _valid(val):
            f = max(0.0, min(1.0, val / vmax))
            col = C["alarm"] if f >= alarm else C["warn"] if f >= warn else C["ok"]
            self.arc(cx, cy, r, A(f), 240 * f, col, 7)
            a = math.radians(A(max(f, 0.06)))  # keep the needle off the readout at zero
            self.line(cx, cy, cx + math.cos(a) * (r - 13), cy - math.sin(a) * (r - 13),
                      fill=C["value"], width=2)
            self.oval(cx, cy, 3.5, fill=C["value"], outline="")
        if wired and _valid(setp):
            a = math.radians(A(max(0.0, min(1.0, setp / vmax))))
            tx, ty = cx + math.cos(a) * (r + 2), cy - math.sin(a) * (r + 2)
            self.oval(tx, ty, 3.5, fill=C["setp"], outline="")
        txt = fmt.format(val) if wired and _valid(val) else "---"
        self.text(cx, cy + r * 0.55, txt, vsize, mono=True, bold=True,
                  fill=C["value"] if wired else C["nowire"])
        self.text(cx, cy + r * 0.55 + vsize * 0.6 + usize * 0.6, unit, usize, fill=C["dim"])

    def bubble(self, x, y, tag):
        """P&ID-style instrument bubble."""
        self.oval(x, y, 10, fill=C["bg"], outline=C["steel_hi"], width=max(1, self.s))
        self.text(x, y, tag, 7, mono=True, bold=True, fill=C["text"])

    def pipe(self, *pts, flowing=False, wide=12):
        self.line(*pts, fill=C["steel"], width=wide, capstyle="projecting", joinstyle="miter")
        self.line(*pts, fill="#2a1d16", width=wide * 0.45, capstyle="projecting", joinstyle="miter")
        if flowing:
            off = int(-self.phase * 3) % 16
            self.line(*pts, fill=C["plastic"], width=wide * 0.45, dash=(8, 8), dashoffset=off)

    def section(self, y, label):
        self.rect(10, y - 5, 13, y + 5, fill=C["accent"], outline="")
        self.text(18, y, label, 9, bold=True, anchor="w", fill=C["dim"])

    # ------------------------------------------------------------ page chrome
    def background(self):
        for x in range(0, self.VW + 1, 40):
            self.line(x, 36, x, self.VH, fill=C["grid"])
        for y in range(40, self.VH + 1, 40):
            self.line(0, y, self.VW, y, fill=C["grid"])

    def title_bar(self, center=None, center_color=None):
        """Top bar: name, a status line in the middle, clock. (The tab names the page.)"""
        self.rect(0, 0, self.VW, 36, fill=C["head"], outline="")
        self.line(0, 36, self.VW, 36, fill=C["accent"], width=1.5)
        self.text(12, 18, "FRANKENMOLDER", 20, bold=True, anchor="w", fill=C["value"])
        if center:
            self.text(self.VW / 2 + 40, 18, center, 16, mono=True, bold=True,
                      fill=center_color or C["dim"])
        self.text(self.VW - 12, 18, time.strftime("%H:%M:%S"), 19, mono=True, bold=True,
                  anchor="e", fill=C["setp"])

    def heaters_ready_text(self, d):
        zones = d["zones"].values()
        ready = sum(1 for z in zones if z.get("state") == "PID")
        n = len(d["zones"])
        return (f"HEATERS AT TEMP  {ready}/{n}",
                C["ok"] if ready == n else C["warn"] if ready else C["dim"])

    def event_panel(self, x0, y0, x1, y1, title="EVENTS"):
        """Newest-first log, one line per event."""
        self.panel(x0, y0, x1, y1, title)
        col = {"info": C["text"], "ok": C["ok"], "alarm": C["alarm"], "cmd": C["setp"]}
        rows = int((y1 - y0 - self.HDR - 6) // 20)
        for i, (t, msg, lvl) in enumerate(list(self.events)[:rows]):
            y = y0 + self.HDR + 15 + i * 20
            self.text(x0 + 8, y, t[:5], 13, mono=True, anchor="w", fill=C["dim"])
            self.text(x0 + 58, y, msg, 13, mono=True, anchor="w", fill=col.get(lvl, C["text"]))

    # ------------------------------------------------------------ controls
    def rrect(self, x0, y0, x1, y1, r, **kw):
        """Rounded rectangle (smoothed polygon)."""
        pts = (x0 + r, y0, x0 + r, y0, x1 - r, y0, x1 - r, y0, x1, y0, x1, y0 + r, x1, y0 + r,
               x1, y1 - r, x1, y1 - r, x1, y1, x1 - r, y1, x1 - r, y1, x0 + r, y1, x0 + r, y1,
               x0, y1, x0, y1 - r, x0, y1 - r, x0, y0 + r, x0, y0 + r, x0, y0)
        return self.poly(*pts, smooth=True, **kw)

    def _lamp(self, x0, y0, x1, y1, on, color, base=None):
        """Recessed indicator lamp; glows when on."""
        self.rrect(x0 - 2, y0 - 2, x1 + 2, y1 + 2, 3, fill="#05080b", outline="")
        if on:
            self.rrect(x0 - 3, y0 - 3, x1 + 3, y1 + 3, 4,
                       fill=blend(base or C["panel"], color, 0.45), outline="")
            self.rrect(x0, y0, x1, y1, 3, fill=color, outline=blend(color, "#ffffff", 0.4))
            self.rect(x0 + 3, y0 + 1.5, x1 - 3, y0 + 3, fill=blend(color, "#ffffff", 0.55),
                      outline="")
        else:
            self.rrect(x0, y0, x1, y1, 3, fill=blend(color, "#000000", 0.72), outline="")

    def lamp_button(self, x0, y0, x1, y1, label, face, lamp_on=False, lamp_color=LAMP["white"],
                    on_press=None, enabled=True, size=18):
        """Industrial push-button: bezel, raised face, label and a lamp window that is
        lit while the thing the button controls is active."""
        pressed = enabled and self.is_pressed(x0, y0, x1, y1)
        face = face if enabled else blend(face, C["panel"], 0.65)
        # housing
        self.rrect(x0, y0, x1, y1, 7, fill="#151c25", outline="#33414f")
        fx0, fy0, fx1, fy1 = x0 + 5, y0 + 5, x1 - 5, y1 - 7
        if pressed:   # face pushed in: no drop shadow, shifted down, darker
            fy0, fy1 = fy0 + 2, fy1 + 2
            face = blend(face, "#000000", 0.25)
        else:
            self.rrect(fx0, fy0 + 3, fx1, fy1 + 3, 5, fill="#06090c", outline="")
        self.rrect(fx0, fy0, fx1, fy1, 5, fill=face, outline=blend(face, "#ffffff", 0.3))
        self.rrect(fx0 + 3, fy0 + 2, fx1 - 3, fy0 + (fy1 - fy0) * 0.42, 4,
                   fill=blend(face, "#ffffff", 0.10), outline="")
        txt = blend("#ffffff", face, 0.15) if enabled else blend("#ffffff", face, 0.6)
        # label centred in the face above the lamp window
        self.text((fx0 + fx1) / 2, (fy0 + fy1 - 16) / 2, label, size, bold=True, fill=txt,
                  justify="center")
        w = fx1 - fx0
        self._lamp(fx0 + w * 0.2, fy1 - 14, fx1 - w * 0.2, fy1 - 6, lamp_on and enabled,
                   lamp_color, base=face)
        if on_press and enabled:
            self._hot.append((x0, y0, x1, y1, on_press))

    def indicator(self, cx, y, label, on, color, w=56):
        """Stand-alone pilot lamp with a caption above it."""
        self.text(cx, y, label, 13, bold=True, fill=C["text"] if on else C["dim"])
        self._lamp(cx - w / 2, y + 12, cx + w / 2, y + 28, on, color)

    def value_button(self, x0, y0, x1, y1, label, value, unit, fmt="{:.1f}", on_press=None,
                     color=None, size=26):
        """Tap-to-edit numeric display (setpoints). Needs ~56 units of height."""
        pressed = self.is_pressed(x0, y0, x1, y1)
        self.rrect(x0, y0, x1, y1, 6, fill="#151c25", outline=C["accent"] if pressed else "#3d8bd1")
        self.rect(x0 + 5, y0 + 5, x1 - 5, y1 - 5, fill="#1b2633" if pressed else C["well"],
                  outline="")
        self.text(x0 + 11, y0 + 16, label, 13, bold=True, anchor="w", fill=C["dim"])
        self.text(x1 - 11, y0 + 16, "TAP ✎", 13, bold=True, anchor="e", fill=C["accent"])
        s = fmt.format(value) if _valid(value) else "---"
        self.text(x1 - 36, y1 - 20, s, size, mono=True, bold=True, anchor="e",
                  fill=(color or C["setp"]) if _valid(value) else C["dim"])
        self.text(x1 - 11, y1 - 18, unit, 13, anchor="e", fill=C["dim"])
        if on_press:
            self._hot.append((x0, y0, x1, y1, on_press))
