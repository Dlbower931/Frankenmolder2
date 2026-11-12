#!/usr/bin/env python3
"""
Frankenmolder - Watchdog page: CAN link health, active alarms, and a switchable view of
live sensor readouts (default), the ROS log, or raw CAN frames.

Pieces:
    LinkMonitor      decides whether the CAN link is up, from the watchdog node's
                     /watchdog/status and the GUI's own "last ESP32 temperature" time,
                     and tracks the failure alarm until the operator acknowledges it.
    build_alarms()   list of active alarms for the page.
    WatchdogPage     the tab (HmiCanvas). set_data() reads data["watchdog"]:
                       {"status": <decoded /watchdog/status or None>, "status_age": s|None,
                        "link": LinkMonitor.snapshot(), "alarms": [...], "logs": [...],
                        "sensors": [...], "raw_can": [...]}
                     logs are newest-last dicts {"t", "lvl", "node", "msg"}.
                     sensors are rows {"name", "value", "unit", "info", "level", "age"}
                     (level: ok / warn / alarm / dim; value already formatted or None).
                     raw_can are newest-last dicts {"t", "id", "data": bytes} from
                     /can_bus_raw_string (frames the Pi received from the ESP32).
    LinkAlarmPopup   flashing must-acknowledge popup shown when the link fails.

Standalone preview with a simulated link failure:  python3 watchdog_page.py
"""

import re
import struct
import time
import tkinter as tk

from hmi import C, FACE, LAMP, HmiCanvas, _valid, blend

LOG_LEVELS = ("DEBUG", "INFO", "WARN", "ERROR", "FATAL")
LEVEL_COLORS = {"DEBUG": C["dim"], "INFO": C["text"], "WARN": C["warn"], "ERROR": C["alarm"],
                "FATAL": C["alarm"]}
FILTERS = (("ALL", 0), ("WARN+", 2), ("ERROR", 3))   # minimum index into LOG_LEVELS
ROSOUT_LEVELS = {1: "DEBUG", 2: "INFO", 4: "WARN", 8: "ERROR", 16: "FATAL"}
VIEWS = ("SENSORS", "ROS LOG", "RAW CAN")
STALE_S = 2.0   # a sensor reading older than this is shown in amber

# CAN IDs -> short names (must match can_bridge_node.py / the ESP32 firmware)
CAN_NAMES = {0x101: "Z1 TEMP", 0x102: "Z2 TEMP", 0x103: "Z3 TEMP", 0x104: "Z4 TEMP",
             0x111: "Z1 STATE", 0x112: "Z2 STATE", 0x113: "Z3 STATE", 0x114: "Z4 STATE",
             0x130: "SERVO STAT", 0x131: "SERVO POS", 0x132: "DRIVE 1", 0x133: "DRIVE 2",
             0x134: "PARAM REPLY", 0x200: "HEARTBEAT", 0x201: "Z1 SETP", 0x202: "Z2 SETP",
             0x203: "Z3 SETP", 0x204: "Z4 SETP", 0x210: "HEAT CMD", 0x230: "SERVO CMD",
             0x231: "SERVO SPEED", 0x232: "SERVO MOVE", 0x233: "PARAM CMD"}
SERVO_STATES = {0: "IDLE", 1: "RUNNING", 2: "MOVING", 3: "STOPPING", 4: "FAULT"}
_RAW_RE = re.compile(r"Timestamp:\s*([\d.]+).*?ID:\s*([0-9a-fA-F]+).*?DL:\s*(\d+)\s+"
                     r"((?:[0-9a-fA-F]{2}\s)*)")


def parse_raw_can(text, now=None):
    """python-can's str(Message) -> {"t", "id", "data"}; None if it does not parse."""
    m = _RAW_RE.search(text + " ")
    if not m:
        return None
    data = bytes(int(b, 16) for b in m.group(4).split())[:int(m.group(3))]
    return {"t": float(m.group(1)) or (now or time.time()), "id": int(m.group(2), 16),
            "data": data}


def decode_can(can_id, data):
    """Human-readable value of a frame, or '' if unknown."""
    try:
        if 0x101 <= can_id <= 0x104 and len(data) == 4:
            v = struct.unpack("<f", data)[0]
            return "FAULT (NaN)" if v != v else f"{v:.1f} °C"
        if 0x201 <= can_id <= 0x204 and len(data) == 4:
            return f"set {struct.unpack('<f', data)[0]:.1f} °C"
        if 0x111 <= can_id <= 0x114:
            return data.decode("ascii", "replace").strip("\x00")
        if can_id == 0x130 and len(data) == 8:
            st, flags, hz = struct.unpack("<BB2xf", data)
            return f"{SERVO_STATES.get(st, st)} {hz:.0f} Hz f=0x{flags:02X}"
        if can_id == 0x131 and len(data) == 8:
            pos, tgt = struct.unpack("<ii", data)
            return f"pos {pos} tgt {tgt} Hz"
        if can_id == 0x231 and len(data) == 8:
            hz, acc = struct.unpack("<ff", data)
            return f"{hz:+.0f} Hz acc {acc:.0f}"
        if can_id == 0x132 and len(data) == 8:
            rpm, tq, cur, alm = struct.unpack("<hhhH", data)
            return f"{rpm} rpm {tq}% alm {alm}"
    except (struct.error, ValueError):
        pass
    return ""


def short_node(name):
    """'/topic_watchdog_1234_1699999999' -> 'topic_watchdog', '/can_bridge_node' -> 'can_bridge'."""
    name = re.sub(r"(_\d+)+$", "", (name or "").lstrip("/"))
    return name[:-5] if name.endswith("_node") else name


def fmt_age(age):
    if not _valid(age):
        return "never"
    if age < 10:
        return f"{age:.1f} s"
    if age < 120:
        return f"{age:.0f} s"
    return f"{age / 60:.0f} min"


class LinkMonitor:
    GRACE_S = 15.0        # after GUI start, before a missing ESP32 counts as a failure
    WD_STALE_S = 3.5      # watchdog publishes every 1 s
    TEMP_TIMEOUT_S = 5.0  # ESP32 sends temperatures every 500 ms

    def __init__(self, now=None):
        self.start = now if now is not None else time.time()
        self.ok = True
        self.reason = "Starting up"
        self.failed_at = None
        self.fail_reason = None
        self.restored_at = None
        self.acked = True

    def evaluate(self, now, wd_status, wd_age, temp_age):
        can = (wd_status or {}).get("can") if (wd_age is not None and wd_age < self.WD_STALE_S) else None
        temp_fresh = temp_age is not None and temp_age < self.TEMP_TIMEOUT_S
        in_grace = now - self.start < self.GRACE_S
        if can is not None and not can.get("ok", True):
            return False, can.get("reason") or "Watchdog reports the CAN link down"
        if not temp_fresh:
            if temp_age is None:
                if in_grace:
                    return True, "Starting up (waiting for ESP32 data)"
                return False, "No data received from the ESP32"
            return False, f"No data from the ESP32 for {temp_age:.0f} s"
        if can is None:
            return True, "OK (watchdog node not reporting)"
        return True, can.get("reason") or "OK"

    def update(self, now, wd_status, wd_age, temp_age):
        """Returns "failed", "restored" or None (no change)."""
        ok, reason = self.evaluate(now, wd_status, wd_age, temp_age)
        change = None
        if not ok and self.ok:
            self.failed_at, self.fail_reason, self.restored_at = now, reason, None
            self.acked = False
            change = "failed"
        elif ok and not self.ok:
            self.restored_at = now
            change = "restored"
        self.ok, self.reason = ok, reason
        return change

    def acknowledge(self):
        self.acked = True

    def snapshot(self):
        return {"ok": self.ok, "reason": self.reason, "failed_at": self.failed_at,
                "fail_reason": self.fail_reason, "restored_at": self.restored_at,
                "acked": self.acked}


def build_alarms(link, status, status_age, now=None):
    """Active alarms, most important first: dicts {"text", "level", "acked"}."""
    now = now if now is not None else time.time()
    alarms = []
    if not link["ok"]:
        alarms.append({"text": f"CAN LINK DOWN - {link['reason']}", "level": "alarm",
                       "acked": link["acked"]})
    elif not link["acked"] and link["failed_at"]:
        alarms.append({"text": f"CAN link failed {time.strftime('%H:%M:%S', time.localtime(link['failed_at']))}"
                               f" ({link['fail_reason']}) - now restored", "level": "warn",
                       "acked": False})
    elif link["reason"].startswith("Link degraded"):
        alarms.append({"text": link["reason"], "level": "warn", "acked": None})
    if status is None or status_age is None or status_age > LinkMonitor.WD_STALE_S:
        alarms.append({"text": "Watchdog node not reporting (/watchdog/status)", "level": "warn",
                       "acked": None})
    else:
        can = status.get("can", {})
        if can.get("restart_ms") == 0:
            alarms.append({"text": f"{can.get('iface', 'can0')} restart-ms is 0: no auto-recovery "
                                   "from bus-off", "level": "warn", "acked": None})
        if link["ok"]:
            for topic, t in sorted(status.get("topics", {}).items()):
                if not t.get("ok"):
                    alarms.append({"text": f"{topic} stale ({fmt_age(t.get('age'))})",
                                   "level": "warn", "acked": None})
    return alarms


class WatchdogPage(HmiCanvas):
    LEFT = (8, 340)
    RIGHT = (348, 992)
    ALARM_Y = (44, 172)
    LOG_Y = (180, 520)
    ROW_H = 20

    def __init__(self, parent, on_clear_logs=None, on_clear_raw=None, tick_ms=250, **kw):
        self.on_clear_logs = on_clear_logs or (lambda: None)
        self.on_clear_raw = on_clear_raw or (lambda: None)
        self.wd = {"status": None, "status_age": None, "link": LinkMonitor().snapshot(),
                   "alarms": [], "logs": [], "sensors": [], "raw_can": []}
        self.view = "SENSORS"
        self.filter_idx = 0
        self.scroll = 0
        self.detail = None      # log entry shown full-size
        self.raw_frozen = None  # raw CAN list while PAUSEd
        super().__init__(parent, tick_ms=tick_ms, **kw)

    def set_data(self, data):
        self.wd = data.get("watchdog", self.wd)

    # ------------------------------------------------------------ actions
    def _cycle_filter(self):
        self.filter_idx = (self.filter_idx + 1) % len(FILTERS)
        self.scroll = 0

    def _scroll(self, rows):
        self.scroll = max(0, self.scroll + rows)

    def _clear(self):
        self.scroll, self.detail = 0, None
        self.on_clear_logs()

    def _set_view(self, view):
        self.view, self.scroll, self.detail = view, 0, None

    def _toggle_pause(self):
        self.raw_frozen = None if self.raw_frozen is not None else list(self.wd.get("raw_can", []))
        self.scroll = 0

    def _clear_raw(self):
        self.raw_frozen, self.scroll = None, 0
        self.on_clear_raw()

    def _raw_frames(self):
        frames = self.raw_frozen if self.raw_frozen is not None else self.wd.get("raw_can", [])
        return list(reversed(frames))

    def _filtered_logs(self):
        lo = FILTERS[self.filter_idx][1]
        return [e for e in reversed(self.wd.get("logs", []))
                if LOG_LEVELS.index(e.get("lvl", "INFO")) >= lo]

    # ------------------------------------------------------------ drawing
    def render(self):
        self.begin()
        self.background()
        link = self.wd["link"]
        n = len(self.wd.get("alarms", []))
        if not link["ok"]:
            self.title_bar("CAN LINK DOWN", C["alarm"])
        elif n:
            self.title_bar(f"{n} ACTIVE ALARM{'S' if n > 1 else ''}", C["warn"])
        else:
            self.title_bar("ALL SYSTEMS OK", C["ok"])
        self._can_panel()
        self._topics_panel()
        self._alarm_panel()
        if self.view == "SENSORS":
            self._sensor_panel()
        elif self.view == "RAW CAN":
            self._raw_panel()
        else:
            self._log_panel()
        if self.detail:
            self._detail_overlay()

    def _blink(self):
        return int(time.time() * 2) % 2 == 0

    def _can_panel(self):
        x0, x1 = self.LEFT
        y0, y1 = 44, 340
        st = self.wd.get("status") or {}
        age = self.wd.get("status_age")
        fresh = st and age is not None and age < LinkMonitor.WD_STALE_S
        can = st.get("can", {}) if fresh else {}
        link = self.wd["link"]
        self.panel(x0, y0, x1, y1, "CAN LINK", chip="OK" if link["ok"] else "DOWN",
                   chip_color=C["ok"] if link["ok"] else C["alarm"])

        # big status banner
        by0, by1 = y0 + self.HDR + 8, y0 + self.HDR + 52
        if link["ok"]:
            col, on = C["ok"], True
        else:
            col, on = C["alarm"], self._blink()
        self.rect(x0 + 8, by0, x1 - 8, by1, fill=col if on else C["well"], outline=col,
                  width=max(1, 2 * self.s))
        self.text((x0 + x1) / 2, (by0 + by1) / 2 - 8, "LINK UP" if link["ok"] else "LINK DOWN",
                  18, mono=True, bold=True, fill="#0a0e13" if on else col)
        reason = link["reason"]
        if len(reason) > 38:
            reason = reason[:37] + "…"
        self.text((x0 + x1) / 2, (by0 + by1) / 2 + 11, reason, 12,
                  fill="#0a0e13" if on else C["text"])

        # lamps
        ly = by1 + 12
        state = can.get("state") if can else None
        iface_ok = state in ("ERROR-ACTIVE", "ERROR-WARNING")
        esp_age = can.get("esp32_age") if can else None
        esp_ok = _valid(esp_age) and esp_age < LinkMonitor.TEMP_TIMEOUT_S
        auto = can.get("restart_ms") if can else None
        w = (x1 - x0) / 3
        self.indicator(x0 + w * 0.5, ly, "PI CAN", iface_ok,
                       LAMP["green"] if iface_ok else LAMP["red"], w=60)
        self.indicator(x0 + w * 1.5, ly, "ESP32", esp_ok, LAMP["green"], w=60)
        self.indicator(x0 + w * 2.5, ly, "AUTO-RST", bool(auto), LAMP["green"], w=60)

        # readouts
        ry = ly + 50
        self.readout(x0, x1, ry, "Controller", state or ("no data" if not fresh else "?"), "",
                     size=15, color=C["ok"] if iface_ok else C["alarm"])
        self.readout(x0, x1, ry + 22, "Last ESP32 data", fmt_age(esp_age) if can else "---", "",
                     size=15)
        br = can.get("bitrate")
        self.readout(x0, x1, ry + 44, "Bitrate / restart-ms",
                     f"{br // 1000 if br else '?'}k / {auto if auto is not None else '?'}", "",
                     size=15, color=C["value"] if auto else C["warn"])

        # counters
        xs, rx, tx = can.get("xstats", {}), can.get("rx", {}), can.get("tx", {})
        counters = (("BUS-OFF", xs.get("bus_off")), ("RESTART", xs.get("restarts")),
                    ("ERR-PASS", xs.get("error_passive")), ("ERR-WARN", xs.get("error_warning")),
                    ("BUS ERR", xs.get("bus_error")), ("TX DROP", tx.get("dropped")))
        cy = ry + 66
        cw = (x1 - x0 - 16) / 3
        for i, (label, val) in enumerate(counters):
            r, c = divmod(i, 3)
            cx, yy = x0 + 8 + c * cw, cy + r * 40
            self.text(cx + 4, yy, label, 11, bold=True, anchor="w", fill=C["dim"])
            bad = isinstance(val, int) and val > 0 and label != "RESTART"
            self.text(cx + 4, yy + 18, "---" if val is None else str(val), 16, mono=True, bold=True,
                      anchor="w", fill=C["warn"] if bad else C["value"])

    def _topics_panel(self):
        x0, x1 = self.LEFT
        y0, y1 = 348, 520
        st = self.wd.get("status") or {}
        age = self.wd.get("status_age")
        fresh = st and age is not None and age < LinkMonitor.WD_STALE_S
        self.panel(x0, y0, x1, y1, "WATCHED TOPICS",
                   chip="LIVE" if fresh else "NO WATCHDOG",
                   chip_color=C["ok"] if fresh else C["warn"])
        topics = sorted((st.get("topics") or {}).items()) if fresh else []
        if not topics:
            self.text((x0 + x1) / 2, (y0 + y1) / 2 + 10,
                      "Waiting for /watchdog/status" if not fresh else "No topics", 14,
                      fill=C["dim"])
            return
        for i, (topic, t) in enumerate(topics[:6]):
            y = y0 + self.HDR + 18 + i * 24
            ok = t.get("ok")
            self._lamp(x0 + 10, y - 5, x0 + 24, y + 5, True, LAMP["green"] if ok else LAMP["red"])
            name = topic.replace("/extruder/", "").replace("/temperature", " temp")
            self.text(x0 + 32, y, name, 14, mono=True, anchor="w", fill=C["text"])
            self.text(x1 - 10, y, fmt_age(t.get("age")), 14, mono=True, bold=True, anchor="e",
                      fill=C["value"] if ok else C["alarm"])

    def _alarm_panel(self):
        x0, x1 = self.RIGHT
        y0, y1 = self.ALARM_Y
        alarms = self.wd.get("alarms", [])
        self.panel(x0, y0, x1, y1, "ACTIVE ALARMS", chip=str(len(alarms)) if alarms else "NONE",
                   chip_color=C["alarm"] if any(a["level"] == "alarm" for a in alarms)
                   else C["warn"] if alarms else C["ok"])
        if not alarms:
            self.text((x0 + x1) / 2, (y0 + self.HDR + y1) / 2, "No active alarms", 15,
                      fill=C["dim"])
            return
        blink = self._blink()
        for i, a in enumerate(alarms[:4]):
            y = y0 + self.HDR + 14 + i * 23
            col = C["alarm"] if a["level"] == "alarm" else C["warn"]
            flashing = a["acked"] is False and not blink
            self.rect(x0 + 8, y - 9, x0 + 14, y + 9, fill=C["well"] if flashing else col,
                      outline="")
            txt = a["text"] if len(a["text"]) <= 64 else a["text"][:63] + "…"
            self.text(x0 + 22, y, txt, 13, mono=True, anchor="w", fill=col)
            if a["acked"] is False:
                self.chip(x1 - 8, y, "UNACK", col, size=11, h=9)
            elif a["acked"]:
                self.chip(x1 - 8, y, "ACKED", C["off"], fg=C["text"], size=11, h=9)

    # ------------------------------------------------------------ right-hand views
    BTN_W = 86

    def _buttons(self, extra):
        """Button column: three view selectors, then the view's own buttons.
        extra: [(label, face, callback, enabled, lamp_on)]"""
        x1 = self.RIGHT[1]
        bx0 = x1 - self.BTN_W
        top, y1 = self.LOG_Y[0] + self.HDR + 4, self.LOG_Y[1] - 6
        btns = [(v, FACE["blue"], (lambda v=v: self._set_view(v)), True, self.view == v)
                for v in VIEWS] + list(extra)
        bh = (y1 - top) / 7
        for i, (label, face, cb, enabled, lamp) in enumerate(btns):
            gap = 6 if i == len(VIEWS) else 0     # separate the view selectors
            self.lamp_button(bx0, top + i * bh + gap, x1 - 4, top + (i + 1) * bh - 2, label, face,
                             lamp_on=lamp, lamp_color=LAMP["amber"] if i >= len(VIEWS)
                             else LAMP["blue"], on_press=cb, enabled=enabled, size=11)
        return bx0

    def _scroll_buttons(self, rows, total):
        return [("▲ NEWER", FACE["neutral"], lambda: self._scroll(-rows // 2), self.scroll > 0,
                 self.scroll > 0),
                ("▼ OLDER", FACE["neutral"], lambda: self._scroll(rows // 2),
                 self.scroll + rows < total, False)]

    def _sensor_panel(self):
        x0, x1 = self.RIGHT
        y0, y1 = self.LOG_Y
        rows = self.wd.get("sensors", [])
        stale = sum(1 for r in rows if _valid(r.get("age")) and r["age"] > STALE_S)
        self.panel(x0, y0, x1, y1, "LIVE SENSOR READOUTS",
                   chip=f"{stale} STALE" if stale else "LIVE",
                   chip_color=C["warn"] if stale else C["ok"])
        bx0 = self._buttons([])
        top = y0 + self.HDR + 4
        # column header
        self.text(x0 + 8, top + 8, "SIGNAL", 11, bold=True, anchor="w", fill=C["dim"])
        self.text(x0 + 236, top + 8, "VALUE", 11, bold=True, anchor="e", fill=C["dim"])
        self.text(x0 + 300, top + 8, "DETAIL", 11, bold=True, anchor="w", fill=C["dim"])
        self.text(bx0 - 10, top + 8, "AGE", 11, bold=True, anchor="e", fill=C["dim"])
        if not rows:
            self.text((x0 + bx0) / 2, (top + y1) / 2, "No sensor data yet", 15, fill=C["dim"])
            return
        lvl_col = {"ok": C["ok"], "warn": C["warn"], "alarm": C["alarm"], "dim": C["dim"]}
        n = int((y1 - top - 22) // self.ROW_H)
        for i, r in enumerate(rows[:n]):
            y = top + 28 + i * self.ROW_H
            if i % 2:
                self.rect(x0 + 2, y - 10, bx0 - 6, y + 10, fill="#0e151d", outline="")
            age = r.get("age")
            old = not _valid(age) or age > STALE_S
            col = lvl_col.get(r.get("level"), C["value"])
            self.rect(x0 + 6, y - 6, x0 + 10, y + 6, fill=col, outline="")
            self.text(x0 + 16, y, r["name"], 13, mono=True, anchor="w", fill=C["text"])
            val = r.get("value")
            self.text(x0 + 236, y, "---" if val is None else str(val), 14, mono=True, bold=True,
                      anchor="e", fill=C["dim"] if val is None else col if r.get("level") in
                      ("warn", "alarm") else C["value"])
            self.text(x0 + 242, y, r.get("unit", ""), 12, anchor="w", fill=C["dim"])
            info = r.get("info", "")
            maxc = int((bx0 - 70 - (x0 + 300)) / 7.2)
            if len(info) > maxc:
                info = info[:maxc - 1] + "…"
            self.text(x0 + 300, y, info, 12, mono=True, anchor="w", fill=C["dim"])
            self.text(bx0 - 10, y, fmt_age(age), 12, mono=True, anchor="e",
                      fill=C["warn"] if old else C["dim"])

    def _raw_panel(self):
        x0, x1 = self.RIGHT
        y0, y1 = self.LOG_Y
        frames = self._raw_frames()
        paused = self.raw_frozen is not None
        self.panel(x0, y0, x1, y1, "RAW CAN  (received from ESP32)",
                   chip=f"PAUSED  {len(frames)}" if paused else f"LIVE  {len(frames)}",
                   chip_color=C["warn"] if paused else C["dim"])
        rows = int((y1 - y0 - self.HDR - 8) // self.ROW_H)
        self.scroll = min(self.scroll, max(0, len(frames) - rows))
        bx0 = self._buttons(self._scroll_buttons(rows, len(frames)) + [
            ("PAUSE", FACE["neutral"], self._toggle_pause, True, paused),
            ("CLEAR", FACE["stop"], self._clear_raw, bool(frames), False)])
        top = y0 + self.HDR + 4
        if not frames:
            self.text((x0 + bx0) / 2, (top + y1) / 2, "No CAN frames received", 15, fill=C["dim"])
            return
        for i, f in enumerate(frames[self.scroll:self.scroll + rows]):
            y = top + 10 + i * self.ROW_H
            if i % 2:
                self.rect(x0 + 2, y - 10, bx0 - 6, y + 10, fill="#0e151d", outline="")
            t = f.get("t", 0)
            ts = time.strftime("%H:%M:%S", time.localtime(t)) + f".{int((t % 1) * 1000):03d}"
            self.text(x0 + 8, y, ts, 12, mono=True, anchor="w", fill=C["dim"])
            cid = f.get("id", 0)
            self.text(x0 + 104, y, f"{cid:03X}", 12, mono=True, bold=True, anchor="w",
                      fill=C["setp"])
            self.text(x0 + 138, y, CAN_NAMES.get(cid, "?")[:11], 12, mono=True, anchor="w",
                      fill=C["text"])
            data = f.get("data", b"")
            self.text(x0 + 230, y, " ".join(f"{b:02X}" for b in data), 12, mono=True,
                      anchor="w", fill=C["value"])
            dec = decode_can(cid, data)
            if len(dec) > 19:
                dec = dec[:18] + "…"
            self.text(x0 + 408, y, dec, 12, mono=True, anchor="w", fill=C["ok"])

    def _log_panel(self):
        x0, x1 = self.RIGHT
        y0, y1 = self.LOG_Y
        logs = self._filtered_logs()
        fname = FILTERS[self.filter_idx][0]
        self.panel(x0, y0, x1, y1, "ROS LOG", chip=f"{fname}  {len(logs)}", chip_color=C["dim"])
        rows = int((y1 - y0 - self.HDR - 8) // self.ROW_H)
        self.scroll = min(self.scroll, max(0, len(logs) - rows))
        bx0 = self._buttons(self._scroll_buttons(rows, len(logs)) + [
            (f"▸ {FILTERS[(self.filter_idx + 1) % len(FILTERS)][0]}", FACE["neutral"],
             self._cycle_filter, True, False),
            ("CLEAR", FACE["stop"], self._clear, bool(self.wd.get("logs")), False)])
        top = y0 + self.HDR + 4
        if not logs:
            self.text((x0 + bx0) / 2, (top + y1) / 2, "No log messages", 15, fill=C["dim"])
        for i, e in enumerate(logs[self.scroll:self.scroll + rows]):
            y = top + 10 + i * self.ROW_H
            lvl = e.get("lvl", "INFO")
            col = LEVEL_COLORS.get(lvl, C["text"])
            if i % 2:
                self.rect(x0 + 2, y - 10, bx0 - 6, y + 10, fill="#0e151d", outline="")
            t = time.strftime("%H:%M:%S", time.localtime(e.get("t", 0)))
            self.text(x0 + 8, y, t, 12, mono=True, anchor="w", fill=C["dim"])
            self.text(x0 + 78, y, lvl[:5], 12, mono=True, bold=True, anchor="w", fill=col)
            node = short_node(e.get("node"))[:14]
            self.text(x0 + 128, y, node, 12, mono=True, anchor="w", fill=C["setp"])
            msg = " ".join(str(e.get("msg", "")).split())
            maxc = int((bx0 - 6 - (x0 + 232)) / 7.4)
            if len(msg) > maxc:
                msg = msg[:maxc - 1] + "…"
            self.text(x0 + 232, y, msg, 12, mono=True, anchor="w",
                      fill=col if lvl in ("WARN", "ERROR", "FATAL") else C["text"])
            self._hot.append((x0 + 2, y - 10, bx0 - 6, y + 10, lambda e=e: self._show(e)))

    def _show(self, entry):
        self.detail = entry

    def _detail_overlay(self):
        e = self.detail
        x0, y0, x1, y1 = self.RIGHT[0] + 20, self.LOG_Y[0] + 30, self.RIGHT[1] - 20, self.LOG_Y[1] - 20
        lvl = e.get("lvl", "INFO")
        col = LEVEL_COLORS.get(lvl, C["text"])
        self.rect(x0, y0, x1, y1, fill=C["head"], outline=col, width=max(1, 2 * self.s))
        t = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(e.get("t", 0)))
        self.text(x0 + 12, y0 + 18, f"{lvl}   {t}   {e.get('node', '')}", 13, mono=True, bold=True,
                  anchor="w", fill=col)
        self.line(x0 + 8, y0 + 32, x1 - 8, y0 + 32, fill=C["edge"])
        self.cv.create_text(*self.P(x0 + 12, y0 + 42), text=str(e.get("msg", "")), anchor="nw",
                            width=(x1 - x0 - 24) * self.s, fill=C["value"],
                            font=(self.mono, -max(6, int(round(14 * self.s)))))
        self.text((x0 + x1) / 2, y1 - 14, "TAP TO CLOSE", 12, bold=True, fill=C["accent"])
        self._hot.append((self.RIGHT[0], self.ALARM_Y[0], self.RIGHT[1], self.LOG_Y[1],
                          lambda: setattr(self, "detail", None)))


class LinkAlarmPopup:
    """Flashing popup for a CAN link failure. It stays until ACKNOWLEDGE is pressed; if the
    link comes back first it changes to a 'restored' message but still needs the ack."""

    W, H = 620, 340
    FLASH_MS = 450

    def __init__(self, master, on_ack, reason, failed_at):
        self.on_ack = on_ack
        self.restored_at = None
        self.flash_on = True
        self.top = tk.Toplevel(master)
        self.top.title("CAN LINK FAILURE")
        self.top.transient(master)
        self.top.attributes("-topmost", True)
        self.top.protocol("WM_DELETE_WINDOW", lambda: None)   # must be acknowledged
        self.top.resizable(False, False)
        sw, sh = self.top.winfo_screenwidth(), self.top.winfo_screenheight()
        w, h = min(self.W, sw - 20), min(self.H, sh - 20)
        self.top.geometry(f"{w}x{h}+{(sw - w) // 2}+{(sh - h) // 2}")

        self.box = tk.Frame(self.top, bd=0, highlightthickness=6)
        self.box.pack(fill="both", expand=True)
        self.title = tk.Label(self.box, font=("DejaVu Sans", 30, "bold"))
        self.title.pack(pady=(18, 4))
        self.reason = tk.Label(self.box, font=("DejaVu Sans", 15, "bold"), wraplength=w - 60,
                               justify="center")
        self.reason.pack(pady=4)
        self.detail = tk.Label(self.box, font=("DejaVu Sans", 12), wraplength=w - 60,
                               justify="center")
        self.detail.pack(pady=(4, 10))
        self.btn = tk.Button(self.box, text="ACKNOWLEDGE", font=("DejaVu Sans", 22, "bold"),
                             command=self.acknowledge, bg=FACE["neutral"], fg="#ffffff",
                             activebackground=blend(FACE["neutral"], "#ffffff", 0.15),
                             activeforeground="#ffffff", relief=tk.FLAT, bd=0,
                             highlightthickness=2, highlightbackground="#ffffff", cursor="hand2")
        self.btn.pack(fill="x", padx=40, pady=(0, 20), ipady=14, side="bottom")
        self.failed(reason, failed_at)
        self._flash()
        self.top.after(150, self._grab)

    def _grab(self):
        try:
            self.top.lift()
            self.top.grab_set()
            self.btn.focus_set()
        except tk.TclError:
            self.top.after(250, self._grab)    # not viewable yet

    def failed(self, reason, failed_at):
        self.restored_at = None
        self.failed_at = failed_at
        self.title.config(text="⚠  CAN LINK FAILURE")
        self.reason.config(text=reason)
        self.detail.config(text=f"Link lost at {time.strftime('%H:%M:%S', time.localtime(failed_at))}."
                                "\nThe ESP32 turns all heaters OFF after 5 s without CAN messages.")
        self._paint()

    def restored(self, restored_at):
        self.restored_at = restored_at
        self.title.config(text="CAN LINK RESTORED")
        self.detail.config(text=f"Link lost at {time.strftime('%H:%M:%S', time.localtime(self.failed_at))}, "
                                f"restored at {time.strftime('%H:%M:%S', time.localtime(restored_at))}."
                                "\nCheck the heater zones - they may have been turned OFF.")
        self._paint()

    def _paint(self):
        if self.restored_at is None:
            bg = C["alarm"] if self.flash_on else "#2a0d0c"
            fg = "#ffffff"
        else:
            bg, fg = C["head"], C["warn"]
        edge = "#ffffff" if (self.restored_at is None and self.flash_on) else C["warn"]
        self.box.config(bg=bg, highlightbackground=edge, highlightcolor=edge)
        for lab in (self.title, self.reason, self.detail):
            lab.config(bg=bg, fg=fg if lab is not self.detail else
                       (fg if self.restored_at is None else C["text"]))

    def _flash(self):
        if not self.top.winfo_exists():
            return
        self.flash_on = not self.flash_on
        self._paint()
        self.top.lift()
        self.top.after(self.FLASH_MS, self._flash)

    def acknowledge(self):
        try:
            self.top.grab_release()
        except tk.TclError:
            pass
        self.top.destroy()
        self.on_ack()


# ---------------------------------------------------------------- demo
def main():
    import argparse
    import random

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--size", default="800x420", help="800x420 = 7in screen minus tabs/status")
    ap.add_argument("--fail-at", type=float, default=6.0, help="seconds until the simulated failure")
    ap.add_argument("--restore-after", type=float, default=8.0)
    args = ap.parse_args()

    root = tk.Tk()
    root.title("Frankenmolder - Watchdog")
    root.geometry(args.size + "+40+60")
    logs = []
    page = WatchdogPage(root, on_clear_logs=logs.clear)
    page.pack(fill="both", expand=True)
    t0 = time.time()
    mon = LinkMonitor(now=t0 - LinkMonitor.GRACE_S)
    popup = {"w": None}
    nodes = ("/can_bridge_node", "/topic_watchdog_1234", "/extruder_gui_node_99")
    xs = {"restarts": 0, "bus_error": 0, "arbitration_lost": 0, "error_warning": 0,
          "error_passive": 0, "bus_off": 0}

    def log(lvl, node, msg):
        logs.append({"t": time.time(), "lvl": lvl, "node": node, "msg": msg})

    def ack():
        mon.acknowledge()
        popup["w"] = None

    def pump():
        now = time.time()
        t = now - t0
        down = args.fail_at <= t < args.fail_at + args.restore_after
        if down and xs["bus_off"] == 0:
            xs.update(bus_off=1, error_passive=1, error_warning=1, restarts=1)
        if random.random() < 0.3:
            log(random.choice(("INFO", "INFO", "INFO", "WARN")), random.choice(nodes),
                random.choice(("GUI: zone1: setpoint 200.0 °C sent.",
                               "Published temp zone2: 201.4",
                               "CAN TX failed: [Errno 105] No buffer space available",
                               "Screw RPM 25.0 sent.")))
        can = {"iface": "can0", "ok": not down,
               "reason": "Pi CAN controller is BUS-OFF" if down else "OK",
               "state": "BUS-OFF" if down else "ERROR-ACTIVE", "operstate": "UP",
               "restart_ms": 100, "bitrate": 500000,
               "esp32_age": 7.0 if down else 0.3, "xstats": dict(xs),
               "rx": {"packets": 5786, "errors": 0, "dropped": 0},
               "tx": {"packets": 426, "errors": 1, "dropped": 2134 if xs["bus_off"] else 0}}
        status = {"stamp": now, "can": can,
                  "topics": {f"/extruder/zone{i}/temperature":
                             {"age": 7.0 if down else 0.4, "ok": not down} for i in range(1, 5)}}
        change = mon.update(now, status, 0.2, (t - args.fail_at) if down else 0.3)
        if change == "failed":
            log("ERROR", "/topic_watchdog_1234", f"Watchdog: CAN LINK DOWN - {mon.reason}")
            if popup["w"] is None:
                popup["w"] = LinkAlarmPopup(root, ack, mon.reason, mon.failed_at)
            else:
                popup["w"].failed(mon.reason, mon.failed_at)
        elif change == "restored":
            log("WARN", "/topic_watchdog_1234", "Watchdog: CAN link restored (OK)")
            if popup["w"] is not None:
                popup["w"].restored(mon.restored_at)
        link = mon.snapshot()
        page.set_data({"watchdog": {"status": status, "status_age": 0.2, "link": link,
                                    "alarms": build_alarms(link, status, 0.2, now),
                                    "logs": list(logs)}})
        root.after(250, pump)

    root.bind("<Escape>", lambda e: root.destroy())
    pump()
    root.mainloop()


if __name__ == "__main__":
    main()
