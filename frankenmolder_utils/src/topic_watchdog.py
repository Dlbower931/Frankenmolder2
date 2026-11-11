#!/usr/bin/env python3
"""
Frankenmolder watchdog.

1. Watches critical topics (the ESP32 zone temperatures) and logs when they go stale.
2. Polls the Pi's CAN interface (`ip -details -statistics -json link show can0`) for its
   controller state (ERROR-ACTIVE / ERROR-PASSIVE / BUS-OFF / STOPPED) and error counters,
   and logs bus-off / restart events.
3. Publishes everything once a second as JSON on /watchdog/status for the GUI's
   Watchdog tab:

    {"stamp": <unix time>,
     "can": {"iface": "can0", "ok": bool, "reason": str, "state": str, "operstate": str,
             "restart_ms": int|None, "bitrate": int|None, "esp32_age": float|None,
             "xstats": {"restarts", "bus_error", "arbitration_lost", "error_warning",
                        "error_passive", "bus_off"},
             "rx": {"packets", "errors", "dropped"}, "tx": {"packets", "errors", "dropped"}},
     "topics": {"<topic>": {"age": float|None, "ok": bool}}}

The parsing is pure Python (parse_ip_json / can_health) so it can be tested without ROS.
"""

import json
import re
import subprocess
import time

CAN_IFACE = "can0"
ESP32_TIMEOUT_S = 5.0          # matches the GUI's link timeout
GOOD_CAN_STATES = ("ERROR-ACTIVE", "ERROR-WARNING")
XSTAT_KEYS = ("restarts", "bus_error", "arbitration_lost", "error_warning", "error_passive",
              "bus_off")


def read_can_iface(iface=CAN_IFACE):
    """Run `ip` and return its output parsed by parse_ip_json(), or an error dict."""
    try:
        out = subprocess.run(["ip", "-details", "-statistics", "-json", "link", "show", iface],
                             capture_output=True, text=True, timeout=2.0)
    except FileNotFoundError:
        return {"error": "'ip' command not found (install iproute2)"}
    except subprocess.TimeoutExpired:
        return {"error": "'ip' command timed out"}
    if out.returncode != 0:
        return {"error": (out.stderr or out.stdout).strip() or f"ip exited {out.returncode}"}
    try:
        return parse_ip_json(out.stdout)
    except (ValueError, KeyError, IndexError, TypeError) as e:
        return parse_ip_text(subprocess.run(
            ["ip", "-details", "-statistics", "link", "show", iface],
            capture_output=True, text=True, timeout=2.0).stdout, json_error=str(e))


def parse_ip_json(text):
    link = json.loads(text)[0]
    info = link.get("linkinfo", {})
    data = info.get("info_data", {})
    xs = info.get("info_xstats", {})
    stats = link.get("stats64") or link.get("stats") or {}
    pick = lambda d: {k: d.get(k) for k in ("packets", "errors", "dropped")}
    return {
        "state": data.get("state", "UNKNOWN"),
        "operstate": link.get("operstate", "UNKNOWN"),
        "flags": link.get("flags", []),
        "restart_ms": data.get("restart_ms"),
        "bitrate": (data.get("bittiming") or {}).get("bitrate"),
        "xstats": {k: xs.get(k) for k in XSTAT_KEYS},
        "rx": pick(stats.get("rx", {})),
        "tx": pick(stats.get("tx", {})),
    }


def parse_ip_text(text, json_error=None):
    """Fallback for an `ip` without CAN JSON support: just the controller state."""
    m = re.search(r"can state (\S+)", text or "")
    up = re.search(r"<[^>]*\bUP\b[^>]*>", text or "")
    res = {"state": m.group(1) if m else "UNKNOWN", "operstate": "UP" if up else "DOWN",
           "flags": [], "restart_ms": None, "bitrate": None,
           "xstats": {k: None for k in XSTAT_KEYS}, "rx": {}, "tx": {}}
    r = re.search(r"restart-ms (\d+)", text or "")
    if r:
        res["restart_ms"] = int(r.group(1))
    if not m:
        res["error"] = f"could not parse ip output ({json_error or 'no can state'})"
    return res


def can_health(iface_info, esp32_age):
    """(ok, reason) for the whole CAN link: Pi interface usable AND ESP32 data arriving."""
    if "error" in iface_info and iface_info.get("state", "UNKNOWN") == "UNKNOWN":
        return False, f"Cannot read {CAN_IFACE}: {iface_info['error']}"
    state = iface_info.get("state", "UNKNOWN")
    if "UP" not in iface_info.get("flags", []) and iface_info.get("operstate") == "DOWN":
        return False, f"Pi interface {CAN_IFACE} is DOWN"
    if state == "BUS-OFF":
        return False, "Pi CAN controller is BUS-OFF"
    if state == "STOPPED":
        return False, "Pi CAN controller is STOPPED"
    if esp32_age is None:
        return False, "No data received from the ESP32 yet"
    if esp32_age > ESP32_TIMEOUT_S:
        return False, f"No data from the ESP32 for {esp32_age:.0f} s"
    if state not in GOOD_CAN_STATES:
        # ERROR-PASSIVE with data still arriving: degraded but working
        return True, f"Link degraded ({state})"
    return True, "OK"


class Watchdog:
    def __init__(self, topics, timeout_s=5.0, grace_s=10.0):
        import rospy
        from std_msgs.msg import Float32, String
        self.rospy = rospy
        rospy.init_node('topic_watchdog', anonymous=True)
        rospy.loginfo("Watchdog starting...")
        self.topics = topics
        self.timeout = timeout_s
        self.grace = grace_s
        self.start = time.time()
        self.last_rx = {t: None for t in topics}
        self.stale_logged = {t: False for t in topics}
        self.prev_can = None
        self.prev_ok = None
        for t in topics:
            rospy.Subscriber(t, Float32, lambda _m, t=t: self._on_msg(t))
            rospy.loginfo(f"Watchdog: monitoring {t}")
        self.pub = rospy.Publisher('/watchdog/status', String, queue_size=1, latch=True)
        rospy.Timer(rospy.Duration(1.0), self._check)
        rospy.loginfo(f"Watchdog initialized. Grace {grace_s:.0f} s, timeout {timeout_s:.0f} s, "
                      f"CAN interface {CAN_IFACE}")

    def _on_msg(self, topic):
        self.last_rx[topic] = time.time()
        if self.stale_logged[topic]:
            self.rospy.logwarn(f"Watchdog: {topic} is publishing again.")
            self.stale_logged[topic] = False

    def _check(self, _event):
        now = time.time()
        in_grace = now - self.start < self.grace

        topics = {}
        for t in self.topics:
            last = self.last_rx[t]
            age = None if last is None else now - last
            ok = age is not None and age <= self.timeout
            topics[t] = {"age": age, "ok": ok}
            if not ok and not in_grace and not self.stale_logged[t]:
                if age is None:
                    self.rospy.logerr(f"Watchdog: no messages ever received on {t}")
                else:
                    self.rospy.logerr(f"Watchdog: no messages on {t} for {age:.0f} s")
                self.stale_logged[t] = True

        ages = [v["age"] for v in topics.values() if v["age"] is not None]
        esp32_age = min(ages) if ages else None
        info = read_can_iface()
        ok, reason = can_health(info, esp32_age)
        if in_grace and esp32_age is None and "ESP32" in reason:
            ok, reason = True, "Starting up (waiting for ESP32 data)"
        self._log_can_changes(info, ok, reason)

        can = dict(info, iface=CAN_IFACE, ok=ok, reason=reason, esp32_age=esp32_age)
        self.pub.publish(json.dumps({"stamp": now, "can": can, "topics": topics}))

    def _log_can_changes(self, info, ok, reason):
        rp = self.rospy
        if self.prev_ok is not None and ok != self.prev_ok:
            if ok:
                rp.logwarn(f"Watchdog: CAN link restored ({reason})")
            else:
                rp.logerr(f"Watchdog: CAN LINK DOWN - {reason}")
        elif self.prev_ok is None and not ok:
            rp.logerr(f"Watchdog: CAN LINK DOWN - {reason}")
        self.prev_ok = ok

        prev = self.prev_can
        if prev is not None:
            if info.get("state") != prev.get("state"):
                lvl = rp.loginfo if info.get("state") == "ERROR-ACTIVE" else rp.logwarn
                lvl(f"Watchdog: {CAN_IFACE} state {prev.get('state')} -> {info.get('state')}")
            px, nx = prev.get("xstats", {}), info.get("xstats", {})
            for key, label, log in (("bus_off", "went BUS-OFF", rp.logerr),
                                    ("restarts", "auto-restarted after bus-off", rp.logwarn),
                                    ("error_passive", "entered ERROR-PASSIVE", rp.logwarn)):
                a, b = px.get(key), nx.get(key)
                if a is not None and b is not None and b > a:
                    log(f"Watchdog: {CAN_IFACE} {label} (total {b})")
        elif info.get("restart_ms") == 0:
            rp.logwarn(f"Watchdog: {CAN_IFACE} restart-ms is 0 - it will NOT recover from "
                       "bus-off by itself (start_node.sh normally sets it)")
        self.prev_can = info


if __name__ == '__main__':
    import rospy
    try:
        critical_topics = [f'/extruder/zone{i}/temperature' for i in range(1, 5)]
        Watchdog(critical_topics)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Watchdog crashed: {e}")
