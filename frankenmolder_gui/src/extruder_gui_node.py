#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, String
from rosgraph_msgs.msg import Log
import json
from collections import deque
import tkinter as tk
from tkinter import ttk # Import the themed toolkit for tabs (Notebook)
from threading import Thread, Lock
import math
import sys # Added for stderr
import time
from process_dashboard import ControlRoomDashboard, blank_data
from extruder_page import ExtruderPage
from watchdog_page import (ROSOUT_LEVELS, LinkAlarmPopup, LinkMonitor, WatchdogPage,
                           build_alarms)
import hmi

# --- Configuration ---
ZONE_COUNT = 4
MIN_SETPOINT = 20.0
MAX_SETPOINT = 350.0
MAX_RPM = 100.0   # screw RPM limit for the number pad
GUI_POLL_INTERVAL_MS = 500 # How often the GUI checks for updates

# --- Shared State (Thread Safety) ---
# Use dictionaries to store data for each zone
latest_temps = {f"zone{i+1}": float('nan') for i in range(ZONE_COUNT)}
# --- RENAMED: This now stores the *true* state from the control node ---
latest_actual_states = {f"zone{i+1}": "OFF" for i in range(ZONE_COUNT)}
# Screw drive = AC servo via the ESP32 (state: IDLE / RUNNING / MOVING / STOPPING / FAULT)
latest_motor_state = None
latest_motor_rpm_act = float('nan')
latest_motor_flags = []
last_motor_time = 0.0
MOTOR_LINK_TIMEOUT_S = 1.0  # ESP32 sends servo status every 100 ms
motor_direction = "FWD"     # operator setting, pushed to the CAN bridge on a latched topic
# Dashboard extras: echoed setpoints, sensor faults and link health
latest_setpoints = {f"zone{i+1}": float('nan') for i in range(ZONE_COUNT)}
latest_error_states = {f"zone{i+1}": "OK" for i in range(ZONE_COUNT)}
latest_motor_rpm_set = float('nan')
last_temp_time = 0.0  # time.time() of the last temperature message from the ESP32
LINK_TIMEOUT_S = 5.0
# Watchdog tab: decoded /watchdog/status and the ROS log (/rosout_agg)
latest_watchdog = None
last_watchdog_time = 0.0
log_entries = deque(maxlen=500)

# Lock to prevent race conditions when accessing shared data
data_lock = Lock()

def _key_style(face):
    """Flat dark keys for the number pad, matching the lamp buttons' faces."""
    return dict(bg=face, fg="#ffffff", activebackground=hmi.blend(face, "#ffffff", 0.15),
                activeforeground="#ffffff", relief=tk.FLAT, bd=0, highlightthickness=1,
                highlightbackground=hmi.C["edge"], cursor="hand2")


class NumberPadPopup:
    """Custom number pad popup for touchscreen-friendly numeric input."""
    def __init__(self, parent, entry_widget, entry_var, min_val=None, max_val=None, callback=None, confirm_text="OK", on_confirm=None):
        self.parent = parent
        self.entry_widget = entry_widget
        self.entry_var = entry_var
        self.min_val = min_val
        self.max_val = max_val
        self.callback = callback
        self.on_confirm = on_confirm  # Additional callback when OK/Set Target is pressed
        self.confirm_text = confirm_text  # Text for confirm button
        
        # Create popup window
        self.popup = tk.Toplevel(parent)
        self.popup.title("Enter Number")
        self.popup.attributes('-topmost', True)  # Keep on top
        self.popup.transient(parent)  # Remove minimize/maximize buttons
        self.popup.configure(bg=hmi.C["panel"], highlightthickness=2,
                             highlightbackground=hmi.C["accent"])
        
        # Set a fixed size for the popup to prevent scrunching (widened by 10px)
        popup_width = 410
        popup_height = 500
        
        # Don't set geometry yet - wait until widgets are created
        
        # Current input string - get initial value from entry variable
        initial_val = ""
        if hasattr(entry_var, 'get'):
            try:
                val = entry_var.get()
                initial_val = str(val) if val else ""
            except:
                initial_val = ""
        self.input_str = tk.StringVar(value=initial_val)
        
        # Display field
        display_frame = tk.Frame(self.popup, bg=hmi.C["well"], bd=0,
                                 highlightthickness=1, highlightbackground=hmi.C["edge"])
        display_frame.pack(padx=10, pady=10, fill=tk.X)
        self.display_label = tk.Label(display_frame, textvariable=self.input_str,
                                      font=("DejaVu Sans Mono", 26, "bold"), bg=hmi.C["well"],
                                      fg=hmi.C["setp"], anchor="e", padx=10, pady=5)
        self.display_label.pack(fill=tk.X)
        
        # Number pad buttons
        button_frame = tk.Frame(self.popup, bg=hmi.C["panel"])
        button_frame.pack(padx=10, pady=10)
        
        # Button layout
        buttons = [
            ['7', '8', '9', 'C'],
            ['4', '5', '6', '⌫'],
            ['1', '2', '3', '0']
        ]
        
        for i, row in enumerate(buttons):
            for j, btn_text in enumerate(row):
                # Make buttons larger for better touchscreen use
                face = hmi.FACE["blue"] if btn_text in ('C', '⌫') else hmi.FACE["neutral"]
                btn = tk.Button(button_frame, text=btn_text, font=("DejaVu Sans", 22, "bold"),
                               width=5, height=3, command=lambda t=btn_text: self.button_click(t),
                               **_key_style(face))
                btn.grid(row=i, column=j, padx=5, pady=5, sticky="nsew")
                button_frame.grid_columnconfigure(j, weight=1, minsize=100)
            button_frame.grid_rowconfigure(i, weight=1, minsize=100)
        
        # Add large "Set Target" button spanning all columns at the bottom
        set_btn = tk.Button(button_frame, text="SET", font=("DejaVu Sans", 22, "bold"),
                           height=3, command=lambda: self.button_click(self.confirm_text),
                           **_key_style(hmi.FACE["start"]))
        set_btn.grid(row=3, column=0, columnspan=4, padx=5, pady=5, sticky="nsew")
        button_frame.grid_rowconfigure(3, weight=1, minsize=100)
        
        # Handle window close
        self.popup.protocol("WM_DELETE_WINDOW", self.cancel)
        
        # Now center popup on screen after all widgets are created
        self.popup.update_idletasks()
        screen_width = self.popup.winfo_screenwidth()
        screen_height = self.popup.winfo_screenheight()
        x = (screen_width // 2) - (popup_width // 2)
        y = (screen_height // 2) - (popup_height // 2)
        self.popup.geometry(f"{popup_width}x{popup_height}+{x}+{y}")
        
        # Focus the popup
        self.popup.focus_set()
        
    def button_click(self, char):
        current = self.input_str.get()
        
        if char == 'C':  # Clear
            self.input_str.set("")
        elif char == '⌫':  # Backspace
            self.input_str.set(current[:-1] if current else "")
        elif char == self.confirm_text:  # Confirm (OK or Set Target)
            self.confirm()
        elif char == '':  # Empty button (spacer)
            return  # Do nothing for empty buttons
        else:  # Number (0-9)
            self.input_str.set(current + char)
    
    def confirm(self):
        try:
            value_str = self.input_str.get()
            if not value_str or value_str == '-':
                value_str = "0"
            
            value = float(value_str)
            
            # Validate range if specified
            if self.min_val is not None and value < self.min_val:
                value = self.min_val
            if self.max_val is not None and value > self.max_val:
                value = self.max_val
            
            # Update the entry variable (works for both DoubleVar and StringVar)
            if hasattr(self.entry_var, 'set'):
                # Format value to 1 decimal place for display
                formatted_value = f"{value:.1f}"
                self.entry_var.set(formatted_value)
            
            # Call callback if provided
            if self.callback:
                self.callback(value)
            
            # Call on_confirm callback if provided (e.g., publish setpoint immediately)
            if self.on_confirm:
                self.on_confirm(value)
            
            self.popup.destroy()
            
        except ValueError:
            # Invalid input, just close
            self.popup.destroy()
    
    def cancel(self):
        self.popup.destroy()
class ExtruderGUI(tk.Frame):
    """Main application class that holds the tabbed notebook."""
    def __init__(self, master):
        super().__init__(master)
        self.master = master
        self.master.title("Frankenmolder Control Interface")
        
        # --- ROS Communication Objects ---
        self.publishers = {}
        self.subscribers = {}

        # --- Dark control-room theme for the window chrome ---
        self.master.configure(bg=hmi.C["bg"])
        style = ttk.Style(self.master)
        style.theme_use("default")
        style.configure("TNotebook", background=hmi.C["bg"], borderwidth=0, tabmargins=(4, 4, 4, 0))
        style.configure("TNotebook.Tab", background=hmi.C["head"], foreground=hmi.C["dim"],
                        padding=(22, 8), borderwidth=0, font=("DejaVu Sans", 12, "bold"))
        style.map("TNotebook.Tab",
                  background=[("selected", hmi.C["accent"])],
                  foreground=[("selected", "#ffffff")])

        # --- Create the Tabbed Notebook ---
        self.notebook = ttk.Notebook(self.master)

        # --- Tab 1: Dashboard (process overview) ---
        self.dashboard_frame = ControlRoomDashboard(
            self.notebook,
            on_zone_click=lambda zone_id: self.notebook.select(self.extruder_frame),
            on_motor_click=lambda: self.notebook.select(self.extruder_frame),
            tick_ms=150)
        self.notebook.add(self.dashboard_frame, text='Dashboard')

        # --- Tab 2: Extruder (heaters + screw drive) ---
        self.extruder_frame = ExtruderPage(
            self.notebook,
            on_setpoint_tap=self.open_setpoint_pad,
            on_zone_cmd=self.publish_state_cmd,
            on_all_cmd=self.publish_all_zones_cmd,
            on_rpm_tap=self.open_rpm_pad,
            on_motor_cmd=self.publish_motor_cmd,
            on_direction_cmd=self.publish_motor_direction)
        self.notebook.add(self.extruder_frame, text='Extruder')

        # --- Tab 3: Watchdog (CAN link health, alarms, ROS log) ---
        self.watchdog_frame = WatchdogPage(self.notebook, on_clear_logs=self.clear_logs)
        self.notebook.add(self.watchdog_frame, text='Watchdog')
        self._watchdog_tab_text = None

        # CAN link failure alarm (flashing popup until acknowledged)
        self.link_monitor = LinkMonitor()
        self.link_popup = None

        self.notebook.pack(expand=True, fill='both', padx=0, pady=0)

        # --- Status Bar (Shared by all tabs) ---
        self.message_var = tk.StringVar(value="System Initialized.")
        status_label = tk.Label(self.master, textvariable=self.message_var, anchor=tk.W,
                                bg=hmi.C["head"], fg=hmi.C["text"], bd=0, padx=8, pady=3,
                                font=("DejaVu Sans Mono", 10))
        status_label.pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=(0, 5))

        # Start the periodic GUI update (polling)
        self.master.after(GUI_POLL_INTERVAL_MS, self.poll_ros_updates)

    def init_ros_comms(self):
        """Initializes all ROS publishers and subscribers for all frames."""
        rospy.loginfo("GUI: Initializing all ROS publishers and subscribers...")
        try:
            # --- Heater Publishers & Subscribers ---
            for i in range(ZONE_COUNT):
                zone_id = f"zone{i+1}"
                # Publishers (GUI sends commands)
                self.publishers[f"{zone_id}_setpoint"] = rospy.Publisher(f'/extruder/{zone_id}/setpoint', Float32, queue_size=1)
                self.publishers[f"{zone_id}_state_cmd"] = rospy.Publisher(f'/extruder/{zone_id}/state_cmd', String, queue_size=1)
                
                # Subscribers (GUI receives status)
                self.subscribers[f"{zone_id}_temp"] = rospy.Subscriber(
                    f'/extruder/{zone_id}/temperature', 
                    Float32, 
                    self.update_temp_callback, 
                    callback_args=zone_id
                )
                
                # --- FIX: Subscribe to the /actual_state topic ---
                self.subscribers[f"{zone_id}_actual_state"] = rospy.Subscriber(
                    f'/extruder/{zone_id}/actual_state', # Listen for the *true* state from the controller
                    String,
                    self.update_actual_state_callback, # Use the renamed callback
                    callback_args=zone_id
                )

                # Dashboard: echo of the setpoint (whoever publishes it) and sensor faults
                self.subscribers[f"{zone_id}_setpoint_echo"] = rospy.Subscriber(
                    f'/extruder/{zone_id}/setpoint', Float32,
                    self.update_setpoint_callback, callback_args=zone_id
                )
                self.subscribers[f"{zone_id}_error_state"] = rospy.Subscriber(
                    f'/extruder/{zone_id}/error_state', String,
                    self.update_error_state_callback, callback_args=zone_id
                )
            
            # --- Screw drive (AC servo, translated by can_bridge_node) ---
            # set_rpm and direction are latched so a restarted bridge picks them up again
            self.publishers["motor_set_rpm"] = rospy.Publisher('/extruder/motor/set_rpm', Float32, queue_size=1, latch=True)
            self.publishers["motor_state_cmd"] = rospy.Publisher('/extruder/motor/state_cmd', String, queue_size=1)
            self.publishers["motor_direction"] = rospy.Publisher('/extruder/motor/direction', String, queue_size=1, latch=True)
            self.publishers["motor_direction"].publish(String(motor_direction))

            self.subscribers["motor_actual_state"] = rospy.Subscriber(
                '/extruder/motor/actual_state', String, self.update_motor_state_callback)
            self.subscribers["motor_actual_rpm"] = rospy.Subscriber(
                '/extruder/motor/actual_rpm', Float32, self.update_motor_rpm_act_callback)
            self.subscribers["motor_flags"] = rospy.Subscriber(
                '/extruder/motor/flags', String, self.update_motor_flags_callback)
            self.subscribers["motor_set_rpm_echo"] = rospy.Subscriber(
                '/extruder/motor/set_rpm', Float32, self.update_motor_rpm_set_callback
            )

            # --- Watchdog ---
            self.subscribers["watchdog_status"] = rospy.Subscriber(
                '/watchdog/status', String, self.update_watchdog_callback)
            self.subscribers["rosout"] = rospy.Subscriber('/rosout_agg', Log, self.update_log_callback)

            rospy.loginfo("GUI: ROS Comms Initialized.")
            self.message_var.set("ROS Connection Established.")
            return True

        except Exception as e:
            err_msg = f"GUI: Error initializing ROS comms: {e}"
            rospy.logerr(err_msg)
            print(err_msg, file=sys.stderr, flush=True) # Ensure it appears
            self.message_var.set("ROS Connection FAILED. Check logs.")
            return False

    # --- Thread-Safe Callbacks (Update shared data) ---
    def update_temp_callback(self, temp_msg, zone_id):
        """ROS thread callback: Safely update shared temp data."""
        try:
            with data_lock:
                global last_temp_time
                latest_temps[zone_id] = temp_msg.data
                last_temp_time = time.time()
        except Exception as e:
            rospy.logerr(f"R_TEMP_CB ({zone_id}): Error storing temp data: {e}")

    # --- RENAMED: This callback now listens to /actual_state ---
    def update_actual_state_callback(self, state_msg, zone_id):
        """ROS thread callback: Safely update shared state data."""
        try:
            with data_lock:
                latest_actual_states[zone_id] = state_msg.data
        except Exception as e:
            rospy.logerr(f"R_STATE_CB ({zone_id}): Error storing state data: {e}")

    def update_motor_state_callback(self, state_msg):
        """ROS thread callback: Safely update shared motor state."""
        try:
            with data_lock:
                global latest_motor_state, last_motor_time
                latest_motor_state = state_msg.data
                last_motor_time = time.time()
        except Exception as e:
            rospy.logerr(f"R_MOTOR_CB: Error storing motor state: {e}")

    def update_motor_rpm_act_callback(self, msg):
        with data_lock:
            global latest_motor_rpm_act
            latest_motor_rpm_act = msg.data

    def update_motor_flags_callback(self, msg):
        with data_lock:
            global latest_motor_flags
            latest_motor_flags = [f for f in msg.data.split(",") if f]

    def update_setpoint_callback(self, msg, zone_id):
        with data_lock:
            latest_setpoints[zone_id] = msg.data

    def update_error_state_callback(self, msg, zone_id):
        with data_lock:
            latest_error_states[zone_id] = msg.data

    def update_motor_rpm_set_callback(self, msg):
        with data_lock:
            global latest_motor_rpm_set
            latest_motor_rpm_set = msg.data

    def update_watchdog_callback(self, msg):
        try:
            status = json.loads(msg.data)
        except ValueError as e:
            rospy.logwarn(f"GUI: bad /watchdog/status JSON: {e}")
            return
        with data_lock:
            global latest_watchdog, last_watchdog_time
            latest_watchdog = status
            last_watchdog_time = time.time()

    def update_log_callback(self, msg):
        entry = {"t": msg.header.stamp.to_sec() or time.time(),
                 "lvl": ROSOUT_LEVELS.get(msg.level, "INFO"), "node": msg.name, "msg": msg.msg}
        with data_lock:
            log_entries.append(entry)

    def clear_logs(self):
        with data_lock:
            log_entries.clear()

    def build_dashboard_data(self):
        """Snapshot the shared ROS state into the dashboard's data format."""
        nan_to_none = lambda v: None if (v is None or math.isnan(v)) else v
        d = blank_data()
        with data_lock:
            for i in range(ZONE_COUNT):
                zone_id = f"zone{i+1}"
                fault = latest_error_states.get(zone_id) == "FAULT"
                d["zones"][zone_id] = {
                    # NaN is drawn as FAULT, None as "---" (no reading yet)
                    "act": float('nan') if fault else nan_to_none(latest_temps[zone_id]),
                    "set": nan_to_none(latest_setpoints[zone_id]),
                    "state": "FAULT" if fault else latest_actual_states[zone_id],
                }
            now = time.time()
            motor_link = (now - last_motor_time) < MOTOR_LINK_TIMEOUT_S
            d["motor"] = {
                # servo state from the ESP32; None when its status frames stop arriving
                "state": latest_motor_state if motor_link else None,
                "rpm_set": nan_to_none(latest_motor_rpm_set),
                "rpm_act": nan_to_none(latest_motor_rpm_act) if motor_link else None,
                "load": None,
                "direction": motor_direction,
                "flags": list(latest_motor_flags) if motor_link else [],
                "link": motor_link,
            }
            wd_age = (now - last_watchdog_time) if last_watchdog_time else None
            temp_age = (now - last_temp_time) if last_temp_time else None
            change = self.link_monitor.update(now, latest_watchdog, wd_age, temp_age)
            link = self.link_monitor.snapshot()
            d["can_ok"] = link["ok"]
            d["watchdog"] = {"status": latest_watchdog, "status_age": wd_age, "link": link,
                             "alarms": build_alarms(link, latest_watchdog, wd_age, now),
                             "logs": list(log_entries)}
            d["link_change"] = change
        return d

    # --- CAN link alarm ---
    def handle_link_alarm(self, data):
        change = data.get("link_change")
        link = data["watchdog"]["link"]
        if change == "failed":
            rospy.logerr(f"GUI: CAN LINK FAILURE - {link['reason']}")
            self.message_var.set(f"CAN LINK FAILURE: {link['reason']}")
            if self.link_popup is None:
                self.link_popup = LinkAlarmPopup(self.master, self.acknowledge_link_alarm,
                                                 link["reason"], link["failed_at"])
            else:
                self.link_popup.failed(link["reason"], link["failed_at"])
        elif change == "restored":
            rospy.logwarn("GUI: CAN link restored.")
            self.message_var.set("CAN link restored." if link["acked"]
                                 else "CAN link restored - acknowledge the alarm.")
            if self.link_popup is not None:
                self.link_popup.restored(link["restored_at"])

        # Tab label shows when there is something to look at
        n = len(data["watchdog"]["alarms"])
        text = "⚠ Watchdog" if not link["ok"] else (f"Watchdog ({n})" if n else "Watchdog")
        if text != self._watchdog_tab_text:
            self.notebook.tab(self.watchdog_frame, text=text)
            self._watchdog_tab_text = text

    def acknowledge_link_alarm(self):
        self.link_popup = None
        self.link_monitor.acknowledge()
        state = "link still DOWN" if not self.link_monitor.ok else "link OK"
        rospy.loginfo(f"GUI: CAN link alarm acknowledged by operator ({state}).")
        self.message_var.set(f"CAN link alarm acknowledged ({state}).")

    # --- Commands from the Extruder page ---
    def open_setpoint_pad(self, zone_id):
        with data_lock:
            current = latest_setpoints[zone_id]
        var = tk.StringVar(value="" if math.isnan(current) else f"{current:.0f}")
        NumberPadPopup(self.master, self.extruder_frame, var,
                       min_val=MIN_SETPOINT, max_val=MAX_SETPOINT, confirm_text="Set",
                       on_confirm=lambda v: self.publish_setpoint(zone_id, v))

    def open_rpm_pad(self):
        with data_lock:
            current = latest_motor_rpm_set
        var = tk.StringVar(value="" if math.isnan(current) else f"{current:.0f}")
        NumberPadPopup(self.master, self.extruder_frame, var, min_val=0.0, max_val=MAX_RPM,
                       confirm_text="Set", on_confirm=self.publish_set_rpm)

    def _publish(self, key, msg, ok_text):
        pub = self.publishers.get(key)
        if not pub:
            rospy.logerr(f"GUI: publisher '{key}' not found.")
            self.message_var.set(f"Error: no publisher for {key} (ROS not connected?)")
            return False
        try:
            pub.publish(msg)
            rospy.loginfo(f"GUI: {ok_text}")
            self.message_var.set(ok_text)
            return True
        except Exception as e:
            rospy.logerr(f"GUI: publish {key} failed: {e}")
            self.message_var.set(f"Error sending {key}: {e}")
            return False

    def publish_setpoint(self, zone_id, value):
        value = float(value)
        if not (MIN_SETPOINT <= value <= MAX_SETPOINT):
            self.message_var.set(f"{zone_id}: setpoint must be {MIN_SETPOINT:.0f}-{MAX_SETPOINT:.0f} °C")
            return
        if self._publish(f"{zone_id}_setpoint", Float32(value),
                         f"{zone_id}: setpoint {value:.1f} °C sent."):
            self.extruder_frame.log(f"Z{zone_id[-1]} SET {value:.0f}", "cmd")

    def publish_state_cmd(self, zone_id, command):
        """command is 'ON' or 'OFF'."""
        self._publish(f"{zone_id}_state_cmd", String(command), f"{zone_id}: {command} sent.")

    def publish_all_zones_cmd(self, command):
        for i in range(ZONE_COUNT):
            self.publish_state_cmd(f"zone{i+1}", command)

    def publish_set_rpm(self, value):
        value = float(value)
        if self._publish("motor_set_rpm", Float32(value), f"Screw RPM {value:.1f} sent."):
            self.extruder_frame.log(f"RPM SET {value:.1f}", "cmd")

    def publish_motor_cmd(self, command):
        """command is 'ON' (start) or 'OFF' (stop)."""
        self._publish("motor_state_cmd", String(command),
                      f"Motor {'START' if command == 'ON' else 'STOP'} sent.")

    def publish_motor_direction(self, direction):
        """direction is 'FWD' or 'REV'. The page only offers it while the screw is stopped."""
        global motor_direction
        if self._publish("motor_direction", String(direction), f"Screw direction {direction} sent."):
            with data_lock:
                motor_direction = direction
            return True
        return False

    # --- GUI Polling Function (Read shared data) ---
    def poll_ros_updates(self):
        """GUI thread loop: Safely read shared data and update all frames."""
        try:
            # Pass data to each frame to update its widgets
            data = self.build_dashboard_data()
            self.dashboard_frame.set_data(data)
            self.extruder_frame.set_data(data)
            self.watchdog_frame.set_data(data)
            self.handle_link_alarm(data)

        except Exception as e:
            err_msg = f"G_POLL: Error during GUI update: {e}"
            rospy.logerr(err_msg)
            self.message_var.set(err_msg)

        # Reschedule the next check
        self.master.after(GUI_POLL_INTERVAL_MS, self.poll_ros_updates)
def ros_spin_thread(app):
    """Handles all ROS communication in a separate thread."""
    rospy.loginfo("GUI: ROS spin thread started.")
    try:
        if not app.init_ros_comms():
            rospy.logerr("GUI: Failed to initialize ROS comms. ROS thread exiting.")
            return
        
        rospy.loginfo("GUI: Starting rospy.spin().")
        rospy.spin() # Keeps the ROS thread alive, handling callbacks
        rospy.loginfo("GUI: rospy.spin() finished.")
        
    except rospy.ROSInterruptException:
        rospy.loginfo("GUI: ROS spin thread interrupted.")
    except Exception as e:
        rospy.logerr(f"GUI: Unhandled exception in ROS spin thread: {e}")

def main():
    """Main function to initialize ROS node and start Tkinter GUI."""
    try:
        # Initialize the ROS node in the main thread
        rospy.init_node('extruder_gui_node', anonymous=True)
        rospy.loginfo("GUI Node initialized in main thread.")
        
        # Create the Tkinter root window
        root = tk.Tk()
        
        # Auto-open in fullscreen
        root.attributes('-fullscreen', True) 
        
        app = ExtruderGUI(root)
        
        # Start the ROS communication thread
        # Pass the 'app' instance to the thread
        ros_thread = Thread(target=ros_spin_thread, args=(app,), daemon=True)
        ros_thread.start()
        
        # Start the Tkinter GUI loop in the main thread
        rospy.loginfo("GUI: Starting Tkinter mainloop().")
        root.mainloop()
        rospy.loginfo("GUI: Tkinter mainloop() exited.")
        
    except rospy.ROSInterruptException:
        rospy.loginfo("GUI: Main thread interrupted. Shutting down.")
    except Exception as e:
        rospy.logerr(f"GUI: Unhandled exception in main: {e}")
        print(f"FATAL: GUI Main thread crashed: {e}", file=sys.stderr, flush=True)

if __name__ == '__main__':
    main()