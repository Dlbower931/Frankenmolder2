#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Bool, String # Keep String for state_cmd
import tkinter as tk
from threading import Thread, Lock
import math # For isnan check
import sys # For stderr

# --- Configuration ---
ZONE_COUNT = 3
MIN_SETPOINT = 20.0
MAX_SETPOINT = 350.0
GUI_POLL_INTERVAL_MS = 500 # How often the GUI checks for updates

# --- Shared State (Thread Safety) ---
# Use dictionaries to store data for each zone
latest_temps = {f"zone{i+1}": float('nan') for i in range(ZONE_COUNT)}
# --- ADDED: Dictionary to store the latest ACTUAL state received from control node ---
latest_state_cmds = {f"zone{i+1}": "OFF" for i in range(ZONE_COUNT)}
# Lock to prevent race conditions when accessing shared data
data_lock = Lock()

# --- Tkinter Class Definition ---
class ExtruderGUI(tk.Frame):
    def __init__(self, master):
        super().__init__(master)
        self.master = master
        self.master.title("Frankenmolder Extruder Control")

        # ROS Communication Objects (Initialized later)
        self.setpoint_pubs = {}
        self.state_cmd_pubs = {} # Still needed to send commands TO control node
        self.temp_subs = {}
        # --- ADDED: Keep track of state_cmd subscribers ---
        self.state_cmd_subs = {}

        # --- Data Storage (Tkinter Variables) ---
        self.current_temps = {} # Dict: { "zone1": tk.StringVar, ... }
        self.target_setpoints = {} # Dict: { "zone1": tk.DoubleVar, ... }
        # --- This now reflects the ACTUAL state received from control node ---
        self.current_mode = {} # Dict: { "zone1": tk.StringVar, ... }

        self.message_var = tk.StringVar(value="System Initialized.") # Status message

        # Initialize Tkinter variables for each zone
        for i in range(ZONE_COUNT):
            zone_id = f"zone{i+1}"
            self.current_temps[zone_id] = tk.StringVar(value="--")
            self.target_setpoints[zone_id] = tk.DoubleVar(value=200.0) # Default target
            # Initialize display to OFF, will be updated by subscriber
            self.current_mode[zone_id] = tk.StringVar(value="OFF") # Start displaying OFF state

        self.create_widgets()

        # Start the periodic GUI update check
        self.master.after(GUI_POLL_INTERVAL_MS, self.check_for_updates)

    def create_widgets(self):
        # --- Main Barrel Frame ---
        barrel_frame = tk.Frame(self.master, bd=2, relief=tk.SUNKEN)
        barrel_frame.grid(row=0, column=0, columnspan=ZONE_COUNT, padx=10, pady=10, sticky="ew") # Adjusted columnspan

        # --- Create Controls for Each Zone ---
        for i in range(ZONE_COUNT):
            zone_id = f"zone{i+1}"
            zone_frame = tk.LabelFrame(barrel_frame, text=f"Zone {i+1}", padx=10, pady=10)
            zone_frame.grid(row=0, column=i, padx=5, pady=5, sticky="nsew") # Place zones side-by-side

            # Temperature Display
            tk.Label(zone_frame, text="Temp:", font=("Arial", 12)).grid(row=0, column=0, sticky="w")
            temp_label = tk.Label(zone_frame, textvariable=self.current_temps[zone_id], font=("Arial", 18, "bold"))
            temp_label.grid(row=0, column=1, columnspan=2, sticky="e")
            tk.Label(zone_frame, text="°C", font=("Arial", 12)).grid(row=0, column=3, sticky="w")

            # Mode Display (Reflects ACTUAL state from control node)
            tk.Label(zone_frame, text="Mode:", font=("Arial", 12)).grid(row=1, column=0, sticky="w")
            mode_label = tk.Label(zone_frame, textvariable=self.current_mode[zone_id], font=("Arial", 12, "italic"))
            mode_label.grid(row=1, column=1, columnspan=3, sticky="e")
            setattr(self, f"{zone_id}_mode_label", mode_label) # Store ref for potential coloring

            # Setpoint Control (Stacked)
            tk.Label(zone_frame, text="Setpoint (°C):", font=("Arial", 12)).grid(row=2, column=0, columnspan=4, pady=(10, 0), sticky="w")
            entry = tk.Entry(zone_frame, textvariable=self.target_setpoints[zone_id], width=10, font=("Arial", 16))
            entry.grid(row=3, column=0, columnspan=4, pady=5, ipady=8)
            set_button = tk.Button(zone_frame, text="Set Target", font=("Arial", 14, "bold"), width=15,
                                   command=lambda zid=zone_id: self.publish_setpoint(zid))
            set_button.grid(row=4, column=0, columnspan=4, pady=5, ipady=8)

            # State Control Buttons (Stacked OFF/START)
            tk.Label(zone_frame, text="Commands:", font=("Arial", 12)).grid(row=5, column=0, columnspan=4, pady=(15, 0), sticky="w")
            button_frame = tk.Frame(zone_frame)
            button_frame.grid(row=6, column=0, columnspan=4, pady=(5, 5))

            off_button = tk.Button(button_frame, text="OFF", bg="red", fg="white", font=("Arial", 12, "bold"), width=12, height=2, # Size params
                                   command=lambda zid=zone_id: self.publish_state_cmd(zid, "OFF"))
            off_button.pack(side=tk.TOP, fill=tk.X, pady=2) # Stack vertically

            start_button = tk.Button(button_frame, text="START", bg="orange", fg="black", font=("Arial", 12, "bold"), width=12, height=2, # Size params
                                     command=lambda zid=zone_id: self.publish_state_cmd(zid, "HEATING"))
            start_button.pack(side=tk.TOP, fill=tk.X, pady=2) # Stack vertically


        # --- Status Bar ---
        status_label = tk.Label(self.master, textvariable=self.message_var, bd=1, relief=tk.SUNKEN, anchor=tk.W)
        status_label.grid(row=1, column=0, columnspan=ZONE_COUNT, sticky='ew', padx=10, pady=(0, 5)) # Adjusted columnspan

    # --- GUI Update Methods ---
    def check_for_updates(self):
        """Periodically check shared data and update Tkinter variables."""
        # rospy.logdebug("G_POLL: Checking for updates...") # Can be noisy
        try:
            with data_lock:
                for i in range(ZONE_COUNT):
                    zone_id = f"zone{i+1}"
                    temp_val = latest_temps.get(zone_id, float('nan')) # Use get for safety
                    # --- Read ACTUAL state from shared dictionary ---
                    actual_state = latest_state_cmds.get(zone_id, "OFF") # Use get for safety

                    # Update Temperature Display
                    if not math.isnan(temp_val):
                        self.current_temps[zone_id].set(f"{temp_val:.1f}")
                    else:
                        self.current_temps[zone_id].set("--")

                    # --- Update Mode Display using ACTUAL state ---
                    self.current_mode[zone_id].set(actual_state) # Update Tkinter Var

                    # Update Mode Label Color based on ACTUAL state
                    mode_label = getattr(self, f"{zone_id}_mode_label", None)
                    if mode_label:
                        if actual_state == "OFF": mode_label.config(fg="red")
                        elif actual_state == "HEATING": mode_label.config(fg="orange")
                        elif actual_state == "PID": mode_label.config(fg="green")
                        else: mode_label.config(fg="black") # Default/Unknown

        except Exception as e:
            err_msg = f"G_POLL: Error during GUI update: {e}"
            rospy.logerr(err_msg)
            self.message_var.set(err_msg) # Show error in status bar

        # Reschedule the next check
        self.master.after(GUI_POLL_INTERVAL_MS, self.check_for_updates)

    # --- ROS Data Callbacks ---
    def update_temp_callback(self, temp_msg, zone_id):
        """ROS thread callback: Safely update shared temp data."""
        # rospy.logdebug(f"R_TEMP_CB ({zone_id}): Received {temp_msg.data}")
        try:
            with data_lock:
                latest_temps[zone_id] = temp_msg.data
        except Exception as e:
            rospy.logerr(f"R_TEMP_CB ({zone_id}): Error storing temp data: {e}")

    # --- ADDED CALLBACK for state_cmd topic ---
    def update_state_cmd_callback(self, state_msg, zone_id):
        """ROS thread callback: Safely update shared state data."""
        rospy.logdebug(f"R_STATE_CB ({zone_id}): Received state '{state_msg.data}'")
        try:
            with data_lock:
                latest_state_cmds[zone_id] = state_msg.data
        except Exception as e:
            rospy.logerr(f"R_STATE_CB ({zone_id}): Error storing state data: {e}")
    # ----------------------------------------------

    # --- ROS Publishing Methods ---
    def publish_setpoint(self, zone_id):
        """Validate and publish the setpoint."""
        pub = self.setpoint_pubs.get(zone_id)
        if not pub:
            rospy.logerr(f"Setpoint Publisher for {zone_id} not initialized.")
            self.message_var.set(f"Error: Publisher for {zone_id} not ready.")
            return

        try:
            setpoint_value = self.target_setpoints[zone_id].get()
            if not isinstance(setpoint_value, (int, float)): raise ValueError("Input must be a number.")
            if not (MIN_SETPOINT <= setpoint_value <= MAX_SETPOINT): raise ValueError(f"Setpoint: {MIN_SETPOINT}-{MAX_SETPOINT}°C.")

            msg = Float32(setpoint_value)
            pub.publish(msg)
            rospy.loginfo(f"GUI published setpoint for {zone_id}: {setpoint_value}")
            self.message_var.set(f"{zone_id}: Setpoint {setpoint_value:.1f}°C sent.")

            # --- REMOVED Implicit PID command publish ---
            # self.publish_state_cmd(zone_id, "PID")

        except ValueError as ve:
             rospy.logwarn(f"GUI Validation Error ({zone_id}): {ve}")
             self.message_var.set(f"Error ({zone_id}): {ve}")
        except Exception as e:
            rospy.logerr(f"GUI publish_setpoint Error ({zone_id}): {e}")
            self.message_var.set(f"Error publishing setpoint for {zone_id}.")

    def publish_state_cmd(self, zone_id, state_command):
        """Publish the state command ('OFF', 'HEATING')."""
        pub = self.state_cmd_pubs.get(zone_id)
        if not pub:
            rospy.logerr(f"State Command Publisher for {zone_id} not initialized.")
            self.message_var.set(f"Error: State Publisher for {zone_id} not ready.")
            return

        # Basic check to prevent spamming the same command (optional)
        # current_displayed_mode = self.current_mode[zone_id].get()
        # if current_displayed_mode == state_command:
        #     rospy.logdebug(f"GUI: State command '{state_command}' for {zone_id} already sent/active.")
        #     return

        try:
            msg = String(state_command)
            pub.publish(msg)
            rospy.loginfo(f"GUI published state_cmd for {zone_id}: {state_command}")
            # --- We NO LONGER update self.current_mode directly here ---
            # self.current_mode[zone_id].set(state_command) # Let the subscriber handle updates
            self.message_var.set(f"{zone_id}: Sent command '{state_command}'.")

        except Exception as e:
            rospy.logerr(f"GUI publish_state_cmd Error ({zone_id}): {e}")
            self.message_var.set(f"Error sending command for {zone_id}.")

    # --- ROS Initialization (Called from Main Thread) ---
    def init_ros_comms(self):
        """Initialize ROS publishers and subscribers. MUST be called from main thread."""
        rospy.loginfo("GUI: Initializing ROS Comms...")
        try:
            for i in range(ZONE_COUNT):
                zone_id = f"zone{i+1}"
                # Publishers
                self.setpoint_pubs[zone_id] = rospy.Publisher(f'/extruder/{zone_id}/setpoint', Float32, queue_size=1)
                self.state_cmd_pubs[zone_id] = rospy.Publisher(f'/extruder/{zone_id}/state_cmd', String, queue_size=1)

                # Subscribers
                self.temp_subs[zone_id] = rospy.Subscriber(
                    f'/extruder/{zone_id}/temperature',
                    Float32,
                    self.update_temp_callback,
                    callback_args=zone_id # Pass zone_id to callback
                )
                # --- ADDED Subscriber for state_cmd ---
                self.state_cmd_subs[zone_id] = rospy.Subscriber(
                    f'/extruder/{zone_id}/state_cmd',
                    String,
                    self.update_state_cmd_callback,
                    callback_args=zone_id # Pass zone_id to callback
                )
                # ------------------------------------
            rospy.loginfo("GUI: ROS Comms Initialized.")
            return True
        except Exception as e:
            rospy.logfatal(f"GUI: Failed to initialize ROS Comms: {e}")
            self.message_var.set(f"FATAL: Failed to init ROS Comms: {e}")
            return False

    # --- ROS Background Thread ---
    def ros_spin_thread(self):
        """Runs rospy.spin() in a separate thread."""
        rospy.loginfo("GUI: ROS Spin thread started.")
        # --- FIX: Call the correct method name `init_ros_comms` ---
        # Also, check `self.init_ros_comms()` not `app.setup_ros_comms()`
        # NOTE: Initialization is now called from main(), this thread just spins
        # if not self.init_ros_comms(): # Correct Name and reference - MOVED TO MAIN
        #    rospy.logerr("GUI: ROS Comms failed to initialize in spin thread. Exiting thread.")
        #    return # Exit thread if comms fail
        # ---------------------------------------
        rospy.spin()
        rospy.loginfo("GUI: ROS Spin thread finished.")


# --- Main Execution Block ---
def main():
    # --- Initialize ROS Node FIRST in the main thread ---
    try:
        rospy.init_node('extruder_gui_node', anonymous=True)
        rospy.loginfo("GUI Node initialized in main thread.")
    except Exception as e:
        print(f"FATAL: Failed to initialize ROS Node in main thread: {e}", file=sys.stderr, flush=True)
        sys.exit(1) # Cannot proceed without ROS node

    # --- Create Tkinter App ---
    root = tk.Tk()
    app = ExtruderGUI(root)

    # --- Initialize ROS Comms AFTER GUI is created ---
    if not app.init_ros_comms():
        # Handle fatal error during pub/sub setup if needed
        # For now, GUI might still run but show errors or no data
        print("ERROR: ROS Communication setup failed. GUI might not function correctly.", file=sys.stderr, flush=True)
        # Optionally: root.destroy() sys.exit(1)

    # --- Start ROS Spin Thread ---
    ros_thread = Thread(target=app.ros_spin_thread)
    ros_thread.daemon = True # Allows GUI to exit cleanly
    ros_thread.start()

    # --- Start Tkinter Main Loop ---
    try:
        root.mainloop()
    except KeyboardInterrupt:
        rospy.loginfo("GUI received KeyboardInterrupt. Shutting down.")
    finally:
        # Optional cleanup if needed when GUI closes
        rospy.loginfo("GUI main loop finished.")


if __name__ == '__main__':
    main()