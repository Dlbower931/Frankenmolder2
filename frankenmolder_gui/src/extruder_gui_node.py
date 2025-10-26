#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Bool, String # Bool might be removed later if not needed
import tkinter as tk
from threading import Thread, Lock
import math # For isnan check

# --- Configuration ---
ZONE_COUNT = 3 
MIN_SETPOINT = 20.0
MAX_SETPOINT = 350.0
GUI_POLL_INTERVAL_MS = 500 # How often the GUI checks for updates

# --- Shared State (Thread Safety) ---
# Use dictionaries to store data for each zone
# These are updated by the ROS thread
latest_temps = {f"zone{i+1}": float('nan') for i in range(ZONE_COUNT)}
# latest_heater_states = {f"zone{i+1}": False for i in range(ZONE_COUNT)} # REMOVED - No longer subscribing
# Lock to prevent race conditions when accessing shared data
class ExtruderGUI(tk.Frame):
    def __init__(self, master):
        super().__init__(master)
        self.master = master
        self.master.title("Frankenmolder Extruder Control")

        # ROS Communication Objects (Initialized later)
        self.setpoint_pubs = {} # Dict: { "zone1": Publisher, ... }
        self.state_cmd_pubs = {} # Dict: { "zone1": Publisher, ... }

        # --- Data Storage (Tkinter Variables) ---
        self.current_temps = {} # Dict: { "zone1": tk.StringVar, ... }
        self.target_setpoints = {} # Dict: { "zone1": tk.DoubleVar, ... }
        self.current_mode = {} # Dict: { "zone1": tk.StringVar, ... } # Tracks commanded state

        self.message_var = tk.StringVar(value="System Initialized.") # Status message

        # Initialize Tkinter variables for each zone
        for i in range(ZONE_COUNT):
            zone_id = f"zone{i+1}"
            self.current_temps[zone_id] = tk.StringVar(value="--")
            self.target_setpoints[zone_id] = tk.DoubleVar(value=200.0) # Default target
            self.current_mode[zone_id] = tk.StringVar(value="OFF") # Start in OFF state

        self.create_widgets()

        # Start the periodic GUI update check
        self.master.after(GUI_POLL_INTERVAL_MS, self.check_for_updates)

    def create_widgets(self):
        # --- Main Barrel Frame ---
        barrel_frame = tk.Frame(self.master, bd=2, relief=tk.SUNKEN)
        # Span across columns needed for 3 zones + potential spacing
        barrel_frame.grid(row=0, column=0, columnspan=ZONE_COUNT * 2, padx=10, pady=10, sticky="ew")

        # --- Create Controls for Each Zone ---
        for i in range(ZONE_COUNT):
            zone_id = f"zone{i+1}"
            zone_frame = tk.LabelFrame(barrel_frame, text=f"Zone {i+1}", padx=10, pady=10)
            # Place zones side-by-side
            zone_frame.grid(row=0, column=i, padx=5, pady=5, sticky="nsew")
            # Configure column weight so zones expand equally if window is resized
            barrel_frame.grid_columnconfigure(i, weight=1)

            # Temperature Display
            tk.Label(zone_frame, text="Temp:").grid(row=0, column=0, sticky="w")
            temp_label = tk.Label(zone_frame, textvariable=self.current_temps[zone_id], font=("Arial", 18, "bold"))
            temp_label.grid(row=0, column=1, sticky="e")
            tk.Label(zone_frame, text="°C").grid(row=0, column=2, sticky="w")

            # Commanded Mode Display
            tk.Label(zone_frame, text="Mode:").grid(row=1, column=0, sticky="w")
            mode_label = tk.Label(zone_frame, textvariable=self.current_mode[zone_id], font=("Arial", 12, "italic"))
            mode_label.grid(row=1, column=1, columnspan=2, sticky="e")
            setattr(self, f"{zone_id}_mode_label", mode_label) # Store ref for color updates

            # Setpoint Control (Stacked)
            tk.Label(zone_frame, text="Setpoint (°C):", font=("Arial", 12)).grid(row=3, column=0, columnspan=3, pady=(10, 0), sticky="w")
            entry = tk.Entry(zone_frame, textvariable=self.target_setpoints[zone_id], width=10, font=("Arial", 16))
            entry.grid(row=4, column=0, columnspan=3, pady=5, ipady=8)
            set_button = tk.Button(zone_frame, text="Set PID Target", font=("Arial", 14, "bold"), width=15,
                                   command=lambda zid=zone_id: self.publish_setpoint(zid))
            set_button.grid(row=5, column=0, columnspan=3, pady=5, ipady=8)

            # --- State Control Buttons (Stacked Vertically) ---
            button_frame = tk.Frame(zone_frame)
            button_frame.grid(row=6, column=0, columnspan=3, pady=(15, 5), sticky="ew") # Allow frame to expand horizontally

            off_button = tk.Button(button_frame, text="OFF", bg="red", fg="white", font=("Arial", 12, "bold"),
                                   command=lambda zid=zone_id: self.publish_state_cmd(zid, "OFF"))
            # Pack vertically, fill horizontal space, add vertical padding
            off_button.pack(side=tk.TOP, fill=tk.X, pady=2)

            # Button text is "START", but command publishes "HEATING"
            start_button = tk.Button(button_frame, text="START", bg="orange", fg="black", font=("Arial", 12, "bold"),
                                    command=lambda zid=zone_id: self.publish_state_cmd(zid, "HEATING")) # Publishes "HEATING"
            # Pack vertically below the OFF button
            start_button.pack(side=tk.TOP, fill=tk.X, pady=2)
            # PID button REMOVED

            # Configure the button frame's column to expand if needed
            zone_frame.grid_columnconfigure(0, weight=1)


        # --- Status Bar ---
        status_label = tk.Label(self.master, textvariable=self.message_var, bd=1, relief=tk.SUNKEN, anchor=tk.W)
        # Span across columns needed for 3 zones
        status_label.grid(row=1, column=0, columnspan=ZONE_COUNT * 2, sticky='ew', padx=10, pady=(0, 5))

    # --- GUI Update Methods ---
    def check_for_updates(self):
        """Periodically check shared data and update Tkinter variables."""
        try:
            with data_lock:
                for i in range(ZONE_COUNT):
                    zone_id = f"zone{i+1}"
                    temp_val = latest_temps[zone_id]

                    # Update Temperature Display
                    if not math.isnan(temp_val):
                        self.current_temps[zone_id].set(f"{temp_val:.1f}")
                    else:
                        self.current_temps[zone_id].set("--")

                    # Update Mode Label Color based on commanded state
                    # NOTE: This reflects the COMMANDED state, not necessarily the ACTUAL state
                    # The control node should publish the ACTUAL state if needed for display
                    mode = self.current_mode[zone_id].get()
                    mode_label = getattr(self, f"{zone_id}_mode_label", None)
                    if mode_label:
                        if mode == "OFF": mode_label.config(fg="red")
                        elif mode == "HEATING": mode_label.config(fg="orange") # Internal state is HEATING
                        elif mode == "PID": mode_label.config(fg="green")
                        else: mode_label.config(fg="black")

        except Exception as e:
            rospy.logerr(f"G_POLL: Error during GUI update: {e}")
            self.message_var.set(f"Error during GUI update: {e}")

        # Reschedule the next check
        self.master.after(GUI_POLL_INTERVAL_MS, self.check_for_updates)

    # --- ROS Publishing Methods ---
    def publish_setpoint(self, zone_id):
        """Validate and publish the setpoint. Does NOT change the state command."""
        if zone_id not in self.setpoint_pubs:
            rospy.logwarn(f"Setpoint Publisher for {zone_id} not initialized.")
            self.message_var.set(f"Error: Publisher for {zone_id} not ready.")
            return

        try:
            setpoint_value = self.target_setpoints[zone_id].get()
            # Validation
            if MIN_SETPOINT <= setpoint_value <= MAX_SETPOINT:
                msg = Float32(data=setpoint_value)
                self.setpoint_pubs[zone_id].publish(msg)
                rospy.loginfo(f"GUI published setpoint for {zone_id}: {setpoint_value}")
                self.message_var.set(f"{zone_id.capitalize()} setpoint updated to {setpoint_value:.1f}°C")
                # --- REMOVED AUTOMATIC PID STATE CHANGE ---
                # self.publish_state_cmd(zone_id, "PID") # Control node handles this transition
                # ----------------------------------------
            else:
                rospy.logwarn(f"Setpoint {setpoint_value} for {zone_id} out of range ({MIN_SETPOINT}-{MAX_SETPOINT}).")
                self.message_var.set(f"Error: {zone_id.capitalize()} setpoint out of range ({MIN_SETPOINT}-{MAX_SETPOINT}°C).")
        except tk.TclError:
            rospy.logwarn(f"Invalid setpoint input for {zone_id}.")
            self.message_var.set(f"Error: Invalid number entered for {zone_id.capitalize()} setpoint.")
        except Exception as e:
             rospy.logerr(f"Error publishing setpoint for {zone_id}: {e}")
             self.message_var.set(f"Error publishing setpoint for {zone_id}.")

    def publish_state_cmd(self, zone_id, state):
        """Publish the desired state command for a specific zone."""
        if zone_id not in self.state_cmd_pubs:
            rospy.logwarn(f"State Command Publisher for {zone_id} not initialized.")
            self.message_var.set(f"Error: State Publisher for {zone_id} not ready.")
            return

        # PID is now only set implicitly by the control node. GUI only sends OFF/HEATING.
        allowed_states = ["OFF", "HEATING"] # GUI can only command these states directly
        # Internal state tracking still uses "PID" if setpoint is published
        internal_states = ["OFF", "HEATING", "PID"]

        if state not in allowed_states and state != "PID": # Allow internal PID state tracking
             rospy.logerr(f"Internal Error: Invalid explicit state command '{state}' for {zone_id}.")
             self.message_var.set(f"Internal Error: Invalid explicit state '{state}' for {zone_id}.")
             return

        try:
            # Publish the commanded state (OFF or HEATING)
            if state in allowed_states:
                msg = String(data=state)
                self.state_cmd_pubs[zone_id].publish(msg)
                rospy.loginfo(f"GUI published state command for {zone_id}: {state}")

            # Update GUI display immediately based on button press or implicit PID
            if state in internal_states:
                self.current_mode[zone_id].set(state)

            # Update user message based on the button pressed or implicit PID
            if state == "OFF":
                self.message_var.set(f"{zone_id.capitalize()} mode commanded to OFF.")
            elif state == "HEATING":
                # User pressed START, but internal state is HEATING
                self.message_var.set(f"{zone_id.capitalize()} mode commanded to START (Heating).")
            elif state == "PID":
                 # This state is now set internally when setpoint is published
                 # The user message is handled in publish_setpoint
                 pass
        except Exception as e:
             rospy.logerr(f"Error publishing state command for {zone_id}: {e}")
             self.message_var.set(f"Error publishing state for {zone_id}.")

    # --- ROS Initialization/Loop (Worker Thread) ---
    def setup_ros_comms(self):
        """Initializes ROS publishers and subscribers for ALL zones."""
        rospy.loginfo("R_THREAD: Setting up ROS publishers and subscribers...")
        try:
            for i in range(ZONE_COUNT): # Iterate through all 3 zones
                zone_id = f"zone{i+1}"

                # Publishers
                self.setpoint_pubs[zone_id] = rospy.Publisher(f'/extruder/{zone_id}/setpoint', Float32, queue_size=1)
                self.state_cmd_pubs[zone_id] = rospy.Publisher(f'/extruder/{zone_id}/state_cmd', String, queue_size=1)

                # Subscribers
                rospy.Subscriber(f'/extruder/{zone_id}/temperature', Float32,
                                 lambda msg, zid=zone_id: self.temp_callback(zid, msg))
                # REMOVED SUBSCRIBER for heater_cmd

            rospy.loginfo("R_THREAD: ROS publishers and subscribers initialized successfully.")
            return True

        except Exception as e:
            rospy.logerr(f"R_THREAD: Failed to initialize ROS comms: {e}")
            self.message_var.set(f"FATAL: ROS Comms Init Error: {e}") # Update GUI on error
            return False

    # --- ROS Callbacks (Worker Thread) ---
    def temp_callback(self, zone_id, temp_msg):
        """Stores the latest temperature data safely."""
        try:
            with data_lock:
                # Ensure the zone_id is valid before writing
                if zone_id in latest_temps:
                    latest_temps[zone_id] = temp_msg.data
                else:
                    rospy.logwarn(f"R_TEMP_CB: Received temp for unknown zone '{zone_id}'")
            # Optional: Log reception for debugging
            # rospy.loginfo(f"R_TEMP_CB ({zone_id}): Received temp {temp_msg.data:.1f}")
        except Exception as e:
            rospy.logerr(f"R_TEMP_CB ({zone_id}): Error storing temp data: {e}")



# --- Main Execution ---
def ros_thread_loop(app):
    """ROS initialization and spin loop run in a separate thread."""
    if not app.setup_ros_comms():
        # Handle ROS initialization failure (e.g., update GUI, log error)
        # For simplicity, just log and exit the thread
        rospy.logfatal("ROS communication setup failed. Exiting ROS thread.")
        # Attempt to update GUI from worker thread (might need queue for safety)
        # Using master.after to schedule GUI update from main thread might be safer
        # For now, directly setting the var, but be cautious.
        app.master.after(100, lambda: app.message_var.set("FATAL: Failed to connect ROS. Check logs."))
        return

    rospy.loginfo("ROS communication thread running and spinning.")
    rospy.spin() # Keeps the ROS thread alive

def main():
    # Must initialize ROS node in the main thread for signal handling
    rospy.init_node('extruder_gui_node', anonymous=True)
    rospy.loginfo("GUI Node initialized in main thread.")

    root = tk.Tk()
    app = ExtruderGUI(root) # Create the GUI instance

    # Start the ROS communication in a separate thread, passing the app instance
    ros_thread = Thread(target=ros_thread_loop, args=(app,))
    ros_thread.daemon = True # Allows the main program to exit when Tkinter closes
    ros_thread.start()

    # Start the Tkinter GUI loop in the main thread
    rospy.loginfo("Starting Tkinter main loop...")
    root.mainloop()
    rospy.loginfo("Tkinter main loop finished.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("GUI node shutting down.")
    except Exception as e:
        rospy.logerr(f"GUI node crashed: {e}")
