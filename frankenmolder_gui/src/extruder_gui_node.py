#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, String
import tkinter as tk
from tkinter import ttk # Import the themed toolkit for tabs (Notebook)
from threading import Thread, Lock
import math

# --- Configuration ---
ZONE_COUNT = 3
MIN_SETPOINT = 20.0
MAX_SETPOINT = 350.0
GUI_POLL_INTERVAL_MS = 500 # How often the GUI checks for updates

# --- Shared State (Thread Safety) ---
# Use dictionaries to store data for each zone
latest_temps = {f"zone{i+1}": float('nan') for i in range(ZONE_COUNT)}
latest_state_cmds = {f"zone{i+1}": "OFF" for i in range(ZONE_COUNT)}
latest_motor_state = "STOPPED" # Shared state for the motor

# Lock to prevent race conditions when accessing shared data
data_lock = Lock()
class ExtruderGUI(tk.Frame):
    """Main application class that holds the tabbed notebook."""
    def __init__(self, master):
        super().__init__(master)
        self.master = master
        self.master.title("Frankenmolder Control Interface")
        # Adjust window size if needed
        # self.master.geometry("800x480") 

        # --- ROS Communication Objects ---
        self.publishers = {}
        self.subscribers = {}

        # --- Create the Tabbed Notebook ---
        self.notebook = ttk.Notebook(self.master)

        # --- Tab 1: Dashboard ---
        self.dashboard_frame = DashboardFrame(self.notebook)
        self.notebook.add(self.dashboard_frame, text='Dashboard')

        # --- Tab 2: Heater Control (with Scrollbar) ---
        # Create a container frame for the tab content
        heater_tab_container = ttk.Frame(self.notebook)
        heater_tab_container.pack(fill="both", expand=True) # Fill the tab

        # Create a Canvas widget
        canvas = tk.Canvas(heater_tab_container)
        # Create a Scrollbar
        scrollbar = ttk.Scrollbar(heater_tab_container, orient="vertical", command=canvas.yview)
        # Configure the canvas to use the scrollbar
        canvas.configure(yscrollcommand=scrollbar.set)

        # Pack the scrollbar and canvas
        scrollbar.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)

        # Create the actual content frame (HeaterControlFrame) *inside* the canvas
        self.heater_frame = HeaterControlFrame(canvas, self) # Pass canvas as parent
        
        # Add the content frame to the canvas
        canvas.create_window((0, 0), window=self.heater_frame, anchor="nw")

        # Bind the <Configure> event of the content frame to update the canvas scrollregion
        # This makes the scrollbar aware of the content's total size
        self.heater_frame.bind("<Configure>", 
                               lambda e: canvas.configure(scrollregion=canvas.bbox("all")))

        # Add the container (with canvas and scrollbar) to the notebook
        self.notebook.add(heater_tab_container, text='Extruder Heaters')
        # --- End of Tab 2 Setup ---

        # --- Tab 3: Motor Control ---
        self.motor_frame = MotorControlFrame(self.notebook, self) # Pass self (main app)
        self.notebook.add(self.motor_frame, text='Extruder Motor')

        self.notebook.pack(expand=True, fill='both', padx=10, pady=10)

        # --- Status Bar (Shared by all tabs) ---
        self.message_var = tk.StringVar(value="System Initialized.")
        status_label = tk.Label(self.master, textvariable=self.message_var, bd=1, relief=tk.SUNKEN, anchor=tk.W)
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
                # Publishers
                self.publishers[f"{zone_id}_setpoint"] = rospy.Publisher(f'/extruder/{zone_id}/setpoint', Float32, queue_size=1)
                self.publishers[f"{zone_id}_state_cmd"] = rospy.Publisher(f'/extruder/{zone_id}/state_cmd', String, queue_size=1)
                
                # Subscribers
                self.subscribers[f"{zone_id}_temp"] = rospy.Subscriber(
                    f'/extruder/{zone_id}/temperature', 
                    Float32, 
                    self.update_temp_callback, 
                    callback_args=zone_id
                )
                self.subscribers[f"{zone_id}_state_cmd"] = rospy.Subscriber(
                    f'/extruder/{zone_id}/state_cmd', # Listen to the same topic we publish to
                    String,
                    self.update_state_cmd_callback,
                    callback_args=zone_id
                )
            
            # --- Motor Publishers & Subscribers ---
            self.publishers["motor_set_rpm"] = rospy.Publisher('/extruder/motor/set_rpm', Float32, queue_size=1)
            self.publishers["motor_state_cmd"] = rospy.Publisher('/extruder/motor/state_cmd', String, queue_size=1)

            # --- UPDATED SUBSCRIBER ---
            # Change subscription from /actual_state to /state_cmd
            self.subscribers["motor_state_cmd"] = rospy.Subscriber(
                '/extruder/motor/state_cmd', # Listen for state from PICO/self
                String,
                self.update_motor_state_callback
            )
            # --------------------------

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
                latest_temps[zone_id] = temp_msg.data
        except Exception as e:
            rospy.logerr(f"R_TEMP_CB ({zone_id}): Error storing temp data: {e}")

    def update_state_cmd_callback(self, state_msg, zone_id):
        """ROS thread callback: Safely update shared state data."""
        try:
            with data_lock:
                latest_state_cmds[zone_id] = state_msg.data
        except Exception as e:
            rospy.logerr(f"R_STATE_CB ({zone_id}): Error storing state data: {e}")

    def update_motor_state_callback(self, state_msg):
        """ROS thread callback: Safely update shared motor state."""
        try:
            with data_lock:
                global latest_motor_state # Ensure we modify the global
                latest_motor_state = state_msg.data
        except Exception as e:
            rospy.logerr(f"R_MOTOR_CB: Error storing motor state: {e}")

    # --- GUI Polling Function (Read shared data) ---
    def poll_ros_updates(self):
        """GUI thread loop: Safely read shared data and update all frames."""
        try:
            # Pass data to each frame to update its widgets
            self.heater_frame.update_gui_widgets()
            self.motor_frame.update_gui_widgets()
            # self.dashboard_frame.update_gui_widgets() # Add when dashboard has data

        except Exception as e:
            err_msg = f"G_POLL: Error during GUI update: {e}"
            rospy.logerr(err_msg)
            self.message_var.set(err_msg)

        # Reschedule the next check
        self.master.after(GUI_POLL_INTERVAL_MS, self.poll_ros_updates)

class DashboardFrame(tk.Frame):
    """A blank frame for the main dashboard/overview page."""
    def __init__(self, parent, **kwargs):
        super().__init__(parent, **kwargs)
        
        label = tk.Label(self, text="Main Dashboard (Overview)", font=("Arial", 24))
        label.pack(padx=50, pady=50)
        # Add high-level status widgets here later

    def update_gui_widgets(self):
        # Update dashboard widgets here
        pass


class HeaterControlFrame(tk.Frame):
    """The frame containing the 3-Zone Heater controls."""
    def __init__(self, parent, main_app, **kwargs):
        # Parent is now the Canvas, which is fine
        super().__init__(parent, **kwargs) 
        self.main_app = main_app # Reference to main app to access publishers

        # --- Tkinter Variables (Local to this frame) ---
        self.current_temps = {}
        self.target_setpoints = {}
        self.current_mode = {}
        self.mode_labels = {} # Store refs to labels for coloring

        # --- Main Barrel Frame ---
        # This frame is now *inside* the canvas
        barrel_frame = tk.Frame(self, bd=2, relief=tk.SUNKEN)
        # Use pack() instead of grid() since this frame is inside the canvas window
        barrel_frame.pack(padx=10, pady=10, fill=tk.X, expand=True)

        # --- Create Controls for Each Zone ---
        for i in range(ZONE_COUNT):
            zone_id = f"zone{i+1}"
            
            # Init Tkinter vars
            self.current_temps[zone_id] = tk.StringVar(value="--")
            self.target_setpoints[zone_id] = tk.DoubleVar(value=200.0)
            self.current_mode[zone_id] = tk.StringVar(value="OFF")

            zone_frame = tk.LabelFrame(barrel_frame, text=f"Zone {i+1}", padx=10, pady=10, font=("Arial", 14, "bold"))
            zone_frame.grid(row=0, column=i, padx=5, pady=5, sticky="nsew")
            barrel_frame.grid_columnconfigure(i, weight=1) # Make zones expand equally

            # Temperature Display
            tk.Label(zone_frame, text="Temp:", font=("Arial", 12)).grid(row=0, column=0, sticky="w")
            temp_label = tk.Label(zone_frame, textvariable=self.current_temps[zone_id], font=("Arial", 18, "bold"))
            temp_label.grid(row=0, column=1, columnspan=2, sticky="e")
            tk.Label(zone_frame, text="°C", font=("Arial", 12)).grid(row=0, column=3, sticky="w")

            # Commanded Mode Display
            tk.Label(zone_frame, text="Mode:", font=("Arial", 12)).grid(row=1, column=0, sticky="w")
            mode_label = tk.Label(zone_frame, textvariable=self.current_mode[zone_id], font=("Arial", 12, "italic"))
            mode_label.grid(row=1, column=1, columnspan=3, sticky="e")
            self.mode_labels[zone_id] = mode_label # Store ref for coloring

            # Setpoint Control (Stacked)
            tk.Label(zone_frame, text="Setpoint (°C):", font=("Arial", 12)).grid(row=2, column=0, columnspan=4, pady=(10, 0), sticky="w")
            entry = tk.Entry(zone_frame, textvariable=self.target_setpoints[zone_id], width=10, font=("Arial", 16))
            entry.grid(row=3, column=0, columnspan=4, pady=5, ipady=8, sticky="ew")
            set_button = tk.Button(zone_frame, text="Set Target", font=("Arial", 14, "bold"), width=15,
                                   command=lambda zid=zone_id: self.publish_setpoint(zid))
            set_button.grid(row=4, column=0, columnspan=4, pady=5, ipady=8, sticky="ew")

            # State Control Buttons (Stacked OFF/START)
            tk.Label(zone_frame, text="Commands:", font=("Arial", 12)).grid(row=5, column=0, columnspan=4, pady=(15, 0), sticky="w")
            button_frame = tk.Frame(zone_frame)
            button_frame.grid(row=6, column=0, columnspan=4, pady=(5, 5), sticky="ew")

            off_button = tk.Button(button_frame, text="OFF", bg="red", fg="white", font=("Arial", 12, "bold"), height=2,
                                   command=lambda zid=zone_id: self.publish_state_cmd(zid, "OFF"))
            off_button.pack(side=tk.TOP, fill=tk.X, pady=2, expand=True)

            start_button = tk.Button(button_frame, text="START", bg="orange", fg="black", font=("Arial", 12, "bold"), height=2,
                                     command=lambda zid=zone_id: self.publish_state_cmd(zid, "HEATING"))
            start_button.pack(side=tk.TOP, fill=tk.X, pady=2, expand=True)

    def update_gui_widgets(self):
        """Update all widgets in this frame with data from shared state."""
        with data_lock:
            for i in range(ZONE_COUNT):
                zone_id = f"zone{i+1}"
                
                # Update Temp
                temp_val = latest_temps.get(zone_id, float('nan'))
                if not math.isnan(temp_val):
                    self.current_temps[zone_id].set(f"{temp_val:.1f}")
                else:
                    self.current_temps[zone_id].set("--")
                
                # Update Mode from state_cmd
                mode = latest_state_cmds.get(zone_id, "OFF")
                self.current_mode[zone_id].set(mode)
                
                # Update Color
                mode_label = self.mode_labels.get(zone_id)
                if mode_label:
                    if mode == "OFF": mode_label.config(fg="red")
                    elif mode == "HEATING": mode_label.config(fg="orange")
                    elif mode == "PID": mode_label.config(fg="green")
                    else: mode_label.config(fg="black")

    def publish_setpoint(self, zone_id):
        """Validate and publish the setpoint."""
        pub = self.main_app.publishers.get(f"{zone_id}_setpoint")
        if not pub:
            rospy.logerr(f"Setpoint Publisher for {zone_id} not found.")
            return

        try:
            setpoint_value = self.target_setpoints[zone_id].get()
            if not (MIN_SETPOINT <= setpoint_value <= MAX_SETPOINT):
                raise ValueError(f"Setpoint must be between {MIN_SETPOINT} and {MAX_SETPOINT}°C.")

            msg = Float32(setpoint_value)
            pub.publish(msg)
            rospy.loginfo(f"GUI published setpoint for {zone_id}: {setpoint_value}")
            self.main_app.message_var.set(f"{zone_id}: Setpoint {setpoint_value:.1f}°C accepted.")

        except Exception as e:
             rospy.logwarn(f"GUI Validation Error ({zone_id}): {e}")
             self.main_app.message_var.set(f"Error ({zone_id}): {e}")

    def publish_state_cmd(self, zone_id, state_command):
        """Publish the state command ('OFF', 'HEATING')."""
        pub = self.main_app.publishers.get(f"{zone_id}_state_cmd")
        if not pub:
            rospy.logerr(f"State Command Publisher for {zone_id} not found.")
            return

        try:
            # We want the button to say "START" but send "HEATING"
            internal_command = state_command
            if state_command == "START": # This check is redundant now but harmless
                internal_command = "HEATING"

            msg = String(internal_command)
            pub.publish(msg)
            rospy.loginfo(f"GUI published state_cmd for {zone_id}: {internal_command}")
            # GUI now updates its mode based on the subscription echo, not locally.
            self.main_app.message_var.set(f"{zone_id}: {state_command} command sent.")

        except Exception as e:
            rospy.logerr(f"GUI publish_state_cmd Error ({zone_id}): {e}")


class MotorControlFrame(tk.Frame):
    """The frame containing the Extruder Motor controls."""
    def __init__(self, parent, main_app, **kwargs):
        super().__init__(parent, **kwargs)
        self.main_app = main_app

        # --- Tkinter Variables ---
        self.target_rpm = tk.DoubleVar(value=10.0)
        self.actual_motor_state = tk.StringVar(value="OFF") # Default to OFF

        # --- Layout ---
        control_frame = tk.LabelFrame(self, text="Motor Control", padx=20, pady=20, font=("Arial", 14, "bold"))
        control_frame.pack(padx=20, pady=20, fill=tk.BOTH, expand=True)

        # State Display
        tk.Label(control_frame, text="Actual State:", font=("Arial", 14)).grid(row=0, column=0, padx=10, pady=10, sticky="w")
        self.state_label = tk.Label(control_frame, textvariable=self.actual_motor_state, font=("Arial", 18, "bold"), fg="red")
        self.state_label.grid(row=0, column=1, padx=10, pady=10, sticky="w")
        
        # RPM Control
        tk.Label(control_frame, text="Target RPM:", font=("Arial", 14)).grid(row=1, column=0, padx=10, pady=10, sticky="w")
        rpm_entry = tk.Entry(control_frame, textvariable=self.target_rpm, width=10, font=("Arial", 18))
        rpm_entry.grid(row=1, column=1, padx=10, pady=10, ipady=8)
        rpm_button = tk.Button(control_frame, text="Set RPM", font=("Arial", 14, "bold"),
                               command=self.publish_set_rpm)
        rpm_button.grid(row=1, column=2, padx=10, pady=10, ipady=8)

        # Command Buttons
        button_frame = tk.Frame(control_frame)
        button_frame.grid(row=2, column=0, columnspan=3, pady=20)
        
        start_button = tk.Button(button_frame, text="START MOTOR", bg="green", fg="white", font=("Arial", 14, "bold"), height=2, width=15,
                                 # Publish "ON" when "START MOTOR" is clicked
                                 command=lambda: self.publish_motor_cmd("ON"))
        start_button.pack(side=tk.LEFT, padx=10)

        stop_button = tk.Button(button_frame, text="STOP MOTOR", bg="red", fg="white", font=("Arial", 14, "bold"), height=2, width=15,
                                # Publish "OFF" when "STOP MOTOR" is clicked
                                command=lambda: self.publish_motor_cmd("OFF"))
        stop_button.pack(side=tk.LEFT, padx=10)

    def update_gui_widgets(self):
        """Update all widgets in this frame with data from shared state."""
        with data_lock:
            state = latest_motor_state # This now comes from /extruder/motor/state_cmd
            self.actual_motor_state.set(state)
            
            # Update color based on state
            if state == "ON":
                self.state_label.config(fg="green")
            elif state == "OFF":
                self.state_label.config(fg="red")
            elif state == "FAULT": # Add other states as needed
                self.state_label.config(fg="orange")
            else:
                self.state_label.config(fg="black") # Default for states like "STARTING", etc.

    def publish_set_rpm(self):
        """Validate and publish the target RPM."""
        pub = self.main_app.publishers.get("motor_set_rpm")
        if not pub:
            rospy.logerr("Motor Set RPM Publisher not found.")
            return

        try:
            rpm_value = self.target_rpm.get()
            # Add validation as needed
            if rpm_value < 0:
                raise ValueError("RPM cannot be negative.")
            
            msg = Float32(rpm_value)
            pub.publish(msg)
            rospy.loginfo(f"GUI published motor_set_rpm: {rpm_value}")
            self.main_app.message_var.set(f"Motor RPM target {rpm_value} sent.")

        except Exception as e:
            rospy.logwarn(f"GUI Validation Error (Motor): {e}")
            self.main_app.message_var.set(f"Error (Motor): {e}")

    def publish_motor_cmd(self, command):
        """Publish the motor command ('ON', 'OFF')."""
        pub = self.main_app.publishers.get("motor_state_cmd")
        if not pub:
            rospy.logerr("Motor State Command Publisher not found.")
            return

        try:
            # The 'command' argument is now "ON" or "OFF"
            msg = String(command)
            pub.publish(msg)
            rospy.loginfo(f"GUI published motor_state_cmd: {command}")
            # Use the button text for the user message
            user_action = "START" if command == "ON" else "STOP"
            self.main_app.message_var.set(f"Motor {user_action} command sent.")
        except Exception as e:
            rospy.logerr(f"GUI publish_motor_cmd Error: {e}")
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
