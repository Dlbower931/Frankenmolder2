#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, String
import tkinter as tk
from tkinter import ttk # Import the themed toolkit for tabs (Notebook)
from threading import Thread, Lock
import math
import sys # Added for stderr

# --- Configuration ---
ZONE_COUNT = 3
MIN_SETPOINT = 20.0
MAX_SETPOINT = 350.0
GUI_POLL_INTERVAL_MS = 500 # How often the GUI checks for updates

# --- Shared State (Thread Safety) ---
# Use dictionaries to store data for each zone
latest_temps = {f"zone{i+1}": float('nan') for i in range(ZONE_COUNT)}
# --- RENAMED: This now stores the *true* state from the control node ---
latest_actual_states = {f"zone{i+1}": "OFF" for i in range(ZONE_COUNT)}
latest_motor_state = "STOPPED" # Shared state for the motor

# Lock to prevent race conditions when accessing shared data
data_lock = Lock()

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
        display_frame = tk.Frame(self.popup, bg="white", bd=2, relief=tk.SUNKEN)
        display_frame.pack(padx=10, pady=10, fill=tk.X)
        self.display_label = tk.Label(display_frame, textvariable=self.input_str, 
                                      font=("Arial", 24, "bold"), bg="white", 
                                      anchor="e", padx=10, pady=5)
        self.display_label.pack(fill=tk.X)
        
        # Number pad buttons
        button_frame = tk.Frame(self.popup)
        button_frame.pack(padx=10, pady=10)
        
        # Button layout
        buttons = [
            ['7', '8', '9', 'C'],
            ['4', '5', '6', '⌫'],
            ['1', '2', '3', '.'],
            ['-', '0', '+', self.confirm_text]
        ]
        
        for i, row in enumerate(buttons):
            for j, btn_text in enumerate(row):
                # Make buttons larger for better touchscreen use
                btn = tk.Button(button_frame, text=btn_text, font=("Arial", 24, "bold"),
                               width=5, height=3, command=lambda t=btn_text: self.button_click(t))
                btn.grid(row=i, column=j, padx=5, pady=5, sticky="nsew")
                button_frame.grid_columnconfigure(j, weight=1, minsize=100)
            button_frame.grid_rowconfigure(i, weight=1, minsize=100)
        
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
        elif char == '.':
            if '.' not in current:
                self.input_str.set(current + char)
        elif char == '-':
            if current.startswith('-'):
                self.input_str.set(current[1:])
            else:
                self.input_str.set('-' + current)
        elif char == '+':
            if current.startswith('-'):
                self.input_str.set(current[1:])
        else:  # Number
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

        # --- Create the Tabbed Notebook ---
        self.notebook = ttk.Notebook(self.master)

        # --- Tab 1: Dashboard ---
        self.dashboard_frame = DashboardFrame(self.notebook)
        self.notebook.add(self.dashboard_frame, text='Dashboard')

        # --- Tab 2: Heater Control (No Scrollbar) ---
        # Create the HeaterControlFrame directly in the notebook
        self.heater_frame = HeaterControlFrame(self.notebook, self) 
        self.notebook.add(self.heater_frame, text='Extruder Heaters')
        # --- End of Tab 2 Setup ---

        # --- Tab 3: Motor Control ---
        self.motor_frame = MotorControlFrame(self.notebook, self)
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
            
            # --- Motor Publishers & Subscribers ---
            self.publishers["motor_set_rpm"] = rospy.Publisher('/extruder/motor/set_rpm', Float32, queue_size=1)
            self.publishers["motor_state_cmd"] = rospy.Publisher('/extruder/motor/state_cmd', String, queue_size=1)

            # This subscription for the motor is already correct
            self.subscribers["motor_actual_state"] = rospy.Subscriber(
                '/extruder/motor/actual_state', # Listen for state from PICO
                String,
                self.update_motor_state_callback
            )

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
        # Parent is now the Notebook
        super().__init__(parent, **kwargs) 
        self.main_app = main_app # Reference to main app to access publishers

        # --- Tkinter Variables (Local to this frame) ---
        self.current_temps = {}
        self.target_setpoints = {}
        self.current_mode = {} # This will now reflect ACTUAL state
        self.mode_labels = {} # Store refs to labels for coloring
        self.setpoint_buttons = {} # Store refs to setpoint buttons

        # --- Main Barrel Frame ---
        barrel_frame = tk.Frame(self, bd=2, relief=tk.SUNKEN)
        barrel_frame.pack(padx=10, pady=10, fill=tk.X, expand=True)

        # --- Create Controls for Each Zone ---
        for i in range(ZONE_COUNT):
            zone_id = f"zone{i+1}"
            
            # Init Tkinter vars
            self.current_temps[zone_id] = tk.StringVar(value="--")
            # Use StringVar for setpoint so button can display formatted text
            self.target_setpoints[zone_id] = tk.StringVar(value="30.0")
            self.current_mode[zone_id] = tk.StringVar(value="OFF") # Displays ACTUAL state

            zone_frame = tk.LabelFrame(barrel_frame, text=f"Zone {i+1}", padx=10, pady=10, font=("Arial", 14, "bold"))
            zone_frame.grid(row=0, column=i, padx=5, pady=5, sticky="nsew")
            barrel_frame.grid_columnconfigure(i, weight=1) # Make zones expand equally

            # Temperature Display
            tk.Label(zone_frame, text="Temp:", font=("Arial", 12)).grid(row=0, column=0, sticky="w")
            temp_label = tk.Label(zone_frame, textvariable=self.current_temps[zone_id], font=("Arial", 18, "bold"))
            temp_label.grid(row=0, column=1, columnspan=2, sticky="e")
            tk.Label(zone_frame, text="°C", font=("Arial", 12)).grid(row=0, column=3, sticky="w")

            # Actual Mode Display
            tk.Label(zone_frame, text="Mode:", font=("Arial", 12)).grid(row=1, column=0, sticky="w")
            mode_label = tk.Label(zone_frame, textvariable=self.current_mode[zone_id], font=("Arial", 12, "italic"))
            mode_label.grid(row=1, column=1, columnspan=3, sticky="e")
            self.mode_labels[zone_id] = mode_label # Store ref for coloring

            # Setpoint Control (Stacked)
            tk.Label(zone_frame, text="Setpoint (°C):", font=("Arial", 12)).grid(row=2, column=0, columnspan=4, pady=(10, 0), sticky="w")
            # Create button that displays current setpoint value and opens number pad when clicked
            setpoint_btn = tk.Button(zone_frame, textvariable=self.target_setpoints[zone_id], 
                                   font=("Arial", 18, "bold"), width=15, height=2,
                                   command=lambda zid=zone_id: self.open_number_pad(zid))
            setpoint_btn.grid(row=3, column=0, columnspan=4, pady=5, ipady=8, sticky="ew")
            self.setpoint_buttons[zone_id] = setpoint_btn
            # Format the button text to show value with decimal
            self.update_setpoint_button_text(zone_id)
            # Removed "Set Target" button - it's now in the number pad popup

            # State Control Buttons (Stacked OFF/START)
            tk.Label(zone_frame, text="Commands:", font=("Arial", 12)).grid(row=5, column=0, columnspan=4, pady=(15, 0), sticky="w")
            button_frame = tk.Frame(zone_frame)
            button_frame.grid(row=6, column=0, columnspan=4, pady=(5, 5), sticky="ew")

            off_button = tk.Button(button_frame, text="OFF", bg="red", fg="white", font=("Arial", 12, "bold"), height=2,
                                   command=lambda zid=zone_id: self.publish_state_cmd(zid, "OFF"))
            off_button.pack(side=tk.TOP, fill=tk.X, pady=2, expand=True)

            start_button = tk.Button(button_frame, text="START", bg="#00008B", fg="white", font=("Arial", 12, "bold"), height=2,
                                     command=lambda zid=zone_id: self.publish_state_cmd(zid, "ON"))
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
                
                # --- FIX: Update Mode from the /actual_state topic data ---
                mode = latest_actual_states.get(zone_id, "OFF")
                self.current_mode[zone_id].set(mode)
                
                # Update Color based on the actual state
                mode_label = self.mode_labels.get(zone_id)
                if mode_label:
                    if mode == "OFF": mode_label.config(fg="red")
                    elif mode == "HEATING": mode_label.config(fg="orange")
                    elif mode == "PID": mode_label.config(fg="green")
                    else: mode_label.config(fg="black") # Default/Unknown

    def update_setpoint_button_text(self, zone_id):
        """Update the button text to show formatted setpoint value."""
        value = self.target_setpoints[zone_id].get()
        self.target_setpoints[zone_id].set(value)  # Trigger update
        # The button textvariable will automatically update
    
    def open_number_pad(self, zone_id):
        """Open number pad popup for setpoint entry."""
        # Create a dummy entry widget for positioning (won't be visible)
        entry_ref = self.setpoint_buttons[zone_id]
        NumberPadPopup(
            self.main_app.master,
            entry_ref,
            self.target_setpoints[zone_id],
            min_val=MIN_SETPOINT,
            max_val=MAX_SETPOINT,
            confirm_text="Set",
            callback=lambda v: self.update_setpoint_button_text(zone_id),
            on_confirm=lambda v: self.publish_setpoint(zone_id)  # Auto-publish when Set Target is pressed
        )
    
    def publish_setpoint(self, zone_id):
        """Validate and publish the setpoint."""
        pub = self.main_app.publishers.get(f"{zone_id}_setpoint")
        if not pub:
            rospy.logerr(f"Setpoint Publisher for {zone_id} not found.")
            return

        try:
            # Get value as float from StringVar
            setpoint_str = self.target_setpoints[zone_id].get()
            setpoint_value = float(setpoint_str)
            
            if not (MIN_SETPOINT <= setpoint_value <= MAX_SETPOINT):
                raise ValueError(f"Setpoint must be between {MIN_SETPOINT} and {MAX_SETPOINT}°C.")

            # Explicitly create Float32 message and set data field
            msg = Float32()
            msg.data = float(setpoint_value)
            pub.publish(msg)
            rospy.loginfo(f"GUI published setpoint for {zone_id}: {setpoint_value} (msg.data={msg.data})")
            self.main_app.message_var.set(f"{zone_id}: Setpoint {setpoint_value:.1f}°C accepted.")
            
            # --- Removed automatic HEATING command ---
            # User must explicitly click START to begin heating

        except ValueError as e:
             rospy.logwarn(f"GUI Validation Error ({zone_id}): {e}")
             self.main_app.message_var.set(f"Error ({zone_id}): {e}")
        except Exception as e:
             rospy.logwarn(f"GUI Error ({zone_id}): {e}")
             self.main_app.message_var.set(f"Error ({zone_id}): {e}")

    def publish_state_cmd(self, zone_id, state_command):
        """Publish the state command ('OFF', 'ON')."""
        pub = self.main_app.publishers.get(f"{zone_id}_state_cmd")
        if not pub:
            rospy.logerr(f"State Command Publisher for {zone_id} not found.")
            return

        try:
            # Send the command as-is (ON or OFF)
            msg = String(state_command)
            pub.publish(msg)
            rospy.loginfo(f"GUI published state_cmd for {zone_id}: {state_command}")
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
        self.actual_motor_state = tk.StringVar(value="STOPPED")

        # --- Layout ---
        control_frame = tk.LabelFrame(self, text="Motor Control", padx=20, pady=20, font=("Arial", 14, "bold"))
        control_frame.pack(padx=20, pady=20, fill=tk.BOTH, expand=True)

        # State Display
        tk.Label(control_frame, text="Actual State:", font=("Arial", 14)).grid(row=0, column=0, padx=10, pady=10, sticky="w")
        self.state_label = tk.Label(control_frame, textvariable=self.actual_motor_state, font=("Arial", 18, "bold"), fg="red")
        self.state_label.grid(row=0, column=1, padx=10, pady=10, sticky="w")
        
        # RPM Control
        tk.Label(control_frame, text="Target RPM:", font=("Arial", 14)).grid(row=1, column=0, padx=10, pady=10, sticky="w")
        # Create button that displays current RPM value and opens number pad when clicked
        self.rpm_btn = tk.Button(control_frame, textvariable=self.target_rpm, 
                                font=("Arial", 20, "bold"), width=8, height=2,
                                command=self.open_number_pad_rpm)
        self.rpm_btn.grid(row=1, column=1, padx=10, pady=10, ipady=8)
        rpm_button = tk.Button(control_frame, text="Set RPM", font=("Arial", 14, "bold"),
                               command=self.publish_set_rpm)
        rpm_button.grid(row=1, column=2, padx=10, pady=10, ipady=8)

        # Command Buttons
        button_frame = tk.Frame(control_frame)
        button_frame.grid(row=2, column=0, columnspan=3, pady=20)
        
        start_button = tk.Button(button_frame, text="START MOTOR", bg="green", fg="white", font=("Arial", 14, "bold"), height=2, width=15,
                                 command=lambda: self.publish_motor_cmd("ON"))
        start_button.pack(side=tk.LEFT, padx=10)

        stop_button = tk.Button(button_frame, text="STOP MOTOR", bg="red", fg="white", font=("Arial", 14, "bold"), height=2, width=15,
                                command=lambda: self.publish_motor_cmd("OFF"))
        stop_button.pack(side=tk.LEFT, padx=10)

    def update_gui_widgets(self):
        """Update all widgets in this frame with data from shared state."""
        with data_lock:
            state = latest_motor_state
            self.actual_motor_state.set(state)
            
            # Update color based on state
            if state == "RUNNING" or state == "ON": # Accept "ON" as running
                self.state_label.config(fg="green")
            elif state == "STOPPED" or state == "OFF": # Accept "OFF" as stopped
                self.state_label.config(fg="red")
            elif state == "FAULT":
                self.state_label.config(fg="orange")
            else:
                self.state_label.config(fg="black")

    def open_number_pad_rpm(self):
        """Open number pad popup for RPM entry."""
        NumberPadPopup(
            self.main_app.master,
            self.rpm_btn,
            self.target_rpm,
            min_val=0.0,
            max_val=100.0  # Reasonable max for RPM
        )
    
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
            
            msg = Float32()
            msg.data = float(rpm_value)
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
            msg = String(command)
            pub.publish(msg)
            rospy.loginfo(f"GUI published motor_state_cmd: {command}")
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