#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Bool
import tkinter as tk
from tkinter import messagebox # For potential error popups (optional)
from threading import Thread, Lock
import time

# --- SECTION: Configuration & Shared State ---

ZONE_COUNT = 3
MIN_SETPOINT = 20.0
MAX_SETPOINT = 350.0

# --- Shared Data Structures (Thread Safety Critical) ---
# Use dictionaries to store data for each zone
# These will be updated by the ROS thread and read by the GUI thread
latest_temps = {}
latest_heater_states = {}
# Initialize with default values
for i in range(1, ZONE_COUNT + 1):
    latest_temps[f"zone{i}"] = float('nan') # Use NaN for uninitialized
    latest_heater_states[f"zone{i}"] = False

# Lock to protect access to shared data structures
data_lock = Lock()
# --- SECTION: GUI Class ---

class ExtruderGUI(tk.Frame):
    def __init__(self, master):
        super().__init__(master)
        self.master = master
        self.master.title("Frankenmolder Extruder Control")

        # --- ROS Communication Objects ---
        self.setpoint_pubs = {}

        # --- Tkinter Variables (GUI Thread Owned) ---
        self.tk_current_temps = {}
        self.tk_heater_states = {}
        self.tk_target_setpoints = {}
        self.message_var = tk.StringVar(value="System Initialized.")

        # --- Widget Storage (For reliable updates) ---
        self._heater_state_labels = {} # Store labels for easy access

        for i in range(1, ZONE_COUNT + 1):
            zone_id = f"zone{i}"
            self.tk_current_temps[zone_id] = tk.StringVar(value="--")
            self.tk_heater_states[zone_id] = tk.StringVar(value="OFF")
            self.tk_target_setpoints[zone_id] = tk.DoubleVar(value=200.0)

        self.create_widgets()
        self.master.after(500, self.check_for_updates) # Start the polling loop

    def create_widgets(self):
        """Creates and arranges the Tkinter widgets."""
        main_frame = tk.Frame(self.master, bd=2, relief=tk.GROOVE)
        main_frame.pack(padx=10, pady=10, fill=tk.X)
        tk.Label(main_frame, text="Extruder Barrel Zones", font=("Arial", 14, "bold")).pack()

        barrel_frame = tk.Frame(main_frame)
        barrel_frame.pack(fill=tk.X, expand=True)

        # Define a larger font for controls
        control_font = ("Arial", 14) # Increased font size
        temp_font = ("Arial", 18, "bold") # Larger font for temperature reading

        for i in range(1, ZONE_COUNT + 1):
            zone_id = f"zone{i}"
            zone_frame = tk.LabelFrame(barrel_frame, text=f"Zone {i}", padx=10, pady=10, font=("Arial", 12)) # Larger frame label
            zone_frame.grid(row=0, column=i-1, padx=5, pady=5, sticky="nsew")
            barrel_frame.grid_columnconfigure(i-1, weight=1)

            # Temperature Display
            tk.Label(zone_frame, text="Temp:").grid(row=0, column=0, sticky="w", pady=2)
            temp_label = tk.Label(zone_frame, textvariable=self.tk_current_temps[zone_id], font=temp_font) # Use larger temp font
            temp_label.grid(row=0, column=1, columnspan=2, sticky="e", pady=2)
            tk.Label(zone_frame, text="°C").grid(row=0, column=3, sticky="w", pady=2)

            # Heater State
            tk.Label(zone_frame, text="Heater:").grid(row=1, column=0, sticky="w", pady=2)
            # **FIX:** Store the label widget for reliable color updates
            state_label = tk.Label(zone_frame, textvariable=self.tk_heater_states[zone_id], font=control_font, fg="red")
            state_label.grid(row=1, column=1, columnspan=3, sticky="e", pady=2)
            self._heater_state_labels[zone_id] = state_label # Store the label

            # Setpoint Control
            tk.Label(zone_frame, text="Set (°C):").grid(row=2, column=0, sticky="w", pady=(10, 2))
            # **FIX:** Increase Entry width significantly
            entry = tk.Entry(zone_frame, textvariable=self.tk_target_setpoints[zone_id], width=10, font=control_font) # Increased width to 10
            entry.grid(row=2, column=1, columnspan=2, sticky="ew", pady=(10, 2)) # Use sticky="ew" to make it expand
            # **FIX:** Increase Button font size and add padding
            set_button = tk.Button(zone_frame, text="Set", font=control_font, padx=10, pady=3, # Added padding
                                   command=lambda zid=zone_id: self.publish_setpoint(zid))
            set_button.grid(row=2, column=3, sticky="ew", padx=(5,0), pady=(10, 2)) # Use sticky="ew"

            # Configure column weights within the zone frame for better resizing
            zone_frame.grid_columnconfigure(1, weight=1)
            zone_frame.grid_columnconfigure(2, weight=1)


        # Status Bar
        status_bar = tk.Label(self.master, textvariable=self.message_var, bd=1, relief=tk.SUNKEN, anchor=tk.W)
        status_bar.pack(side=tk.BOTTOM, fill=tk.X)

    # --- GUI Update Polling Method (Runs in GUI Thread) ---
    def check_for_updates(self):
        """Periodically checks shared data and updates Tkinter variables."""
        global latest_temps, latest_heater_states, data_lock

        with data_lock:
            for i in range(1, ZONE_COUNT + 1):
                zone_id = f"zone{i}"
                # Update Temperature
                temp = latest_temps[zone_id]
                if not isinstance(temp, float) or temp != temp: # Check for NaN
                     self.tk_current_temps[zone_id].set("--")
                else:
                    self.tk_current_temps[zone_id].set(f"{temp:.1f}")

                # Update Heater State
                state = latest_heater_states[zone_id]
                state_text = "ON" if state else "OFF"
                color = "green" if state else "red"
                self.tk_heater_states[zone_id].set(state_text)

                # **FIX:** Update color using the stored label reference
                if zone_id in self._heater_state_labels:
                    self._heater_state_labels[zone_id].config(fg=color)

        # Schedule the next check
        self.master.after(500, self.check_for_updates) # Check every 500ms

    # --- ROS Publishing Method (Called by GUI Thread) ---
    def publish_setpoint(self, zone_id):
        """Validates and publishes the setpoint for a specific zone."""
        if zone_id in self.setpoint_pubs:
            try:
                setpoint_value = self.tk_target_setpoints[zone_id].get()
                if not (MIN_SETPOINT <= setpoint_value <= MAX_SETPOINT):
                    self.message_var.set(f"Error: {zone_id} setpoint ({setpoint_value:.1f}) out of range ({MIN_SETPOINT}-{MAX_SETPOINT}).")
                    rospy.logwarn(f"GUI: Invalid setpoint {setpoint_value} for {zone_id}")
                    return
                msg = Float32(data=setpoint_value)
                self.setpoint_pubs[zone_id].publish(msg)
                self.message_var.set(f"{zone_id} setpoint updated to {setpoint_value:.1f}°C.")
                rospy.loginfo(f"GUI published setpoint for {zone_id}: {setpoint_value}")
            except tk.TclError:
                self.message_var.set(f"Error: Invalid numeric input for {zone_id}.")
                rospy.logwarn(f"GUI: Non-numeric input for {zone_id} setpoint.")
            except Exception as e:
                self.message_var.set(f"Error publishing {zone_id} setpoint: {e}")
                rospy.logerr(f"GUI Error publishing setpoint for {zone_id}: {e}")
        else:
            self.message_var.set(f"Error: Publisher for {zone_id} not ready.")
            rospy.logwarn(f"GUI: Setpoint Publisher for {zone_id} not initialized.")

    # --- ROS Callback Methods (Run in ROS Thread) ---
    def _update_temp_cb(self, temp_msg, zone_id):
        global latest_temps, data_lock
        with data_lock:
            latest_temps[zone_id] = temp_msg.data

    def _update_heater_state_cb(self, state_msg, zone_id):
        global latest_heater_states, data_lock
        with data_lock:
            latest_heater_states[zone_id] = state_msg.data

    # --- ROS Initialization ---
    def initialize_ros_comms(self):
        rospy.loginfo("GUI: Initializing ROS Publishers and Subscribers...")
        try:
            for i in range(1, ZONE_COUNT + 1):
                zone_id = f"zone{i}"
                pub_topic = f'/extruder/{zone_id}/setpoint'
                self.setpoint_pubs[zone_id] = rospy.Publisher(pub_topic, Float32, queue_size=1)
                rospy.loginfo(f"GUI: Advertising {pub_topic}")

                temp_topic = f'/extruder/{zone_id}/temperature'
                rospy.Subscriber(temp_topic, Float32, lambda msg, zid=zone_id: self._update_temp_cb(msg, zid))
                rospy.loginfo(f"GUI: Subscribing to {temp_topic}")

                state_topic = f'/extruder/{zone_id}/heater_state'
                rospy.Subscriber(state_topic, Bool, lambda msg, zid=zone_id: self._update_heater_state_cb(msg, zid))
                rospy.loginfo(f"GUI: Subscribing to {state_topic}")
            rospy.loginfo("GUI: ROS Comms Initialized.")
            return True
        except Exception as e:
            rospy.logerr(f"GUI: Failed to initialize ROS comms: {e}")
            messagebox.showerror("ROS Initialization Error", f"Failed to initialize ROS publishers/subscribers: {e}")
            return False


# --- SECTION: Main Execution ---

def ros_thread_loop(app_instance):
    """Handles ROS callbacks in a separate thread."""
    rospy.loginfo("ROS communication thread started.")
    # Initialize publishers/subscribers using the app instance
    if app_instance.initialize_ros_comms():
        rospy.spin() # Keeps the ROS thread alive and processing callbacks
    else:
        rospy.logerr("Exiting ROS thread due to initialization failure.")


def main():
    try:
        # Initialize ROS Node *before* creating GUI and starting threads
        # This MUST run in the main thread
        rospy.init_node('extruder_gui_node', anonymous=True)
        rospy.loginfo("GUI Node initialized in main thread.")

        # Create the Tkinter window and application instance
        root = tk.Tk()
        app = ExtruderGUI(root)

        # Start the ROS communication in a separate thread, passing the app instance
        ros_thread = Thread(target=ros_thread_loop, args=(app,))
        ros_thread.daemon = True # Allows the main program to exit if Tkinter closes
        ros_thread.start()

        # Start the Tkinter GUI loop (this blocks the main thread)
        rospy.loginfo("Starting Tkinter main loop...")
        root.mainloop()

        rospy.loginfo("Tkinter main loop finished.")

    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interrupt received, shutting down GUI.")
    except Exception as e:
        rospy.logfatal(f"Unhandled exception in main: {e}")
        # Optionally show an error popup
        try:
            root = tk.Tk()
            root.withdraw() # Hide the main window
            messagebox.showerror("Fatal Error", f"An unhandled error occurred: {e}")
        except Exception:
            pass # Ignore if Tkinter itself fails

if __name__ == '__main__':
    main()
