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
        # Publishers need to be initialized *after* rospy.init_node
        self.setpoint_pubs = {} # Dictionary to hold publishers for each zone

        # --- Tkinter Variables (GUI Thread Owned) ---
        self.tk_current_temps = {}
        self.tk_heater_states = {}
        self.tk_target_setpoints = {}
        self.message_var = tk.StringVar(value="System Initialized.")

        for i in range(1, ZONE_COUNT + 1):
            zone_id = f"zone{i}"
            self.tk_current_temps[zone_id] = tk.StringVar(value="--")
            self.tk_heater_states[zone_id] = tk.StringVar(value="OFF")
            self.tk_target_setpoints[zone_id] = tk.DoubleVar(value=200.0) # Default target

        self.create_widgets()
        self.master.after(500, self.check_for_updates) # Start the polling loop

    def create_widgets(self):
        """Creates and arranges the Tkinter widgets."""
        main_frame = tk.Frame(self.master, bd=2, relief=tk.GROOVE)
        main_frame.pack(padx=10, pady=10, fill=tk.X)
        tk.Label(main_frame, text="Extruder Barrel Zones", font=("Arial", 14, "bold")).pack()

        barrel_frame = tk.Frame(main_frame)
        barrel_frame.pack(fill=tk.X, expand=True)

        # Create a control panel for each zone horizontally
        for i in range(1, ZONE_COUNT + 1):
            zone_id = f"zone{i}"
            zone_frame = tk.LabelFrame(barrel_frame, text=f"Zone {i}", padx=10, pady=10)
            # Use grid within barrel_frame for horizontal alignment
            zone_frame.grid(row=0, column=i-1, padx=5, pady=5, sticky="nsew")
            barrel_frame.grid_columnconfigure(i-1, weight=1) # Make columns expand equally

            # --- Temperature Display ---
            tk.Label(zone_frame, text="Temp:").grid(row=0, column=0, sticky="w")
            temp_label = tk.Label(zone_frame, textvariable=self.tk_current_temps[zone_id], font=("Arial", 16))
            temp_label.grid(row=0, column=1, sticky="e")
            tk.Label(zone_frame, text="°C").grid(row=0, column=2, sticky="w")

            # --- Heater State ---
            tk.Label(zone_frame, text="Heater:").grid(row=1, column=0, sticky="w")
            state_label = tk.Label(zone_frame, textvariable=self.tk_heater_states[zone_id], fg="red")
            state_label.grid(row=1, column=1, columnspan=2, sticky="e")
            state_label.widget_name = f"heater_state_label_{zone_id}" # Give it a name to find later

            # --- Setpoint Control ---
            tk.Label(zone_frame, text="Set (°C):").grid(row=2, column=0, sticky="w")
            entry = tk.Entry(zone_frame, textvariable=self.tk_target_setpoints[zone_id], width=7)
            entry.grid(row=2, column=1, sticky="e")
            # Use lambda to pass the zone_id to the publish function
            set_button = tk.Button(zone_frame, text="Set",
                                   command=lambda zid=zone_id: self.publish_setpoint(zid))
            set_button.grid(row=2, column=2, sticky="w", padx=(5,0))

        # --- Status Bar ---
        status_bar = tk.Label(self.master, textvariable=self.message_var, bd=1, relief=tk.SUNKEN, anchor=tk.W)
        status_bar.pack(side=tk.BOTTOM, fill=tk.X)

    # --- GUI Update Polling Method (Runs in GUI Thread) ---
    def check_for_updates(self):
        """Periodically checks shared data and updates Tkinter variables."""
        global latest_temps, latest_heater_states, data_lock
        # rospy.loginfo("G_POLL: Checking for updates...") # DEBUG

        with data_lock:
            for i in range(1, ZONE_COUNT + 1):
                zone_id = f"zone{i}"
                # Update Temperature
                temp = latest_temps[zone_id]
                if not isinstance(temp, float) or temp != temp: # Check for NaN
                     self.tk_current_temps[zone_id].set("--")
                else:
                    self.tk_current_temps[zone_id].set(f"{temp:.1f}")
                    # rospy.loginfo(f"G_POLL: Updated Tkinter temp var for {zone_id} to {temp:.1f}") # DEBUG

                # Update Heater State
                state = latest_heater_states[zone_id]
                state_text = "ON" if state else "OFF"
                color = "green" if state else "red"
                self.tk_heater_states[zone_id].set(state_text)
                try:
                    # Find the specific label widget to update its color
                    state_label_widget = self.master.nametowidget(f".!frame.!frame.!labelframe{i}.!label4") # Adjust path if layout changes!
                    if state_label_widget:
                        state_label_widget.config(fg=color)
                except KeyError:
                    rospy.logwarn(f"Could not find heater state label for {zone_id} to update color.")


        # Schedule the next check
        self.master.after(500, self.check_for_updates) # Check every 500ms

    # --- ROS Publishing Method (Called by GUI Thread) ---
    def publish_setpoint(self, zone_id):
        """Validates and publishes the setpoint for a specific zone."""
        if zone_id in self.setpoint_pubs:
            try:
                setpoint_value = self.tk_target_setpoints[zone_id].get()

                # --- Validation ---
                if not (MIN_SETPOINT <= setpoint_value <= MAX_SETPOINT):
                    self.message_var.set(f"Error: {zone_id} setpoint ({setpoint_value:.1f}) out of range ({MIN_SETPOINT}-{MAX_SETPOINT}).")
                    rospy.logwarn(f"GUI: Invalid setpoint {setpoint_value} for {zone_id}")
                    return # Stop processing

                # If valid, publish
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
    # These methods MUST NOT directly modify Tkinter variables/widgets
    def _update_temp_cb(self, temp_msg, zone_id):
        """Internal callback to store latest temp data safely."""
        global latest_temps, data_lock
        # rospy.loginfo(f"R_TEMP_CB: Received temp for {zone_id}: {temp_msg.data}") # DEBUG
        with data_lock:
            latest_temps[zone_id] = temp_msg.data

    def _update_heater_state_cb(self, state_msg, zone_id):
        """Internal callback to store latest heater state data safely."""
        global latest_heater_states, data_lock
        # rospy.loginfo(f"R_STATE_CB: Received state for {zone_id}: {state_msg.data}") # DEBUG
        with data_lock:
            latest_heater_states[zone_id] = state_msg.data

    # --- ROS Initialization (Called once by main thread before starting ROS thread) ---
    def initialize_ros_comms(self):
        """Initializes ROS publishers and subscribers."""
        rospy.loginfo("GUI: Initializing ROS Publishers and Subscribers...")
        try:
            for i in range(1, ZONE_COUNT + 1):
                zone_id = f"zone{i}"

                # Initialize Publisher for setpoints
                pub_topic = f'/extruder/{zone_id}/setpoint'
                self.setpoint_pubs[zone_id] = rospy.Publisher(pub_topic, Float32, queue_size=1)
                rospy.loginfo(f"GUI: Advertising {pub_topic}")

                # Initialize Subscriber for temperature
                temp_topic = f'/extruder/{zone_id}/temperature'
                # Use lambda to pass the zone_id to the callback wrapper
                rospy.Subscriber(temp_topic, Float32,
                                 lambda msg, zid=zone_id: self._update_temp_cb(msg, zid))
                rospy.loginfo(f"GUI: Subscribing to {temp_topic}")


                # Initialize Subscriber for heater state
                state_topic = f'/extruder/{zone_id}/heater_state'
                rospy.Subscriber(state_topic, Bool,
                                 lambda msg, zid=zone_id: self._update_heater_state_cb(msg, zid))
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
