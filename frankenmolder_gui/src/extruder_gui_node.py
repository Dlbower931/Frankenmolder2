#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Bool
import tkinter as tk
from threading import Thread # Essential for running ROS and GUI simultaneously
import threading # Used for the Lock

class ExtruderGUI(tk.Frame):
    # --- Polling interval (milliseconds) for GUI updates ---
    POLL_INTERVAL = 500

    def __init__(self, master):
        super().__init__(master)
        self.master = master
        self.master.title("Frankenmolder Extruder Control")
        
        # ROS Communication Objects
        self.setpoint_pub = None 
        
        # --- THREAD-SAFE DATA STORAGE (Standard Python variables) ---
        self.latest_temp_data = None
        self.latest_heater_state = None
        self.data_lock = threading.Lock() # Lock to protect the data storage

        # --- TKINTER VARIABLES (Only updated in the main thread) ---
        self.current_temp = tk.StringVar(value="--")
        self.heater_state = tk.StringVar(value="OFF")
        self.target_setpoint = tk.DoubleVar(value=200.0) # Default target

        self.create_widgets()
        
        # Start the thread-safe poller immediately
        self.master.after(self.POLL_INTERVAL, self.check_for_updates)


    def create_widgets(self):
        # Configure layout for a cleaner look
        self.master.columnconfigure(1, weight=1)
        
        # --- Zone 1 Temperature Display ---
        tk.Label(self.master, text="Zone 1 Temperature:").grid(row=0, column=0, padx=10, pady=5, sticky="w")
        tk.Label(self.master, textvariable=self.current_temp, font=("Arial", 24, "bold"), anchor="e").grid(row=0, column=1, padx=10, pady=5, sticky="e")
        tk.Label(self.master, text="°C", font=("Arial", 14)).grid(row=0, column=2, sticky="w")

        # --- Heater State Display ---
        tk.Label(self.master, text="Heater Status:").grid(row=1, column=0, padx=10, pady=5, sticky="w")
        # Get a reference to the label widget for later configuration (color update)
        self.heater_label = tk.Label(self.master, textvariable=self.heater_state, font=("Arial", 14), fg="red")
        self.heater_label.grid(row=1, column=1, padx=10, pady=5, sticky="e")

        # --- Setpoint Control ---
        tk.Label(self.master, text="New Setpoint (°C):").grid(row=2, column=0, padx=10, pady=10, sticky="w")
        
        entry = tk.Entry(self.master, textvariable=self.target_setpoint, width=8, font=("Arial", 12))
        entry.grid(row=2, column=1, padx=10, pady=10, sticky="e")
        
        tk.Button(self.master, text="Set Setpoint", command=self.publish_setpoint, bg="#4CAF50", fg="white", activebackground="#45a049").grid(row=2, column=2, padx=10, pady=10, sticky="w")

    # --- ROS CALLBACKS (Run in ROS Thread - ONLY write to standard variables) ---
    def update_temp(self, temp_msg):
        """Called by the ROS subscriber to store the temperature data."""
        with self.data_lock:
            self.latest_temp_data = temp_msg.data

    def update_heater_state(self, state_msg):
        """Called by the ROS subscriber to store the heater state."""
        with self.data_lock:
            self.latest_heater_state = state_msg.data
    
    # --- GUI POLLING FUNCTION (Run in Main Thread via after()) ---
    def check_for_updates(self):
        """
        Runs periodically on the main Tkinter thread to safely update the GUI
        based on data received by the ROS thread.
        """
        temp_data = None
        heater_state_data = None
        
        # Safely retrieve data from the shared storage
        with self.data_lock:
            temp_data = self.latest_temp_data
            heater_state_data = self.latest_heater_state
            
        # 1. Update Temperature Display
        if temp_data is not None:
            try:
                self.current_temp.set(f"{temp_data:.2f}")
            except Exception as e:
                 # Print error to ROS log, not Tkinter console
                rospy.logerr(f"Tkinter update error: {e}") 

        # 2. Update Heater State Display
        if heater_state_data is not None:
            state = "ON" if heater_state_data else "OFF"
            color = "green" if heater_state_data else "red"
            self.heater_state.set(state)
            self.heater_label.config(fg=color) # Use the stored label reference

        # Reschedule this function to run again
        self.master.after(self.POLL_INTERVAL, self.check_for_updates)


    # --- ROS Publishing Method (Run in Main Thread when button is clicked) ---
    def publish_setpoint(self):
        if self.setpoint_pub:
            setpoint_value = self.target_setpoint.get()
            msg = Float32(setpoint_value)
            self.setpoint_pub.publish(msg)
            rospy.loginfo(f"GUI published setpoint: {setpoint_value}")
        else:
            rospy.logwarn("Setpoint Publisher not initialized.")

    # --- ROS Initialization/Loop ---
    def ros_thread_loop(self):
        """This function runs in a separate thread to handle all ROS communication."""
        # Use try-except to catch cases where the GUI thread might be shutting down
        try:
            rospy.init_node('extruder_gui_node', anonymous=True)
            
            # 1. Initialize Publisher
            self.setpoint_pub = rospy.Publisher('/extruder/zone1/setpoint', Float32, queue_size=1)
            
            # 2. Initialize Subscribers
            rospy.Subscriber('/extruder/zone1/temperature', Float32, self.update_temp)
            rospy.Subscriber('/extruder/zone1/heater_state', Bool, self.update_heater_state)
            
            rospy.loginfo("ROS communication thread running.")
            rospy.spin() # Keeps the ROS thread alive
        except rospy.ROSInterruptException:
            pass # Clean shutdown

def main():
    root = tk.Tk()
    # Handle the window closing event to shut down ROS cleanly
    def on_closing():
        rospy.signal_shutdown("GUI closed by user")
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)

    app = ExtruderGUI(root)
    
    # Start the ROS communication in a separate thread
    ros_thread = Thread(target=app.ros_thread_loop)
    ros_thread.daemon = True 
    ros_thread.start()
    
    # Start the Tkinter GUI loop in the main thread
    root.mainloop()

if __name__ == '__main__':
    main()