#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Bool
import tkinter as tk
from threading import Thread # Essential for running ROS and GUI simultaneously

class ExtruderGUI(tk.Frame):
    def __init__(self, master):
        super().__init__(master)
        self.master = master
        self.master.title("Frankenmolder Extruder Control")
        
        # ROS Communication Objects (Initialized in ros_thread)
        self.setpoint_pub = None 
        
        # --- Data Storage ---
        self.current_temp = tk.StringVar(value="--")
        self.heater_state = tk.StringVar(value="OFF")
        self.target_setpoint = tk.DoubleVar(value=200.0) # Default target

        self.create_widgets()

    def create_widgets(self):
        # --- Zone 1 Temperature Display ---
        tk.Label(self.master, text="Zone 1 Temperature:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        tk.Label(self.master, textvariable=self.current_temp, font=("Arial", 20)).grid(row=0, column=1, padx=5, pady=5, sticky="e")
        tk.Label(self.master, text="°C").grid(row=0, column=2, sticky="w")

        # --- Heater State Display ---
        tk.Label(self.master, text="Heater Status:").grid(row=1, column=0, padx=5, pady=5, sticky="w")
        tk.Label(self.master, textvariable=self.heater_state, fg="red").grid(row=1, column=1, padx=5, pady=5, sticky="e")

        # --- Setpoint Control ---
        tk.Label(self.master, text="New Setpoint (°C):").grid(row=2, column=0, padx=5, pady=5, sticky="w")
        
        entry = tk.Entry(self.master, textvariable=self.target_setpoint, width=8)
        entry.grid(row=2, column=1, padx=5, pady=5, sticky="e")
        
        tk.Button(self.master, text="Set", command=self.publish_setpoint).grid(row=2, column=2, padx=5, pady=5, sticky="w")

    # --- GUI Update Methods ---
    def update_temp(self, temp_msg):
        """Called by the ROS subscriber to update the GUI variable."""
        # Use try/except to catch any formatting errors
        try:
            self.current_temp.set(f"{temp_msg.data:.2f}")
        except Exception as e:
            rospy.logerr(f"GUI Update Error: {e}")

    def update_heater_state(self, state_msg):
        """Called by the ROS subscriber to update the GUI variable."""
        state = "ON" if state_msg.data else "OFF"
        color = "green" if state_msg.data else "red"
        self.heater_state.set(state)
        self.master.nametowidget(".!label2").config(fg=color) # Find the heater status label by path

    # --- ROS Publishing Method ---
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
        rospy.init_node('extruder_gui_node', anonymous=True)
        
        # 1. Initialize Publisher (Sends data to control node)
        self.setpoint_pub = rospy.Publisher('/extruder/zone1/setpoint', Float32, queue_size=1)
        
        # 2. Initialize Subscribers (Receives data from sensor/control nodes)
        rospy.Subscriber('/extruder/zone1/temperature', Float32, self.update_temp)
        rospy.Subscriber('/extruder/zone1/heater_state', Bool, self.update_heater_state)
        
        rospy.loginfo("ROS communication thread running.")
        rospy.spin() # Keeps the ROS thread alive

def main():
    root = tk.Tk()
    app = ExtruderGUI(root)
    
    # Start the ROS communication in a separate thread
    ros_thread = Thread(target=app.ros_thread_loop)
    ros_thread.daemon = True # Allows the main program to exit when Tkinter closes
    ros_thread.start()
    
    # Start the Tkinter GUI loop in the main thread
    root.mainloop()

if __name__ == '__main__':
    main()