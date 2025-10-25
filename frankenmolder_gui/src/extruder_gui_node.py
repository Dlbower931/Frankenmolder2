#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Bool
import tkinter as tk
from threading import Thread
import threading

class ExtruderGUI(tk.Frame):
    POLL_INTERVAL = 500

    def __init__(self, master):
        super().__init__(master)
        self.master = master
        self.master.title("Frankenmolder Extruder Control")
        
        self.setpoint_pub = None 
        
        # --- THREAD-SAFE DATA STORAGE ---
        self.latest_temp_data = None
        self.latest_heater_state = None
        self.data_lock = threading.Lock() 

        # --- TKINTER VARIABLES ---
        self.current_temp = tk.StringVar(value="--")
        self.heater_state = tk.StringVar(value="OFF")
        self.target_setpoint = tk.DoubleVar(value=200.0)

        self.create_widgets()
        
        # Start the GUI polling loop
        self.master.after(self.POLL_INTERVAL, self.check_for_updates)

    def setup_ros_comms(self):
        """Initializes ROS publishers and subscribers. Called from main thread."""
        
        # 1. Initialize Publisher
        self.setpoint_pub = rospy.Publisher('/extruder/zone1/setpoint', Float32, queue_size=1)
        
        # 2. Initialize Subscribers
        rospy.Subscriber('/extruder/zone1/temperature', Float32, self.update_temp)
        rospy.Subscriber('/extruder/zone1/heater_state', Bool, self.update_heater_state)
        
        rospy.loginfo("ROS communication publishers/subscribers set up successfully.")

    def create_widgets(self):
        self.master.columnconfigure(1, weight=1)
        
        # --- Zone 1 Temperature Display ---
        tk.Label(self.master, text="Zone 1 Temperature:").grid(row=0, column=0, padx=10, pady=5, sticky="w")
        tk.Label(self.master, textvariable=self.current_temp, font=("Arial", 24, "bold"), anchor="e").grid(row=0, column=1, padx=10, pady=5, sticky="e")
        tk.Label(self.master, text="°C", font=("Arial", 14)).grid(row=0, column=2, sticky="w")

        # --- Heater State Display ---
        tk.Label(self.master, text="Heater Status:").grid(row=1, column=0, padx=10, pady=5, sticky="w")
        self.heater_label = tk.Label(self.master, textvariable=self.heater_state, font=("Arial", 14), fg="red")
        self.heater_label.grid(row=1, column=1, padx=10, pady=5, sticky="e")

        # --- Setpoint Control ---
        tk.Label(self.master, text="New Setpoint (°C):").grid(row=2, column=0, padx=10, pady=10, sticky="w")
        
        entry = tk.Entry(self.master, textvariable=self.target_setpoint, width=8, font=("Arial", 12))
        entry.grid(row=2, column=1, padx=10, pady=10, sticky="e")
        
        tk.Button(self.master, text="Set Setpoint", command=self.publish_setpoint, bg="#4CAF50", fg="white", activebackground="#45a049").grid(row=2, column=2, padx=10, pady=10, sticky="w")

    # --- ROS CALLBACKS (Run in ROS Thread) ---
    def update_temp(self, temp_msg):
        """CALLED BY ROS. Stores temperature data safely."""
        rospy.loginfo("R_TEMP_CB: Data received. Attempting to acquire lock.")
        with self.data_lock:
            self.latest_temp_data = temp_msg.data
            rospy.loginfo(f"R_TEMP_CB: Successfully stored temp: {self.latest_temp_data:.2f}")

    def update_heater_state(self, state_msg):
        """CALLED BY ROS. Stores heater state safely."""
        with self.data_lock:
            self.latest_heater_state = state_msg.data
            rospy.loginfo(f"R_HEATER_CB: Stored state: {self.latest_heater_state}")
    
    # --- GUI POLLING FUNCTION (Run in Main Tkinter Thread) ---
    def check_for_updates(self):
        """Pulls data from storage and updates Tkinter variables."""
        
        # Check if the ROS thread has stored any data yet
        if self.latest_temp_data is None:
            rospy.loginfo("G_POLL: Data storage empty. Skipping update.")
            self.master.after(self.POLL_INTERVAL, self.check_for_updates)
            return

        temp_data = None
        heater_state_data = None
        
        # Safely retrieve data from the shared storage
        rospy.loginfo("G_POLL: Data available. Attempting to acquire lock for read.")
        with self.data_lock:
            temp_data = self.latest_temp_data
            heater_state_data = self.latest_heater_state
        rospy.loginfo(f"G_POLL: Lock released. Read temp: {temp_data:.2f}")

        # 1. Update Temperature Display
        try:
            self.current_temp.set(f"{temp_data:.2f}")
            rospy.loginfo("G_POLL: Successfully updated Tkinter temp variable.")
        except Exception as e:
            rospy.logerr(f"G_POLL: Tkinter update error on temp: {e}") 

        # 2. Update Heater State Display
        if heater_state_data is not None:
            state = "ON" if heater_state_data else "OFF"
            color = "green" if heater_state_data else "red"
            self.heater_state.set(state)
            self.heater_label.config(fg=color)

        # Reschedule this function to run again
        self.master.after(self.POLL_INTERVAL, self.check_for_updates)


    # --- ROS Publishing Method ---
    def publish_setpoint(self):
        if self.setpoint_pub:
            setpoint_value = self.target_setpoint.get()
            msg = Float32(setpoint_value)
            self.setpoint_pub.publish(msg)
            rospy.loginfo(f"GUI published setpoint: {setpoint_value}")
        else:
            rospy.logwarn("Setpoint Publisher not initialized.")

    # --- ROS Listener Loop (Run in ROS Thread) ---
    def ros_thread_loop(self):
        """This function runs in the separate thread to run the ROS listener."""
        try:
            rospy.loginfo("ROS thread listening for data (rospy.spin()).")
            # This must be called in the secondary thread to keep listeners active
            rospy.spin() 
        except rospy.ROSInterruptException:
            pass

def main():
    # --- STEP 1: Initialize ROS in the Main Thread (CRITICAL FIX) ---
    # This prevents the "signal only works in main thread" error
    try:
        rospy.init_node('extruder_gui_node', anonymous=True)
    except rospy.exceptions.ROSInitException as e:
        # This handles the case where ROS is already initialized, though unlikely here
        print(f"ROS Initialization failed: {e}")
        return

    root = tk.Tk()
    
    def on_closing():
        rospy.signal_shutdown("GUI closed by user")
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)

    app = ExtruderGUI(root)
    
    # --- STEP 2: Setup Pub/Sub after init_node ---
    app.setup_ros_comms()

    # --- STEP 3: Start the ROS listener in a secondary thread ---
    ros_thread = Thread(target=app.ros_thread_loop)
    ros_thread.daemon = True 
    ros_thread.start()
    
    # --- STEP 4: Start the Tkinter GUI loop in the main thread ---
    root.mainloop()

if __name__ == '__main__':
    main()
