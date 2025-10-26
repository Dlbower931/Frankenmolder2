#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, String, Bool
import time

class ExtruderZoneControl:
    def __init__(self, zone_id):
        self.zone_id = zone_id
        self.node_name = f'extruder_{zone_id}_control_node'
        rospy.init_node(self.node_name, anonymous=True)
        rospy.loginfo(f"Starting {self.node_name}...")

        # --- Parameters ---
        self.hysteresis = rospy.get_param("~hysteresis", 2.0) # Degrees C band
        self.update_rate = rospy.get_param("~update_rate", 1.0) # Hz

        # --- State Variables ---
        self.current_state_cmd = "OFF" # Desired state from GUI
        self.current_setpoint = 0.0    # Desired temperature (for PID)
        self.current_temperature = float('nan') # Actual temperature from sensor
        self.heater_on = False         # Internal state for PID logic

        # --- Publishers ---
        # NOTE: We are NOT publishing /heater_cmd as per user request.
        # If needed later, uncomment the following line:
        # self.heater_cmd_pub = rospy.Publisher(f'/extruder/{zone_id}/heater_cmd', Bool, queue_size=10)

        # --- Subscribers ---
        rospy.Subscriber(f'/extruder/{zone_id}/state_cmd', String, self.state_cmd_callback)
        rospy.Subscriber(f'/extruder/{zone_id}/setpoint', Float32, self.setpoint_callback)
        rospy.Subscriber(f'/extruder/{zone_id}/temperature', Float32, self.temperature_callback)

        # --- Control Loop Timer ---
        self.control_timer = rospy.Timer(rospy.Duration(1.0 / self.update_rate), self.control_loop)

        rospy.loginfo(f"{self.node_name} initialized. Hysteresis: {self.hysteresis}, Rate: {self.update_rate} Hz")

    def state_cmd_callback(self, msg):
        """Update the desired operating state."""
        new_state = msg.data.upper() # Ensure consistent casing
        if new_state in ["OFF", "HEATING", "PID"]:
            if new_state != self.current_state_cmd:
                rospy.loginfo(f"{self.zone_id.capitalize()}: State command changed to {new_state}")
                self.current_state_cmd = new_state
                # Reset heater state when changing modes for safety
                self.heater_on = False
        else:
            rospy.logwarn(f"{self.zone_id.capitalize()}: Received invalid state command '{msg.data}'")

    def setpoint_callback(self, msg):
        """Update the target temperature for PID mode."""
        # Add validation if needed (e.g., check against MIN/MAX_SETPOINT)
        self.current_setpoint = msg.data
        # rospy.loginfo(f"{self.zone_id.capitalize()}: Setpoint updated to {self.current_setpoint:.1f} C") # Can be noisy

    def temperature_callback(self, msg):
        """Update the current measured temperature."""
        self.current_temperature = msg.data

    def control_loop(self, event):
        """Main logic executed periodically."""
        # Determine heater action based on the commanded state
        if self.current_state_cmd == "OFF":
            self.heater_on = False
            # rospy.loginfo_throttle(10, f"{self.zone_id.capitalize()}: State is OFF.")

        elif self.current_state_cmd == "HEATING":
            # Simple "HEATING" mode: Heater is always ON (full power)
            # Useful for initial heat-up. Be cautious with this mode.
            self.heater_on = True
            # rospy.loginfo_throttle(10, f"{self.zone_id.capitalize()}: State is HEATING (Heater ON).")

        elif self.current_state_cmd == "PID":
            # Simple ON/OFF control with hysteresis for "PID" mode
            if math.isnan(self.current_temperature):
                rospy.logwarn_throttle(5, f"{self.zone_id.capitalize()}: PID waiting for valid temperature reading.")
                self.heater_on = False # Safety: Turn off if temp is invalid
            elif self.current_temperature < (self.current_setpoint - self.hysteresis):
                # Below target band, turn heater ON
                if not self.heater_on:
                    rospy.loginfo(f"{self.zone_id.capitalize()} PID: Temp ({self.current_temperature:.1f}) < Target ({self.current_setpoint:.1f} - {self.hysteresis:.1f}). Turning Heater ON.")
                self.heater_on = True
            elif self.current_temperature > self.current_setpoint:
                # Above target, turn heater OFF
                # (Stays off between setpoint-hysteresis and setpoint)
                if self.heater_on:
                     rospy.loginfo(f"{self.zone_id.capitalize()} PID: Temp ({self.current_temperature:.1f}) > Target ({self.current_setpoint:.1f}). Turning Heater OFF.")
                self.heater_on = False
            # else: # Within hysteresis band, maintain current state
                # rospy.loginfo_throttle(10, f"{self.zone_id.capitalize()}: PID Temp within hysteresis band.")
                # pass

        # --- Publish Heater Command (If needed) ---
        # If your heater hardware needs a separate boolean command, uncomment this:
        # if self.heater_cmd_pub:
        #     self.heater_cmd_pub.publish(self.heater_on)

if __name__ == '__main__':
    import math # Needed for isnan
    try:
        # --- IMPORTANT: Change 'zone1' if creating nodes for other zones ---
        controller = ExtruderZoneControl('zone1')
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Control node shutting down.")
    except Exception as e:
        rospy.logerr(f"Control node crashed: {e}")