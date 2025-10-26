#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Bool, String # Import String
import time

# --- Configuration ---
HYSTERESIS = 2.0 # Temperature band (e.g., +/- 2.0 C around setpoint for PID)

# --- Global State Variables ---
# These store the latest known values
current_setpoint = 0.0
current_temp = float('nan') # Initialize as NaN
# State commanded by the GUI ("OFF", "HEATING") - PID is implicit
commanded_state = "OFF"
# Actual state managed by this node ("OFF", "HEATING", "PID")
actual_state = "OFF"
# Output state for the heater mechanism
heater_output_state = False # True = ON, False = OFF

# --- Publishers ---
# Publish the actual state for the GUI to display
actual_state_pub = None
# Publish the command for the physical heater mechanism
heater_cmd_pub = None

# --- ROS Callbacks ---
def setpoint_callback(msg):
    """Subscribes to the setpoint published by the GUI."""
    global current_setpoint
    # Basic validation (optional, GUI should handle range)
    if msg.data >= 0: # Ensure non-negative
        current_setpoint = msg.data
        rospy.loginfo(f"Control Node: Received new setpoint: {current_setpoint:.1f} C")
    else:
        rospy.logwarn(f"Control Node: Received invalid setpoint: {msg.data}")

def temp_callback(msg):
    """Subscribes to the actual temperature published by the sensor node."""
    global current_temp
    current_temp = msg.data
    # rospy.logdebug(f"Control Node: Received temp: {current_temp:.1f} C") # Too verbose for normal operation

def state_cmd_callback(msg):
    """Subscribes to the state command published by the GUI ('OFF', 'HEATING')."""
    global commanded_state, actual_state
    new_cmd_state = msg.data
    if new_cmd_state in ["OFF", "HEATING"]:
        if commanded_state != new_cmd_state:
            rospy.loginfo(f"Control Node: Received state command: {new_cmd_state}")
            commanded_state = new_cmd_state
            # If commanded OFF, immediately set actual state to OFF
            if commanded_state == "OFF":
                actual_state = "OFF"
                rospy.loginfo("Control Node: State changed to OFF by command.")
            # If commanded HEATING, start the heating process
            elif commanded_state == "HEATING":
                actual_state = "HEATING" # Start heating up
                rospy.loginfo("Control Node: State changed to HEATING by command.")
    else:
        rospy.logwarn(f"Control Node: Received invalid state command: {new_cmd_state}")

# --- Control Logic ---
def control_loop():
    global current_temp, current_setpoint, commanded_state, actual_state, heater_output_state

    # Don't do anything if temp is invalid or setpoint not set
    if math.isnan(current_temp) or current_setpoint <= 0:
        actual_state = "OFF" # Safety: turn off if sensor fails or no setpoint
        heater_output_state = False
        return # Exit loop early

    # --- State Machine Logic ---
    if actual_state == "OFF":
        heater_output_state = False
        # Stay OFF unless commanded to HEAT
        if commanded_state == "HEATING":
             actual_state = "HEATING" # Transition to HEATING based on command
             rospy.loginfo("Control Node: Transitioning OFF -> HEATING")

    elif actual_state == "HEATING":
        heater_output_state = True # Full power heating
        # Check if commanded OFF
        if commanded_state == "OFF":
            actual_state = "OFF"
            rospy.loginfo("Control Node: Transitioning HEATING -> OFF")
        # Check if temp has reached the PID band lower threshold
        elif current_temp >= (current_setpoint - HYSTERESIS):
            actual_state = "PID" # Transition to PID control
            rospy.loginfo(f"Control Node: Temp {current_temp:.1f}C reached PID band. Transitioning HEATING -> PID")

    elif actual_state == "PID":
        # Check if commanded OFF
        if commanded_state == "OFF":
            actual_state = "OFF"
            rospy.loginfo("Control Node: Transitioning PID -> OFF")
        else:
            # Simple PID logic (On/Off within hysteresis band)
            # Turn ON if below setpoint, OFF if at or above
            if current_temp < current_setpoint:
                heater_output_state = True
            else:
                heater_output_state = False
            # Optional: More sophisticated PID calculation here

    # --- Publish Outputs ---
    heater_cmd_pub.publish(heater_output_state)
    actual_state_pub.publish(actual_state) # Let GUI know the true state


def extruder_zone1_control():
    global actual_state_pub, heater_cmd_pub

    rospy.init_node('extruder_zone1_control_node', anonymous=True)

    # --- Publishers ---
    # Publishes the ACTUAL state back to the GUI/other nodes
    actual_state_pub = rospy.Publisher('/extruder/zone1/actual_state', String, queue_size=10, latch=True)
    # Publishes the ON/OFF command to the heater mechanism/relay node
    heater_cmd_pub = rospy.Publisher('/extruder/zone1/heater_cmd', Bool, queue_size=10, latch=True)

    # --- Subscribers ---
    rospy.Subscriber('/extruder/zone1/setpoint', Float32, setpoint_callback)
    rospy.Subscriber('/extruder/zone1/temperature', Float32, temp_callback)
    rospy.Subscriber('/extruder/zone1/state_cmd', String, state_cmd_callback)

    rate = rospy.Rate(1) # Run the control loop once per second

    rospy.loginfo("Extruder Zone 1 Control Node Started.")
    # Initialize state publications
    actual_state_pub.publish(actual_state)
    heater_cmd_pub.publish(heater_output_state)


    while not rospy.is_shutdown():
        control_loop()
        rate.sleep()

if __name__ == '__main__':
    import math # Needed for isnan check
    try:
        extruder_zone1_control()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logfatal(f"Control Node Crashed: {e}")