#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float32
import serial
import time
import sys

# --- Configuration ---
# USB port the Pico is connected to.
# This MUST be mapped in docker-compose.yml
PICO_PORT = '/dev/ttyACM0' 
BAUD_RATE = 115200

# Global serial object
ser = None

def state_cmd_callback(msg):
    """Receives motor state command ('ON' or 'OFF') and sends to Pico."""
    global ser
    if ser is None:
        rospy.logwarn_throttle(5, "Pico serial port not available.")
        return
        
    command = msg.data.upper() # "ON" or "OFF"
    
    try:
        rospy.loginfo(f"Sending command to Pico: {command}")
        ser.write(f"{command}\n".encode('utf-8')) # Send with newline
    except Exception as e:
        rospy.logerr(f"Failed to send command to Pico: {e}")

def set_rpm_callback(msg):
    """Receives RPM command and sends to Pico (Example for later)."""
    global ser
    if ser is None:
        rospy.logwarn_throttle(5, "Pico serial port not available.")
        return
    
    rpm = msg.data
    command = f"RPM {rpm}\n" # Example of a different command
    
    try:
        rospy.loginfo(f"Sending command to Pico: {command.strip()}")
        ser.write(command.encode('utf-8'))
    except Exception as e:
        rospy.logerr(f"Failed to send command to Pico: {e}")

def pico_bridge_node():
    global ser
    rospy.init_node('pico_bridge_node', anonymous=True)
    rospy.loginfo("Starting Pico Bridge Node...")

    # --- Initialize Serial Port ---
    try:
        ser = serial.Serial(PICO_PORT, BAUD_RATE, timeout=1)
        # Wait a moment for Pico to reset after serial connection
        time.sleep(2) 
        rospy.loginfo(f"Successfully connected to Pico on {PICO_PORT}")
    except serial.SerialException as e:
        rospy.logerr(f"Failed to open serial port {PICO_PORT}: {e}")
        rospy.logerr("Make sure Pico is plugged in and /dev/ttyACM0 is mapped in docker-compose.yml")
        # We don't exit, just set ser to None so callbacks fail gracefully
        ser = None
    except Exception as e:
        rospy.logerr(f"An unknown error occurred opening serial port: {e}")
        ser = None
        
    # --- Subscribers ---
    # Subscribes to the GUI's motor state command
    rospy.Subscriber('/extruder/motor/state_cmd', String, state_cmd_callback)
    
    # Subscribes to the GUI's RPM setpoint
    rospy.Subscriber('/extruder/motor/set_rpm', Float32, set_rpm_callback)
    
    rospy.loginfo("Pico Bridge Node ready. Listening for topics...")
    rospy.spin()

if __name__ == '__main__':
    try:
        pico_bridge_node()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Pico Bridge Node crashed: {e}")
    finally:
        if ser is not None and ser.is_open:
            ser.close()
            rospy.loginfo("Pico serial port closed.")
