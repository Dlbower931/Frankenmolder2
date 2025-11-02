#!/usr/bin/env python3

"""
Receives CAN bus messages from the ESP32 controller (Temps),
unpacks the data, and publishes it to ROS topics for the GUI.

Also, subscribes to ROS command topics (Motor, Heaters) from the GUI
and sends the corresponding commands over the CAN bus to the ESP32.
"""

import rospy
import can
import struct # Used to unpack/pack bytes
import os
import sys
from std_msgs.msg import Float32, String

# --- Configuration ---
CAN_INTERFACE = 'can0'

# --- CAN IDs for Receiving Data (ESP32 -> Pi) ---
# These MUST match the definitions in your ESP32 code
CAN_ID_STATUS_TEMP_1 = 0x101 # (float) Actual Temp Zone 1
CAN_ID_STATUS_TEMP_2 = 0x102 # (float) Actual Temp Zone 2
CAN_ID_STATUS_TEMP_3 = 0x103 # (float) Actual Temp Zone 3
# Add future status IDs here
# CAN_ID_STATUS_MOTOR_STATE = 0x110
# CAN_ID_STATUS_MOTOR_RPM = 0x111

# --- CAN IDs for Sending Commands (Pi -> ESP32) ---
# These MUST match what your ESP32 code will listen for
# CAN_ID_CMD_MOTOR_STATE = 0x201
# ... (add other command IDs here when ready)


class CANBridgeNode:
    def __init__(self):
        rospy.loginfo(f"Initializing CAN Bridge Node...")
        
        # --- Publishers (CAN -> ROS) ---
        # These publish data received from the CAN bus to the ROS system
        self.temp_pubs = {
            CAN_ID_STATUS_TEMP_1: rospy.Publisher('/extruder/zone1/temperature', Float32, queue_size=10),
            CAN_ID_STATUS_TEMP_2: rospy.Publisher('/extruder/zone2/temperature', Float32, queue_size=10),
            CAN_ID_STATUS_TEMP_3: rospy.Publisher('/extruder/zone3/temperature', Float32, queue_size=10),
        }
        
        # Add other publishers here when ready
        # self.motor_state_pub = rospy.Publisher('/extruder/motor/actual_state', String, queue_size=10, latch=True)

        # Helper for logging
        self.zone_names = {
            CAN_ID_STATUS_TEMP_1: "Zone 1 Temp",
            CAN_ID_STATUS_TEMP_2: "Zone 2 Temp",
            CAN_ID_STATUS_TEMP_3: "Zone 3 Temp",
        }

        # --- CAN Bus Setup ---
        self.bus = None
        try:
            if CAN_INTERFACE not in os.listdir('/sys/class/net/'):
                rospy.logfatal(f"CRITICAL Error: CAN interface '{CAN_INTERFACE}' not found in container.")
                rospy.logfatal(f"Ensure '{CAN_INTERFACE}' is UP on the Pi HOST: sudo ip link set {CAN_INTERFACE} up type can bitrate 500000")
                sys.exit(1)

            self.bus = can.interface.Bus(channel=CAN_INTERFACE, bustype='socketcan', receive_own_messages=False)
            rospy.loginfo(f"Successfully connected to CAN bus '{CAN_INTERFACE}'.")
        except Exception as e:
            rospy.logfatal(f"Fatal error initializing CAN bus: {e}")
            sys.exit(1)
            
        # --- Subscribers (ROS -> CAN) ---
        # (Add subscribers here when you're ready to send commands from GUI to CAN)
        # rospy.Subscriber('/extruder/motor/state_cmd', String, self.ros_motor_state_callback)
        # ...

        rospy.loginfo("CAN Bridge Node initialized. Listening for CAN messages.")

    # --- ROS -> CAN Callback Methods ---
    # (Add callback functions here)
    # def ros_motor_state_callback(self, msg):
    #     ...
            
    # --- CAN Sending Methods ---
    # (Add send_can_string, send_can_float here)
    # ...

    # --- CAN -> ROS Listener Loop ---
            
    def run_listener(self):
        """Main loop: receives CAN messages and publishes to ROS."""
        if not self.bus:
            rospy.logerr("CAN bus not initialized. Shutting down listener.")
            return

        rospy.loginfo("CAN listener loop running... Waiting for messages.")
        
        try:
            for message in self.bus: # This loop blocks until a message is received
                if rospy.is_shutdown():
                    break
                    
                # --- Handle Temperature Status Messages ---
                if message.arbitration_id in self.temp_pubs:
                    # Look up the publisher based on the CAN ID
                    publisher = self.temp_pubs[message.arbitration_id]
                    zone_name = self.zone_names.get(message.arbitration_id, "Unknown Zone")
                    
                    if message.dlc == 4:
                        # Unpack the 4 bytes into a 32-bit float ('<f')
                        temp = struct.unpack('<f', message.data)[0]
                        
                        if temp != temp: # Check for NaN (Not a Number)
                            rospy.logwarn_throttle(10, f"CAN->ROS ({zone_name}): SENSOR FAULT (NaN)")
                        else:
                            # Publish the valid temperature to the correct ROS topic
                            publisher.publish(Float32(temp))
                            # Log only one zone to reduce spam, or use logdebug
                            if message.arbitration_id == CAN_ID_STATUS_TEMP_1:
                                rospy.loginfo_throttle(5, f"CAN->ROS ({zone_name}): {temp:.2f} °C (Zone 2/3 data also processed)")
                    else:
                        rospy.logwarn(f"CAN->ROS ({zone_name}): Invalid temp data length: {message.dlc} bytes (expected 4)")
                
                # (Add elif blocks here for other CAN IDs, like motor status)
                
        except rospy.ROSInterruptException:
            pass # ROS is shutting down
        except Exception as e:
            rospy.logerr(f"Error in CAN listener loop: {e}")
        finally:
            self.bus.shutdown()
            rospy.loginfo("CAN bus shut down.")

if __name__ == "__main__":
    try:
        rospy.init_node('can_bridge_node', anonymous=True)
        bridge = CANBridgeNode()
        bridge.run_listener() # Run the listener loop
    except rospy.ROSInterruptException:
        rospy.loginfo("CAN Bridge Node shutting down.")
    except Exception as e:
        rospy.logfatal(f"CAN Bridge Node crashed: {e}")
