#!/usr/bin/env python3

import rospy
import can
import struct # Used to unpack/pack bytes
import os
import sys
from std_msgs.msg import Float32, String

# --- Configuration ---
# This must match the interface name on the Pi host (e.g., 'can0')
CAN_INTERFACE = 'can0'

# --- CAN IDs for Receiving Data (ESP32 -> Pi) ---
# These MUST match the definitions in your ESP32 code
CAN_ID_STATUS_TEMP_1 = 0x101 # (float) Actual Temp Zone 1
CAN_ID_STATUS_TEMP_2 = 0x102 # (float) Actual Temp Zone 2
CAN_ID_STATUS_TEMP_3 = 0x103 # (float) Actual Temp Zone 3
# Add future status IDs here (e.g., motor actual RPM, motor state)
CAN_ID_STATUS_MOTOR_STATE = 0x110 # (string) e.g., "ON", "OFF"
CAN_ID_STATUS_MOTOR_RPM = 0x111 # (float) Actual RPM

# --- CAN IDs for Sending Commands (Pi -> ESP32) ---
# These MUST match what your ESP32 code will listen for
CAN_ID_CMD_MOTOR_STATE = 0x201  # (string) "ON" or "OFF"
CAN_ID_CMD_MOTOR_RPM = 0x202  # (float) Target RPM
CAN_ID_CMD_HEATER_STATE = 0x210 # (string) "OFF", "HEATING", "PID" (Example)
CAN_ID_CMD_HEATER_SETPOINT = 0x211 # (float) Target Temp (Example)


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
        
        # Example publishers for motor status (if ESP32 sends it back)
        self.motor_state_pub = rospy.Publisher('/extruder/motor/actual_state', String, queue_size=10, latch=True)
        self.motor_rpm_pub = rospy.Publisher('/extruder/motor/actual_rpm', Float32, queue_size=10)

        # Helper for logging
        self.zone_names = {
            CAN_ID_STATUS_TEMP_1: "Zone 1 Temp",
            CAN_ID_STATUS_TEMP_2: "Zone 2 Temp",
            CAN_ID_STATUS_TEMP_3: "Zone 3 Temp",
            CAN_ID_STATUS_MOTOR_STATE: "Motor State",
            CAN_ID_STATUS_MOTOR_RPM: "Motor RPM",
        }

        # --- CAN Bus Setup ---
        self.bus = None
        try:
            # Verify the can0 interface exists in the container's /sys/class/net/
            if CAN_INTERFACE not in os.listdir('/sys/class/net/'):
                rospy.logfatal(f"CRITICAL Error: CAN interface '{CAN_INTERFACE}' not found in container.")
                rospy.logfatal("Ensure 'network_mode: host' is in docker-compose.yml")
                rospy.logfatal(f"Ensure '{CAN_INTERFACE}' is UP on the Pi HOST: sudo ip link set {CAN_INTERFACE} up type can bitrate 500000")
                sys.exit(1) # Exit if bus is not found

            # Use 'socketcan' for native Linux CAN
            self.bus = can.interface.Bus(channel=CAN_INTERFACE, bustype='socketcan', receive_own_messages=False)
            rospy.loginfo(f"Successfully connected to CAN bus '{CAN_INTERFACE}'.")
        except Exception as e:
            rospy.logfatal(f"Fatal error initializing CAN bus: {e}")
            sys.exit(1) # Node cannot function without CAN bus
            
        # --- Subscribers (ROS -> CAN) ---
        # These listen for commands from the GUI to send *to* the CAN bus
        
        # Motor Commands
        rospy.Subscriber('/extruder/motor/state_cmd', String, self.ros_motor_state_callback)
        rospy.Subscriber('/extruder/motor/set_rpm', Float32, self.ros_motor_rpm_callback)
        
        # Heater Commands
        rospy.Subscriber('/extruder/zone1/state_cmd', String, self.ros_heater_state_callback, "zone1")
        rospy.Subscriber('/extruder/zone2/state_cmd', String, self.ros_heater_state_callback, "zone2")
        rospy.Subscriber('/extruder/zone3/state_cmd', String, self.ros_heater_state_callback, "zone3")
        
        rospy.Subscriber('/extruder/zone1/setpoint', Float32, self.ros_heater_setpoint_callback, "zone1")
        rospy.Subscriber('/extruder/zone2/setpoint', Float32, self.ros_heater_setpoint_callback, "zone2")
        rospy.Subscriber('/extruder/zone3/setpoint', Float32, self.ros_heater_setpoint_callback, "zone3")

        rospy.loginfo("CAN Bridge Node initialized. Listening for ROS topics and CAN messages.")

    # --- ROS -> CAN Callback Methods ---

    def ros_motor_state_callback(self, msg):
        """Called when GUI sends /extruder/motor/state_cmd"""
        command_str = msg.data # e.g., "ON" or "OFF"
        rospy.loginfo(f"ROS->CAN: Received motor state command: {command_str}")
        self.send_can_string(CAN_ID_CMD_MOTOR_STATE, command_str)

    def ros_motor_rpm_callback(self, msg):
        """Called when GUI sends /extruder/motor/set_rpm"""
        rpm_val = msg.data
        # Format as "RPM10.0" to match what Pico expects
        command_str = f"RPM{rpm_val:.1f}" 
        rospy.loginfo(f"ROS->CAN: Received motor RPM: {rpm_val}, Sending string: '{command_str}'")
        self.send_can_string(CAN_ID_CMD_MOTOR_RPM, command_str)

    def ros_heater_state_callback(self, msg, zone_id):
        """Called when GUI sends /extruder/zoneX/state_cmd"""
        # This function will need a more complex CAN frame definition
        # For now, just log it.
        rospy.loginfo(f"ROS->CAN: Received {zone_id} state command: {msg.data} (Sending logic not implemented yet)")
        # Example: self.send_can_string(CAN_ID_CMD_HEATER_STATE, f"{zone_id}:{msg.data}")

    def ros_heater_setpoint_callback(self, msg, zone_id):
        """Called when GUI sends /extruder/zoneX/setpoint"""
        rospy.loginfo(f"ROS->CAN: Received {zone_id} setpoint: {msg.data} (Sending logic not implemented yet)")
        # Example: self.send_can_float(CAN_ID_CMD_HEATER_SETPOINT, msg.data)

    # --- CAN Sending Methods ---
    
    def send_can_string(self, can_id, command_string):
        """Sends a simple string command over the CAN bus (max 8 bytes)."""
        if not self.bus: return

        # Encode the string as bytes, truncate to 8 bytes if longer
        data_bytes = command_string.encode('utf-8')
        if len(data_bytes) > 8:
            rospy.logwarn(f"CAN TX (ID 0x{can_id:X}): String '{command_string}' is > 8 bytes, truncating!")
            data_bytes = data_bytes[:8]
        
        try:
            msg = can.Message(arbitration_id=can_id,
                              data=data_bytes,
                              dlc=len(data_bytes), # Set DLC to actual length
                              is_extended_id=False)
            self.bus.send(msg)
            rospy.loginfo(f"CAN TX (ID 0x{can_id:X}): Sent string '{command_string}'")
        except Exception as e:
            rospy.logerr(f"CAN TX (ID 0x{can_id:X}): Failed to send command: {e}")

    def send_can_float(self, can_id, value):
        """Packs a float into 4 bytes and sends it over CAN."""
        if not self.bus: return
        
        try:
            # Pack the float into 4 bytes, little-endian
            data_bytes = struct.pack('<f', value)
            
            msg = can.Message(arbitration_id=can_id,
                              data=data_bytes,
                              dlc=4, # A float is 4 bytes
                              is_extended_id=False)
            self.bus.send(msg)
            rospy.loginfo(f"CAN TX (ID 0x{can_id:X}): Sent float {value:.2f}")
        except Exception as e:
            rospy.logerr(f"CAN TX (ID 0x{can_id:X}): Failed to send float: {e}")

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
                    zone_name = self.zone_names.get(message.arbitration_id, "Unknown Zone")
                    
                    if message.dlc == 4:
                        # Unpack the 4 bytes into a 32-bit float ('<f')
                        temp = struct.unpack('<f', message.data)[0]
                        
                        if temp != temp: # Check for NaN (Not a Number)
                            rospy.logwarn_throttle(10, f"CAN->ROS ({zone_name}): SENSOR FAULT (NaN)")
                        else:
                            # Publish the valid temperature to the correct ROS topic
                            self.temp_pubs[message.arbitration_id].publish(Float32(temp))
                            rospy.loginfo_throttle(5, f"CAN->ROS ({zone_name}): {temp:.2f} °C")
                    else:
                        rospy.logwarn(f"CAN->ROS ({zone_name}): Invalid temp data length: {message.dlc} bytes (expected 4)")
                
                # --- Handle Motor Status Messages (Example) ---
                elif message.arbitration_id == CAN_ID_STATUS_MOTOR_STATE:
                    state_str = message.data.decode('utf-8').strip()
                    rospy.loginfo(f"CAN->ROS (Motor State): {state_str}")
                    self.motor_state_pub.publish(String(state_str))

                elif message.arbitration_id == CAN_ID_STATUS_MOTOR_RPM:
                    if message.dlc == 4:
                        rpm = struct.unpack('<f', message.data)[0]
                        rospy.loginfo(f"CAN->ROS (Motor RPM): {rpm:.1f}")
                        self.motor_rpm_pub.publish(Float32(rpm))
                    else:
                        rospy.logwarn(f"CAN->ROS (Motor RPM): Invalid data length: {message.dlc} bytes (expected 4)")
                
                # else:
                #     rospy.logdebug(f"CAN RX: Ignoring ID 0x{message.arbitration_id:X}")

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