#!/usr/bin/env python3

"""
Receives CAN bus messages from the ESP32 controller (Temps, States),
unpacks the data, and publishes it to ROS topics for the GUI.
...
"""

import rospy
import can
import struct # Used to unpack/pack bytes
import os
import sys
from std_msgs.msg import Float32, String
# from can_msgs.msg import Frame # Removed - using String for debug

# --- Configuration ---
CAN_INTERFACE = 'can0'

# --- CAN ID Protocol (Must match ESP32 code) ---
# ... (CAN_ID_STATUS... definitions are the same) ...
CAN_ID_STATUS_TEMP_1 = 0x101
CAN_ID_STATUS_TEMP_2 = 0x102
CAN_ID_STATUS_TEMP_3 = 0x103
CAN_ID_STATUS_STATE_1 = 0x111
CAN_ID_STATUS_STATE_2 = 0x112
CAN_ID_STATUS_STATE_3 = 0x113

# --- Pi -> ESP32 (Commands) ---
CAN_ID_CMD_SETPOINT_1 = 0x201
CAN_ID_CMD_SETPOINT_2 = 0x202
CAN_ID_CMD_SETPOINT_3 = 0x203
# --- NEW PROTOCOL for Heater State ---
# Uses 2 bytes: [ZoneIndex (1-3), StateCode (0=OFF, 1=HEATING, 2=PID)]
CAN_ID_CMD_STATE = 0x210 
# ---------------------------------
CAN_ID_CMD_MOTOR_STATE = 0x220
CAN_ID_CMD_MOTOR_RPM = 0x221


class CANBridgeNode:
    def __init__(self):
        # ... (Same __init__... as before) ...
        # ... (Publishers, Subscribers, CAN Bus Setup) ...
# ... (File content redacted for brevity, assume __init__ is correct) ...
        rospy.loginfo(f"Initializing CAN Bridge Node...")
        
        # --- Publishers (CAN -> ROS) ---
        self.temp_pubs = {
            CAN_ID_STATUS_TEMP_1: rospy.Publisher('/extruder/zone1/temperature', Float32, queue_size=10),
            CAN_ID_STATUS_TEMP_2: rospy.Publisher('/extruder/zone2/temperature', Float32, queue_size=10),
            CAN_ID_STATUS_TEMP_3: rospy.Publisher('/extruder/zone3/temperature', Float32, queue_size=10),
        }
        
        self.actual_state_pubs = {
            CAN_ID_STATUS_STATE_1: rospy.Publisher('/extruder/zone1/actual_state', String, queue_size=10, latch=True),
            CAN_ID_STATUS_STATE_2: rospy.Publisher('/extruder/zone2/actual_state', String, queue_size=10, latch=True),
            CAN_ID_STATUS_STATE_3: rospy.Publisher('/extruder/zone3/actual_state', String, queue_size=10, latch=True),
        }

        self.error_state_pubs = {
            CAN_ID_STATUS_TEMP_1: rospy.Publisher('/extruder/zone1/error_state', String, queue_size=10, latch=True),
            CAN_ID_STATUS_TEMP_2: rospy.Publisher('/extruder/zone2/error_state', String, queue_size=10, latch=True),
            CAN_ID_STATUS_TEMP_3: rospy.Publisher('/extruder/zone3/error_state', String, queue_size=10, latch=True),
        }
        
        self.motor_state_pub = rospy.Publisher('/extruder/motor/actual_state', String, queue_size=10, latch=True)
        # self.motor_rpm_pub = rospy.Publisher('/extruder/motor/actual_rpm', Float32, queue_size=10)
        self.raw_can_pub = rospy.Publisher('/can_bus_raw_string', String, queue_size=50)

        # Helper for logging
        self.zone_names = {
            CAN_ID_STATUS_TEMP_1: "Zone 1 Temp",
            CAN_ID_STATUS_TEMP_2: "Zone 2 Temp",
            CAN_ID_STATUS_TEMP_3: "Zone 3 Temp",
            CAN_ID_STATUS_STATE_1: "Zone 1 State",
            CAN_ID_STATUS_STATE_2: "Zone 2 State",
            CAN_ID_STATUS_STATE_3: "Zone 3 State",
        }
        
        self.setpoint_can_ids = {
            "zone1": CAN_ID_CMD_SETPOINT_1,
            "zone2": CAN_ID_CMD_SETPOINT_2,
            "zone3": CAN_ID_CMD_SETPOINT_3,
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
        rospy.Subscriber('/extruder/motor/state_cmd', String, self.ros_motor_state_callback)
        rospy.Subscriber('/extruder/motor/set_rpm', Float32, self.ros_motor_rpm_callback)
        rospy.Subscriber('/extruder/zone1/state_cmd', String, lambda msg: self.ros_heater_state_callback(msg, "zone1"))
        rospy.Subscriber('/extruder/zone2/state_cmd', String, lambda msg: self.ros_heater_state_callback(msg, "zone2"))
        rospy.Subscriber('/extruder/zone3/state_cmd', String, lambda msg: self.ros_heater_state_callback(msg, "zone3"))
        rospy.Subscriber('/extruder/zone1/setpoint', Float32, lambda msg: self.ros_heater_setpoint_callback(msg, "zone1"))
        rospy.Subscriber('/extruder/zone2/setpoint', Float32, lambda msg: self.ros_heater_setpoint_callback(msg, "zone2"))
        rospy.Subscriber('/extruder/zone3/setpoint', Float32, lambda msg: self.ros_heater_setpoint_callback(msg, "zone3"))

        rospy.loginfo("CAN Bridge Node initialized. Listening for ROS topics and CAN messages.")


    # --- ROS -> CAN Callback Methods ---

    def ros_motor_state_callback(self, msg):
        # ... (This logic remains the same) ...
        command_str = msg.data # e.g., "ON" or "OFF"
        rospy.loginfo(f"ROS->CAN: Received motor state command: {command_str}")
        self.send_can_string(CAN_ID_CMD_MOTOR_STATE, command_str)

    def ros_motor_rpm_callback(self, msg):
        # ... (This logic remains the same) ...
        rpm_val = msg.data
        command_str = f"RPM{rpm_val:.1f}" 
        rospy.loginfo(f"ROS->CAN: Received motor RPM: {rpm_val}, Sending string: '{command_str}'")
        self.send_can_string(CAN_ID_CMD_MOTOR_RPM, command_str)

    def ros_heater_state_callback(self, msg, zone_id):
        """Called when GUI sends /extruder/zoneX/state_cmd"""
        # --- NEW PROTOCOL ---
        rospy.loginfo(f"ROS->CAN: Received {zone_id} state command: {msg.data}")
        
        # 1. Convert zone_id string "zone1" to integer 1
        try:
            zone_index = int(zone_id.replace('zone', '')) # 1, 2, or 3
        except Exception:
            rospy.logerr(f"Invalid zone_id '{zone_id}' in state callback.")
            return

        # 2. Convert state string to integer code
        state_code = 0 # Default to OFF
        if msg.data == "HEATING":
            state_code = 1
        elif msg.data == "PID":
            state_code = 2
        
        # 3. Pack into 2 bytes
        data_bytes = [zone_index, state_code]
        
        # 4. Send the 2-byte CAN message
        self.send_can_bytes(CAN_ID_CMD_STATE, data_bytes)

    def ros_heater_setpoint_callback(self, msg, zone_id):
        # ... (This logic remains the same) ...
        can_id = self.setpoint_can_ids.get(zone_id)
        if can_id is not None:
            rospy.loginfo(f"ROS->CAN: Received {zone_id} setpoint: {msg.data}, sending to ID 0x{can_id:X}")
            self.send_can_float(can_id, msg.data)
        else:
            rospy.logwarn(f"ROS->CAN: No CAN ID defined for setpoint on {zone_id}")

    # --- CAN Sending Methods ---
    
    def send_can_string(self, can_id, command_string):
        # ... (This function remains the same) ...
        if not self.bus: return
        data_bytes = command_string.encode('utf-8')
        if len(data_bytes) > 8:
            rospy.logwarn(f"CAN TX (ID 0x{can_id:X}): String '{command_string}' is > 8 bytes, truncating!")
            data_bytes = data_bytes[:8]
        try:
            msg = can.Message(arbitration_id=can_id,
                              data=data_bytes,
                              dlc=len(data_bytes),
                              is_extended_id=False)
            self.bus.send(msg)
            rospy.loginfo(f"CAN TX (ID 0x{can_id:X}): Sent string '{command_string}'")
        except Exception as e:
            rospy.logerr(f"CAN TX (ID 0x{can_id:X}): Failed to send command: {e}")

    def send_can_float(self, can_id, value):
        # ... (This function remains the same) ...
        if not self.bus: return
        try:
            data_bytes = struct.pack('<f', value)
            msg = can.Message(arbitration_id=can_id,
                              data=data_bytes,
                              dlc=4,
                              is_extended_id=False)
            self.bus.send(msg)
            rospy.loginfo(f"CAN TX (ID 0x{can_id:X}): Sent float {value:.2f}")
        except Exception as e:
            rospy.logerr(f"CAN TX (ID 0x{can_id:X}): Failed to send float: {e}")
            
    def send_can_bytes(self, can_id, data_bytes):
        """Sends a raw list/array of bytes over CAN."""
        if not self.bus: return
        
        try:
            msg = can.Message(arbitration_id=can_id,
                              data=data_bytes,
                              dlc=len(data_bytes),
                              is_extended_id=False)
            self.bus.send(msg)
            rospy.loginfo(f"CAN TX (ID 0x{can_id:X}): Sent bytes {data_bytes}")
        except Exception as e:
            rospy.logerr(f"CAN TX (ID 0x{can_id:X}): Failed to send bytes: {e}")

    # --- CAN -> ROS Listener Loop ---
            
    def run_listener(self):
        # ... (This function remains the same) ...
        if not self.bus:
            rospy.logerr("CAN bus not initialized. Shutting down listener.")
            return

        rospy.loginfo("CAN listener loop running... Waiting for messages.")
        
        try:
            for message in self.bus:
                if rospy.is_shutdown():
                    break
                
                # --- Publish raw CAN message as a string ---
                try:
                    raw_string = str(message)
                    self.raw_can_pub.publish(String(raw_string))
                except Exception as e:
                    rospy.logerr(f"Failed to publish raw CAN string: {e}")
                # ---------------------------------------------
                    
                # --- Handle Temperature Status Messages ---
                if message.arbitration_id in self.temp_pubs:
                    temp_publisher = self.temp_pubs[message.arbitration_id]
                    status_publisher = self.error_state_pubs[message.arbitration_id]
                    zone_name = self.zone_names.get(message.arbitration_id, "Unknown Zone")
                    
                    if message.dlc == 4:
                        temp = struct.unpack('<f', message.data)[0]
                        
                        if temp != temp: # Check for NaN
                            rospy.logwarn_throttle(10, f"CAN->ROS ({zone_name}): SENSOR FAULT (NaN)")
                            status_publisher.publish(String("FAULT"))
                        else:
                            temp_publisher.publish(Float32(temp))
                            status_publisher.publish(String("OK"))
                            if message.arbitration_id == CAN_ID_STATUS_TEMP_1:
                                rospy.loginfo_throttle(5, f"CAN->ROS ({zone_name}): {temp:.2f} °C (OK)")
                    else:
                        rospy.logwarn(f"CAN->ROS ({zone_name}): Invalid temp data length: {message.dlc} bytes (expected 4)")
                
                # --- Handle Actual State Status Messages ---
                elif message.arbitration_id in self.actual_state_pubs:
                    state_publisher = self.actual_state_pubs[message.arbitration_id]
                    zone_name = self.zone_names.get(message.arbitration_id, "Unknown State")
                    try:
                        state_str = message.data.decode('utf-8').strip('\x00')
                        rospy.loginfo(f"CAN->ROS ({zone_name}): {state_str}")
                        state_publisher.publish(String(state_str))
                    except Exception as e:
                        rospy.logwarn(f"CAN->ROS ({zone_name}): Error decoding string: {e}")
                
                # (Handle motor status, etc. here)

        except rospy.ROSInterruptException:
            pass
        except Exception as e:
            rospy.logerr(f"Error in CAN listener loop: {e}")
        finally:
            if self.bus:
                self.bus.shutdown()
            rospy.loginfo("CAN bus shut down.")

if __name__ == "__main__":
    try:
        rospy.init_node('can_bridge_node', anonymous=True)
        bridge = CANBridgeNode()
        bridge.run_listener()
    except rospy.ROSInterruptException:
        rospy.loginfo("CAN Bridge Node shutting down.")
    except Exception as e:
        rospy.logfatal(f"CAN Bridge Node crashed: {e}")
