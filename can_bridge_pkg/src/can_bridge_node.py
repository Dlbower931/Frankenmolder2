#!/usr/bin/env python3

"""
Receives CAN bus messages from the ESP32 controller (Temps),
unpacks the data, and publishes it to ROS topics for the GUI.
Also publishes raw CAN frames to /can_bus_raw for debugging.

Also, subscribes to ROS command topics (Motor, Heaters) from the GUI
and sends the corresponding commands over the CAN bus to the ESP32.
"""

import rospy
import can
import struct # Used to unpack/pack bytes
import os
import sys
from std_msgs.msg import Float32, String
from can_msgs.msg import Frame # <-- Added import for raw CAN frames

# --- Configuration ---
CAN_INTERFACE = 'can0'

# --- CAN IDs for Receiving Data (ESP32 -> Pi) ---
CAN_ID_STATUS_TEMP_1 = 0x101 # (float) Actual Temp Zone 1
CAN_ID_STATUS_TEMP_2 = 0x102 # (float) Actual Temp Zone 2
CAN_ID_STATUS_TEMP_3 = 0x103 # (float) Actual Temp Zone 3
CAN_ID_STATUS_MOTOR_STATE = 0x110 # (String) "ON" or "OFF"
CAN_ID_STATUS_MOTOR_RPM = 0x111 # (float) Actual RPM

# --- CAN IDs for Sending Commands (Pi -> ESP32) ---
CAN_ID_CMD_MOTOR_STATE = 0x201  # (String) "ON" or "OFF"
CAN_ID_CMD_MOTOR_RPM = 0x202  # (String) "RPM10.0"
# Define separate IDs for each zone's setpoint command
CAN_ID_CMD_SETPOINT_1 = 0x211 # (float) Target Temp Zone 1
CAN_ID_CMD_SETPOINT_2 = 0x212 # (float) Target Temp Zone 2
CAN_ID_CMD_SETPOINT_3 = 0x213 # (float) Target Temp Zone 3
CAN_ID_CMD_HEATER_STATE = 0x210 # (String) "zone1:HEATING" (Shared ID)


class CANBridgeNode:
    def __init__(self):
        rospy.loginfo(f"Initializing CAN Bridge Node...")
        
        # --- Publishers (CAN -> ROS) ---
        self.temp_pubs = {
            CAN_ID_STATUS_TEMP_1: rospy.Publisher('/extruder/zone1/temperature', Float32, queue_size=10),
            CAN_ID_STATUS_TEMP_2: rospy.Publisher('/extruder/zone2/temperature', Float32, queue_size=10),
            CAN_ID_STATUS_TEMP_3: rospy.Publisher('/extruder/zone3/temperature', Float32, queue_size=10),
        }
        
        self.status_pubs = {
            CAN_ID_STATUS_TEMP_1: rospy.Publisher('/extruder/zone1/error_state', String, queue_size=10, latch=True),
            CAN_ID_STATUS_TEMP_2: rospy.Publisher('/extruder/zone2/error_state', String, queue_size=10, latch=True),
            CAN_ID_STATUS_TEMP_3: rospy.Publisher('/extruder/zone3/error_state', String, queue_size=10, latch=True),
        }
        
        self.motor_state_pub = rospy.Publisher('/extruder/motor/actual_state', String, queue_size=10, latch=True)
        self.motor_rpm_pub = rospy.Publisher('/extruder/motor/actual_rpm', Float32, queue_size=10)

        # --- Added: Publisher for raw CAN frames ---
        self.raw_can_pub = rospy.Publisher('/can_bus_raw', Frame, queue_size=50)

        # Helper for logging
        self.zone_names = {
            CAN_ID_STATUS_TEMP_1: "Zone 1 Temp",
            CAN_ID_STATUS_TEMP_2: "Zone 2 Temp",
            CAN_ID_STATUS_TEMP_3: "Zone 3 Temp",
            CAN_ID_STATUS_MOTOR_STATE: "Motor State",
            CAN_ID_STATUS_MOTOR_RPM: "Motor RPM",
        }
        
        # Map ROS topic setpoints to CAN IDs
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
        
        # Heater Commands
        rospy.Subscriber('/extruder/zone1/state_cmd', String, lambda msg: self.ros_heater_state_callback(msg, "zone1"))
        rospy.Subscriber('/extruder/zone2/state_cmd', String, lambda msg: self.ros_heater_state_callback(msg, "zone2"))
        rospy.Subscriber('/extruder/zone3/state_cmd', String, lambda msg: self.ros_heater_state_callback(msg, "zone3"))
        
        rospy.Subscriber('/extruder/zone1/setpoint', Float32, lambda msg: self.ros_heater_setpoint_callback(msg, "zone1"))
        rospy.Subscriber('/extruder/zone2/setpoint', Float32, lambda msg: self.ros_heater_setpoint_callback(msg, "zone2"))
        rospy.Subscriber('/extruder/zone3/setpoint', Float32, lambda msg: self.ros_heater_setpoint_callback(msg, "zone3"))

        rospy.loginfo("CAN Bridge Node initialized. Listening for ROS topics and CAN messages.")

    # --- ROS -> CAN Callback Methods ---

    def ros_motor_state_callback(self, msg):
        command_str = msg.data
        rospy.loginfo(f"ROS->CAN: Received motor state command: {command_str}")
        self.send_can_string(CAN_ID_CMD_MOTOR_STATE, command_str)

    def ros_motor_rpm_callback(self, msg):
        rpm_val = msg.data
        command_str = f"RPM{rpm_val:.1f}" 
        rospy.loginfo(f"ROS->CAN: Received motor RPM: {rpm_val}, Sending string: '{command_str}'")
        self.send_can_string(CAN_ID_CMD_MOTOR_RPM, command_str)

    def ros_heater_state_callback(self, msg, zone_id):
        command_str = f"{zone_id}:{msg.data}"
        rospy.loginfo(f"ROS->CAN: Received {zone_id} state command: {msg.data}")
        self.send_can_string(CAN_ID_CMD_HEATER_STATE, command_str)

    def ros_heater_setpoint_callback(self, msg, zone_id):
        can_id = self.setpoint_can_ids.get(zone_id)
        if can_id is not None:
            rospy.loginfo(f"ROS->CAN: Received {zone_id} setpoint: {msg.data}, sending to ID 0x{can_id:X}")
            self.send_can_float(can_id, msg.data)
        else:
            rospy.logwarn(f"ROS->CAN: No CAN ID defined for setpoint on {zone_id}")

    # --- CAN Sending Methods ---
    
    def send_can_string(self, can_id, command_string):
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

    # --- CAN -> ROS Listener Loop ---
            
    def run_listener(self):
        if not self.bus:
            rospy.logerr("CAN bus not initialized. Shutting down listener.")
            return

        rospy.loginfo("CAN listener loop running... Waiting for messages.")
        
        try:
            for message in self.bus:
                if rospy.is_shutdown():
                    break
                
                # --- Added: Publish raw CAN message for Foxglove ---
                try:
                    ros_can_msg = Frame()
                    ros_can_msg.header.stamp = rospy.Time.now()
                    ros_can_msg.id = message.arbitration_id
                    ros_can_msg.dlc = message.dlc
                    ros_can_msg.is_extended = message.is_extended_id
                    # --- FIX: Map python-can's 'is_remote_frame' to ROS's 'is_rtr' ---
                    ros_can_msg.is_rtr = message.is_remote_frame
                    # -----------------------------------------------------------
                    ros_can_msg.is_error = message.is_error
                    ros_can_msg.data = list(message.data) 
                    self.raw_can_pub.publish(ros_can_msg)
                except Exception as e:
                    rospy.logerr(f"Failed to publish raw CAN frame: {e}")
                # ---------------------------------------------
                    
                # --- Handle Temperature Status Messages ---
                if message.arbitration_id in self.temp_pubs:
                    temp_publisher = self.temp_pubs[message.arbitration_id]
                    status_publisher = self.status_pubs[message.arbitration_id]
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
                
                # --- Handle Motor Status Messages (Example) ---
                elif message.arbitration_id == CAN_ID_STATUS_MOTOR_STATE:
                    try:
                        state_str = message.data.decode('utf-8').strip('\x00')
                        rospy.loginfo(f"CAN->ROS (Motor State): {state_str}")
                        self.motor_state_pub.publish(String(state_str))
                    except Exception as e:
                        rospy.logwarn(f"CAN->ROS (Motor State): Error decoding string: {e}")

                elif message.arbitration_id == CAN_ID_STATUS_MOTOR_RPM:
                    if message.dlc == 4:
                        rpm = struct.unpack('<f', message.data)[0]
                        rospy.loginfo(f"CAN->ROS (Motor RPM): {rpm:.1f}")
                        self.motor_rpm_pub.publish(Float32(rpm))
                    else:
                        rospy.logwarn(f"CAN->ROS (Motor RPM): Invalid data length: {message.dlc} bytes (expected 4)")

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
