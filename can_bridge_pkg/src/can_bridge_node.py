#!/usr/bin/env python3

"""
Receives CAN bus messages from the ESP32 controller (Temps, States),
unpacks the data, and publishes it to ROS topics for the GUI.
...
Also sends a 1Hz heartbeat to the ESP32 to prevent watchdog timeout.
"""

import rospy
import can
import struct # Used to unpack/pack bytes
import os
import sys
from std_msgs.msg import Float32, Int32, String
# from can_msgs.msg import Frame # Removed - using String for debug

# --- Configuration ---
CAN_INTERFACE = 'can0'
HEARTBEAT_RATE_HZ = 1.0 # Send heartbeat once per second

# --- CAN ID Protocol (Must match ESP32 code) ---

# --- ESP32 -> Pi (Status/Data) ---
# (float) Actual Temp
CAN_ID_STATUS_TEMP_1 = 0x101
CAN_ID_STATUS_TEMP_2 = 0x102
CAN_ID_STATUS_TEMP_3 = 0x103
CAN_ID_STATUS_TEMP_4 = 0x104
# (string) Actual State "OFF", "HEAT", "PID"
CAN_ID_STATUS_STATE_1 = 0x111
CAN_ID_STATUS_STATE_2 = 0x112
CAN_ID_STATUS_STATE_3 = 0x113
CAN_ID_STATUS_STATE_4 = 0x114
# (string) Motor Actual State "ON", "OFF"
CAN_ID_STATUS_MOTOR_STATE = 0x120
# CAN_ID_STATUS_MOTOR_RPM = 0x121

# --- Pi -> ESP32 (Commands) ---
# --- ADDED HEARTBEAT ID ---
CAN_ID_CMD_HEARTBEAT = 0x200 # (Empty frame) Pi is alive
# (float) Setpoint
CAN_ID_CMD_SETPOINT_1 = 0x201
CAN_ID_CMD_SETPOINT_2 = 0x202
CAN_ID_CMD_SETPOINT_3 = 0x203
CAN_ID_CMD_SETPOINT_4 = 0x204
# (string) "zoneX:STATE"
CAN_ID_CMD_STATE = 0x210
# (string) "ON", "OFF"
CAN_ID_CMD_MOTOR_STATE = 0x220
# (string) "RPMxx.x"
CAN_ID_CMD_MOTOR_RPM = 0x221

# --- AC servo test (binary, little-endian; see esp32_firmware/ac_servo.h) ---
CAN_ID_CMD_SERVO = 0x230         # [u8 cmd] 1=RUN 2=STOP 3=ESTOP 4=ZERO
CAN_ID_CMD_SERVO_SPEED = 0x231   # [f32 target_hz signed][f32 accel Hz/s]
CAN_ID_CMD_SERVO_MOVE = 0x232    # [i32 pulses signed][f32 hz]
CAN_ID_STATUS_SERVO = 0x130      # [u8 state][u8 flags][2 pad][f32 actual_hz signed]
CAN_ID_STATUS_SERVO_POS = 0x131  # [i32 position][i32 target_hz echo]
SERVO_CMD_CODES = {"RUN": 1, "STOP": 2, "ESTOP": 3, "ZERO": 4}
SERVO_STATES = {0: "IDLE", 1: "RUNNING", 2: "MOVING", 3: "STOPPING", 4: "FAULT"}
SERVO_FLAG_NAMES = ((0x01, "ALARM"), (0x02, "WATCHDOG"), (0x04, "AT_SPEED"), (0x08, "ALARM_INPUT"))


def pack_servo_speed(target_hz, accel):
    return list(struct.pack('<ff', float(target_hz), float(accel)))


def pack_servo_move(pulses, hz):
    return list(struct.pack('<if', int(pulses), float(hz)))


def unpack_servo_status(data):
    """-> (state_name, [flag names], actual_hz)"""
    state, flags, actual_hz = struct.unpack('<BB2xf', bytes(data[:8]))
    names = [name for bit, name in SERVO_FLAG_NAMES if flags & bit]
    return SERVO_STATES.get(state, f"UNKNOWN{state}"), names, actual_hz


def unpack_servo_pos(data):
    """-> (position_pulses, target_hz_echo)"""
    return struct.unpack('<ii', bytes(data[:8]))


class CANBridgeNode:
    def __init__(self):
        rospy.loginfo(f"Initializing CAN Bridge Node...")
        
        # --- Publishers (CAN -> ROS) ---
        self.temp_pubs = {
            CAN_ID_STATUS_TEMP_1: rospy.Publisher('/extruder/zone1/temperature', Float32, queue_size=10),
            CAN_ID_STATUS_TEMP_2: rospy.Publisher('/extruder/zone2/temperature', Float32, queue_size=10),
            CAN_ID_STATUS_TEMP_3: rospy.Publisher('/extruder/zone3/temperature', Float32, queue_size=10),
            CAN_ID_STATUS_TEMP_4: rospy.Publisher('/extruder/zone4/temperature', Float32, queue_size=10),
        }
        
        self.actual_state_pubs = {
            CAN_ID_STATUS_STATE_1: rospy.Publisher('/extruder/zone1/actual_state', String, queue_size=10, latch=True),
            CAN_ID_STATUS_STATE_2: rospy.Publisher('/extruder/zone2/actual_state', String, queue_size=10, latch=True),
            CAN_ID_STATUS_STATE_3: rospy.Publisher('/extruder/zone3/actual_state', String, queue_size=10, latch=True),
            CAN_ID_STATUS_STATE_4: rospy.Publisher('/extruder/zone4/actual_state', String, queue_size=10, latch=True),
        }

        self.error_state_pubs = {
            CAN_ID_STATUS_TEMP_1: rospy.Publisher('/extruder/zone1/error_state', String, queue_size=10, latch=True),
            CAN_ID_STATUS_TEMP_2: rospy.Publisher('/extruder/zone2/error_state', String, queue_size=10, latch=True),
            CAN_ID_STATUS_TEMP_3: rospy.Publisher('/extruder/zone3/error_state', String, queue_size=10, latch=True),
            CAN_ID_STATUS_TEMP_4: rospy.Publisher('/extruder/zone4/error_state', String, queue_size=10, latch=True),
        }
        
        self.motor_state_pub = rospy.Publisher('/extruder/motor/actual_state', String, queue_size=10, latch=True)
        # self.motor_rpm_pub = rospy.Publisher('/extruder/motor/actual_rpm', Float32, queue_size=10)
        self.raw_can_pub = rospy.Publisher('/can_bus_raw_string', String, queue_size=50)

        # AC servo test status (CAN -> ROS)
        self.servo_state_pub = rospy.Publisher('/servo/state', String, queue_size=10, latch=True)
        self.servo_flags_pub = rospy.Publisher('/servo/flags', String, queue_size=10, latch=True)
        self.servo_hz_pub = rospy.Publisher('/servo/actual_hz', Float32, queue_size=10)
        self.servo_pos_pub = rospy.Publisher('/servo/position', Int32, queue_size=10)
        self.servo_target_echo_pub = rospy.Publisher('/servo/target_echo_hz', Float32, queue_size=10)
        # Last speed settings; 0x231 always carries both
        self.servo_target_hz = 0.0
        self.servo_accel = 20000.0

        # Helper for logging
        self.zone_names = {
            CAN_ID_STATUS_TEMP_1: "Zone 1 Temp",
            CAN_ID_STATUS_TEMP_2: "Zone 2 Temp",
            CAN_ID_STATUS_TEMP_3: "Zone 3 Temp",
            CAN_ID_STATUS_TEMP_4: "Zone 4 Temp",
            CAN_ID_STATUS_STATE_1: "Zone 1 State",
            CAN_ID_STATUS_STATE_2: "Zone 2 State",
            CAN_ID_STATUS_STATE_3: "Zone 3 State",
            CAN_ID_STATUS_STATE_4: "Zone 4 State",
        }
        
        self.setpoint_can_ids = {
            "zone1": CAN_ID_CMD_SETPOINT_1,
            "zone2": CAN_ID_CMD_SETPOINT_2,
            "zone3": CAN_ID_CMD_SETPOINT_3,
            "zone4": CAN_ID_CMD_SETPOINT_4,
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
        rospy.Subscriber('/extruder/zone4/state_cmd', String, lambda msg: self.ros_heater_state_callback(msg, "zone4"))
        rospy.Subscriber('/extruder/zone1/setpoint', Float32, lambda msg: self.ros_heater_setpoint_callback(msg, "zone1"))
        rospy.Subscriber('/extruder/zone2/setpoint', Float32, lambda msg: self.ros_heater_setpoint_callback(msg, "zone2"))
        rospy.Subscriber('/extruder/zone3/setpoint', Float32, lambda msg: self.ros_heater_setpoint_callback(msg, "zone3"))
        rospy.Subscriber('/extruder/zone4/setpoint', Float32, lambda msg: self.ros_heater_setpoint_callback(msg, "zone4"))

        # AC servo test (ROS -> CAN)
        rospy.Subscriber('/servo/command', String, self.ros_servo_command_callback)
        rospy.Subscriber('/servo/target_hz', Float32, self.ros_servo_target_callback)
        rospy.Subscriber('/servo/accel', Float32, self.ros_servo_accel_callback)
        rospy.Subscriber('/servo/move_pulses', Int32, self.ros_servo_move_callback)

        # --- ADDED: Start the heartbeat timer ---
        self.heartbeat_timer = rospy.Timer(rospy.Duration(1.0 / HEARTBEAT_RATE_HZ), self.send_heartbeat_callback)
        
        rospy.loginfo("CAN Bridge Node initialized. Listening for ROS topics and CAN messages.")


    # --- ROS -> CAN Callback Methods ---
    
    def send_heartbeat_callback(self, event=None):
        """Called by rospy.Timer, sends a heartbeat to keep the ESP32 watchdog happy."""
        # This message is empty. Its only purpose is to be *received*
        # by the ESP32's `checkCanMessages()` function to reset the watchdog.
        try:
            msg = can.Message(arbitration_id=CAN_ID_CMD_HEARTBEAT,
                              dlc=0, # 0 bytes of data
                              is_extended_id=False)
            self.bus.send(msg)
            rospy.loginfo_throttle(60, f"CAN TX (ID 0x{CAN_ID_CMD_HEARTBEAT:X}): Sent Heartbeat.")
        except Exception as e:
            rospy.logerr(f"CAN TX (ID 0x{CAN_ID_CMD_HEARTBEAT:X}): Failed to send heartbeat: {e}")

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
        """Called when GUI sends /extruder/zoneX/state_cmd"""
        rospy.loginfo(f"ROS->CAN: Received {zone_id} state command: {msg.data}")
        
        try:
            zone_index = int(zone_id.replace('zone', ''))
        except Exception:
            rospy.logerr(f"Invalid zone_id '{zone_id}' in state callback.")
            return

        state_code = 0 # Default to OFF
        if msg.data == "ON":
            state_code = 1
        elif msg.data == "HEATING":  # Legacy support - map to ON
            state_code = 1
        elif msg.data == "PID":
            state_code = 2
        
        data_bytes = [zone_index, state_code]
        self.send_can_bytes(CAN_ID_CMD_STATE, data_bytes)

    def ros_heater_setpoint_callback(self, msg, zone_id):
        can_id = self.setpoint_can_ids.get(zone_id)
        if can_id is not None:
            # Ensure we extract the float value correctly from Float32 message
            setpoint_value = float(msg.data)
            rospy.loginfo(f"ROS->CAN: Received {zone_id} setpoint: {setpoint_value}, sending to ID 0x{can_id:X}")
            self.send_can_float(can_id, setpoint_value)
        else:
            rospy.logwarn(f"ROS->CAN: No CAN ID defined for setpoint on {zone_id}")

    # --- AC servo test (ROS -> CAN) ---
    def ros_servo_command_callback(self, msg):
        code = SERVO_CMD_CODES.get(msg.data.strip().upper())
        if code is None:
            rospy.logwarn(f"ROS->CAN: unknown servo command '{msg.data}' (use RUN/STOP/ESTOP/ZERO)")
            return
        rospy.loginfo(f"ROS->CAN: servo {msg.data}")
        self.send_can_bytes(CAN_ID_CMD_SERVO, [code])

    def _send_servo_speed(self):
        self.send_can_bytes(CAN_ID_CMD_SERVO_SPEED,
                            pack_servo_speed(self.servo_target_hz, self.servo_accel))

    def ros_servo_target_callback(self, msg):
        self.servo_target_hz = float(msg.data)
        self._send_servo_speed()

    def ros_servo_accel_callback(self, msg):
        self.servo_accel = float(msg.data)
        self._send_servo_speed()

    def ros_servo_move_callback(self, msg):
        hz = abs(self.servo_target_hz) or 1000.0
        rospy.loginfo(f"ROS->CAN: servo move {msg.data} pulses at {hz:.0f} Hz")
        self.send_can_bytes(CAN_ID_CMD_SERVO_MOVE, pack_servo_move(msg.data, hz))

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

    def send_can_float(self, can_id, value):
        if not self.bus: return
        try:
            # Pack as little-endian 32-bit float
            data_bytes = struct.pack('<f', float(value))
            # Debug: show raw bytes being sent
            byte_str = ' '.join([f'{b:02X}' for b in data_bytes])
            rospy.loginfo(f"CAN TX (ID 0x{can_id:X}): Packing float {value:.2f} as bytes: {byte_str}")
            
            msg = can.Message(arbitration_id=can_id,
                              data=data_bytes,
                              dlc=4,
                              is_extended_id=False)
            self.bus.send(msg)
            rospy.loginfo(f"CAN TX (ID 0x{can_id:X}): Sent float {value:.2f}")
        except Exception as e:
            rospy.logerr(f"CAN TX (ID 0x{can_id:X}): Failed to send float: {e}")
            rospy.logerr(f"  Value was: {value}, Type: {type(value)}")

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
                
                # --- Publish raw CAN message as a string ---
                try:
                    raw_string = str(message)
                    self.raw_can_pub.publish(String(raw_string))
                except Exception as e:
                    rospy.logerr(f"Failed to publish raw CAN string: {e}")
                
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
                
                # --- AC servo test status ---
                elif message.arbitration_id == CAN_ID_STATUS_SERVO and message.dlc == 8:
                    state, flags, actual_hz = unpack_servo_status(message.data)
                    self.servo_state_pub.publish(String(state))
                    self.servo_flags_pub.publish(String(",".join(flags)))
                    self.servo_hz_pub.publish(Float32(actual_hz))
                elif message.arbitration_id == CAN_ID_STATUS_SERVO_POS and message.dlc == 8:
                    position, target_echo = unpack_servo_pos(message.data)
                    self.servo_pos_pub.publish(Int32(position))
                    self.servo_target_echo_pub.publish(Float32(target_echo))

                # --- Handle Motor State Status Messages ---
                elif message.arbitration_id == CAN_ID_STATUS_MOTOR_STATE:
                    try:
                        state_str = message.data.decode('utf-8').strip('\x00')
                        rospy.loginfo(f"CAN->ROS (Motor State): {state_str}")
                        self.motor_state_pub.publish(String(state_str))
                    except Exception as e:
                        rospy.logwarn(f"CAN->ROS (Motor State): Error decoding string: {e}")

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