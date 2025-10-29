#!/usr/bin/env python3

import rospy
import serial
import serial.tools.list_ports
import threading
import time
import json
from std_msgs.msg import String, Float32, Bool

class PicoSerialComm:
    def __init__(self):
        rospy.init_node('pico_serial_node', anonymous=True)
        
        # Serial configuration
        self.serial_port = None
        self.baud_rate = 115200  # High speed for injection molding
        self.connection_timeout = 5.0
        self.reconnect_interval = 2.0
        self.is_connected = False
        
        # Threading
        self.serial_thread = None
        self.running = True
        self.lock = threading.Lock()
        
        # Command queue for high-frequency sending
        self.command_queue = []
        self.queue_lock = threading.Lock()
        
        # ROS Publishers
        self.status_pub = rospy.Publisher('/pico/status', String, queue_size=10)
        self.connection_pub = rospy.Publisher('/pico/connected', Bool, queue_size=1)
        self.actual_rpm_pub = rospy.Publisher('/extruder/actual_rpm', Float32, queue_size=10)

        # Minimal ROS API
        self.run_sub = rospy.Subscriber('/extruder/run', Bool, self.run_callback)
        self.rpm_sub = rospy.Subscriber('/extruder/target_rpm', Float32, self.rpm_callback)
        
        # Extruder control state
        self.target_rpm = 0.0
        self.is_running = False
        
        rospy.loginfo("Pico Serial Communication Node initialized")
        
        # Start connection attempt
        self.connect_to_pico()
        
        # Start command processing thread
        self.start_command_processor()

    def find_pico_port(self):
        """Find the Pi Pico 2W USB serial port"""
        rospy.loginfo("Scanning for Pi Pico 2W...")
        
        # Common Pi Pico identifiers
        pico_identifiers = [
            "USB Serial Device",
            "Pico",
            "Raspberry Pi Pico",
            "MicroPython"
        ]
        
        ports = serial.tools.list_ports.comports()
        for port in ports:
            rospy.loginfo(f"Found device: {port.description} on {port.device}")
            for identifier in pico_identifiers:
                if identifier.lower() in port.description.lower():
                    rospy.loginfo(f"Pi Pico 2W found on {port.device}")
                    return port.device
        
        # Fallback: try common USB serial ports
        common_ports = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1']
        for port in common_ports:
            try:
                test_serial = serial.Serial(port, self.baud_rate, timeout=1)
                test_serial.close()
                rospy.loginfo(f"Using fallback port: {port}")
                return port
            except:
                continue
                
        return None

    def connect_to_pico(self):
        """Establish serial connection to Pi Pico 2W"""
        port = self.find_pico_port()
        if not port:
            rospy.logerr("Pi Pico 2W not found! Check USB connection.")
            self.publish_connection_status(False)
            return False
            
        try:
            self.serial_port = serial.Serial(
                port=port,
                baudrate=self.baud_rate,
                timeout=self.connection_timeout,
                write_timeout=1.0
            )
            
            # Wait for connection to stabilize
            time.sleep(2)
            
            # Send handshake
            self.send_command("HANDSHAKE", {"node": "raspberry_pi"})
            
            self.is_connected = True
            self.publish_connection_status(True)
            rospy.loginfo(f"Connected to Pi Pico 2W on {port}")
            
            # Start serial reading thread
            self.serial_thread = threading.Thread(target=self.serial_reader)
            self.serial_thread.daemon = True
            self.serial_thread.start()
            
            return True
            
        except Exception as e:
            rospy.logerr(f"Failed to connect to Pi Pico 2W: {e}")
            self.publish_connection_status(False)
            return False

    def serial_reader(self):
        """Background thread to read from Pi Pico"""
        while self.running and self.is_connected:
            try:
                if self.serial_port and self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode('utf-8').strip()
                    if line:
                        self.process_incoming_data(line)
                else:
                    time.sleep(0.001)  # Small delay to prevent busy waiting
                    
            except Exception as e:
                rospy.logerr(f"Serial read error: {e}")
                self.handle_disconnection()
                break

    def process_incoming_data(self, data):
        """Process incoming data from Pi Pico"""
        try:
            # Try to parse as JSON
            if data.startswith('{') and data.endswith('}'):
                message = json.loads(data)
                self.handle_json_message(message)
            else:
                # Handle simple text messages
                self.handle_text_message(data)
                
        except json.JSONDecodeError:
            rospy.logwarn(f"Received non-JSON data: {data}")
            self.handle_text_message(data)
        except Exception as e:
            rospy.logerr(f"Error processing incoming data: {e}")

    def handle_json_message(self, message):
        """Handle structured JSON messages from Pi Pico"""
        msg_type = message.get('type', 'unknown')
        
        if msg_type == 'status':
            self.status_pub.publish(String(json.dumps(message)))
            rospy.loginfo(f"Pico Status: {message.get('data', 'No data')}")
            
        elif msg_type == 'error':
            rospy.logerr(f"Pico Error: {message.get('data', 'Unknown error')}")
            
        elif msg_type == 'handshake_ack':
            rospy.loginfo("Pi Pico 2W handshake acknowledged")
        
        elif msg_type == 'status':
            data = message.get('data', {})
            if 'actual_rpm' in data:
                try:
                    self.actual_rpm_pub.publish(Float32(float(data['actual_rpm'])))
                except Exception:
                    pass
            
        else:
            rospy.logwarn(f"Unknown message type from Pico: {msg_type}")

    def handle_text_message(self, data):
        """Handle simple text messages from Pi Pico"""
        rospy.loginfo(f"Pico: {data}")

    def send_command(self, command_type, data=None, priority=False):
        """Send command to Pi Pico 2W"""
        if not self.is_connected or not self.serial_port:
            rospy.logwarn("Cannot send command: Not connected to Pi Pico")
            return False
            
        try:
            message = {
                'type': command_type,
                'timestamp': time.time(),
                'data': data or {}
            }
            
            command_str = json.dumps(message) + '\n'
            
            with self.lock:
                if priority:
                    # High priority commands go to front of queue
                    with self.queue_lock:
                        self.command_queue.insert(0, command_str)
                else:
                    with self.queue_lock:
                        self.command_queue.append(command_str)
                        
            return True
            
        except Exception as e:
            rospy.logerr(f"Failed to queue command: {e}")
            return False

    def start_command_processor(self):
        """Start background thread to process command queue"""
        def process_commands():
            while self.running:
                try:
                    with self.queue_lock:
                        if self.command_queue and self.is_connected:
                            command = self.command_queue.pop(0)
                            
                            with self.lock:
                                if self.serial_port:
                                    self.serial_port.write(command.encode('utf-8'))
                                    rospy.logdebug(f"Sent to Pico: {command.strip()}")
                        else:
                            time.sleep(0.001)  # Small delay when queue is empty
                            
                except Exception as e:
                    rospy.logerr(f"Command processing error: {e}")
                    time.sleep(0.1)
                    
        self.command_thread = threading.Thread(target=process_commands)
        self.command_thread.daemon = True
        self.command_thread.start()

    # Minimal ROS Callbacks
    def run_callback(self, msg: Bool):
        should_run = bool(msg.data)
        self.is_running = should_run
        self.send_command('RUN' if should_run else 'STOP', {}, priority=True)

    def rpm_callback(self, msg: Float32):
        try:
            rpm = float(msg.data)
        except Exception:
            rospy.logwarn("Invalid RPM value received")
            return
        self.target_rpm = rpm
        self.send_command('SET_RPM', { 'rpm': rpm }, priority=False)

    def publish_connection_status(self, connected):
        """Publish connection status"""
        self.is_connected = connected
        self.connection_pub.publish(Bool(connected))
        
        if connected:
            self.status_pub.publish(String("Connected to Pi Pico 2W"))
        else:
            self.status_pub.publish(String("Disconnected from Pi Pico 2W"))

    def handle_disconnection(self):
        """Handle disconnection from Pi Pico"""
        rospy.logwarn("Lost connection to Pi Pico 2W")
        self.is_connected = False
        self.publish_connection_status(False)
        
        # Attempt reconnection
        threading.Timer(self.reconnect_interval, self.connect_to_pico).start()

    def shutdown(self):
        """Clean shutdown"""
        rospy.loginfo("Shutting down Pico Serial Communication Node")
        self.running = False
        
        if self.serial_port:
            try:
                self.serial_port.close()
            except:
                pass

def main():
    try:
        comm_node = PicoSerialComm()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'comm_node' in locals():
            comm_node.shutdown()

if __name__ == '__main__':
    main()
