#!/usr/bin/env python3
"""
Pi Pico 2W Firmware - Minimal Extruder Control
Commands: RUN, STOP, SET_RPM
Publishes: status { actual_rpm }
"""

import json
import time
from machine import Pin, PWM
import uasyncio as asyncio

class ExtruderController:
    def __init__(self):
        # Motor PWM pin (adjust pin number as wired)
        self.motor_pin = Pin(2, Pin.OUT)
        self.motor_pwm = PWM(self.motor_pin)
        self.motor_pwm.freq(1000)
        self.motor_pwm.duty_u16(0)

        # Status LED
        self.status_led = Pin("LED", Pin.OUT)
        
        # State variables
        self.is_connected = False
        self.is_running = False
        self.target_rpm = 0.0
        self.actual_rpm = 0.0

        # Command processing
        self.command_queue = []
        self.running = True
        
        print("Extruder Controller initialized (RUN/STOP/RPM)")

    def set_motor_speed_percent(self, speed_percent):
        try:
            duty = int(max(0, min(100, speed_percent)) * 655.35)
            self.motor_pwm.duty_u16(duty)
            return True
        except Exception as e:
            print("Error setting motor duty:", e)
            return False
    
    def send_response(self, response_type, data=None):
        """Send response back to Raspberry Pi"""
        try:
            message = {
                'type': response_type,
                'timestamp': time.time(),
                'data': data or {}
            }
            response = json.dumps(message) + '\n'
            print(response, end='')  # Send via USB serial
        except Exception as e:
            print(f"Error sending response: {e}")
    
    def process_command(self, command_data):
        """Process incoming command from Raspberry Pi"""
        try:
            command_type = command_data.get('type', 'unknown')
            data = command_data.get('data', {})
            
            if command_type == 'HANDSHAKE':
                self.is_connected = True
                self.status_led.value(1)
                self.send_response('handshake_ack', {'status': 'ready'})
                print("Handshake acknowledged")
            elif command_type == 'RUN':
                self.is_running = True
                self.status_led.value(1)
                # Use current target_rpm to set duty
                speed_percent = max(0.0, min(100.0, self.target_rpm_to_percent(self.target_rpm)))
                self.set_motor_speed_percent(speed_percent)
                self.send_response('status', {'running': True})

            elif command_type == 'STOP':
                self.is_running = False
                self.set_motor_speed_percent(0.0)
                self.status_led.value(0)
                self.send_response('status', {'running': False})

            elif command_type == 'SET_RPM':
                rpm = float(data.get('rpm', 0.0))
                self.target_rpm = rpm
                if self.is_running:
                    speed_percent = max(0.0, min(100.0, self.target_rpm_to_percent(rpm)))
                    self.set_motor_speed_percent(speed_percent)
                self.send_response('status', {'target_rpm': self.target_rpm})
                
            else:
                self.send_response('error', {'message': f'Unknown command: {command_type}'})
                
        except Exception as e:
            self.send_response('error', {'message': f'Command processing error: {str(e)}'})
            print(f"Error processing command: {e}")

    def target_rpm_to_percent(self, rpm):
        # Map RPM to PWM percent. Adjust scaling to your system.
        # Example: 0..3000 RPM -> 0..100%
        max_rpm = 3000.0
        return (max(0.0, min(max_rpm, rpm)) / max_rpm) * 100.0
    
    async def status_monitor(self):
        """Monitor system status and send periodic updates"""
        while self.running:
            try:
                if self.is_connected:
                    # Simple model: actual_rpm approaches target when running
                    if self.is_running:
                        # Slew towards target
                        self.actual_rpm += (self.target_rpm - self.actual_rpm) * 0.2
                    else:
                        self.actual_rpm *= 0.8

                    status_data = {
                        'running': self.is_running,
                        'target_rpm': self.target_rpm,
                        'actual_rpm': round(self.actual_rpm, 1)
                    }
                    self.send_response('status', status_data)

                await asyncio.sleep(0.1)

            except Exception as e:
                print(f"Status monitor error: {e}")
                await asyncio.sleep(1)
    
    async def command_processor(self):
        """Process incoming commands from serial (stdin)"""
        import sys
        reader = asyncio.StreamReader()
        proto = asyncio.StreamReaderProtocol(reader)
        await asyncio.get_event_loop().connect_read_pipe(lambda: proto, sys.stdin)
        while self.running:
            try:
                line = await reader.readline()
                if not line:
                    await asyncio.sleep(0.01)
                    continue
                text = line.decode().strip()
                if text:
                    try:
                        cmd = json.loads(text)
                        self.process_command(cmd)
                    except Exception as e:
                        self.send_response('error', {'message': 'Invalid JSON'})
            except Exception as e:
                print(f"Command processor error: {e}")
                await asyncio.sleep(0.1)
    
    async def main_loop(self):
        """Main control loop"""
        print("Starting Extruder Controller...")
        
        # Start status monitoring
        status_task = asyncio.create_task(self.status_monitor())
        command_task = asyncio.create_task(self.command_processor())
        
        try:
            # Run both tasks concurrently
            await asyncio.gather(status_task, command_task)
        except KeyboardInterrupt:
            print("Shutting down...")
        finally:
            self.running = False
            # Cleanup
            self.set_motor_speed_percent(0)
            self.status_led.value(0)

# Initialize and run the controller
if __name__ == '__main__':
    controller = ExtruderController()
    try:
        asyncio.run(controller.main_loop())
    except KeyboardInterrupt:
        print("Controller stopped by user")
    except Exception as e:
        print(f"Fatal error: {e}")
