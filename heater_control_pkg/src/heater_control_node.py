#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, String
import time
import math
import RPi.GPIO as GPIO # Library for direct GPIO control

# --- Configuration ---
ZONE_COUNT = 3
HYSTERESIS = 2.0 # Temperature band for PID state transition
# IMPORTANT: Define GPIO pins connected to your heater relays/SSRs
HEATER_PINS = {
    "zone1": 17, # Example BCM pin number for Zone 1 heater
    "zone2": 27, # Example BCM pin number for Zone 2 heater
    "zone3": 22, # Example BCM pin number for Zone 3 heater
}

# --- Global State Variables (using dictionaries) ---
current_setpoints = {f"zone{i+1}": 0.0 for i in range(ZONE_COUNT)}
current_temps = {f"zone{i+1}": float('nan') for i in range(ZONE_COUNT)}
commanded_states = {f"zone{i+1}": "OFF" for i in range(ZONE_COUNT)}
actual_states = {f"zone{i+1}": "OFF" for i in range(ZONE_COUNT)}
heater_pin_states = {f"zone{i+1}": False for i in range(ZONE_COUNT)} # Tracks physical output

# --- Publishers ---
# Dictionary to hold publishers for each zone's actual state
actual_state_pubs = {}

# --- GPIO Setup ---
def setup_gpio():
    GPIO.setmode(GPIO.BCM) # Use Broadcom pin numbering
    GPIO.setwarnings(False)
    for zone_id, pin in HEATER_PINS.items():
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW) # Set pin as output, initially LOW (OFF)
        heater_pin_states[zone_id] = False # Ensure internal state matches hardware
    rospy.loginfo("GPIO pins configured for heaters.")

# --- ROS Callbacks (using closures/lambdas to pass zone_id) ---
def create_callback(zone_id, data_dict):
    """Factory function to create topic callbacks that update the correct zone's data."""
    def callback(msg):
        # Basic validation (optional)
        if isinstance(msg, Float32) and msg.data >= 0:
            data_dict[zone_id] = msg.data
            # rospy.logdebug(f"HeaterControl({zone_id}): Received {type(msg).__name__}: {msg.data}")
        elif isinstance(msg, String) and msg.data in ["OFF", "HEATING"]:
            if data_dict[zone_id] != msg.data: # Only log changes
                 rospy.loginfo(f"HeaterControl({zone_id}): Received State Command: {msg.data}")
                 data_dict[zone_id] = msg.data
                 # If commanded OFF, force actual state OFF
                 if msg.data == "OFF" and actual_states[zone_id] != "OFF":
                     rospy.loginfo(f"HeaterControl({zone_id}): State forced to OFF by command.")
                     actual_states[zone_id] = "OFF"
                     # Update GPIO immediately on OFF command
                     control_heater(zone_id, False)
                     actual_state_pubs[zone_id].publish(actual_states[zone_id])
        else:
             rospy.logwarn(f"HeaterControl({zone_id}): Received invalid data: {msg.data}")
    return callback

# --- Control & GPIO Logic ---
def control_heater(zone_id, turn_on):
    """Controls the GPIO pin for the specified zone's heater."""
    global heater_pin_states
    pin = HEATER_PINS.get(zone_id)
    if pin is None:
        rospy.logerr(f"HeaterControl({zone_id}): No GPIO pin defined!")
        return

    desired_gpio_state = GPIO.HIGH if turn_on else GPIO.LOW
    current_gpio_state = GPIO.input(pin) # Read current physical state (optional check)

    # Only change GPIO if the desired state is different
    if heater_pin_states[zone_id] != turn_on:
        try:
            GPIO.output(pin, desired_gpio_state)
            heater_pin_states[zone_id] = turn_on # Update internal tracking
            rospy.loginfo(f"HeaterControl({zone_id}): Heater GPIO {pin} set to {'ON' if turn_on else 'OFF'}")
        except Exception as e:
            rospy.logerr(f"HeaterControl({zone_id}): Failed to set GPIO {pin}: {e}")

def control_loop():
    """Runs the state machine and GPIO control logic for all zones."""
    global actual_states

    for i in range(ZONE_COUNT):
        zone_id = f"zone{i+1}"
        previous_actual_state = actual_states[zone_id]

        temp = current_temps[zone_id]
        setpoint = current_setpoints[zone_id]
        commanded = commanded_states[zone_id]
        current_actual = actual_states[zone_id]

        heater_on = False # Default to heater off

        # --- Safety Check ---
        if math.isnan(temp) or setpoint <= 0:
            if current_actual != "OFF":
                rospy.logwarn(f"HeaterControl({zone_id}): Sensor/Setpoint invalid. Forcing state to OFF.")
                actual_states[zone_id] = "OFF"
            heater_on = False # Ensure heater is off

        # --- State Machine ---
        elif current_actual == "OFF":
            heater_on = False
            # Transition to HEATING if commanded and safe
            if commanded == "HEATING" and not math.isnan(temp) and setpoint > 0:
                actual_states[zone_id] = "HEATING"
                rospy.loginfo(f"HeaterControl({zone_id}): Transitioning OFF -> HEATING")

        elif current_actual == "HEATING":
            heater_on = True # Heater is ON during heat-up
            # Transition to OFF if commanded
            if commanded == "OFF":
                actual_states[zone_id] = "OFF"
                rospy.loginfo(f"HeaterControl({zone_id}): Transitioning HEATING -> OFF")
            # Transition to PID when reaching the band
            elif temp >= (setpoint - HYSTERESIS):
                actual_states[zone_id] = "PID"
                rospy.loginfo(f"HeaterControl({zone_id}): Temp {temp:.1f}C in band. Transitioning HEATING -> PID")

        elif current_actual == "PID":
            # Transition to OFF if commanded
            if commanded == "OFF":
                actual_states[zone_id] = "OFF"
                rospy.loginfo(f"HeaterControl({zone_id}): Transitioning PID -> OFF")
            else:
                # Simple ON/OFF PID logic based on setpoint
                if temp < setpoint:
                    heater_on = True
                else:
                    heater_on = False
                # Future: Implement actual PID calculation for PWM output here

        # --- Control GPIO ---
        control_heater(zone_id, heater_on)

        # --- Publish Actual State (if changed) ---
        if previous_actual_state != actual_states[zone_id]:
            rospy.loginfo(f"HeaterControl({zone_id}): Actual state is now {actual_states[zone_id]}")
            actual_state_pubs[zone_id].publish(actual_states[zone_id])

def main_heater_control():
    global actual_state_pubs
    rospy.init_node('heater_control_node', anonymous=True)
    setup_gpio()

    # --- Create Publishers and Subscribers for each zone ---
    for i in range(ZONE_COUNT):
        zone_id = f"zone{i+1}"
        # Publishers
        actual_state_pubs[zone_id] = rospy.Publisher(f'/extruder/{zone_id}/actual_state', String, queue_size=10, latch=True)
        # Subscribers
        rospy.Subscriber(f'/extruder/{zone_id}/setpoint', Float32, create_callback(zone_id, current_setpoints))
        rospy.Subscriber(f'/extruder/{zone_id}/temperature', Float32, create_callback(zone_id, current_temps))
        rospy.Subscriber(f'/extruder/{zone_id}/state_cmd', String, create_callback(zone_id, commanded_states))

        # Initialize state publication
        actual_state_pubs[zone_id].publish(actual_states[zone_id])

    rate = rospy.Rate(1) # Control loop frequency (Hz)
    rospy.loginfo("Heater Control Node Started. Monitoring all zones.")

    while not rospy.is_shutdown():
        control_loop()
        rate.sleep()

if __name__ == '__main__':
    try:
        main_heater_control()
    except rospy.ROSInterruptException:
        rospy.loginfo("Heater Control Node shutting down.")
    except Exception as e:
        rospy.logfatal(f"Heater Control Node Crashed: {e}")
    finally:
        # Cleanup GPIO pins on exit
        rospy.loginfo("Cleaning up GPIO...")
        GPIO.cleanup()
        rospy.loginfo("GPIO cleanup complete.")
