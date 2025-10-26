#!/usr/bin/env python3

# --- VERY EARLY DEBUG PRINT ---
print("DEBUG: heater_control_node.py starting execution...")
import sys # For stderr

try:
    import rospy
    from std_msgs.msg import Float32, String
    import time # Import time for sleep
    import math
    import RPi.GPIO as GPIO # Library for direct GPIO control
    # --- VERY EARLY DEBUG PRINT ---
    print("DEBUG: heater_control_node.py imports successful.")
except Exception as import_e:
    # --- VERY EARLY DEBUG PRINT ---
    print(f"FATAL: Import error: {import_e}", file=sys.stderr)
    # Exit immediately if imports fail
    sys.exit(1)


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
    # --- ADDED DEBUG ---
    rospy.loginfo("Attempting GPIO setup...")
    GPIO.setmode(GPIO.BCM) # Use Broadcom pin numbering
    GPIO.setwarnings(False)
    for zone_id, pin in HEATER_PINS.items():
        if pin is not None: # Check if pin is defined
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW) # Set pin as output, initially LOW (OFF)
            heater_pin_states[zone_id] = False # Ensure internal state matches hardware
    rospy.loginfo("GPIO pins configured for heaters.")

# --- ROS Callbacks (using closures/lambdas to pass zone_id) ---
def create_callback(zone_id, data_dict, data_type):
    """Factory function to create topic callbacks that update the correct zone's data."""
    def callback(msg):
        # --- Need global actual_states to modify it directly ---
        global actual_states
        valid = False
        value = None
        # Basic validation (optional)
        if data_type == Float32:
            if isinstance(msg, Float32) and msg.data >= 0:
                 value = msg.data
                 valid = True
            else:
                 rospy.logwarn(f"HeaterControl({zone_id}): Received invalid Float32 data: {msg.data}")
        elif data_type == String:
             if isinstance(msg, String) and msg.data in ["OFF", "HEATING"]:
                 value = msg.data
                 valid = True
             else:
                  rospy.logwarn(f"HeaterControl({zone_id}): Received invalid String data: {msg.data}")

        if valid:
            if data_dict[zone_id] != value: # Only log changes
                 # Decide log level based on type
                 if data_type == String:
                     rospy.loginfo(f"HeaterControl({zone_id}): Received State Command: {value}")
                 elif zone_id in current_setpoints and data_dict == current_setpoints: # Log setpoint changes
                     rospy.loginfo(f"HeaterControl({zone_id}): Received new setpoint: {value:.1f} C")

                 data_dict[zone_id] = value

                 # --- CRITICAL UPDATE: Handle state commands immediately ---
                 if data_type == String: # Only act on state commands here
                     if value == "OFF" and actual_states[zone_id] != "OFF":
                         rospy.loginfo(f"HeaterControl({zone_id}): State forced to OFF by command.")
                         actual_states[zone_id] = "OFF"
                         # Update GPIO immediately on OFF command
                         control_heater(zone_id, False)
                         if zone_id in actual_state_pubs:
                             actual_state_pubs[zone_id].publish(actual_states[zone_id])
                     # --- ADDED THIS BLOCK ---
                     elif value == "HEATING" and actual_states[zone_id] == "OFF":
                         rospy.loginfo(f"HeaterControl({zone_id}): State forced to HEATING by command.")
                         actual_states[zone_id] = "HEATING"
                         # Optionally turn on heater immediately, or let control loop handle it
                         # control_heater(zone_id, True) # Turn on immediately
                         if zone_id in actual_state_pubs:
                             actual_state_pubs[zone_id].publish(actual_states[zone_id]) # Publish new state
                     # ------------------------

    return callback


# --- Control & GPIO Logic ---
def control_heater(zone_id, turn_on):
    """Controls the GPIO pin for the specified zone's heater."""
    global heater_pin_states
    pin = HEATER_PINS.get(zone_id)
    if pin is None:
        # Don't log error continuously if pin isn't defined for this zone yet
        # rospy.logerr(f"HeaterControl({zone_id}): No GPIO pin defined!")
        return

    desired_gpio_state = GPIO.HIGH if turn_on else GPIO.LOW
    # current_gpio_state = GPIO.input(pin) # Reading state can be slow/unreliable sometimes

    # Only change GPIO if the desired state is different from our tracked state
    if heater_pin_states.get(zone_id) != turn_on:
        try:
            GPIO.output(pin, desired_gpio_state)
            heater_pin_states[zone_id] = turn_on # Update internal tracking
            rospy.loginfo(f"HeaterControl({zone_id}): Heater GPIO {pin} set to {'ON' if turn_on else 'OFF'}")
        except Exception as e:
            rospy.logerr(f"HeaterControl({zone_id}): Failed to set GPIO {pin}: {e}")

def control_loop():
    """Runs the state machine and GPIO control logic for all zones."""
    global actual_states
    # rospy.loginfo("Control loop executing...") # Make less frequent if needed

    for i in range(ZONE_COUNT):
        zone_id = f"zone{i+1}"
        previous_actual_state = actual_states[zone_id]

        # --- ENHANCED DEBUG LOGGING AT START OF LOOP ITERATION ---
        temp = current_temps[zone_id]
        setpoint = current_setpoints[zone_id]
        commanded = commanded_states[zone_id]
        current_actual = actual_states[zone_id] # Use local copy for logic clarity
        rospy.loginfo_throttle(5, f"LOOP_START({zone_id}): Temp={temp:.1f}, Setpoint={setpoint:.1f}, Commanded='{commanded}', Actual='{current_actual}'")
        # ---------------------------------------------------------


        heater_on = False # Default to heater off

        # --- Safety Check ---
        if math.isnan(temp) or setpoint <= 0:
            if current_actual != "OFF":
                rospy.logwarn(f"HeaterControl({zone_id}): Sensor/Setpoint invalid. Forcing state to OFF.")
                actual_states[zone_id] = "OFF" # Update global state
            heater_on = False # Ensure heater is off

        # --- State Machine ---
        elif current_actual == "OFF":
            heater_on = False
            # Transition to HEATING ONLY happens in callback now based on command

        elif current_actual == "HEATING":
            heater_on = True # Heater is ON during heat-up

            # --- DEBUG LOGGING ADDED HERE ---
            lower_band = setpoint - HYSTERESIS
            comparison_result = temp >= lower_band
            # Throttle debug log to once every 5 seconds to reduce noise
            rospy.loginfo_throttle(5, f"DEBUG({zone_id}): State=HEATING, Temp={temp:.1f}, Setpoint={setpoint:.1f}, LowerBand={lower_band:.1f}, Temp>=LowerBand? {comparison_result}")
            # --------------------------------

            # Transition to OFF is handled in callback now
            # Transition to PID when reaching the band
            if not math.isnan(temp) and comparison_result: # Ensure temp is valid before comparing
                actual_states[zone_id] = "PID"
                rospy.loginfo(f"HeaterControl({zone_id}): Temp {temp:.1f}C in band. Transitioning HEATING -> PID")

        elif current_actual == "PID":
            # Transition to OFF is handled in callback now
            # Simple ON/OFF PID logic based on setpoint
            if not math.isnan(temp): # Ensure temp is valid
                if temp < setpoint:
                    heater_on = True
                else:
                    heater_on = False
            else:
                 heater_on = False # Safety off if sensor fails in PID mode
            # Future: Implement actual PID calculation for PWM output here

        # --- Control GPIO ---
        # Make sure pin is defined before trying to control
        if HEATER_PINS.get(zone_id) is not None:
             control_heater(zone_id, heater_on)
        elif heater_on: # Log warning if trying to turn on heater without a pin
             rospy.logwarn_throttle(10, f"HeaterControl({zone_id}): Trying to turn heater ON but no GPIO pin defined.")


        # --- Publish Actual State (if changed) ---
        if previous_actual_state != actual_states[zone_id]:
            rospy.loginfo(f"HeaterControl({zone_id}): Actual state is now {actual_states[zone_id]}")
            if zone_id in actual_state_pubs:
                actual_state_pubs[zone_id].publish(actual_states[zone_id])

def main_heater_control():
    # --- ADDED DEBUG ---
    print("DEBUG: Entering main_heater_control function...") # Use print before rospy is init
    global actual_state_pubs
    try:
        rospy.init_node('heater_control_node', anonymous=True)
        # --- ADDED DEBUG ---
        rospy.loginfo("ROS node initialized.")
    except Exception as init_e:
        print(f"FATAL: Failed to initialize ROS node: {init_e}", file=sys.stderr)
        sys.exit(1)

    try:
        # --- ADDED DELAY ---
        rospy.loginfo("Waiting 2 seconds before GPIO setup...")
        time.sleep(2.0)
        # -----------------
        setup_gpio()
    except Exception as gpio_e:
        rospy.logerr(f"HeaterControl: Failed to setup GPIO. Check permissions/wiring. Error: {gpio_e}")
        # Decide if GPIO failure is fatal
        sys.exit(1) # Exit if GPIO fails

    # --- Create Publishers and Subscribers for each zone ---
    # --- ADDED DEBUG ---
    rospy.loginfo("Setting up publishers and subscribers...")
    for i in range(ZONE_COUNT):
        zone_id = f"zone{i+1}"
        try:
            # Publishers
            actual_state_pubs[zone_id] = rospy.Publisher(f'/extruder/{zone_id}/actual_state', String, queue_size=10, latch=True)
            # Subscribers
            rospy.Subscriber(f'/extruder/{zone_id}/setpoint', Float32, create_callback(zone_id, current_setpoints, Float32))
            rospy.Subscriber(f'/extruder/{zone_id}/temperature', Float32, create_callback(zone_id, current_temps, Float32))
            rospy.Subscriber(f'/extruder/{zone_id}/state_cmd', String, create_callback(zone_id, commanded_states, String))

            # Initialize state publication
            actual_state_pubs[zone_id].publish(actual_states[zone_id])
        except Exception as pubsub_e:
            rospy.logerr(f"HeaterControl({zone_id}): Failed during Pub/Sub setup: {pubsub_e}")
            # Decide if failure for one zone is fatal for all
            # sys.exit(1)

    # --- ADDED DEBUG ---
    rospy.loginfo("Pub/Sub setup complete.")

    rate = rospy.Rate(1) # Control loop frequency (Hz)
    rospy.loginfo("Heater Control Node Started. Monitoring all zones.")
    # --- ADDED DEBUG ---
    rospy.loginfo("Entering main control loop...")

    while not rospy.is_shutdown():
        try:
            control_loop()
        except Exception as loop_e:
             rospy.logerr(f"HeaterControl: Error in control loop: {loop_e}")
             # Decide if the error is recoverable
        rate.sleep()

if __name__ == '__main__':
    # --- ADDED TOP-LEVEL TRY/EXCEPT ---
    try:
        print("DEBUG: Script __main__ block executing...")
        main_heater_control()
    except rospy.ROSInterruptException:
        print("Heater Control Node shutting down (ROSInterrupt).")
    except Exception as main_e:
        print(f"FATAL: Heater Control Node Crashed: {main_e}", file=sys.stderr)
    finally:
        # Cleanup GPIO pins on exit, only if GPIO was initialized successfully
        try:
            # Check if GPIO has been set up before trying cleanup
             if 'GPIO' in locals() or 'GPIO' in globals():
                 if any(pin is not None for pin in HEATER_PINS.values()):
                     print("DEBUG: Cleaning up GPIO...")
                     GPIO.cleanup()
                     print("DEBUG: GPIO cleanup complete.")
        except NameError:
             pass # GPIO object might not exist
        except Exception as cleanup_e:
             print(f"ERROR: Error during GPIO cleanup: {cleanup_e}", file=sys.stderr)

