#!/usr/bin/env python3

# --- VERY EARLY DEBUG PRINT ---
print("DEBUG: heater_control_node.py starting execution...")
import sys # For stderr

try:
    import rospy
    from std_msgs.msg import Float32, String
    import time
    import math
    # --- Temporarily comment out GPIO import ---
    # import RPi.GPIO as GPIO # Library for direct GPIO control
    # --- VERY EARLY DEBUG PRINT ---
    print("DEBUG: heater_control_node.py imports successful (GPIO skipped).")
except Exception as import_e:
    # --- VERY EARLY DEBUG PRINT ---
    print(f"FATAL: Import error: {import_e}", file=sys.stderr)
    # Exit immediately if imports fail
    sys.exit(1)


# --- Configuration ---
ZONE_COUNT = 3
HYSTERESIS = 2.0 # Temperature band for PID state transition
# --- GPIO Pins commented out ---
# HEATER_PINS = {
#     "zone1": 17, # Example BCM pin number for Zone 1 heater
#     "zone2": 27, # Example BCM pin number for Zone 2 heater
#     "zone3": 22, # Example BCM pin number for Zone 3 heater
# }

# --- Global State Variables (using dictionaries) ---
current_setpoints = {f"zone{i+1}": 0.0 for i in range(ZONE_COUNT)}
current_temps = {f"zone{i+1}": float('nan') for i in range(ZONE_COUNT)}
commanded_states = {f"zone{i+1}": "OFF" for i in range(ZONE_COUNT)}
actual_states = {f"zone{i+1}": "OFF" for i in range(ZONE_COUNT)}
# --- GPIO state tracking commented out ---
# heater_pin_states = {f"zone{i+1}": False for i in range(ZONE_COUNT)} # Tracks physical output

# --- Publishers ---
actual_state_pubs = {}

# --- GPIO Setup (Commented Out) ---
# def setup_gpio():
#     rospy.loginfo("Attempting GPIO setup...")
#     try:
#         GPIO.setmode(GPIO.BCM)
#         GPIO.setwarnings(False)
#         for zone_id, pin in HEATER_PINS.items():
#             if pin is not None:
#                 GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
#                 heater_pin_states[zone_id] = False
#         rospy.loginfo("GPIO pins configured for heaters.")
#     except Exception as e:
#          # Log error but continue if possible, maybe some zones don't use GPIO
#         rospy.logerr(f"HeaterControl: Error during GPIO setup: {e}. Check permissions/hardware.")


# --- ROS Callbacks ---
def create_callback(zone_id, data_dict, data_type):
    """Factory function to create topic callbacks that update the correct zone's data."""
    def callback(msg):
        global actual_states # Allow modification for immediate state change
        valid = False
        value = None
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
            if data_dict[zone_id] != value:
                 if data_type == String:
                     rospy.loginfo(f"HeaterControl({zone_id}): Received State Command: {value}")
                 elif zone_id in current_setpoints and data_dict == current_setpoints:
                     rospy.loginfo(f"HeaterControl({zone_id}): Received new setpoint: {value:.1f} C")

                 data_dict[zone_id] = value

                 if data_type == String:
                     if value == "OFF" and actual_states[zone_id] != "OFF":
                         rospy.loginfo(f"HeaterControl({zone_id}): State forced to OFF by command.")
                         actual_states[zone_id] = "OFF"
                         # --- GPIO call commented out ---
                         # control_heater(zone_id, False)
                         if zone_id in actual_state_pubs:
                             actual_state_pubs[zone_id].publish(actual_states[zone_id])
                     elif value == "HEATING" and actual_states[zone_id] == "OFF":
                         # Only transition to HEATING if safe (temp received, setpoint > 0)
                         if not math.isnan(current_temps[zone_id]) and current_setpoints[zone_id] > 0:
                             rospy.loginfo(f"HeaterControl({zone_id}): State forced to HEATING by command.")
                             actual_states[zone_id] = "HEATING"
                             if zone_id in actual_state_pubs:
                                 actual_state_pubs[zone_id].publish(actual_states[zone_id])
                         else:
                             rospy.logwarn(f"HeaterControl({zone_id}): Received HEATING command but conditions not met (No Temp or Setpoint <= 0). Staying OFF.")
                             # Ensure commanded state reflects reality if we ignore it
                             commanded_states[zone_id] = "OFF"

    return callback


# --- Control & GPIO Logic (GPIO Part Commented Out) ---
# def control_heater(zone_id, turn_on):
#     """Controls the GPIO pin for the specified zone's heater."""
#     global heater_pin_states
#     pin = HEATER_PINS.get(zone_id)
#     if pin is None:
#         return
#
#     desired_gpio_state = GPIO.HIGH if turn_on else GPIO.LOW
#     if heater_pin_states.get(zone_id) != turn_on:
#         try:
#             GPIO.output(pin, desired_gpio_state)
#             heater_pin_states[zone_id] = turn_on
#             rospy.loginfo(f"HeaterControl({zone_id}): Heater GPIO {pin} set to {'ON' if turn_on else 'OFF'}")
#         except Exception as e:
#             rospy.logerr(f"HeaterControl({zone_id}): Failed to set GPIO {pin}: {e}")

def control_loop():
    """Runs the state machine logic for all zones."""
    global actual_states

    for i in range(ZONE_COUNT):
        zone_id = f"zone{i+1}"
        # Use copies for logic
        temp = current_temps[zone_id]
        setpoint = current_setpoints[zone_id]
        commanded = commanded_states[zone_id]
        current_actual = actual_states[zone_id]
        previous_actual_state = current_actual

        rospy.loginfo_throttle(5, f"LOOP_START({zone_id}): Temp={temp:.1f}, Setpoint={setpoint:.1f}, Commanded='{commanded}', Actual='{current_actual}'")

        heater_should_be_on = False # Track logical heater state

        # --- State Machine ---
        if math.isnan(temp) or setpoint <= 0:
            if current_actual != "OFF":
                rospy.logwarn(f"HeaterControl({zone_id}): Sensor/Setpoint invalid. Forcing state to OFF.")
                actual_states[zone_id] = "OFF"
            heater_should_be_on = False

        elif current_actual == "OFF":
            heater_should_be_on = False
            # Transition handled in callback

        elif current_actual == "HEATING":
            heater_should_be_on = True
            lower_band = setpoint - HYSTERESIS
            comparison_result = temp >= lower_band
            rospy.loginfo_throttle(5, f"DEBUG({zone_id}): State=HEATING, Temp={temp:.1f}, Setpoint={setpoint:.1f}, LowerBand={lower_band:.1f}, Temp>=LowerBand? {comparison_result}")

            if not math.isnan(temp) and comparison_result:
                actual_states[zone_id] = "PID"
                rospy.loginfo(f"HeaterControl({zone_id}): Temp {temp:.1f}C in band. Transitioning HEATING -> PID")
            # OFF transition handled in callback

        elif current_actual == "PID":
            if not math.isnan(temp):
                if temp < setpoint:
                    heater_should_be_on = True
                    rospy.loginfo_throttle(5, f"DEBUG({zone_id}): State=PID, Temp={temp:.1f} < Setpoint={setpoint:.1f}. Heater SHOULD BE ON")
                else:
                    heater_should_be_on = False
                    rospy.loginfo_throttle(5, f"DEBUG({zone_id}): State=PID, Temp={temp:.1f} >= Setpoint={setpoint:.1f}. Heater SHOULD BE OFF")
            else:
                 rospy.logwarn_throttle(5, f"HeaterControl({zone_id}): State=PID, Temp is NaN. Forcing Heater SHOULD BE OFF.")
                 heater_should_be_on = False
            # OFF transition handled in callback

        # --- Control GPIO (Commented Out) ---
        # --- FIX: Comment out the block checking HEATER_PINS ---
        # if HEATER_PINS.get(zone_id) is not None:
        #      control_heater(zone_id, heater_should_be_on)
        # elif heater_should_be_on:
        #      rospy.logwarn_throttle(10, f"HeaterControl({zone_id}): Trying to turn heater ON but no GPIO pin defined.")
        # --- Log the logical state instead ---
        # if HEATER_PINS.get(zone_id) is not None: # <-- Comment out this check
        rospy.loginfo_throttle(5, f"LOGIC({zone_id}): Heater SHOULD BE {'ON' if heater_should_be_on else 'OFF'}")


        # --- Publish Actual State (if changed) ---
        if previous_actual_state != actual_states[zone_id]:
            rospy.loginfo(f"HeaterControl({zone_id}): Actual state changed {previous_actual_state} -> {actual_states[zone_id]}")
            if zone_id in actual_state_pubs:
                actual_state_pubs[zone_id].publish(actual_states[zone_id])


def main_heater_control():
    # --- ADDED PRINT ---
    print("DEBUG: Entering main_heater_control function...")
    global actual_state_pubs
    try:
        # --- ADDED PRINT ---
        print("DEBUG: Attempting rospy.init_node...")
        rospy.init_node('heater_control_node', anonymous=True)
        # --- Use print here as rospy logging might not be ready yet ---
        print("DEBUG: ROS node initialized.")
    except Exception as init_e:
        print(f"FATAL: Failed to initialize ROS node: {init_e}", file=sys.stderr)
        sys.exit(1)

    # --- GPIO Setup Call Commented Out ---
    # try:
    #     setup_gpio()
    # except Exception as gpio_e:
    #     rospy.logerr(f"HeaterControl: Failed to setup GPIO. Error: {gpio_e}")

    # --- Use print here as well ---
    print("DEBUG: Setting up publishers and subscribers...")
    rospy.loginfo("Setting up publishers and subscribers...") # Keep rospy log too
    for i in range(ZONE_COUNT):
        zone_id = f"zone{i+1}"
        try:
            actual_state_pubs[zone_id] = rospy.Publisher(f'/extruder/{zone_id}/actual_state', String, queue_size=10, latch=True)
            rospy.Subscriber(f'/extruder/{zone_id}/setpoint', Float32, create_callback(zone_id, current_setpoints, Float32))
            rospy.Subscriber(f'/extruder/{zone_id}/temperature', Float32, create_callback(zone_id, current_temps, Float32))
            rospy.Subscriber(f'/extruder/{zone_id}/state_cmd', String, create_callback(zone_id, commanded_states, String))
            actual_state_pubs[zone_id].publish(actual_states[zone_id]) # Publish initial state
        except Exception as pubsub_e:
             # Use print for critical setup errors
            print(f"ERROR: HeaterControl({zone_id}): Failed during Pub/Sub setup: {pubsub_e}", file=sys.stderr)
            rospy.logerr(f"HeaterControl({zone_id}): Failed during Pub/Sub setup: {pubsub_e}")

    # --- Use print ---
    print("DEBUG: Pub/Sub setup complete.")
    rospy.loginfo("Pub/Sub setup complete.") # Keep rospy log

    rate = rospy.Rate(1) # Control loop frequency (Hz)
    # --- Use print ---
    print("DEBUG: Heater Control Node Started (No GPIO). Entering main control loop...")
    rospy.loginfo("Heater Control Node Started (No GPIO). Entering main control loop...") # Keep rospy log

    while not rospy.is_shutdown():
        try:
            control_loop()
        except Exception as loop_e:
            # Use print for loop errors
             print(f"ERROR: HeaterControl: Error in control loop: {loop_e}", file=sys.stderr)
             rospy.logerr(f"HeaterControl: Error in control loop: {loop_e}")
        rate.sleep()

if __name__ == '__main__':
    # --- Use print in top-level handler ---
    try:
        print("DEBUG: Script __main__ block executing...")
        main_heater_control()
    except rospy.ROSInterruptException:
        print("Heater Control Node shutting down (ROSInterrupt).") # Use print
    except Exception as main_e:
        print(f"FATAL: Heater Control Node Crashed: {main_e}", file=sys.stderr) # Use print
    finally:
        # --- GPIO Cleanup Call Commented Out ---
        pass
        # try:
        #      if 'GPIO' in locals() or 'GPIO' in globals():
        #          if any(pin is not None for pin in HEATER_PINS.values()):
        #              print("DEBUG: Cleaning up GPIO...")
        #              GPIO.cleanup()
        #              print("DEBUG: GPIO cleanup complete.")
        # except NameError:
        #      pass
        # except Exception as cleanup_e:
        #      print(f"ERROR: Error during GPIO cleanup: {cleanup_e}", file=sys.stderr)

