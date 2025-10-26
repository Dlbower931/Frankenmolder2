#!/usr/bin/env python3

import sys
try:
    import rospy
    from std_msgs.msg import Float32, String
    import time
    import math
    # --- Re-enable GPIO import ---
    import RPi.GPIO as GPIO # Library for direct GPIO control
    print("DEBUG: heater_control_node.py imports successful (GPIO Enabled).", flush=True)
except Exception as import_e:
    print(f"FATAL: Import error: {import_e}", file=sys.stderr, flush=True)
    sys.exit(1)


# --- Configuration ---
ZONE_COUNT = 3
HYSTERESIS = 2.0 # Temperature band for PID state transition
PID_DROP_TIMEOUT = 5.0 # Seconds below band before reverting to HEATING

# --- Re-enable GPIO Pins dictionary ---
HEATER_PINS = {
    "zone1": 17, # Example BCM pin number for Zone 1 heater
    "zone2": 27, # Example BCM pin number for Zone 2 heater
    "zone3": 22, # Example BCM pin number for Zone 3 heater
}

# --- Global State Variables (using dictionaries) ---
current_setpoints = {f"zone{i+1}": 0.0 for i in range(ZONE_COUNT)}
current_temps = {f"zone{i+1}": float('nan') for i in range(ZONE_COUNT)}
actual_states = {f"zone{i+1}": "OFF" for i in range(ZONE_COUNT)}
pid_below_band_start_time = {f"zone{i+1}": None for i in range(ZONE_COUNT)}
# --- Re-enable GPIO state tracking ---
heater_pin_states = {f"zone{i+1}": False for i in range(ZONE_COUNT)} # Tracks physical output

# --- Publishers ---
state_cmd_pubs = {}

# --- GPIO Setup (Re-enabled) ---
def setup_gpio():
    rospy.loginfo("Attempting GPIO setup...")
    try:
        GPIO.setmode(GPIO.BCM) # Use Broadcom pin numbering
        GPIO.setwarnings(False)
        for zone_id, pin in HEATER_PINS.items():
            if pin is not None: # Check if pin is defined
                GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW) # Set pin as output, initially LOW (OFF)
                heater_pin_states[zone_id] = False # Ensure internal state matches hardware
        rospy.loginfo("GPIO pins configured for heaters.")
    except Exception as e:
         rospy.logerr(f"HeaterControl: Error during GPIO setup: {e}. Check permissions/hardware.")
         # Depending on severity, you might want to raise the exception or handle it
         # raise e # Re-raise if GPIO is absolutely critical


# --- ROS Callbacks ---
def create_callback(zone_id, data_dict, data_type):
    """Factory function to create topic callbacks that update the correct zone's data."""
    def callback(msg):
        global actual_states
        valid = False
        value = None
        if data_type == Float32:
            if isinstance(msg, Float32) and msg.data >= 0:
                 value = msg.data
                 valid = True
            else:
                 rospy.logwarn(f"HeaterControl({zone_id}): Received invalid Float32 data: {msg.data}")
        elif data_type == String:
             if isinstance(msg, String) and msg.data in ["OFF", "HEATING", "PID"]:
                 value = msg.data
                 valid = True
             else:
                  rospy.logwarn(f"HeaterControl({zone_id}): Received invalid String data: {msg.data}")

        if valid:
            current_value = data_dict.get(zone_id)
            if current_value != value:
                 if data_type == String:
                     rospy.loginfo(f"HeaterControl({zone_id}): Received State Command Request: {value}")
                     current_actual_state = actual_states.get(zone_id)
                     if value == "OFF" and current_actual_state != "OFF":
                         rospy.loginfo(f"HeaterControl({zone_id}): Forcing state to OFF due to external command.")
                         actual_states[zone_id] = "OFF"
                         pid_below_band_start_time[zone_id] = None
                         # --- Re-enable GPIO control on OFF command ---
                         control_heater(zone_id, False)
                         if zone_id in state_cmd_pubs:
                             state_cmd_pubs[zone_id].publish(String(actual_states[zone_id]))
                     elif value == "HEATING" and current_actual_state == "OFF":
                         temp_ok = not math.isnan(current_temps.get(zone_id, float('nan')))
                         setpoint_ok = current_setpoints.get(zone_id, 0.0) > 0
                         if temp_ok and setpoint_ok:
                             rospy.loginfo(f"HeaterControl({zone_id}): Accepting HEATING command, starting heat cycle.")
                             actual_states[zone_id] = "HEATING"
                             pid_below_band_start_time[zone_id] = None
                             if zone_id in state_cmd_pubs:
                                 state_cmd_pubs[zone_id].publish(String(actual_states[zone_id]))
                         else:
                             rospy.logwarn(f"HeaterControl({zone_id}): Ignoring HEATING command, conditions not met (Temp={current_temps.get(zone_id, 'NaN')}, Setpoint={current_setpoints.get(zone_id, 0.0)}). Staying OFF.")
                             if zone_id in state_cmd_pubs:
                                 state_cmd_pubs[zone_id].publish(String("OFF"))

                 elif data_type == Float32:
                     if zone_id in current_setpoints and data_dict == current_setpoints:
                         rospy.loginfo(f"HeaterControl({zone_id}): Received new setpoint: {value:.1f} C")
                         if actual_states.get(zone_id) == "PID":
                             pid_below_band_start_time[zone_id] = None
                     data_dict[zone_id] = value

    return callback


# --- Control & GPIO Logic (Re-enabled) ---
def control_heater(zone_id, turn_on):
    """Controls the GPIO pin for the specified zone's heater."""
    global heater_pin_states
    pin = HEATER_PINS.get(zone_id)
    if pin is None:
        # Log only once if pin not defined but trying to control
        rospy.logwarn_once(f"HeaterControl({zone_id}): No GPIO pin defined in HEATER_PINS.")
        return

    desired_gpio_state = GPIO.HIGH if turn_on else GPIO.LOW
    # Check internal state first to minimize redundant GPIO calls
    if heater_pin_states.get(zone_id) != turn_on:
        try:
            GPIO.output(pin, desired_gpio_state)
            heater_pin_states[zone_id] = turn_on # Update internal tracking AFTER successful write
            rospy.loginfo(f"HeaterControl({zone_id}): Heater GPIO {pin} set to {'ON' if turn_on else 'OFF'}")
        except RuntimeError as e: # Catch potential GPIO errors (e.g., if library not init properly)
            rospy.logerr(f"HeaterControl({zone_id}): GPIO Runtime Error setting pin {pin}: {e}")
        except Exception as e:
            rospy.logerr(f"HeaterControl({zone_id}): Failed to set GPIO {pin}: {e}")

def control_loop():
    """Runs the state machine and GPIO control logic for all zones."""
    global actual_states, pid_below_band_start_time

    for i in range(ZONE_COUNT):
        zone_id = f"zone{i+1}"
        temp = current_temps.get(zone_id, float('nan'))
        setpoint = current_setpoints.get(zone_id, 0.0)
        current_actual = actual_states.get(zone_id, "OFF")
        previous_actual_state = current_actual

        # Restore normal logging frequency
        rospy.loginfo_throttle(10, f"LOOP_START({zone_id}): Temp={temp:.1f}, Setpoint={setpoint:.1f}, Actual='{current_actual}'")

        heater_should_be_on = False

        # --- State Machine (Driven by actual_states) ---
        if math.isnan(temp) or setpoint <= 0:
            if current_actual != "OFF":
                rospy.logwarn(f"HeaterControl({zone_id}): Sensor/Setpoint invalid. Forcing state to OFF.")
                actual_states[zone_id] = "OFF"
                pid_below_band_start_time[zone_id] = None
            heater_should_be_on = False

        elif current_actual == "OFF":
            heater_should_be_on = False
            pid_below_band_start_time[zone_id] = None

        elif current_actual == "HEATING":
            heater_should_be_on = True
            lower_band = setpoint - HYSTERESIS
            comparison_result = False
            pid_below_band_start_time[zone_id] = None
            # Restore normal logging frequency
            # rospy.loginfo_throttle(5, f"DEBUG({zone_id}): In HEATING state. Temp={temp:.1f}, Setpoint={setpoint:.1f}, LowerBand={lower_band:.1f}")
            try:
                 if not math.isnan(temp):
                     temp_float = float(temp)
                     lower_band_float = float(lower_band)
                     comparison_result = temp_float >= lower_band_float
                     # rospy.loginfo_throttle(5, f"DEBUG({zone_id}): Comparison: {temp_float:.1f} >= {lower_band_float:.1f} is {comparison_result}")
                 # else: rospy.loginfo_throttle(5, f"DEBUG({zone_id}): Temp is NaN, cannot compare.")
            except (TypeError, ValueError) as e:
                 rospy.logerr(f"ERROR({zone_id}): Error during comparison: Temp={temp}, LowerBand={lower_band}, Error={e}")

            if not math.isnan(temp) and comparison_result:
                actual_states[zone_id] = "PID"
                rospy.loginfo(f"INFO({zone_id}): Temp {temp:.1f}C in band. Transitioning HEATING -> PID")

        elif current_actual == "PID":
            if not math.isnan(temp):
                lower_band = setpoint - HYSTERESIS
                if temp < lower_band:
                    if pid_below_band_start_time[zone_id] is None:
                        pid_below_band_start_time[zone_id] = rospy.Time.now()
                        rospy.logwarn(f"WARN({zone_id}): Temp {temp:.1f}C dropped below PID band ({lower_band:.1f}C). Starting timer...")
                        heater_should_be_on = True # Keep heater ON during initial drop
                    else:
                        time_below_band = rospy.Time.now() - pid_below_band_start_time[zone_id]
                        if time_below_band > rospy.Duration(PID_DROP_TIMEOUT):
                            rospy.logerr(f"ERROR({zone_id}): Temp {temp:.1f}C stayed below PID band for {time_below_band.to_sec():.1f}s. Transitioning PID -> HEATING")
                            actual_states[zone_id] = "HEATING"
                            heater_should_be_on = True
                            pid_below_band_start_time[zone_id] = None
                        else:
                            rospy.logwarn_throttle(2, f"WARN({zone_id}): Temp {temp:.1f}C still below PID band. Timer: {time_below_band.to_sec():.1f}s / {PID_DROP_TIMEOUT:.1f}s. Heater ON.")
                            heater_should_be_on = True
                else:
                    # Temp is within or above band, reset timer and do normal PID
                    pid_below_band_start_time[zone_id] = None
                    if temp < setpoint:
                        heater_should_be_on = True
                        # rospy.loginfo_throttle(5, f"DEBUG({zone_id}): State=PID, Temp={temp:.1f} < Setpoint={setpoint:.1f}. Heater SHOULD BE ON")
                    else:
                        heater_should_be_on = False
                        # rospy.loginfo_throttle(5, f"DEBUG({zone_id}): State=PID, Temp={temp:.1f} >= Setpoint={setpoint:.1f}. Heater SHOULD BE OFF")
            else:
                 rospy.logwarn_throttle(5, f"WARN({zone_id}): State=PID, Temp is NaN. Forcing Heater SHOULD BE OFF.")
                 heater_should_be_on = False
                 pid_below_band_start_time[zone_id] = None

        # --- Re-enable GPIO control ---
        control_heater(zone_id, heater_should_be_on)

        # --- Publish State ---
        current_state_for_pub = actual_states.get(zone_id, "OFF")
        if zone_id in state_cmd_pubs:
            if previous_actual_state != current_state_for_pub:
                rospy.loginfo(f"INFO({zone_id}): Actual state changed {previous_actual_state} -> {current_state_for_pub}. Publishing to state_cmd.")
            # Always publish for reliability
            state_cmd_pubs[zone_id].publish(String(current_state_for_pub))


def main_heater_control():
    global state_cmd_pubs
    try:
        rospy.init_node('heater_control_node', anonymous=True)
        rospy.loginfo("ROS node initialized.")
    except Exception as init_e:
        rospy.logfatal(f"Failed to initialize ROS node: {init_e}")
        sys.exit(1)

    # --- Re-enable GPIO Setup Call ---
    try:
        setup_gpio()
    except Exception as gpio_e:
        # Log error, but decide if it's fatal
        rospy.logerr(f"HeaterControl: CRITICAL - Failed during initial GPIO setup: {gpio_e}. Exiting.")
        # If GPIO is essential, exit the node
        sys.exit(1) # Make GPIO setup mandatory

    rospy.loginfo("Setting up publishers and subscribers...")
    # ... (Pub/Sub setup remains the same) ...
    for i in range(ZONE_COUNT):
        zone_id = f"zone{i+1}"
        try:
            state_cmd_pubs[zone_id] = rospy.Publisher(f'/extruder/{zone_id}/state_cmd', String, queue_size=10, latch=True)
            rospy.Subscriber(f'/extruder/{zone_id}/setpoint', Float32, create_callback(zone_id, current_setpoints, Float32))
            rospy.Subscriber(f'/extruder/{zone_id}/temperature', Float32, create_callback(zone_id, current_temps, Float32))
            rospy.Subscriber(f'/extruder/{zone_id}/state_cmd', String, create_callback(zone_id, {}, String))
            state_cmd_pubs[zone_id].publish(String(actual_states.get(zone_id, "OFF")))
        except Exception as pubsub_e:
            rospy.logerr(f"HeaterControl({zone_id}): Failed during Pub/Sub setup: {pubsub_e}")


    rospy.loginfo("Pub/Sub setup complete.")
    rate = rospy.Rate(1)
    rospy.loginfo("Heater Control Node Started (GPIO Enabled). Entering main control loop...")

    loop_counter = 0
    while not rospy.is_shutdown():
        loop_counter += 1
        try:
            control_loop()
        except Exception as loop_e:
             rospy.logerr(f"HeaterControl: Error in control loop: {loop_e}")
        rate.sleep()

if __name__ == '__main__':
    try:
        main_heater_control()
    except rospy.ROSInterruptException:
        print("Heater Control Node shutting down (ROSInterrupt).", flush=True)
    except Exception as main_e:
        print(f"FATAL: Heater Control Node Crashed: {main_e}", file=sys.stderr, flush=True)
    finally:
        # --- Re-enable GPIO Cleanup Call ---
        try:
             # Check if GPIO module was successfully imported and setup ran
             if 'GPIO' in sys.modules:
                 # Check if any pins were actually configured to avoid errors if HEATER_PINS was empty/None
                 # A simple check if the module object exists might suffice if setup is guaranteed to run if module loads
                 print("DEBUG: Cleaning up GPIO...", flush=True)
                 GPIO.cleanup()
                 print("DEBUG: GPIO cleanup complete.", flush=True)
             else:
                 print("DEBUG: GPIO module not imported or setup failed, skipping cleanup.", flush=True)
        except NameError:
             print("DEBUG: GPIO object not defined, skipping cleanup.", flush=True)
        except Exception as cleanup_e:
             print(f"ERROR: Error during GPIO cleanup: {cleanup_e}", file=sys.stderr, flush=True)

# --- Add top-level exception handler ---
except Exception as top_level_e:
    print(f"FATAL: Uncaught exception at top level: {top_level_e}", file=sys.stderr, flush=True)
    sys.exit(1) # Ensure non-zero exit code on crash
