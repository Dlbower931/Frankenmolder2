#!/usr/bin/env python3

import sys
try:
    import rospy
    from std_msgs.msg import Float32, String
    import time
    import math
    import RPi.GPIO as GPIO
    print("DEBUG: heater_control_node.py imports successful (GPIO Enabled).", flush=True)
except Exception as import_e:
    print(f"FATAL: Import error: {import_e}", file=sys.stderr, flush=True)
    sys.exit(1)


# --- Configuration ---
ZONE_COUNT = 3
HYSTERESIS = 2.0 # Temperature band for PID state transition
PID_DROP_TIMEOUT = 5.0 # Seconds below band before reverting to HEATING

HEATER_PINS = {
    "zone1": 17, # Example BCM pin number for Zone 1 heater
    "zone2": 27, # Example BCM pin number for Zone 2 heater
    "zone3": 22, # Example BCM pin number for Zone 3 heater
}

# --- Global State Variables ---
current_setpoints = {f"zone{i+1}": 0.0 for i in range(ZONE_COUNT)}
current_temps = {f"zone{i+1}": float('nan') for i in range(ZONE_COUNT)}
actual_states = {f"zone{i+1}": "OFF" for i in range(ZONE_COUNT)}
pid_below_band_start_time = {f"zone{i+1}": None for i in range(ZONE_COUNT)}
heater_pin_states = {f"zone{i+1}": False for i in range(ZONE_COUNT)}

# --- Publishers ---
state_cmd_pubs = {}

# --- GPIO Setup ---
def setup_gpio():
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        for zone_id, pin in HEATER_PINS.items():
            if pin is not None:
                GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
                heater_pin_states[zone_id] = False
        rospy.loginfo("GPIO pins configured for heaters.")
    except Exception as e:
         rospy.logerr(f"HeaterControl: Error during GPIO setup: {e}. Check permissions/hardware.")
         raise e # Re-raise if GPIO is critical


# --- ROS Callbacks ---
def create_callback(zone_id, data_dict, data_type):
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
                             # --- FIX: DO NOT PUBLISH "OFF" HERE ---
                             # if zone_id in state_cmd_pubs:
                             #     state_cmd_pubs[zone_id].publish(String("OFF"))

                 elif data_type == Float32:
                     if zone_id in current_setpoints and data_dict == current_setpoints:
                         rospy.loginfo(f"HeaterControl({zone_id}): Received new setpoint: {value:.1f} C")
                         if actual_states.get(zone_id) == "PID":
                             pid_below_band_start_time[zone_id] = None
                     data_dict[zone_id] = value

    return callback


# --- Control & GPIO Logic ---
def control_heater(zone_id, turn_on):
    global heater_pin_states
    pin = HEATER_PINS.get(zone_id)
    if pin is None:
        rospy.logwarn_once(f"HeaterControl({zone_id}): No GPIO pin defined in HEATER_PINS.")
        return

    desired_gpio_state = GPIO.HIGH if turn_on else GPIO.LOW
    current_pin_state = heater_pin_states.get(zone_id)
    if current_pin_state is None or current_pin_state != turn_on:
        try:
            GPIO.output(pin, desired_gpio_state)
            heater_pin_states[zone_id] = turn_on
            rospy.loginfo(f"HeaterControl({zone_id}): Heater GPIO {pin} set to {'ON' if turn_on else 'OFF'}")
        except RuntimeError as e:
            rospy.logerr(f"HeaterControl({zone_id}): GPIO Runtime Error setting pin {pin}: {e}")
        except Exception as e:
            rospy.logerr(f"HeaterControl({zone_id}): Failed to set GPIO {pin}: {e}")

def control_loop():
    global actual_states, pid_below_band_start_time

    for i in range(ZONE_COUNT):
        zone_id = f"zone{i+1}"
        temp = current_temps.get(zone_id, float('nan'))
        setpoint = current_setpoints.get(zone_id, 0.0)
        current_actual = actual_states.get(zone_id, "OFF")
        previous_actual_state = current_actual

        rospy.loginfo_throttle(10, f"LOOP_START({zone_id}): Temp={temp:.1f}, Setpoint={setpoint:.1f}, Actual='{current_actual}'")

        heater_should_be_on = False

        # --- State Machine (Driven by actual_states) ---
        # --- FIX: Modified Safety Check ---
        if math.isnan(temp) or setpoint <= 0:
            if current_actual != "OFF":
                rospy.logwarn(f"HeaterControl({zone_id}): Sensor/Setpoint invalid. Forcing HEATER OFF (state remains {current_actual}).")
                # actual_states[zone_id] = "OFF" # <-- DO NOT FORCE STATE CHANGE
                pid_below_band_start_time[zone_id] = None
            heater_should_be_on = False # Force heater OFF, but leave state as is

        elif current_actual == "OFF":
            heater_should_be_on = False
            pid_below_band_start_time[zone_id] = None

        elif current_actual == "HEATING":
            heater_should_be_on = True
            lower_band = setpoint - HYSTERESIS
            comparison_result = False
            pid_below_band_start_time[zone_id] = None
            try:
                 if not math.isnan(temp):
                     temp_float = float(temp)
                     lower_band_float = float(lower_band)
                     comparison_result = temp_float >= lower_band_float
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
                        heater_should_be_on = True
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
                    pid_below_band_start_time[zone_id] = None
                    if temp < setpoint:
                        heater_should_be_on = True
                    else:
                        heater_should_be_on = False
            else:
                 rospy.logwarn_throttle(5, f"WARN({zone_id}): State=PID, Temp is NaN. Forcing Heater SHOULD BE OFF.")
                 heater_should_be_on = False
                 pid_below_band_start_time[zone_id] = None

        control_heater(zone_id, heater_should_be_on)

        # --- Publish State ---
        current_state_for_pub = actual_states.get(zone_id, "OFF")
        if zone_id in state_cmd_pubs:
            if previous_actual_state != current_state_for_pub:
                rospy.loginfo(f"INFO({zone_id}): Actual state changed {previous_actual_state} -> {current_state_for_pub}. Publishing to state_cmd.")
            state_cmd_pubs[zone_id].publish(String(current_state_for_pub))


def main_heater_control():
    global state_cmd_pubs
    try:
        rospy.init_node('heater_control_node', anonymous=True)
        rospy.loginfo("ROS node initialized.")
    except Exception as init_e:
        rospy.logfatal(f"Failed to initialize ROS node: {init_e}")
        sys.exit(1)

    try:
        setup_gpio()
    except Exception as gpio_e:
        rospy.logerr(f"HeaterControl: CRITICAL - Failed during initial GPIO setup: {gpio_e}. Exiting.")
        sys.exit(1)

    rospy.loginfo("Setting up publishers and subscribers...")
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
    finally:
        try:
             if 'GPIO' in sys.modules:
                 print("DEBUG: Cleaning up GPIO...", flush=True)
                 GPIO.cleanup()
                 print("DEBUG: GPIO cleanup complete.", flush=True)
             else:
                 print("DEBUG: GPIO module not imported or setup failed, skipping cleanup.", flush=True)
        except NameError:
             print("DEBUG: GPIO object not defined, skipping cleanup.", flush=True)