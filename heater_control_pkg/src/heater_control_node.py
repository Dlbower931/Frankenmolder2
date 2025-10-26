#!/usr/bin/env python3

# --- ABSOLUTE FIRST LINE DEBUG ---
print("DEBUG: heater_control_node.py EXECUTION STARTED.", flush=True)
import sys # For stderr

# --- Wrap EVERYTHING in a try/except ---
try:
    # --- VERY EARLY DEBUG PRINT ---
    # print("DEBUG: heater_control_node.py starting execution...") # Redundant now

    try:
        import rospy
        from std_msgs.msg import Float32, String
        import time
        import math
        # --- Temporarily comment out GPIO import ---
        # import RPi.GPIO as GPIO # Library for direct GPIO control
        # --- VERY EARLY DEBUG PRINT ---
        print("DEBUG: heater_control_node.py imports successful (GPIO skipped).", flush=True)
    except Exception as import_e:
        # --- VERY EARLY DEBUG PRINT ---
        print(f"FATAL: Import error: {import_e}", file=sys.stderr, flush=True)
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
                # Use .get for safety, though dicts should be initialized
                current_value = data_dict.get(zone_id)
                if current_value != value:
                     if data_type == String:
                         rospy.loginfo(f"HeaterControl({zone_id}): Received State Command: {value}")
                     elif zone_id in current_setpoints and data_dict == current_setpoints:
                         rospy.loginfo(f"HeaterControl({zone_id}): Received new setpoint: {value:.1f} C")

                     data_dict[zone_id] = value # Update the dictionary

                     if data_type == String:
                         if value == "OFF" and actual_states.get(zone_id) != "OFF":
                             rospy.loginfo(f"HeaterControl({zone_id}): State forced to OFF by command.")
                             actual_states[zone_id] = "OFF"
                             # --- GPIO call commented out ---
                             # control_heater(zone_id, False)
                             if zone_id in actual_state_pubs:
                                 actual_state_pubs[zone_id].publish(actual_states[zone_id])
                         elif value == "HEATING" and actual_states.get(zone_id) == "OFF":
                             # Only transition to HEATING if safe (temp received, setpoint > 0)
                             if not math.isnan(current_temps.get(zone_id, float('nan'))) and current_setpoints.get(zone_id, 0.0) > 0:
                                 rospy.loginfo(f"HeaterControl({zone_id}): State forced to HEATING by command.")
                                 actual_states[zone_id] = "HEATING"
                                 if zone_id in actual_state_pubs:
                                     actual_state_pubs[zone_id].publish(actual_states[zone_id])
                             else:
                                 rospy.logwarn(f"HeaterControl({zone_id}): Received HEATING command but conditions not met (Temp={current_temps.get(zone_id, 'NaN')}, Setpoint={current_setpoints.get(zone_id, 0.0)}). Staying OFF.")
                                 commanded_states[zone_id] = "OFF" # Reflect ignored command

        return callback


    # --- Control & GPIO Logic (GPIO Part Commented Out) ---
    # def control_heater(zone_id, turn_on): ...

    def control_loop():
        """Runs the state machine logic for all zones."""
        global actual_states
        # --- ADDED PRINT ---
        print("DEBUG: control_loop called", flush=True)

        for i in range(ZONE_COUNT):
            zone_id = f"zone{i+1}"
            # Use copies for logic
            temp = current_temps.get(zone_id, float('nan'))
            setpoint = current_setpoints.get(zone_id, 0.0)
            commanded = commanded_states.get(zone_id, "OFF")
            current_actual = actual_states.get(zone_id, "OFF")
            previous_actual_state = current_actual

            # --- Use print instead of rospy.loginfo_throttle for initial loop check ---
            print(f"LOOP_START({zone_id}): Temp={temp:.1f}, Setpoint={setpoint:.1f}, Commanded='{commanded}', Actual='{current_actual}'", flush=True)

            heater_should_be_on = False

            # --- State Machine ---
            if math.isnan(temp) or setpoint <= 0:
                if current_actual != "OFF":
                    print(f"WARN({zone_id}): Sensor/Setpoint invalid. Forcing state to OFF.", flush=True) # Use print
                    actual_states[zone_id] = "OFF"
                heater_should_be_on = False

            elif current_actual == "OFF":
                heater_should_be_on = False

            elif current_actual == "HEATING":
                heater_should_be_on = True
                lower_band = setpoint - HYSTERESIS
                comparison_result = False # Default
                # --- Use print for core logic debug ---
                print(f"DEBUG({zone_id}): In HEATING state. Temp={temp:.1f}, Setpoint={setpoint:.1f}, LowerBand={lower_band:.1f}", flush=True)
                try:
                     if not math.isnan(temp): # Check before casting
                         temp_float = float(temp)
                         lower_band_float = float(lower_band)
                         comparison_result = temp_float >= lower_band_float
                         print(f"DEBUG({zone_id}): Comparison: {temp_float:.1f} >= {lower_band_float:.1f} is {comparison_result}", flush=True)
                     else:
                         print(f"DEBUG({zone_id}): Temp is NaN, cannot compare.", flush=True)
                except (TypeError, ValueError) as e:
                     print(f"ERROR({zone_id}): Error during comparison: Temp={temp}, LowerBand={lower_band}, Error={e}", file=sys.stderr, flush=True)

                if not math.isnan(temp) and comparison_result:
                    actual_states[zone_id] = "PID"
                    print(f"INFO({zone_id}): Temp {temp:.1f}C in band. Transitioning HEATING -> PID", flush=True) # Use print

            elif current_actual == "PID":
                if not math.isnan(temp):
                    if temp < setpoint:
                        heater_should_be_on = True
                        print(f"DEBUG({zone_id}): State=PID, Temp={temp:.1f} < Setpoint={setpoint:.1f}. Heater SHOULD BE ON", flush=True) # Use print
                    else:
                        heater_should_be_on = False
                        print(f"DEBUG({zone_id}): State=PID, Temp={temp:.1f} >= Setpoint={setpoint:.1f}. Heater SHOULD BE OFF", flush=True) # Use print
                else:
                     print(f"WARN({zone_id}): State=PID, Temp is NaN. Forcing Heater SHOULD BE OFF.", flush=True) # Use print
                     heater_should_be_on = False

            print(f"LOGIC({zone_id}): State='{actual_states.get(zone_id, 'ERR')}': Heater SHOULD BE {'ON' if heater_should_be_on else 'OFF'}", flush=True) # Use print


            if previous_actual_state != actual_states.get(zone_id):
                current_state_for_pub = actual_states.get(zone_id, "OFF")
                print(f"INFO({zone_id}): Actual state changed {previous_actual_state} -> {current_state_for_pub}", flush=True) # Use print
                if zone_id in actual_state_pubs:
                    actual_state_pubs[zone_id].publish(current_state_for_pub)


    def main_heater_control():
        print("DEBUG: Entering main_heater_control function...", flush=True)
        global actual_state_pubs
        try:
            print("DEBUG: Attempting rospy.init_node...", flush=True)
            rospy.init_node('heater_control_node', anonymous=True)
            print("DEBUG: ROS node initialized.", flush=True)
        except Exception as init_e:
            print(f"FATAL: Failed to initialize ROS node: {init_e}", file=sys.stderr, flush=True)
            sys.exit(1)

        # --- GPIO Setup Call Commented Out ---

        print("DEBUG: Setting up publishers and subscribers...", flush=True)
        # Keep rospy log too
        rospy.loginfo("Setting up publishers and subscribers...")
        for i in range(ZONE_COUNT):
            zone_id = f"zone{i+1}"
            try:
                actual_state_pubs[zone_id] = rospy.Publisher(f'/extruder/{zone_id}/actual_state', String, queue_size=10, latch=True)
                rospy.Subscriber(f'/extruder/{zone_id}/setpoint', Float32, create_callback(zone_id, current_setpoints, Float32))
                rospy.Subscriber(f'/extruder/{zone_id}/temperature', Float32, create_callback(zone_id, current_temps, Float32))
                rospy.Subscriber(f'/extruder/{zone_id}/state_cmd', String, create_callback(zone_id, commanded_states, String))
                actual_state_pubs[zone_id].publish(actual_states.get(zone_id, "OFF"))
            except Exception as pubsub_e:
                print(f"ERROR: HeaterControl({zone_id}): Failed during Pub/Sub setup: {pubsub_e}", file=sys.stderr, flush=True)
                rospy.logerr(f"HeaterControl({zone_id}): Failed during Pub/Sub setup: {pubsub_e}")

        print("DEBUG: Pub/Sub setup complete.", flush=True)
        rospy.loginfo("Pub/Sub setup complete.") # Keep rospy log

        rate = rospy.Rate(1)
        print("DEBUG: Heater Control Node Started (No GPIO). Entering main control loop...", flush=True)
        rospy.loginfo("Heater Control Node Started (No GPIO). Entering main control loop...") # Keep rospy log

        # --- ADDED PRINT INSIDE LOOP ---
        loop_counter = 0
        while not rospy.is_shutdown():
            loop_counter += 1
            print(f"DEBUG: Main while loop iteration {loop_counter}", flush=True) # Check if the loop is running
            try:
                control_loop()
            except Exception as loop_e:
                 print(f"ERROR: HeaterControl: Error in control loop: {loop_e}", file=sys.stderr, flush=True)
                 rospy.logerr(f"HeaterControl: Error in control loop: {loop_e}")
            rate.sleep()

    if __name__ == '__main__':
        try:
            print("DEBUG: Script __main__ block executing...", flush=True)
            main_heater_control()
        except rospy.ROSInterruptException:
            print("Heater Control Node shutting down (ROSInterrupt).", flush=True)
        except Exception as main_e:
            print(f"FATAL: Heater Control Node Crashed: {main_e}", file=sys.stderr, flush=True)
        finally:
            # --- GPIO Cleanup Call Commented Out ---
            pass

# --- Add top-level exception handler ---
except Exception as top_level_e:
    print(f"FATAL: Uncaught exception at top level: {top_level_e}", file=sys.stderr, flush=True)
    sys.exit(1) # Ensure non-zero exit code on crash

