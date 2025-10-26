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
    #     # ... (rest of function commented out)

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
                     # Use rospy logging now that node should be initialized
                     rospy.logwarn(f"HeaterControl({zone_id}): Received invalid Float32 data: {msg.data}")
            elif data_type == String:
                 if isinstance(msg, String) and msg.data in ["OFF", "HEATING"]:
                     value = msg.data
                     valid = True
                 else:
                      rospy.logwarn(f"HeaterControl({zone_id}): Received invalid String data: {msg.data}")

            if valid:
                current_value = data_dict.get(zone_id)
                if current_value != value:
                     if data_type == String:
                         rospy.loginfo(f"HeaterControl({zone_id}): Received State Command: {value}")
                     elif zone_id in current_setpoints and data_dict == current_setpoints:
                         rospy.loginfo(f"HeaterControl({zone_id}): Received new setpoint: {value:.1f} C")

                     data_dict[zone_id] = value # Update the dictionary

                     if data_type == String:
                         current_actual_state = actual_states.get(zone_id)
                         if value == "OFF" and current_actual_state != "OFF":
                             rospy.loginfo(f"HeaterControl({zone_id}): State forced to OFF by command.")
                             actual_states[zone_id] = "OFF"
                             # --- GPIO call commented out ---
                             if zone_id in actual_state_pubs:
                                 # Publish the state change immediately
                                 actual_state_pubs[zone_id].publish(actual_states[zone_id])
                         elif value == "HEATING" and current_actual_state == "OFF":
                             # Check safety conditions before forcing HEATING
                             temp_ok = not math.isnan(current_temps.get(zone_id, float('nan')))
                             setpoint_ok = current_setpoints.get(zone_id, 0.0) > 0
                             if temp_ok and setpoint_ok:
                                 rospy.loginfo(f"HeaterControl({zone_id}): State forced to HEATING by command.")
                                 actual_states[zone_id] = "HEATING"
                                 if zone_id in actual_state_pubs:
                                     # Publish the state change immediately
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
        # Use rospy logging now
        # rospy.logdebug("DEBUG: control_loop called") # Too verbose

        for i in range(ZONE_COUNT):
            zone_id = f"zone{i+1}"
            temp = current_temps.get(zone_id, float('nan'))
            setpoint = current_setpoints.get(zone_id, 0.0)
            commanded = commanded_states.get(zone_id, "OFF")
            current_actual = actual_states.get(zone_id, "OFF")
            previous_actual_state = current_actual # Store state at loop start

            rospy.loginfo_throttle(5, f"LOOP_START({zone_id}): Temp={temp:.1f}, Setpoint={setpoint:.1f}, Commanded='{commanded}', Actual='{current_actual}'")

            heater_should_be_on = False

            # --- State Machine ---
            if math.isnan(temp) or setpoint <= 0:
                if current_actual != "OFF":
                    rospy.logwarn(f"HeaterControl({zone_id}): Sensor/Setpoint invalid. Forcing state to OFF.")
                    actual_states[zone_id] = "OFF" # Update global immediately
                heater_should_be_on = False

            elif current_actual == "OFF":
                heater_should_be_on = False

            elif current_actual == "HEATING":
                heater_should_be_on = True
                lower_band = setpoint - HYSTERESIS
                comparison_result = False
                rospy.loginfo_throttle(5, f"DEBUG({zone_id}): In HEATING state. Temp={temp:.1f}, Setpoint={setpoint:.1f}, LowerBand={lower_band:.1f}")
                try:
                     if not math.isnan(temp):
                         temp_float = float(temp)
                         lower_band_float = float(lower_band)
                         comparison_result = temp_float >= lower_band_float
                         rospy.loginfo_throttle(5, f"DEBUG({zone_id}): Comparison: {temp_float:.1f} >= {lower_band_float:.1f} is {comparison_result}")
                     else:
                         rospy.loginfo_throttle(5, f"DEBUG({zone_id}): Temp is NaN, cannot compare.")
                except (TypeError, ValueError) as e:
                     rospy.logerr(f"ERROR({zone_id}): Error during comparison: Temp={temp}, LowerBand={lower_band}, Error={e}")

                if not math.isnan(temp) and comparison_result:
                    actual_states[zone_id] = "PID" # Update global immediately
                    rospy.loginfo(f"INFO({zone_id}): Temp {temp:.1f}C in band. Transitioning HEATING -> PID")

            elif current_actual == "PID":
                if not math.isnan(temp):
                    if temp < setpoint:
                        heater_should_be_on = True
                        rospy.loginfo_throttle(5, f"DEBUG({zone_id}): State=PID, Temp={temp:.1f} < Setpoint={setpoint:.1f}. Heater SHOULD BE ON")
                    else:
                        heater_should_be_on = False
                        rospy.loginfo_throttle(5, f"DEBUG({zone_id}): State=PID, Temp={temp:.1f} >= Setpoint={setpoint:.1f}. Heater SHOULD BE OFF")
                else:
                     rospy.logwarn_throttle(5, f"WARN({zone_id}): State=PID, Temp is NaN. Forcing Heater SHOULD BE OFF.")
                     heater_should_be_on = False

            # Use rospy logging
            rospy.loginfo_throttle(5, f"LOGIC({zone_id}): State='{actual_states.get(zone_id, 'ERR')}': Heater SHOULD BE {'ON' if heater_should_be_on else 'OFF'}")


            # --- FIX: Always publish the current actual state ---
            current_state_for_pub = actual_states.get(zone_id, "OFF")
            if zone_id in actual_state_pubs:
                actual_state_pubs[zone_id].publish(current_state_for_pub)
                # Log if state changed FOR DEBUGGING
                if previous_actual_state != current_state_for_pub:
                     rospy.loginfo(f"INFO({zone_id}): Actual state changed {previous_actual_state} -> {current_state_for_pub} (Published)")
            # ----------------------------------------------------


    def main_heater_control():
        # Use rospy logging now
        # print("DEBUG: Entering main_heater_control function...", flush=True)
        global actual_state_pubs
        try:
            # print("DEBUG: Attempting rospy.init_node...", flush=True)
            rospy.init_node('heater_control_node', anonymous=True)
            # print("DEBUG: ROS node initialized.", flush=True)
            rospy.loginfo("ROS node initialized.") # Use rospy log
        except Exception as init_e:
            # Use rospy logfatal if possible, else print
            try:
                rospy.logfatal(f"Failed to initialize ROS node: {init_e}")
            except:
                 print(f"FATAL: Failed to initialize ROS node: {init_e}", file=sys.stderr, flush=True)
            sys.exit(1)

        # --- GPIO Setup Call Commented Out ---

        # print("DEBUG: Setting up publishers and subscribers...", flush=True)
        rospy.loginfo("Setting up publishers and subscribers...")
        for i in range(ZONE_COUNT):
            zone_id = f"zone{i+1}"
            try:
                actual_state_pubs[zone_id] = rospy.Publisher(f'/extruder/{zone_id}/actual_state', String, queue_size=10, latch=True)
                rospy.Subscriber(f'/extruder/{zone_id}/setpoint', Float32, create_callback(zone_id, current_setpoints, Float32))
                rospy.Subscriber(f'/extruder/{zone_id}/temperature', Float32, create_callback(zone_id, current_temps, Float32))
                rospy.Subscriber(f'/extruder/{zone_id}/state_cmd', String, create_callback(zone_id, commanded_states, String))
                # Publish initial state safely
                actual_state_pubs[zone_id].publish(actual_states.get(zone_id, "OFF"))
            except Exception as pubsub_e:
                rospy.logerr(f"HeaterControl({zone_id}): Failed during Pub/Sub setup: {pubsub_e}")

        # print("DEBUG: Pub/Sub setup complete.", flush=True)
        rospy.loginfo("Pub/Sub setup complete.")

        rate = rospy.Rate(1)
        # print("DEBUG: Heater Control Node Started (No GPIO). Entering main control loop...", flush=True)
        rospy.loginfo("Heater Control Node Started (No GPIO). Entering main control loop...")

        # print("DEBUG: Main while loop starting...", flush=True) # Check if the loop starts
        loop_counter = 0
        while not rospy.is_shutdown():
            loop_counter += 1
            # rospy.logdebug(f"DEBUG: Main while loop iteration {loop_counter}") # Too verbose
            try:
                control_loop()
            except Exception as loop_e:
                 rospy.logerr(f"HeaterControl: Error in control loop: {loop_e}")
            rate.sleep()

    if __name__ == '__main__':
        try:
            # print("DEBUG: Script __main__ block executing...", flush=True)
            main_heater_control()
        except rospy.ROSInterruptException:
            # Use print for shutdown messages as rospy might be gone
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
