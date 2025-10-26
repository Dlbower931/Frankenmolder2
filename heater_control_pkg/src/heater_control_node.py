#!/usr/bin/env python3

# --- ABSOLUTE FIRST LINE DEBUG ---
# print("DEBUG: heater_control_node.py EXECUTION STARTED.", flush=True)
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
        # print("DEBUG: heater_control_node.py imports successful (GPIO skipped).", flush=True)
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
    # commanded_states = {f"zone{i+1}": "OFF" for i in range(ZONE_COUNT)} # No longer primary input, just log received
    actual_states = {f"zone{i+1}": "OFF" for i in range(ZONE_COUNT)} # This node determines the actual state
    # --- GPIO state tracking commented out ---
    # heater_pin_states = {f"zone{i+1}": False for i in range(ZONE_COUNT)} # Tracks physical output

    # --- Publishers ---
    # --- CHANGE: Publish back to state_cmd ---
    state_cmd_pubs = {}
    # actual_state_pubs = {} # Removed

    # --- GPIO Setup (Commented Out) ---
    # def setup_gpio(): ...

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
            elif data_type == String: # Callback for state_cmd topic
                 if isinstance(msg, String) and msg.data in ["OFF", "HEATING", "PID"]: # Allow PID to be received but maybe ignored
                     value = msg.data
                     valid = True
                 else:
                      rospy.logwarn(f"HeaterControl({zone_id}): Received invalid String data: {msg.data}")

            if valid:
                current_value = data_dict.get(zone_id)
                if current_value != value:
                     # --- MODIFIED: Handle state_cmd callback ---
                     if data_type == String: # This is the state_cmd callback
                         rospy.loginfo(f"HeaterControl({zone_id}): Received State Command Request: {value}")
                         # Logic to potentially override actual_state based on GUI command
                         if value == "OFF" and actual_states.get(zone_id) != "OFF":
                             rospy.loginfo(f"HeaterControl({zone_id}): Forcing state to OFF due to external command.")
                             actual_states[zone_id] = "OFF"
                             # Publish the change back immediately
                             if zone_id in state_cmd_pubs:
                                 state_cmd_pubs[zone_id].publish(String(actual_states[zone_id]))
                         elif value == "HEATING" and actual_states.get(zone_id) == "OFF":
                             # Check safety conditions before allowing HEATING state
                             temp_ok = not math.isnan(current_temps.get(zone_id, float('nan')))
                             setpoint_ok = current_setpoints.get(zone_id, 0.0) > 0
                             if temp_ok and setpoint_ok:
                                 rospy.loginfo(f"HeaterControl({zone_id}): Accepting HEATING command, starting heat cycle.")
                                 actual_states[zone_id] = "HEATING"
                                 if zone_id in state_cmd_pubs:
                                     state_cmd_pubs[zone_id].publish(String(actual_states[zone_id]))
                             else:
                                 rospy.logwarn(f"HeaterControl({zone_id}): Ignoring HEATING command, conditions not met (Temp={current_temps.get(zone_id, 'NaN')}, Setpoint={current_setpoints.get(zone_id, 0.0)}). Staying OFF.")
                                 # Optionally publish OFF state back if ignoring HEATING
                                 if zone_id in state_cmd_pubs:
                                     state_cmd_pubs[zone_id].publish(String("OFF"))

                     # --- Handle Float32 (Setpoint/Temperature) ---
                     elif data_type == Float32:
                         if zone_id in current_setpoints and data_dict == current_setpoints:
                             rospy.loginfo(f"HeaterControl({zone_id}): Received new setpoint: {value:.1f} C")
                         # Update the dictionary (happens for temp too, but no specific log needed)
                         data_dict[zone_id] = value

        return callback


    # --- Control & GPIO Logic (GPIO Part Commented Out) ---
    # def control_heater(zone_id, turn_on): ...

    def control_loop():
        """Runs the state machine logic for all zones."""
        global actual_states

        for i in range(ZONE_COUNT):
            zone_id = f"zone{i+1}"
            temp = current_temps.get(zone_id, float('nan'))
            setpoint = current_setpoints.get(zone_id, 0.0)
            # commanded = commanded_states.get(zone_id, "OFF") # No longer the primary driver
            current_actual = actual_states.get(zone_id, "OFF")
            previous_actual_state = current_actual # Store state at loop start

            rospy.loginfo_throttle(5, f"LOOP_START({zone_id}): Temp={temp:.1f}, Setpoint={setpoint:.1f}, Actual='{current_actual}'") # Removed Commanded log

            heater_should_be_on = False

            # --- State Machine (Driven by actual_states) ---
            if math.isnan(temp) or setpoint <= 0:
                if current_actual != "OFF":
                    rospy.logwarn(f"HeaterControl({zone_id}): Sensor/Setpoint invalid. Forcing state to OFF.")
                    actual_states[zone_id] = "OFF"
                heater_should_be_on = False

            elif current_actual == "OFF":
                heater_should_be_on = False
                # Transition to HEATING ONLY happens in callback now based on command

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
                # OFF transition handled in callback

            elif current_actual == "PID":
                 # Check if commanded OFF (still allow external OFF command)
                # Note: commanded_states is updated by the callback
                # if commanded_states.get(zone_id) == "OFF": # Check the *received* command
                #      actual_states[zone_id] = "OFF"
                #      rospy.loginfo(f"HeaterControl({zone_id}): Transitioning PID -> OFF (commanded)")
                # else: # If not commanded OFF, continue PID logic
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
                 # OFF transition handled in callback

            # Use rospy logging
            rospy.loginfo_throttle(5, f"LOGIC({zone_id}): State='{actual_states.get(zone_id, 'ERR')}': Heater SHOULD BE {'ON' if heater_should_be_on else 'OFF'}")


            # --- CHANGE: Publish the determined state back to state_cmd topic if changed ---
            current_state_for_pub = actual_states.get(zone_id, "OFF")
            if previous_actual_state != current_state_for_pub:
                rospy.loginfo(f"INFO({zone_id}): Actual state changed {previous_actual_state} -> {current_state_for_pub}. Publishing to state_cmd.")
                if zone_id in state_cmd_pubs: # Use the new publisher dict
                    state_cmd_pubs[zone_id].publish(String(current_state_for_pub))
            # -----------------------------------------------------------------------------


    def main_heater_control():
        # print("DEBUG: Entering main_heater_control function...", flush=True)
        # --- CHANGE: Use state_cmd_pubs ---
        global state_cmd_pubs
        # -------------------------------
        try:
            rospy.init_node('heater_control_node', anonymous=True)
            rospy.loginfo("ROS node initialized.")
        except Exception as init_e:
            try:
                rospy.logfatal(f"Failed to initialize ROS node: {init_e}")
            except:
                 print(f"FATAL: Failed to initialize ROS node: {init_e}", file=sys.stderr, flush=True)
            sys.exit(1)

        # --- GPIO Setup Call Commented Out ---

        rospy.loginfo("Setting up publishers and subscribers...")
        for i in range(ZONE_COUNT):
            zone_id = f"zone{i+1}"
            try:
                # --- CHANGE: Publish state_cmd ---
                state_cmd_pubs[zone_id] = rospy.Publisher(f'/extruder/{zone_id}/state_cmd', String, queue_size=10, latch=True)
                # actual_state_pubs[zone_id] = rospy.Publisher(...) # Removed
                # --------------------------------

                # Subscribers (Setpoint, Temperature, and incoming State Command)
                rospy.Subscriber(f'/extruder/{zone_id}/setpoint', Float32, create_callback(zone_id, current_setpoints, Float32))
                rospy.Subscriber(f'/extruder/{zone_id}/temperature', Float32, create_callback(zone_id, current_temps, Float32))
                rospy.Subscriber(f'/extruder/{zone_id}/state_cmd', String, create_callback(zone_id, {}, String)) # Pass empty dict, callback handles logic now

                # Publish initial state to state_cmd
                state_cmd_pubs[zone_id].publish(String(actual_states.get(zone_id, "OFF")))

            except Exception as pubsub_e:
                rospy.logerr(f"HeaterControl({zone_id}): Failed during Pub/Sub setup: {pubsub_e}")

        rospy.loginfo("Pub/Sub setup complete.")
        rate = rospy.Rate(1)
        rospy.loginfo("Heater Control Node Started (No GPIO). Entering main control loop...")

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
            # --- GPIO Cleanup Call Commented Out ---
            pass

# --- Add top-level exception handler ---
except Exception as top_level_e:
    print(f"FATAL: Uncaught exception at top level: {top_level_e}", file=sys.stderr, flush=True)
    sys.exit(1) # Ensure non-zero exit code on crash