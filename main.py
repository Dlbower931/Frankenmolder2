import machine
import time
import sys
import select

# --- Servo Configuration ---
SERVO_PIN_NUM = 15  # GPIO pin connected to servo signal wire (e.g., GP15)
PWM_FREQ = 50       # Standard servo frequency (50 Hz = 20ms period)

# --- Duty Cycle Values for Positional Servo (0-180 degrees) ---
MIN_DUTY = 1640 # Approx 0.5ms pulse -> ~0 degrees
MAX_DUTY = 7860 # Approx 2.5ms pulse -> ~180 degrees
CENTER_DUTY = (MIN_DUTY + MAX_DUTY) // 2

# --- Sweep Speed Configuration ---
# Define the range for sweep delay based on RPM (1-100)
# Smaller delay = faster sweep
MIN_SWEEP_DELAY = 0.005 # Fastest delay (at RPM 100)
MAX_SWEEP_DELAY = 0.05  # Slowest delay (at RPM 1)
DEFAULT_RPM = 50.0 # Default speed if none set (now a float)

# --- Servo Initialization ---
try:
    servo_pin = machine.Pin(SERVO_PIN_NUM)
    pwm = machine.PWM(servo_pin)
    pwm.freq(PWM_FREQ)
    print(f"Positional Servo PWM initialized on GP{SERVO_PIN_NUM} at {PWM_FREQ}Hz.")
    # Start servo at 0 degrees
    pwm.duty_u16(MIN_DUTY)
except Exception as e:
    print(f"FATAL: Failed to initialize Servo PWM on GP{SERVO_PIN_NUM}: {e}")
    try: pwm.deinit()
    except: pass
    sys.exit(1)

# --- Serial Setup ---
poll_obj = select.poll()
poll_obj.register(sys.stdin, select.POLLIN)

# Updated print message for new command format
print(f"Pico serial servo sweep control ready. Waiting for commands (ON=Sweep, OFF=Stop, RPM1-100=Speed)...")

# --- State Variables ---
sweeping = False # Flag to control whether the servo should be sweeping
# --- Global variable for sweep delay, calculated from RPM ---
current_sweep_delay = MAX_SWEEP_DELAY # Start slow

# --- Helper Function for Angle to Duty ---
def angle_to_duty(angle):
# ... (rest of function) ...
[Immersive content redacted for brevity.]
    """ Converts angle (0-180) to PWM duty cycle (u16) """
    if angle < 0: angle = 0
    if angle > 180: angle = 180
    # Linear interpolation
    return int(MIN_DUTY + (MAX_DUTY - MIN_DUTY) * (angle / 180.0))

# --- Helper Function for RPM to Delay ---
def rpm_to_delay(rpm):
# ... (rest of function) ...
[Immersive content redacted for brevity.]
    """ Converts RPM (1.0-100.0) to sweep delay (seconds) """
    if rpm < 1.0: rpm = 1.0
    if rpm > 100.0: rpm = 100.0
    # Linear interpolation: RPM=1 -> MAX_DELAY, RPM=100 -> MIN_DELAY
    # Formula: D = MaxD - (MaxD - MinD) * (R - 1) / (100 - 1)
    delay = MAX_SWEEP_DELAY - (MAX_SWEEP_DELAY - MIN_SWEEP_DELAY) * (rpm - 1.0) / 99.0
    return delay

# --- Set Initial Delay ---
current_sweep_delay = rpm_to_delay(DEFAULT_RPM)
# ... (rest of function) ...
[Immersive content redacted for brevity.]
print(f"Initial sweep delay set for RPM {DEFAULT_RPM}: {current_sweep_delay:.4f}s")


# --- Main Loop ---
current_angle = 0
# ... (rest of main loop setup) ...
[Immersive content redacted for brevity.]
sweep_direction = 1 # 1 for increasing angle, -1 for decreasing

while True:
    # --- Check for Serial Commands ---
# ... (rest of loop) ...
[Immersive content redacted for brevity.]
    if poll_obj.poll(0):
        try:
            cmd_raw = sys.stdin.readline()
            if cmd_raw:
                cmd_str = cmd_raw.strip()
                print(f"Received command string: {repr(cmd_str)}") # Debugging

                if cmd_str == "ON":
                    if not sweeping:
                        print("Command ON: Starting sweep.")
                        sweeping = True
                elif cmd_str == "OFF":
                    if sweeping:
                        print("Command OFF: Stopping sweep.")
                        sweeping = False
                # --- Updated RPM Logic ---
                # Check for prefix "RPM" (3 chars)
                elif cmd_str.startswith("RPM"):
                    rpm_val_str = "" # Initialize for error logging
                    try:
                        # --- FIX: Use split() and strip() for robust parsing ---
                        # This handles "RPM10.0" or "RPM 10.0"
                        parts = cmd_str.split("RPM", 1) # Split only on the first "RPM"
                        if len(parts) > 1:
                            rpm_val_str = parts[1].strip() # Get the value part, remove spaces
                        else:
                            # This case should be rare if startswith("RPM") is true
                            rpm_val_str = cmd_str[3:] # Fallback to slicing
                        # ----------------------------------------------------
                        
                        # --- Convert to float ---
                        target_rpm = float(rpm_val_str) 
                        
                        if 1.0 <= target_rpm <= 100.0:
                             # Calculate and update the global delay variable
                             new_delay = rpm_to_delay(target_rpm)
                             current_sweep_delay = new_delay
                             print(f"Received RPM {target_rpm:.2f}. Setting sweep delay to {current_sweep_delay:.4f}s")
                        else:
                             print(f"Invalid RPM value: {target_rpm:.2f} (must be 1.0-100.0)")
                    except ValueError:
                        # --- ADDED: Show the representation of the string that failed ---
                        print(f"Invalid RPM float value: Failed converting {repr(rpm_val_str)} to float.")
                    except Exception as e_rpm:
                         print(f"Error processing RPM command: {e_rpm}")
                # --- End Updated RPM Logic ---
                else:
                    # Interpret unknown commands as angle settings (0-180)
# ... (rest of loop) ...
[Immersive content redacted for brevity.]
                    try:
                        # Try to convert to int first for angle
                        target_angle = int(cmd_str)
                        if 0 <= target_angle <= 180:
                            print(f"Setting servo angle directly: {target_angle} degrees")
                            sweeping = False # Stop sweep if setting specific angle
                            pwm.duty_u16(angle_to_duty(target_angle))
                            current_angle = target_angle # Update angle tracking
                        else:
                            if len(cmd_str) > 0: # Avoid printing for empty lines
                                print(f"Unknown command/angle: {repr(cmd_str)}")
                    except ValueError: # If it's not ON/OFF/RPM/Angle
                         if len(cmd_str) > 0:
                            print(f"Unknown command string: {repr(cmd_str)}")
            else:
                print("Read empty line from stdin.")
        except Exception as e:
             print(f"Error reading/processing stdin: {e}")

    # --- Sweeping Logic ---
    if sweeping:
# ... (rest of loop) ...
[Immersive content redacted for brevity.]
        # Update angle
        current_angle += sweep_direction

        # Reverse direction at limits
        if current_angle >= 180:
            current_angle = 180
            sweep_direction = -1
        elif current_angle <= 0:
            current_angle = 0
            sweep_direction = 1

        # Set servo position
        duty = angle_to_duty(current_angle)
        pwm.duty_u16(duty)

    # --- Delay (Uses the updated global variable) ---
    time.sleep(current_sweep_delay)