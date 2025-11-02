# pi_can_handler.py
# Connects to the CAN bus on the Raspberry Pi,
# listens for temperature status messages from the ESP32,
# decodes the float values, and prints them to the console.

import can
import struct
import time
import os

# --- CAN ID DEFINITIONS (Must match ESP32) ---
CAN_ID_STATUS_TEMP_1 = 0x101
CAN_ID_STATUS_TEMP_2 = 0x102
CAN_ID_STATUS_TEMP_3 = 0x103

# Dictionary to hold the last known state
# We initialize with 'None' to show we haven't received data yet
temperatures = {
    "zone_1_temp_c": None,
    "zone_2_temp_c": None,
    "zone_3_temp_c": None,
}

def decode_can_message(msg):
    """Decodes a CAN message and updates the global state."""
    
    # Check if the message payload is 4 bytes (for a float)
    if msg.dlc != 4:
        print(f"WARN: Received msg ID {hex(msg.arbitration_id)} with invalid DLC: {msg.dlc}")
        return

    # Unpack the 4 bytes into a little-endian float
    # The ESP32 (little-endian) copied a float, so we unpack as '<f'
    try:
        value = struct.unpack('<f', msg.data)[0]
    except struct.error as e:
        print(f"ERROR: Failed to unpack data from ID {hex(msg.arbitration_id)}. {e}")
        return

    # Update the correct temperature in our state dictionary
    if msg.arbitration_id == CAN_ID_STATUS_TEMP_1:
        temperatures["zone_1_temp_c"] = value
    elif msg.arbitration_id == CAN_ID_STATUS_TEMP_2:
        temperatures["zone_2_temp_c"] = value
    elif msg.arbitration_id == CAN_ID_STATUS_TEMP_3:
        temperatures["zone_3_temp_c"] = value

def main():
    """Main function to initialize CAN bus and run the listener loop."""
    
    # Use 'socketcan' for the interface and 'can0' for the channel
    # This assumes you have run:
    # 1. sudo ip link set can0 up type can bitrate 500000
    can_interface = 'can0'
    
    print(f"--- RASPBERRY PI CAN DECODER ---")
    print(f"Attempting to connect to '{can_interface}'...")

    try:
        # Initialize the CAN bus
        bus = can.interface.Bus(channel=can_interface, bustype='socketcan')
        print(f"Successfully connected to '{can_interface}'. Listening for messages...")
        
        last_print_time = time.time()

        while True:
            # Wait for a message
            message = bus.recv()

            if message is None:
                continue
                
            # Check if the message ID is one we care about
            if message.arbitration_id in [CAN_ID_STATUS_TEMP_1, CAN_ID_STATUS_TEMP_2, CAN_ID_STATUS_TEMP_3]:
                decode_can_message(message)
            
            # Print the current state to the console every 0.5 seconds
            current_time = time.time()
            if current_time - last_print_time >= 0.5:
                last_print_time = current_time
                
                # Clear the console and print the formatted state
                # os.system('clear') # Uncomment this for a cleaner, refreshing display
                
                # Format the output
                z1 = f"Z1: {temperatures['zone_1_temp_c']:.2f}C" if temperatures['zone_1_temp_c'] is not None else "Z1: --"
                z2 = f"Z2: {temperatures['zone_2_temp_c']:.2f}C" if temperatures['zone_2_temp_c'] is not None else "Z2: --"
                z3 = f"Z3: {temperatures['zone_3_temp_c']:.2f}C" if temperatures['zone_3_temp_c'] is not None else "Z3: --"
                
                print(f"STATE: | {z1} | {z2} | {z3} |", flush=True)


    except can.CanError as e:
        print(f"\nCRITICAL ERROR: Failed to connect to '{can_interface}'.")
        print("Please ensure the 'can0' interface is up and configured:")
        print("  sudo ip link set can0 up type can bitrate 500000")
        print(f"Error details: {e}")
    except KeyboardInterrupt:
        print("\nShutting down CAN listener.")
    finally:
        if 'bus' in locals():
            bus.shutdown()
        print("Goodbye.")

if __name__ == "__main__":
    main()

