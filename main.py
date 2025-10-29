import sys
import time
from machine import Pin, Timer

# The onboard LED on a standard Pico is Pin 25
# On a Pico W, it's Pin("LED")
try:
    led = Pin("LED", Pin.OUT)
except TypeError:
    led = Pin(25, Pin.OUT)

# Timer object for blinking
led_timer = Timer()

def start_blinking(timer):
    """Callback function for the timer to toggle the LED."""
    led.toggle()

def process_command(command):
    """Acts on a command received from the Pi."""
    command = command.strip().upper() # Clean up the command string
    
    if command == "ON":
        print("PICO: Received ON, starting blink.")
        # Start a 2Hz blink (5 times a second)
        led_timer.init(freq=2.5, mode=Timer.PERIODIC, callback=start_blinking)
    
    elif command == "OFF":
        print("PICO: Received OFF, stopping blink.")
        # Stop the timer
        led_timer.deinit()
        # Ensure the LED is off
        led.value(0)
    
    else:
        print(f"PICO: Received unknown command: {command}")

# --- Main Loop ---
print("PICO: Ready for commands...")
while True:
    try:
        # Read a line from the USB serial
        if sys.stdin.any():
            command_in = sys.stdin.readline()
            if command_in:
                process_command(command_in)
        
        # Small delay to prevent busy-looping
        time.sleep(0.1)
        
    except KeyboardInterrupt:
        break
    except Exception as e:
        print(f"PICO Error: {e}")
        time.sleep(1)

# Cleanup
led_timer.deinit()
led.value(0)
print("PICO: Shutting down.")
