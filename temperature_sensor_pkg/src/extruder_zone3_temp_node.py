#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import time
import spidev # Library for hardware SPI communication
import math
import sys
# --- ADD RPi.GPIO import ---
import RPi.GPIO as GPIO

# --- Configuration ---
# Software Chip Select Pin (Connected to MAX6675 CS)
SOFTWARE_CS_PIN = 16 # GPIO 16 (Physical Pin 36)

# Hardware SPI Configuration (We need to initialize spidev, but won't use its CS)
# We can use either device 0 or 1, it doesn't matter since we control CS manually.
# Let's use device 0 for consistency.
SPI_BUS = 0
SPI_DEVICE = 0 # Using CE0 hardware pin, but controlling manually via SOFTWARE_CS_PIN

# --- Initialize SPI and GPIO ---
try:
    # Initialize spidev library (controls SCK, MOSI, MISO)
    spi = spidev.SpiDev(SPI_BUS, SPI_DEVICE)
    spi.max_speed_hz = 500000 # Set SPI speed
    # spi.mode = 0 # Default mode usually works

    # Initialize RPi.GPIO library (controls the software CS pin)
    GPIO.setmode(GPIO.BCM) # Use BCM pin numbering
    GPIO.setwarnings(False)
    GPIO.setup(SOFTWARE_CS_PIN, GPIO.OUT, initial=GPIO.HIGH) # Set pin as output, HIGH (inactive) initially

    rospy.loginfo(f"Zone 3: Hardware SPI({SPI_BUS},{SPI_DEVICE}) and Software CS (GPIO {SOFTWARE_CS_PIN}) initialized.")

except Exception as e:
    # Log fatal error if SPI or GPIO fails
    print(f"FATAL [Zone 3]: Failed to initialize SPI/GPIO: {e}", file=sys.stderr, flush=True)
    try:
        rospy.logfatal(f"Failed to initialize SPI/GPIO: {e}")
    except: pass
    sys.exit(1)


def read_max6675_sw_cs():
    """Reads temperature using hardware SPI and software chip select."""
    try:
        # 1. Assert Chip Select LOW (Activate the sensor)
        GPIO.output(SOFTWARE_CS_PIN, GPIO.LOW)
        # Short delay might be needed after CS goes low, depending on sensor/speed
        # time.sleep(0.0001)

        # 2. Read two bytes using spidev
        data = spi.readbytes(2)

        # 3. De-assert Chip Select HIGH (Deactivate the sensor)
        GPIO.output(SOFTWARE_CS_PIN, GPIO.HIGH)

    except Exception as e:
        rospy.logerr(f"Extruder Zone 3: SPI/GPIO Read Error: {e}")
        # Ensure CS is HIGH if an error occurs
        try:
            GPIO.output(SOFTWARE_CS_PIN, GPIO.HIGH)
        except: pass # Ignore errors during cleanup
        return float('nan')

    # --- Process the received data (same as before) ---
    if len(data) != 2:
        rospy.logwarn("Extruder Zone 3: SPI read returned wrong number of bytes.")
        return float('nan')

    word = (data[0] << 8) | data[1]

    if word & 0x04:
        rospy.logwarn_throttle(10, "Extruder Zone 3: Thermocouple connection fault.")
        return float('nan')

    temp_bits = word >> 3
    temperature_celsius = temp_bits * 0.25

    return temperature_celsius

def temp_publisher_zone3():
    pub = rospy.Publisher('/extruder/zone3/temperature', Float32, queue_size=10)
    rospy.init_node('extruder_zone3_temp_node', anonymous=True)
    rate = rospy.Rate(4) # Publish rate

    rospy.loginfo("Extruder Zone 3 Temp Publisher Started (Software CS).")

    while not rospy.is_shutdown():
        # --- Use the new reading function ---
        temp = read_max6675_sw_cs()

        if not math.isnan(temp):
            pub.publish(temp)
        rate.sleep()

if __name__ == '__main__':
    try:
        temp_publisher_zone3()
    except rospy.ROSInterruptException:
        rospy.loginfo("Extruder Zone 3 Temp Publisher shutting down.")
    except Exception as e:
        rospy.logfatal(f"Extruder Zone 3 Temp Publisher crashed: {e}")
    finally:
        # Cleanup SPI and GPIO
        try:
            if 'spi' in locals() and spi:
                spi.close()
                rospy.loginfo("Extruder Zone 3 SPI device closed.")
        except Exception as close_e:
            rospy.logerr(f"Error closing Zone 3 SPI device: {close_e}")
        try:
             # Check if GPIO module was successfully imported before cleanup
            if 'GPIO' in sys.modules:
                print("DEBUG: Cleaning up GPIO for Zone 3...", flush=True)
                GPIO.cleanup(SOFTWARE_CS_PIN) # Clean up only the pin we used
                print("DEBUG: GPIO cleanup complete for Zone 3.", flush=True)
        except Exception as gpio_clean_e:
            print(f"ERROR: Error cleaning up GPIO for Zone 3: {gpio_clean_e}", file=sys.stderr, flush=True)

