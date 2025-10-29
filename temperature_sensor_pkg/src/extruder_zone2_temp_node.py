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
# Software Chip Select Pin (Using the original CE1 pin)
SOFTWARE_CS_PIN = 7 # GPIO 7 (Physical Pin 26)
SPI_BUS = 0
SPI_DEVICE = 1 # Open device 1

# --- Initialize SPI and GPIO ---
try:
    # Initialize spidev library (controls SCK, MOSI, MISO)
    spi = spidev.SpiDev() # Create object
    spi.open(SPI_BUS, SPI_DEVICE) # Open device 1
    spi.max_speed_hz = 500000
    # --- CRITICAL: Disable hardware chip select ---
    spi.no_cs = True 
    
    # Initialize RPi.GPIO library (controls the software CS pin)
    GPIO.setmode(GPIO.BCM) # Use BCM pin numbering
    GPIO.setwarnings(False)
    GPIO.setup(SOFTWARE_CS_PIN, GPIO.OUT, initial=GPIO.HIGH) # Set pin as output, HIGH (inactive)

    rospy.loginfo(f"Zone 2: Hardware SPI({SPI_BUS},{SPI_DEVICE} with no_cs=True) and Software CS (GPIO {SOFTWARE_CS_PIN}) initialized.")

except Exception as e:
    print(f"FATAL [Zone 2]: Failed to initialize SPI/GPIO: {e}", file=sys.stderr, flush=True)
    try:
        rospy.logfatal(f"Failed to initialize SPI/GPIO: {e}")
    except: pass
    sys.exit(1)


def read_max6675_sw_cs():
    """Reads temperature using hardware SPI and software chip select."""
    try:
        # 1. Assert Chip Select LOW
        GPIO.output(SOFTWARE_CS_PIN, GPIO.LOW)
        
        # 2. Read two bytes
        data = spi.readbytes(2)

        # 3. De-assert Chip Select HIGH
        GPIO.output(SOFTWARE_CS_PIN, GPIO.HIGH)

    except Exception as e:
        rospy.logerr(f"Extruder Zone 2: SPI/GPIO Read Error: {e}")
        try: GPIO.output(SOFTWARE_CS_PIN, GPIO.HIGH)
        except: pass
        return float('nan')

    # --- Process the received data ---
    if len(data) != 2:
        rospy.logwarn("Extruder Zone 2: SPI read returned wrong number of bytes.")
        return float('nan')

    word = (data[0] << 8) | data[1]

    if word & 0x04:
        rospy.logwarn_throttle(10, "Extruder Zone 2: Thermocouple connection fault.")
        return float('nan')

    temp_bits = word >> 3
    temperature_celsius = temp_bits * 0.25

    return temperature_celsius

def temp_publisher_zone2():
    pub = rospy.Publisher('/extruder/zone2/temperature', Float32, queue_size=10)
    rospy.init_node('extruder_zone2_temp_node', anonymous=True)
    rate = rospy.Rate(4) # Publish rate

    rospy.loginfo("Extruder Zone 2 Temp Publisher Started (Software CS).")

    while not rospy.is_shutdown():
        temp = read_max6675_sw_cs() # Call the software CS function

        if not math.isnan(temp):
            pub.publish(temp)
        rate.sleep()

if __name__ == '__main__':
    try:
        temp_publisher_zone2()
    except rospy.ROSInterruptException:
        rospy.loginfo("Extruder Zone 2 Temp Publisher shutting down.")
    except Exception as e:
        rospy.logfatal(f"Extruder Zone 2 Temp Publisher crashed: {e}")
    finally:
        # Cleanup SPI and GPIO
        try:
            if 'spi' in locals() and spi:
                spi.close()
                rospy.loginfo("Extruder Zone 2 SPI device closed.")
        except Exception as close_e:
            rospy.logerr(f"Error closing Zone 2 SPI device: {close_e}")
        try:
            if 'GPIO' in sys.modules:
                # Only cleanup the pin this script initialized
                GPIO.cleanup(SOFTWARE_CS_PIN) 
        except Exception as gpio_clean_e:
            # Use print as rospy logging might be gone
            print(f"ERROR: Error cleaning up GPIO for Zone 2: {gpio_clean_e}", file=sys.stderr, flush=True)
