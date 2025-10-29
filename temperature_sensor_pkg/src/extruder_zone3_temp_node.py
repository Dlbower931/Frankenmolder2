#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import time
import spidev # Library for SPI communication
import math # For isnan
import sys # For stderr

# --- CONFIRMED: SPI Device 2 for GPIO 16 (CE2) ---
# MAX6675 Setup (Zone 3 - Connected to GPIO 16 / CE2)
# Bus 0, Device 2
try:
    spi = spidev.SpiDev(0, 2) # Use device 2
    spi.max_speed_hz = 500000
    # Optional: Set SPI mode if needed, though default (0) usually works for MAX6675
    # spi.mode = 0
except Exception as e:
    print(f"FATAL [Zone 3]: Failed to open SPI device (0, 2): {e}", file=sys.stderr, flush=True)
    try:
        rospy.logfatal(f"Failed to open SPI device (0, 2): {e}")
    except:
        pass
    sys.exit(1)


def read_max6675():
    """Reads temperature from MAX6675 via SPI and returns Celsius or NaN."""
    try:
        data = spi.readbytes(2)
    except Exception as e:
        rospy.logerr(f"Extruder Zone 3: SPI Read Error: {e}")
        return float('nan')

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
    rate = rospy.Rate(4) # Publish rate (e.g., 4 Hz)

    rospy.loginfo("Extruder Zone 3 Temp Publisher Started.")

    while not rospy.is_shutdown():
        temp = read_max6675()
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
        try:
            if 'spi' in locals() and spi:
                spi.close()
                rospy.loginfo("Extruder Zone 3 SPI device closed.")
        except Exception as close_e:
            rospy.logerr(f"Error closing Zone 3 SPI device: {close_e}")

