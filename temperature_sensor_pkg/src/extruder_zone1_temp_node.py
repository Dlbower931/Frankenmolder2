#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import time
import spidev # Library for SPI communication

# MAX6675 Setup (Zone 1 - connected to GPIO CS0)
# Bus 0, Device 0
spi = spidev.SpiDev(0, 0)
spi.max_speed_hz = 500000

def read_max6675():
    # Read two bytes from the MAX6675
    # The read8 and read16 are methods on the spidev object
    data = spi.readbytes(2)
    
    # Combine bytes: [Byte 0 (MSB), Byte 1 (LSB)]
    word = (data[0] << 8) | data[1]
    
    # Check for a thermocouple connection fault (bit 2 is 1 if fault)
    if word & 0x04:
        rospy.logwarn("Extruder Zone 1: Thermocouple connection fault.")
        return float('nan')
        
    # Data is 12-bit, shifted 3 bits to the left (bits 3-14)
    # 0xFFF8 = 1111 1111 1111 1000 (Clears bits 0, 1, 2)
    temp_bits = (word & 0xFFFC) >> 3
    
    # Temp resolution is 0.25°C
    temperature_celsius = temp_bits * 0.25
    
    return temperature_celsius

def temp_publisher_zone1():
    # 1. Initialize Publisher
    pub = rospy.Publisher('/extruder/zone1/temperature', Float32, queue_size=10)
    # 2. Initialize Node
    rospy.init_node('extruder_zone1_temp_node', anonymous=True)
    rate = rospy.Rate(4) # 4 Hz
    
    rospy.loginfo("Extruder Zone 1 Temp Publisher Started.")
    
    while not rospy.is_shutdown():
        temp = read_max6675()
        
        if not math.isnan(temp):
            # 3. Publish Data
            pub.publish(temp)
            
        rate.sleep()

if __name__ == '__main__':
    # We need the 'math' library for the NaN check
    import math 
    try:
        temp_publisher_zone1()
    except rospy.ROSInterruptException:
        pass