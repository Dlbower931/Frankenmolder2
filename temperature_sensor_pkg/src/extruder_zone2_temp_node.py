#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import time
import spidev # Library for SPI communication

# MAX6675 Setup (Zone 2 - connected to GPIO CS1)
# Bus 0, Device 1 (This is the change!)
spi = spidev.SpiDev(0, 1)
spi.max_speed_hz = 500000

def read_max6675():
    # Read two bytes from the MAX6675
    data = spi.readbytes(2)
    
    word = (data[0] << 8) | data[1]
    
    # Check for a thermocouple connection fault
    if word & 0x04:
        rospy.logwarn("Extruder Zone 2: Thermocouple connection fault.")
        return float('nan')
        
    temp_bits = (word & 0xFFFC) >> 3
    temperature_celsius = temp_bits * 0.25
    
    return temperature_celsius

def temp_publisher_zone2():
    # 1. Initialize Publisher (New Topic!)
    pub = rospy.Publisher('/extruder/zone2/temperature', Float32, queue_size=10)
    # 2. Initialize Node (New Node Name!)
    rospy.init_node('extruder_zone2_temp_node', anonymous=True)
    rate = rospy.Rate(4) # 4 Hz
    
    rospy.loginfo("Extruder Zone 2 Temp Publisher Started.")
    
    while not rospy.is_shutdown():
        temp = read_max6675()
        
        if not math.isnan(temp):
            # 3. Publish Data
            pub.publish(temp)
            
        rate.sleep()

if __name__ == '__main__':
    import math
    try:
        temp_publisher_zone2()
    except rospy.ROSInterruptException:
        pass