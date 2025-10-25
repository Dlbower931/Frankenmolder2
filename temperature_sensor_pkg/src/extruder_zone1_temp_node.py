#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32 
import time

# Hardware imports for SPI
import spidev

# --- MAX6675/SPI Configuration ---
# SPI bus 0, device 0 (CE0)
SPI_BUS = 0
SPI_DEVICE = 0

# Initialize SPI communication
# This must happen outside the main loop
try:
    spi = spidev.SpiDev()
    spi.open(SPI_BUS, SPI_DEVICE)
    spi.max_speed_hz = 4000000
    spi.mode = 0  # CPOL=0, CPHA=0
except Exception as e:
    # Log an error if SPI fails to initialize (e.g., if /dev/spidev0.0 is missing)
    rospy.logfatal(f"Failed to open SPI device: {e}")
    # A simple exit might be needed if this is fatal for the program
    # Note: ROS node will exit if this is fatal
    raise

# --- MAX6675 Reading Function ---
def read_temp_c():
    """Reads temperature from MAX6675 via SPI and returns Celsius."""
    try:
        raw = spi.readbytes(2)
    except Exception as e:
        rospy.logerr(f"SPI Read Error: {e}")
        return None # Return None on a communication error

    if len(raw) != 2:
        rospy.logwarn("SPI read returned wrong number of bytes.")
        return None

    value = (raw[0] << 8) | raw[1]

    # bit 2 (0x04) indicates open thermocouple
    if value & 0x04:
        # Thermocouple not connected/open
        return None

    # Mask out the status bits and convert to Celsius (0.25 C/bit)
    temp_c = (value >> 3) * 0.25
    return temp_c

# --- ROS Publisher Function ---
def temperature_publisher():
    # Initialize the ROS Node
    rospy.init_node('extruder_zone1_temp_node')
    
    # Create the Publisher
    pub = rospy.Publisher('/extruder/zone1/temperature', Float32, queue_size=10)
    
    # Set the publishing rate (4 Hz to match sensor's max update rate)
    rate = rospy.Rate(4) # 4 Hz 
    
    rospy.loginfo("MAX6675 ROS Publisher Node started.")
    
    # Main publishing loop
    while not rospy.is_shutdown():
        temp_c = read_temp_c()
        
        if temp_c is not None:
            # Create and populate the message
            temp_msg = Float32()
            temp_msg.data = temp_c
            
            # Publish the message
            pub.publish(temp_msg)
            
            # Log the action
            # Note: rospy.loginfo is visible in the terminal running this script
            # rospy.loginfo(f"Published Temperature: {temp_c:.2f} C") 
            
        else:
            # Publish a warning to the ROS log
            rospy.logwarn("Thermocouple is open or read error occurred. Skipping publish.")
            
        # Wait for the required time to maintain the 4 Hz rate
        rate.sleep()

# --- Main Execution ---
if __name__ == '__main__':
    try:
        temperature_publisher()
    except rospy.ROSInterruptException:
        # Graceful exit when Ctrl+C is pressed or ROS shuts down
        rospy.loginfo("ROS node shutting down.")
    finally:
        # Close the SPI device when the script terminates
        rospy.loginfo("Closing SPI device.")
        spi.close()