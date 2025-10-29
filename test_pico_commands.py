#!/usr/bin/env python3
"""
Test script to demonstrate Pi Pico communication
Run this to test injection molding commands
"""

import rospy
from std_msgs.msg import String, Bool, Float32
import json
import time

def test_extruder_commands():
    """Test minimal extruder commands"""
    rospy.init_node('pico_test_client', anonymous=True)
    
    # Minimal publishers
    run_pub = rospy.Publisher('/extruder/run', Bool, queue_size=10)
    rpm_pub = rospy.Publisher('/extruder/target_rpm', Float32, queue_size=10)
    status_sub = rospy.Subscriber('/extruder/actual_rpm', Float32, lambda m: None)
    
    # Wait for publishers to connect
    rospy.sleep(2)
    
    print("Testing minimal extruder interface...")

    # RUN
    print("1. RUN extruder")
    run_pub.publish(Bool(True))
    rospy.sleep(1)

    # SET RPMs
    for rpm in [200.0, 500.0, 1000.0, 1500.0]:
        print(f"2. Set RPM -> {rpm}")
        rpm_pub.publish(Float32(rpm))
        rospy.sleep(1)

    # STOP
    print("3. STOP extruder")
    run_pub.publish(Bool(False))
    rospy.sleep(1)

    print("\nMinimal tests completed!")

if __name__ == '__main__':
    try:
        test_extruder_commands()
    except rospy.ROSInterruptException:
        pass
