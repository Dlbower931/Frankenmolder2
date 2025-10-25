#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32 # Or the specific type of your topics
import time

class TopicWatchdog:
    def __init__(self, topics_to_watch, timeout_duration=5.0, initial_grace_period=10.0):
        """
        Monitors ROS topics and logs errors if messages stop arriving.

        Args:
            topics_to_watch (list): A list of topic names (strings) to monitor.
            timeout_duration (float): Seconds without messages before logging an error.
            initial_grace_period (float): Seconds to wait after startup before monitoring begins.
        """
        rospy.init_node('topic_watchdog', anonymous=True)
        rospy.loginfo("Topic Watchdog starting...")

        self.topics = topics_to_watch
        self.timeout = rospy.Duration(timeout_duration)
        self.grace_period = rospy.Duration(initial_grace_period)
        self.start_time = rospy.Time.now()

        self.last_received_time = {topic: None for topic in self.topics}
        self.subscribers = {}
        self.has_received_first = {topic: False for topic in self.topics} # Track if we got at least one msg
        self.error_logged = {topic: False for topic in self.topics} # Prevent spamming logs

        for topic in self.topics:
            # Subscribe to each topic using a lambda to pass the topic name
            # Replace Float32 if your topics have different types
            self.subscribers[topic] = rospy.Subscriber(
                topic,
                Float32, # ASSUMPTION: Monitoring Float32 topics
                lambda msg, t=topic: self.message_callback(t)
            )
            rospy.loginfo(f"Monitoring topic: {topic}")

        # Timer to check for timeouts
        self.check_timer = rospy.Timer(rospy.Duration(1.0), self.check_timeouts) # Check every second

        rospy.loginfo(f"Watchdog initialized. Grace period: {initial_grace_period}s, Timeout: {timeout_duration}s")

    def message_callback(self, topic_name):
        """Called whenever a message is received on a monitored topic."""
        now = rospy.Time.now()
        self.last_received_time[topic_name] = now
        if not self.has_received_first[topic_name]:
             self.has_received_first[topic_name] = True
             rospy.loginfo(f"First message received on {topic_name}")
        # Reset error logged flag if topic comes back online
        if self.error_logged[topic_name]:
            rospy.logwarn(f"Topic {topic_name} is publishing messages again.")
            self.error_logged[topic_name] = False


    def check_timeouts(self, event):
        """Periodically checks if any topic has timed out."""
        now = rospy.Time.now()

        # Only start checking after the initial grace period
        if (now - self.start_time) < self.grace_period:
            # rospy.logdebug("Still in grace period...")
            return

        for topic in self.topics:
            last_time = self.last_received_time[topic]

            # Case 1: Never received any message after grace period
            if last_time is None and not self.error_logged[topic]:
                 rospy.logerr(f"CRITICAL: No messages ever received on topic '{topic}' after grace period!")
                 self.error_logged[topic] = True
                 continue # Move to next topic

            # Case 2: Received messages, but they stopped
            if last_time is not None and (now - last_time) > self.timeout:
                if not self.error_logged[topic]:
                    rospy.logerr(f"CRITICAL: No messages received on topic '{topic}' for more than {self.timeout.to_sec()} seconds!")
                    self.error_logged[topic] = True

if __name__ == '__main__':
    try:
        # Define the critical topics to monitor
        # IMPORTANT: Add all essential topics here!
        critical_topics = [
            '/extruder/zone1/temperature',
            '/extruder/zone2/temperature',
            # '/extruder/zone3/temperature', # Add when Zone 3 is active
            # '/extruder/servo/rpm',       # Add when servo is active
        ]
        watchdog = TopicWatchdog(critical_topics)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Topic Watchdog shutting down.")
    except Exception as e:
        rospy.logerr(f"Topic Watchdog crashed: {e}")
```

---

## 3. Update `Dockerfile`

Make the new watchdog script executable.

```dockerfile
# ... (chmod lines for sensor/control nodes)
RUN chmod +x /app/src/temperature_sensor_pkg/src/extruder_zone1_control_node.py

# --- ADD THIS LINE for the watchdog ---
RUN chmod +x /app/src/temperature_sensor_pkg/src/topic_watchdog.py

# ... (Build step)
