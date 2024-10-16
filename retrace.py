#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point
import time


class RoverPathRetrace:
    def __init__(self):
        rospy.init_node('rover_path_retrace')


        self.gps_topic = "/mavros/global_position/global"
        self.rover_cmd_topic = "/rover"


        self.velocity_history = []  
        self.gps_signal_lost = False
        self.gps_timeout = 2.0
        self.last_gps_time = time.time()

        self.gps_subscriber = rospy.Subscriber(self.gps_topic, NavSatFix, self.gps_callback)
        self.rover_cmd_subscriber = rospy.Subscriber(self.rover_cmd_topic, Point, self.velocity_callback)
        self.rover_cmd_publisher = rospy.Publisher(self.rover_cmd_topic, Point, queue_size=10)

    def gps_callback(self, msg):
        
        self.last_gps_time = time.time()

        if self.gps_signal_lost:
            rospy.loginfo("GPS signal restored!")
            self.gps_signal_lost = False
            self.velocity_history.clear() # Clear velocity data when gps is back
    
    def check_gps_timeout(self):
        # Check if the GPS timeout period has been exceeded
        if time.time() - self.last_gps_time > self.gps_timeout:
            if not self.gps_signal_lost:
                rospy.logwarn("GPS signal lost due to timeout!")
                self.gps_signal_lost = True
                self.start_retracing()

    def velocity_callback(self, msg):
        if not self.gps_signal_lost:
            # Store current velocity command if GPS is available
            self.velocity_history.append((msg.x, msg.z))

    def start_retracing(self):
        rospy.loginfo("Starting retrace...")
        while self.gps_signal_lost and self.velocity_history and not rospy.is_shutdown():
            # Retrieve the last velocity and apply the opposite
            left_velocity, right_velocity = self.velocity_history.pop()
            reverse_msg = Point(-left_velocity, 0.0, -right_velocity)
            
            rospy.loginfo(f"Retracing with reversed velocities: left={-left_velocity}, right={-right_velocity}")
            self.rover_cmd_publisher.publish(reverse_msg)
            rospy.sleep(0.1)  

    def stop_rover(self):
        stop_msg = Point(0.0, 0.0, 0.0)
        self.rover_cmd_publisher.publish(stop_msg)
        rospy.loginfo("Rover stopped.")

    def run(self):
        rate = rospy.Rate(10)  
        while not rospy.is_shutdown():
            self.check_gps_timeout()
            rate.sleep()

if __name__ == '__main__':
    rover = RoverPathRetrace()
    try:
        rover.run()
    except rospy.ROSInterruptException:
        rover.stop_rover()
