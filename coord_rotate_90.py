#!/usr/bin/env python3
import rospy
import cv_bridge
import numpy as np
import controls_auto.msg
import actionlib
from sensor_msgs.msg import Image, NavSatFix
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion
from math import pi
import time
import matplotlib.pyplot as plt

# Create a list to store latitude and longitude values
latitudes = []
longitudes = []

# Create a figure object to plot in real time
fig, ax = plt.subplots()

class Rotate:
    _feedback = controls_auto.msg.Rotate90Feedback()
    _result = controls_auto.msg.Rotate90Result()
    
    # Initialization of relevant parameters
    init_angle = 0.0
    cur_angle = 0.0
    target_angle = 90.0
    turned = False
    direction = 0
    angular_vel = 0.3
    linear_vel = 0.3
    first_run = True
    time_rotate = 10
    
    def __init__(self, name):
        self.bridge = cv_bridge.CvBridge()
        self.lat = 0
        self.lon = 0
        self.cardinal_direction = ''
        self.degree = 0

        # Subscribers
        self.sub = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gps_callback)
        self.compass_sub = rospy.Subscriber('mavros/global_position/compass_hdg', Float64, self.compass_callback)
        
        # Publisher
        self.pub_vel = rospy.Publisher('/rover', Point, queue_size=10)

        # Start the action server
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, controls_auto.msg.Rotate90Action, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def move_forward(self):
        msg = Point()
        msg.x = self.linear_vel * 100
        msg.y = 0.0
        msg.z = self.linear_vel * 100
        self.pub_vel.publish(msg)

    def move_backward(self):
        msg = Point()
        msg.x = -self.linear_vel * 100
        msg.y = 0.0
        msg.z = -self.linear_vel * 100
        self.pub_vel.publish(msg)

    def rotate(self, rotation):
        msg = Point()
        msg.x = rotation * self.angular_vel * 100
        msg.y = 0.0
        msg.z = -rotation * self.angular_vel * 100
        self.pub_vel.publish(msg)
    
    def stop(self):
        msg = Point()
        msg.x = msg.y = msg.z = 0.0
        self.pub_vel.publish(msg)

    def compass_callback(self, degree):
        # Determine cardinal direction based on compass heading
        directions = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW']
        self.degree = degree.data

        if 0 <= degree < 22.5 or degree >= 337.5:
            self.cardinal_direction = 'N'
        elif 22.5 <= degree < 67.5:
            self.cardinal_direction = 'NE'
        elif 67.5 <= degree < 112.5:
            self.cardinal_direction = 'E'
        elif 112.5 <= degree < 157.5:
            self.cardinal_direction = 'SE'
        elif 157.5 <= degree < 202.5:
            self.cardinal_direction = 'S'
        elif 202.5 <= degree < 247.5:
            self.cardinal_direction = 'SW'
        elif 247.5 <= degree < 292.5:
            self.cardinal_direction = 'W'
        elif 292.5 <= degree < 337.5:
            self.cardinal_direction = 'NW'
        
        print(self.cardinal_direction)

    def gps_callback(self, msg):
        # Store GPS coordinates
        self.lat = msg.latitude
        self.lon = msg.longitude
        print(f'Latitude: {self.lat}, Longitude: {self.lon}')

    def execute_cb(self, goal):
        self.stop()
        rospy.sleep(1)

        # Set rotation direction based on goal
        self.target_angle = goal.goal_angle
        self.direction = 1 if goal.direction == "left" else -1

        init_time = time.time()
        rospy.loginfo("Rotation started")
        
        initial_orientation = self.degree
        final_orientation = initial_orientation + 90 * (-self.direction)
        while abs((self.degree - final_orientation)) > 5:
            self.rotate(self.direction)
            rospy.sleep(0.1)
        self.stop()
    
        
        rospy.loginfo(f"Rotation of rover by 90 in direction: {self.direction} completed")
        self.stop()


        # Mark as turned and move forward
        self.turned = True
        self._result.turned = True
        rospy.loginfo("Rotated 90°, now moving forward")
        self.move_forward()
        rospy.sleep(1)
        self.stop()

        # Save coordinates to list and file
        latitudes.append(self.lat / 10**4)
        longitudes.append(self.lon / 10**4)
        with open("lat_long_arrow.txt", "a") as file:
            file.write(f"({self.lat}, {self.lon}, {self.cardinal_direction})\n")

        # Stop the rover after moving forward
        rospy.loginfo("Stopped after moving")
        self._as.set_succeeded(self._result, 'Turned successfully.')

if __name__ == '__main__':
    rospy.init_node('rotate_90')
    server = Rotate(rospy.get_name())
    rospy.spin()
