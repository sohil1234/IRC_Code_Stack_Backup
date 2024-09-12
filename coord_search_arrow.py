#!/usr/bin/env python3
import rospy
import cv2
import cv_bridge
import numpy as np
import controls_auto.msg
import actionlib
import time
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from darknet_ros_msgs.msg import BoundingBoxes
from math import pi


class Search:
    """
    Improvements:
    1) If arrow is detected for 10 frames, then stop instead of 1 frame.
    """

    _feedback = controls_auto.msg.SearchArrowFeedback()
    _result = controls_auto.msg.SearchArrowResult()

    # Class variables
    req_class_name = "arrow"
    detected = False
    total_distance = 0.0
    stop_search = 20.0
    first_run = True
    cur_x = 0.0
    cur_y = 0.0
    previous_x = 0.0
    previous_y = 0.0
    image = np.array
    bounding_boxes = []
    linear_vel = 0.3
    angular_vel = 0.17
    cx, cy, xmin, ymin, xmax, ymax = 0, 0, 0, 0, 0, 0
    center = 320
    area = 0

    def __init__(self, name):
        self.bridge = cv_bridge.CvBridge()

        # Initialize subscribers
        self.box_sub = rospy.Subscriber('darknet_ros/bounding_boxes', BoundingBoxes, self.box_callback)
        self.image_sub = rospy.Subscriber('/camera1/usb_cam1/image_raw/', Image, self.image_callback)

        # Initialize publisher
        self.pub_vel = rospy.Publisher('/rover', Point, queue_size=10)

        # Action server
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, controls_auto.msg.SearchArrowAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def box_callback(self, msg):
        self.bounding_boxes = msg.bounding_boxes

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def move_forward(self):
        msg = Point()
        msg.x = self.linear_vel * 100  # Right wheel
        msg.z = self.linear_vel * 100  # Left wheel
        self.pub_vel.publish(msg)

    def stop(self):
        msg = Point()
        msg.x = msg.y = msg.z = 0.0
        self.pub_vel.publish(msg)

    def rotate(self, direction):
        msg = Point()
        msg.x = direction * self.angular_vel * 100  # Right wheel
        msg.z = -direction * self.angular_vel * 100  # Left wheel
        self.pub_vel.publish(msg)

    def get_cx_cy(self):
        mx_area = 0
        self.area = 0
        self.cx = 0
        for box in self.bounding_boxes:
            xmin, xmax, ymin, ymax = box.xmin, box.xmax, box.ymin, box.ymax
            area = (xmax - xmin) * (ymax - ymin)
            if area > mx_area:
                self.xmin, self.xmax, self.ymin, self.ymax = xmin, xmax, ymin, ymax
                self.cx = (xmin + xmax) // 2
                self.cy = (ymin + ymax) // 2
                mx_area = area
        self.area = mx_area
        rospy.loginfo(f"Area of Bounding Box: {self.area}")
        self.bounding_boxes = []

    def execute_cb(self, goal):
        self.detected = False
        self.req_class_name = goal.class_name
        error = 10000
        no_of_detection = 0
        r = rospy.Rate(10)

        while not self.detected:
            self.get_cx_cy()
            if self.area != 0:
                no_of_detection += 1
                self.move_forward()
                rospy.sleep(1)
                self.stop()
            else:
                self.rotate(-1)
                rospy.sleep(1)
                self.stop()
                self.get_cx_cy()
                if self.area != 0:
                    no_of_detection += 1
                    self.move_forward()
                    rospy.sleep(1)
                    self.stop()
                else:
                    self.rotate(1)
                    rospy.sleep(2)
                    self.stop()
                    self.get_cx_cy()
                    if self.area != 0:
                        no_of_detection += 1
                        self.move_forward()
                        rospy.sleep(1)
                        self.stop()
                    else:
                        self.rotate(-1)
                        rospy.sleep(1)
                        self.stop()
                        rospy.logerr("Arrow Lost Maybe Motion blur is there, let's wait for a few moments")
                        for i in range(10):
                            self.get_cx_cy()
                            r.sleep()
                            if self.area != 0:
                                no_of_detection += 1
                        
            

            if no_of_detection > 5:
                self.detected = True
                self.stop()
                break

            self.rotate(1)
            rospy.sleep(2)
            self.stop()
            self.get_cx_cy()
            if self.area != 0:
                no_of_detection += 1
            if no_of_detection > 5:
                self.detected = True
                self.stop()
                break

            self.rotate(-1)
            rospy.sleep(4)
            self.stop()

        if self.detected:
            rospy.loginfo("Rotating towards the Arrow before going into follow_object")
            while abs(error) > 30 and not rospy.is_shutdown():
                self.get_cx_cy()
                error = self.center - self.cx
                direction = 1 if error > 0 else -1
                start_time = time.time()

                while self.area == 0 and not rospy.is_shutdown():
                    rospy.logwarn("Lost Object - Waiting")
                    self.stop()
                    rospy.sleep(0.5)
                    self.get_cx_cy()
                    if time.time() - start_time > 2:
                        self.rotate(direction)
                        rospy.sleep(0.2)
                        self.stop()
                        self.move_forward()
                        rospy.sleep(1)
                        start_time = time.time()

                error = self.center - self.cx
                direction = 1 if error > 0 else -1
                self.rotate(direction)
                rospy.sleep(0.3)
                self.stop()
                r.sleep()

            rospy.loginfo("Rotation towards object Completed : Going to follow object :)")

            self._result.found = True
            self._result.class_name = self.req_class_name
            self._result.cx = self.cx
            self._result.cy = self.cy
            self._as.set_succeeded(self._result, 'Found')
        else:
            self._result.found = False
            self._as.set_succeeded(self._result, 'Not Found')


if __name__ == '__main__':
    rospy.init_node('search_arrow')
    server = Search(rospy.get_name())
    rospy.spin()
