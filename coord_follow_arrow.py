#!/usr/bin/env python3

import rospy
import cv2
import cv_bridge
import numpy as np
import controls_auto.msg
import actionlib
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from darknet_ros_msgs.msg import BoundingBoxes


class FollowArrow:
    # Feedback and Result messages for action server
    _feedback = controls_auto.msg.FollowArrowFeedback()
    _result = controls_auto.msg.FollowArrowResult()

    # Default parameters
    req_class_name = "arrow"
    det_class_name = ""
    direction = 'none'
    center = 320  # Center of the image for error calculation
    cx, cy, xmin, ymin, xmax, ymax = 0, 0, 0, 0, 0, 0
    depth = 0.0
    bounding_boxes = []
    yaw_threshold = 40  # Error threshold for rotation adjustment
    depth_threshold = 1.0  # Distance threshold to stop
    linear_vel = 0.22  # Linear velocity
    angular_vel = 0.2  # Angular velocity
    area = 0  # Bounding box area
    area_threshold = 50000  # Area threshold to stop

    def __init__(self, name):
        self.bridge = cv_bridge.CvBridge()

        # Subscribers
        self.box_sub = rospy.Subscriber('darknet_ros/bounding_boxes', BoundingBoxes, self.box_callback)
        self.image_sub = rospy.Subscriber('/camera/image_raw/', Image, self.image_callback)

        # Publisher
        self.pub_vel = rospy.Publisher('/rover', Point, queue_size=10)

        # Action server
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, controls_auto.msg.FollowArrowAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def box_callback(self, msg):
        """Updates bounding boxes from detection."""
        self.bounding_boxes = msg.bounding_boxes

    def image_callback(self, msg):
        """Converts image from ROS message to OpenCV format."""
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def get_cx_cy(self):
        """Fetches the centroid coordinates (cx, cy) of the largest bounding box."""
        bb = self.bounding_boxes
        mx_area = 0

        if len(bb) > 0:
            for box in bb:
                xmin, xmax, ymin, ymax = box.xmin, box.xmax, box.ymin, box.ymax
                area = (xmax - xmin) * (ymax - ymin)
                
                if area > mx_area:
                    self.xmin, self.xmax, self.ymin, self.ymax = xmin, xmax, ymin, ymax
                    self.cx = (xmin + xmax) // 2
                    self.cy = (ymin + ymax) // 2
                    mx_area = area
            self.area = mx_area
        rospy.logdebug(f"Area of Bounding Box : {self.area}")

    def stop(self):
        """Stops the rover."""
        msg = Point(0.0, 0.0, 0.0)
        self.pub_vel.publish(msg)

    def move_forward(self):
        """Moves the rover forward."""
        msg = Point(self.linear_vel * 100, 0.0, self.linear_vel * 100)
        return msg

    def rotate(self, rotation):
        """Rotates the rover based on the provided direction."""
        msg = Point(rotation * self.angular_vel * 100, 0.0, -rotation * self.angular_vel * 100)
        self.pub_vel.publish(msg)

    def rotate_with_linear(self, forward, error):
        """Moves and rotates the rover while compensating for angular error."""
        linear = self.linear_vel if forward else 0.0
        angular = error * 0.2 / 80
        angular = min(max(angular, -0.15), 0.15)

        msg = Point((linear + angular) * 100, 0.0, (linear - angular) * 100)
        return msg

    def execute_cb(self, goal):
        """Executes the action to follow the detected arrow."""
        self.req_class_name = goal.class_name
        self.cx, self.cy = goal.cx, goal.cy
        r = rospy.Rate(10)
        count = 0

        while count < 5 and not rospy.is_shutdown():
            if len(self.bounding_boxes) == 0:
                rospy.logwarn("Bounding Box Lost (type-1): Doing Something (~_^)")
                self.stop()
                rospy.sleep(1)
                msg = Point(0.1 * 100, 0.0, 0.0)
                self.pub_vel.publish(msg)
                continue

            self.get_cx_cy()
            rospy.loginfo(f"Area: {self.area}, Error: {abs(self.cx - self.center)}")

            if self.area > self.area_threshold:
                count += 1

            if abs(self.cx - self.center) > self.yaw_threshold:
                error = self.center - self.cx
                msg = self.rotate_with_linear(True, error)
            else:
                msg=Point()
                if len(self.bounding_boxes)==0:
                    rospy.logwarn("Bounding Box Lost (type-2): Doing Something (>_<)")
                    self.stop()
                    rospy.sleep(2)
                    msg.x=0.1 * 100
                    rospy.sleep(0.1)
                else:
                    msg = self.move_forward()

            self.pub_vel.publish(msg)
            r.sleep()
            self.bounding_boxes = []

        if self.area > self.area_threshold:
            self.stop()
            self.get_cx_cy()
            self._result.reached = True
            self._result.direction = self.direction
            self._as.set_succeeded(self._result, 'Reached within 2m of the arrow')


if __name__ == '__main__':
    rospy.init_node('follow_arrow')
    server = FollowArrow(rospy.get_name())
    rospy.spin()
