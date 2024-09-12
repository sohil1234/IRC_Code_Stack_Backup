#!/usr/bin/env python3
import rospy
import torch
import cv2
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import Float32
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from torchvision import transforms
from PIL import Image as PILImage
import actionlib
import controls_auto.msg
from direction_check import CNN


class Controller:
    """
    Main controller class to handle the search, detection, and navigation towards arrows.
    Pipeline:
    1. Search for the arrow:
        a. Move straight for 2m.
        b. If detected, return True; else rotate 360 degrees and check again.
        c. If not found after rotation, return False.
    2. Follow the arrow:
        a. Move towards the detected arrow until the depth is < 2m, then stop.
    3. Rotate by 90 degrees and repeat.
    """

    def __init__(self):
        self.cur_x = 0.0
        self.cur_y = 0.0
        self.arrow_cnt = 0
        self.object = 'arrow'
        self.direction = None
        self.status = False
        self.reached = False
        self.first_arrow_gps = True

        # Initialize ROS subscribers
        rospy.Subscriber('/gps_node/fix', NavSatFix, self.gps_cb)
        rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.box_callback)
        rospy.Subscriber('/mavros/global_position/compass_hdg', Float32, self.compass_cb)

        self.bridge = CvBridge()
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = CNN().to(self.device)
        self.model.load_state_dict(torch.load('/home/kratos/catkin_ws_2/src/autonomous_irc/cnn_direction.pth', map_location=self.device))
        self.model.eval()

        # Publishers
        self.pub_vel = rospy.Publisher('/rover', Point, queue_size=10)
        rospy.loginfo("Initialized all publishers/subscribers!")

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.first = True

    def compass_cb(self, msg):
        if self.reached:
            self.log_to_file(f"arrow{self.arrow_cnt}_compass.txt", f"{msg}\n")
            print(f"Compass heading: {msg}")

    def gps_cb(self, msg):
        if self.reached:
            lat, lon = msg.latitude, msg.longitude
            self.log_to_file(f"arrow{self.arrow_cnt}_latlong.txt", f"({lat}, {lon})\n")
            print(f"Latitude: {lat}, Longitude: {lon}")

    def box_callback(self, msg):
        if len(msg.bounding_boxes):
            box = msg.bounding_boxes[0]
            self.class_name = box.Class
            self.detected = True
            self.cx = (box.xmin + box.xmax) // 2
            self.cy = (box.ymin + box.ymax) // 2

    def arrow_direction(self, img):
        torch.cuda.empty_cache()
        img = img[self.ymin:self.ymax, self.xmin:self.xmax]
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img = PILImage.fromarray(img)
        transform = transforms.Compose([transforms.Resize((32, 32)), transforms.ToTensor()])
        img = transform(img).unsqueeze(1).to(self.device) / 255.0
        y_pred = torch.sigmoid(self.model(img))
        return "left" if (y_pred >= 0.5).int() == 0 else "right"

    def move_forward(self, duration=0):
        self.publish_velocity(0.3)
        rospy.sleep(duration)
        self.stop()

    def move_backward(self, duration=0):
        self.publish_velocity(-0.3)
        rospy.sleep(duration)
        self.stop()

    def stop(self):
        self.publish_velocity(0.0)

    def publish_velocity(self, velocity):
        msg = Point(x=velocity * 100, y=0.0, z=velocity * 100)
        self.pub_vel.publish(msg)

    def log_to_file(self, filename, data):
        file_path = f"/home/kratos/catkin_ws_2/src/autonomous_irc/controls_auto/gps/{filename}"
        with open(file_path, "a") as file:
            file.write(data)

    def main(self):
        while self.arrow_cnt < 5 and not rospy.is_shutdown():
            self.reached = False

            # Search for the arrow
            if not self.search_for_arrow():
                print("Failed to find the arrow.")
                return self.status

            # Follow the arrow
            if self.object == 'arrow' and not self.follow_arrow():
                print("Failed to reach the arrow within 2m.")
                return self.status

            # Rotate the bot by 90 degrees
            if not self.rotate_90_degrees():
                print("Failed to rotate 90 degrees.")
                return self.status

            self.arrow_cnt += 1

        # Testing movement
        self.move_forward(20)  # Move forward for 20 meters
        print("Testing complete. Returning home.")
        self.status = True
        return self.status

    def search_for_arrow(self):
        search_arrow_client = actionlib.SimpleActionClient('search_arrow', controls_auto.msg.SearchArrowAction)
        search_arrow_client.wait_for_server()
        search_goal = controls_auto.msg.SearchArrowGoal(class_name="arrow")
        search_arrow_client.send_goal(search_goal, feedback_cb=self.search_cb)
        search_arrow_client.wait_for_result()
        return search_arrow_client.get_result().found

    def follow_arrow(self):
        follow_arrow_client = actionlib.SimpleActionClient('follow_arrow', controls_auto.msg.FollowArrowAction)
        follow_arrow_client.wait_for_server()
        follow_goal = controls_auto.msg.FollowArrowGoal(class_name=self.object, cx=self.cx, cy=self.cy)
        follow_arrow_client.send_goal(follow_goal, feedback_cb=self.follow_cb)
        follow_arrow_client.wait_for_result()
        if follow_arrow_client.get_result().reached:
            self.reached = True
            rospy.sleep(10)
            self.stop()
            self.direction = self.arrow_direction(self.image)
            return True
        return False

    def rotate_90_degrees(self):
        rotate_90_client = actionlib.SimpleActionClient('rotate_90', controls_auto.msg.Rotate90Action)
        rotate_90_client.wait_for_server()
        rotate_goal = controls_auto.msg.Rotate90Goal(goal_angle=90.0, direction=self.direction)
        rotate_90_client.send_goal(rotate_goal, feedback_cb=self.rotate_cb)
        rotate_90_client.wait_for_result()
        return rotate_90_client.get_result().turned

    def search_cb(self, fb_msg):
        print("Distance searched:", fb_msg.distance)

    def follow_cb(self, fb_msg):
        print("Arrow depth:", fb_msg.depth)

    def rotate_cb(self, fb_msg):
        print("Angle turned:", fb_msg.angle)


if __name__ == "__main__":
    rospy.init_node("controller_client")
    controller = Controller()
    controller.main()
    rospy.spin()
    rospy.logwarn("Node shutting down.")
