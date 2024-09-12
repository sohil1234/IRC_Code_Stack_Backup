#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
import utm
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt 
from std_msgs.msg import Float64
import os

plt.ion()
fig, ax = plt.subplots()

class Rover:
    def __init__(self):
        rospy.init_node('node1')
        rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.gps_callback)
        self.rate = rospy.Rate(10)
        self.x_list = []
        self.y_list = []
        self.anim = FuncAnimation(fig, self.update_plot, interval=100)
        plt.show(block = False)

    def gps_callback(self, msg):
        self.x_list.append(msg.latitude)
        self.y_list.append(msg.longitude)

    def update_plot(self, frame):
        ax.clear()
        ax.scatter(self.x_list, self.y_list, s=10, c='b', alpha=0.5)
        ax.set_xlabel('X Position')
        ax.set_ylabel('Y Position')
        ax.set_title('Rover Path')
        ax.spines['left'].set_position('center')
        ax.spines['bottom'].set_position('center')
        plt.pause(0.01)
        
        fig.canvas.draw()

    def execute(self):
        plt.show(block = True)
        plt.s
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = Rover()
        controller.execute()
    except rospy.ROSInterruptException:
        pass