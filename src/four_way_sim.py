#!/usr/bin/env python3
import math
import rospy
import tf
import cv2 as cv
import cv_bridge
import numpy
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist, Point
from matplotlib import pyplot as plt
from std_msgs.msg import String, Bool
import numpy as np

class FourWaySim:
    def __init__(self):

        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        self.my_odom_sub = rospy.Subscriber('my_odom', Point, self.my_odom_cb)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.signal_sub = rospy.Subscriber('signal_sim', Bool, self.signal_cb)

        self.centroid_image = None
        self.cur_signal = False
        self.init_states()

    def init_states(self):
        """Initialize the states of the robot."""
        self.states = {
            'following_line': True,
            'avoiding_obstacle': False,
        }

    def my_odom_cb(self, msg):
        """Callback to `self.my_odom_sub`."""
        #raise NotImplementedError

    def signal_cb(self, msg):
        """Callback to 'self.signal_sub'. """
        self.cur_signal = msg.data


    def scan_cb(self, msg):
        """Callback function for `self.scan_sub`."""
        #raise NotImplementedError

    def move(self):
        twist = Twist()
        if self.cur_signal:
            twist.linear.x = 0.2
        else:
            twist.linear.x = 0.0

        self.cmd_vel_pub.publish(twist)
            
  
        
    def run(self):
        """Run the program."""
        rate = rospy.Rate(10)
        

        while not rospy.is_shutdown():
            self.move()
            rate.sleep()

    #### Calculation of the filtered images
    
           
if __name__ == '__main__':
    rospy.init_node('four_way_sim')
    FourWaySim().run()
    rospy.spin()
