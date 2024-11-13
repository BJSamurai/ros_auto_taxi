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

        self.roba_scan_sub = rospy.Subscriber('/roba/scan', LaserScan, self.roba_scan_cb)
        self.roba_my_odom_sub = rospy.Subscriber('/roba/my_odom', Point, self.roba_my_odom_cb)
        self.roba_cmd_vel_pub = rospy.Publisher('/roba/cmd_vel', Twist, queue_size=1)

        self.robb_scan_sub = rospy.Subscriber('/robb/scan', LaserScan, self.robb_scan_cb)
        self.robb_my_odom_sub = rospy.Subscriber('/robb/my_odom', Point, self.robb_my_odom_cb)
        self.robb_cmd_vel_pub = rospy.Publisher('/robb/cmd_vel', Twist, queue_size=1)

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

    def signal_cb(self, msg):
        """Callback to 'self.signal_sub'. """
        self.cur_signal = msg.data

    def roba_my_odom_cb(self, msg):
        """Callback to `self.my_odom_sub`."""
        #raise NotImplementedError


    def roba_scan_cb(self, msg):
        """Callback function for `self.scan_sub`."""
        #raise NotImplementedError

    def robb_my_odom_cb(self, msg):
        """Callback to `self.my_odom_sub`."""
        #raise NotImplementedError


    def robb_scan_cb(self, msg):
        """Callback function for `self.scan_sub`."""
        #raise NotImplementedError

    def move(self):
        roba_twist = Twist()
        robb_twist = Twist()
        if self.cur_signal:
            roba_twist.linear.x = 0.2
            robb_twist.linear.x = 0.0
        else:
            roba_twist.linear.x = 0.0
            robb_twist.linear.x = 0.2

        self.roba_cmd_vel_pub.publish(roba_twist)
        self.robb_cmd_vel_pub.publish(robb_twist)
            
  
        
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
