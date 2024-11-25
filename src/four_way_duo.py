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

        # roba
        self.roba_scan_sub = rospy.Subscriber('/roba/scan', LaserScan, self.roba_scan_cb)
        self.roba_my_odom_sub = rospy.Subscriber('my_odom', PointArray, self.roba_my_odom_cb)
        self.roba_cmd_vel_pub = rospy.Publisher('/roba/cmd_vel', Twist, queue_size=1)

        # robb
        self.robb_scan_sub = rospy.Subscriber('/robb/scan', LaserScan, self.robb_scan_cb)
        self.robb_my_odom_sub = rospy.Subscriber('my_odom', PointArray, self.robb_my_odom_cb)
        self.robb_cmd_vel_pub = rospy.Publisher('/robb/cmd_vel', Twist, queue_size=1)

        # Signal Related
        self.signal_sub = rospy.Subscriber('signal_sim', Bool, self.signal_cb)
        self.cur_signal = False

        # Initialized Value
        self.roba_dist = 0.0
        self.robb_dist = 0.0

        self.roba_indi = False
        self.robb_indi = False
        self.roba_indi_two = False
        self.robb_indi_two = False



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
        # msg.points[0] contains roba data
        self.roba_dist = msg.points[0].x  # roba distance
        self.roba_yaw = msg.points[0].y   # roba yaw if needed
        #raise NotImplementedError


    def roba_scan_cb(self, msg):
        """Callback function for `self.scan_sub`."""
        #raise NotImplementedError

    def robb_my_odom_cb(self, msg):
        """Callback to `self.my_odom_sub`."""
        # msg.points[1] contains robb data
        self.robb_dist = msg.points[1].x  # robb distance
        self.robb_yaw = msg.points[1].y   # robb yaw if needed
        #raise NotImplementedError


    def robb_scan_cb(self, msg):
        """Callback function for `self.scan_sub`."""
        #raise NotImplementedError

    def move(self):
        roba_twist = Twist()
        robb_twist = Twist()

        # Default Speed
        roba_twist.linear.x = 0.2
        robb_twist.linear.x = 0.2
        

        #''' # Demo variables, Cross Intersections.
        if (self.roba_dist >= 0.75):
            self.roba_indi = True
        if (self.robb_dist >= 0.3):
            self.robb_indi = True

        if (self.roba_dist >= 1.25):
            self.roba_indi_two = True
        if (self.robb_dist >= 0.8):
            self.robb_indi_two = True

        #'''

        if self.cur_signal:
            if (self.roba_indi):
                roba_twist.linear.x = 0.2
            if (self.robb_indi) and (not self.robb_indi_two):
                robb_twist.linear.x = 0.0
        else:
            if (self.roba_indi) and (not self.roba_indi_two):
                roba_twist.linear.x = 0.0
            if (self.robb_indi):
                robb_twist.linear.x = 0.2

        self.roba_cmd_vel_pub.publish(roba_twist)
        self.robb_cmd_vel_pub.publish(robb_twist)
            
  #roba X 0.8,0.3 robb Y 1.2,0.3
        
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
