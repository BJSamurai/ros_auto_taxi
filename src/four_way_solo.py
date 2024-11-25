#!/usr/bin/env python3
import math
import rospy
import tf2_ros
import cv2 as cv
import cv_bridge
import numpy
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist, Point, TransformStamped
from matplotlib import pyplot as plt
from std_msgs.msg import String, Bool, Float32MultiArray, Int32
import numpy as np
from fiducial_msgs.msg import FiducialTransformArray
from tf.transformations import quaternion_from_euler
from mapper_real import Mapper

# Potential bugs:
# 1. When robot is facing 2nd signal, and leaving 1st signal


class FourWaySim:
    def __init__(self):

        # roba
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.roba_scan_cb)
        self.my_odom_sub = rospy.Subscriber('my_odom', Float32MultiArray, self.roba_my_odom_cb)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # Signal Related
        self.signal_sub = rospy.Subscriber('signal_sim', Bool, self.signal_cb)
        self.cur_signal = False
        self.close_to_signal = False
        self.facing_signal = False

        # Stop Sign Related
        self.stop_sign_sub = rospy.Subscriber('stop_sign_sim', Int32, self.stop_sign_cb)
        self.stop_sign_pub = rospy.Publisher('stop_sign_sim', Int32, queue_size=1)

        self.cur_cars_count = 0
        self.stop_car_list = []

        self.close_to_stop_sign = False
        self.facing_stop_sign = False
        self.encountered = False
        self.sleeped = False

        # tf rostopic
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Initialized Value
        self.dist = 0.0
        self.yaw = 0.0

        # Call mapper_real.py - For SLAM demo
        self.mapper = Mapper()

    def signal_cb(self, msg):
        """Callback to 'self.signal_sub'. """
        self.cur_signal = msg.data
    
    def stop_sign_cb(self, msg):
        self.cur_cars_count = msg.data

    def roba_my_odom_cb(self, msg):
        """Callback to `self.my_odom_sub`."""
        # msg.points[0] contains roba data
        self.dist = msg.data[0]  # roba distance
        self.yaw = msg.data[1]   # roba yaw
        #raise NotImplementedError


    def roba_scan_cb(self, msg):
        """Callback function for `self.scan_sub`."""
        #raise NotImplementedError

    def move(self):
        twist = Twist()
        # Default Speed
        twist.linear.x = 0.1     

        #Dealing with signal (finished)
        if (self.close_to_signal is True) and (self.facing_signal is True):
            if self.cur_signal:
                twist.linear.x = 0.2
            else:
                twist.linear.x = 0.0

        #Dealing with STOP sign
        if (self.close_to_stop_sign is True) and (self.facing_stop_sign is True):
            if (self.encountered is False): # Stop for 1.5s when encounter a STOP sign
                twist.linear.x = 0.0
                self.encountered = True
                # Add the car to list
                self.cur_cars_count += 1
                self.stop_car_list.append('roba')
            else:
                return # if still encountering stop sign, break the move(), go to stop_sign_helper()
        else:
            self.encountered = False
            # Remove the car from list
            if ('roba' in self.stop_car_list):
                self.stop_car_list.remove('roba')
                self.cur_cars_count -= 1
                self.sleeped = False
                
        self.cmd_vel_pub.publish(twist)

    def stop_sign_helper(self):
            if (self.sleeped is False):
                rospy.sleep(1.5)
                self.sleeped = True
            twist = Twist()

            if not (self.stop_car_list[0] == 'roba'):
                twist.linear.x = 0.0
            else:
                twist.linear.x = 0.2

            self.cmd_vel_pub.publish(twist)



    def get_pin_to_robot_position(self, pin_id):
        """Get x,y position of robot's base_link to pin_id frame"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                f'pin_{pin_id}', 
                rospy.Time())
            
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            
            return x, y
       
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            return None

    def signal_notification(self, pin_id):
        '''Let robot know when it is closing to one of the signal lights'''
        dist = self.get_pin_to_robot_position(pin_id)
        threshold = 0.2
        if dist:
            x, y = dist
            #Detect whether is CLOSE to the pin
            if ((x**2 + y**2) < 0.2): 
                # The distance to the intersection
                if (self.close_to_signal is False):
                    self.close_to_signal = True
            else:
                self.close_to_signal = False

            #Detect whether is FACING the pin
            angle = math.atan2(y, x)

            if (abs(angle) < threshold):
                if (self.facing_signal is False):
                    self.facing_signal = True
            else:
                self.facing_signal = False

    def stop_sign_notification(self, pin_id):
        dist = self.get_pin_to_robot_position(pin_id)
        threshold = 0.2
        if dist:
            x, y = dist
            #Detect whether is CLOSE to the pin
            rospy.loginfo(f"Fiducial {pin_id} is at x:{x:.2f}, y:{y:.2f}") 
            if ((x**2 + y**2) < 0.2): 
                # The distance to the intersection
                if (self.close_to_stop_sign is False):
                    self.close_to_stop_sign = True
            else:
                self.close_to_stop_sign = False

            #Detect whether is FACING the pin
            angle = math.atan2(y, x)

            if (abs(angle) < threshold):
                if (self.facing_stop_sign is False):
                    self.facing_stop_sign = True
            else:
                self.facing_stop_sign = False

                    
 

    def run(self):
        """Run the program."""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.signal_notification(110)
            self.stop_sign_notification(100)

            if self.encountered and self.facing_stop_sign and self.close_to_stop_sign:
                self.stop_sign_helper()
            else:
                self.move()

            rate.sleep()
           
if __name__ == '__main__':
    rospy.init_node('four_way_solo')
    FourWaySim().run()
    rospy.spin()
