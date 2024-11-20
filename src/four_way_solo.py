#!/usr/bin/env python3
import math
import rospy
import tf2_ros
import cv2 as cv
import cv_bridge
import numpy
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist, Point
from matplotlib import pyplot as plt
from std_msgs.msg import String, Bool, Float32MultiArray
import numpy as np
from fiducial_msgs.msg import FiducialTransformArray


class FourWaySim:
    def __init__(self):

        # roba
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.roba_scan_cb)
        self.my_odom_sub = rospy.Subscriber('my_odom', Float32MultiArray, self.roba_my_odom_cb)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # Signal Related
        self.signal_sub = rospy.Subscriber('signal_sim', Bool, self.signal_cb)
        self.cur_signal = False

        # tf rostopic
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Aruco_detect package rostopic
        rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.fid_cb)

        # Initialized Value
        self.roba_dist = 0.0
        self.robb_dist = 0.0

        self.roba_indi = False
        self.robb_indi = False
        self.roba_indi_two = False
        self.robb_indi_two = False



    def fid_cb(self,msg):
        '''Callback function for 'fiducial_transforms' '''
        '''
            What is contained in msg from 'fiducial_transforms'?
                # A set of camera to fiducial transform with supporting data corresponding to an image
                Header header
                int32 image_seq
                FiducialTransform[] transforms 
        '''
        imageTime = msg.header.stamp

        for m in msg.transforms:
            id = m.fiducial_id
            trans = m.transform.translation
            rot = m.transform.rotation

            # Create a Transform
            t = TransformStamped()
            t.child_frame_id = f'pin_{id}'
            t.header.frame_id = 'odom'
            t.header.stamp = imageTime

            t.transform.translation.x = trans.x
            t.transform.translation.y = trans.y
            t.transform.translation.z = trans.z
            t.transform.rotation.x = rot.x
            t.transform.rotation.y = rot.y
            t.transform.rotation.z = rot.z
            t.transform.rotation.w = rot.w

            # Publish the tf
            self.tf_broadcaster.sendTransform(t)

    def signal_cb(self, msg):
        """Callback to 'self.signal_sub'. """
        self.cur_signal = msg.data

    def roba_my_odom_cb(self, msg):
        """Callback to `self.my_odom_sub`."""
        # msg.points[0] contains roba data
        self.roba_dist = msg.data[0]  # roba distance
        self.roba_yaw = msg.data[1]   # roba yaw
        #raise NotImplementedError


    def roba_scan_cb(self, msg):
        """Callback function for `self.scan_sub`."""
        #raise NotImplementedError

    def move(self):
        roba_twist = Twist()
        robb_twist = Twist()

        # Default Speed
        roba_twist.linear.x = 0.2
        robb_twist.linear.x = 0.2
        

        ''' # Demo variables, Cross Intersections.
        if (self.roba_dist >= 0.75):
            self.roba_indi = True


        if (self.roba_dist >= 1.25):
            self.roba_indi_two = True


        '''

        if self.cur_signal:
            #if (self.roba_indi):
                roba_twist.linear.x = 0.2

        else:
            #if (self.roba_indi) and (not self.roba_indi_two):
                roba_twist.linear.x = 0.0


        self.cmd_vel_pub.publish(roba_twist)
            
  #roba X 0.8,0.3 robb Y 1.2,0.3
        
    def run(self):
        """Run the program."""
        rate = rospy.Rate(10)
        

        while not rospy.is_shutdown():
            self.move()
            rate.sleep()

    #### Calculation of the filtered images
    
           
if __name__ == '__main__':
    rospy.init_node('four_way_solo')
    FourWaySim().run()
    rospy.spin()
