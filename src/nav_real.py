#!/usr/bin/env python3
# Video link for Real Life bot:
# https://drive.google.com/file/d/1RQR6xB1OOFSjUODKeO7_1Wr0f6qr0yVd/view?usp=sharing

import math
import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import Point, Pose, Twist
from tf.transformations import euler_from_quaternion
from fiducial_msgs.msg import FiducialTransformArray

PI = math.pi
STD_ROTATION_SPEED = 0.2
STD_MOVING_SPEED = 0.3

class NavReal:
    def __init__(self):
        # Odom and Cmd_vel rostopic
        self.my_odom_sub = rospy.Subscriber('my_odom', Point, self.my_odom_cb)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # tf rostopic
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Aruco_detect package rostopic
        rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.fid_cb)

        # Odom related variables
        self.cur_dist = self.cur_yaw = 0.0

       
    # Odom callback 
    def my_odom_cb(self, msg):
        """Callback function for `my_odom_sub`."""
        self.cur_dist = msg.x
        self.cur_yaw = msg.y
        #raise NotImplementedError

    # Aruco detect callback
    # This function will update the robot's current location that is relatively in the global tf system
    # Without this, the robot will turn to wrong direction & move wrong distance, 
    # because it was trying to find the route based on its initial position
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

    # Turning function helper function
    # This function return the target_angle in [pi, -pi] range
    def standardize_angle(self,target_angle):
        return math.atan2(math.sin(target_angle), math.cos(target_angle))

    # Turning function
    def turn_to_heading(self, target_yaw, base_vel):
        """
        Turns the robot to heading `target_yaw` with a base velocity of
        `base_vel`.
        """
        twist = Twist()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            yaw = self.cur_yaw
            diff = self.standardize_angle(target_yaw - yaw)

            # Stop turning if lower than threshold
            if (abs(diff) < 0.03):
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                break

            twist.angular.z = base_vel * np.sign(diff)
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        #raise NotImplementedError
        
    # Rotate the robot 360
    def scan_for_fids(self):
        """
        Scans for fiducials by rotating in place. Note that the `mapper` node
        does the actual mapping.
        """

        self.turn_to_heading(PI, STD_ROTATION_SPEED)  
        self.turn_to_heading(0, STD_ROTATION_SPEED)  

        #raise NotImplementedError

    def match_pin_rotation(self, pin_id):
        """
        Rotates the robot so that its `base_link` frame's orientation matches
        that of the target pin's frame.
        """
        try:
            # Get the robot's destinition's tf based on current location, that is published in fid_cb
            transform = self.tf_buffer.lookup_transform('odom', f'pin_{pin_id}', rospy.Time(0))
            # Get the pin's yaw
            pin_rotation = transform.transform.rotation
            pin_euler = euler_from_quaternion([pin_rotation.x, pin_rotation.y, pin_rotation.z, pin_rotation.w])
            yaw = pin_euler[2]  # Get yaw

            # Rotate robot to match the pin's yaw
            self.turn_to_heading(yaw,STD_ROTATION_SPEED)

        # If there's no pin with pin_id
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Error looking up transform for pin {pin_id}: {e}")

        #raise NotImplementedError
        
    def face_pin(self, pin_id):
        """
        Rotates the robot so that it faces the target pin.
        """
        
        try:
            # Get the transform from base_link to pin
            transform = self.tf_buffer.lookup_transform('base_link', f'pin_{pin_id}', rospy.Time(0))
            
            # Get the translation (x, y) from the transform
            cur_translation = transform.transform.translation
            pin_x = cur_translation.x
            pin_y = cur_translation.y
            
            target_angle = math.atan2(pin_y, pin_x)
            cur_yaw = self.cur_yaw 
            
            # Calculate the difference between the cur_yaw and target_yaw
            yaw_diff = target_angle - cur_yaw
            yaw_diff = self.standardize_angle(yaw_diff)
            
            # Rotate the robot to face the pin
            self.turn_to_heading(cur_yaw + yaw_diff, STD_ROTATION_SPEED)
            rospy.loginfo(f"Rotating to face pin {pin_id} with target angle {target_angle}")
    
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Error looking up transform for pin {pin_id}: {e}")

        #raise NotImplementedError

    # move_to_pin helper function
    def move_forward(self, target_distance, base_vel):
        twist = Twist()
        rate = rospy.Rate(10)

        dist_traveled = 0.0

        while not rospy.is_shutdown():
            dist_traveled += self.cur_dist
            diff = abs(target_distance - dist_traveled)
            if (diff < 0.02):
                twist.linear.x = 0.0
                self.cmd_vel_pub.publish(twist)
                break

            twist.linear.x = base_vel * diff
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

    def move_to_pin(self, pin_id):
        """
        Moves the robot to the target pin.
        """

        try:
            transform = self.tf_buffer.lookup_transform('base_link', f'pin_{pin_id}', rospy.Time(0))
            cur_translation = transform.transform.translation

            # Get the pin's position (relative to the robot's base_link)
            pin_x = cur_translation.x
            pin_y = cur_translation.y

            distance_to_pin = math.sqrt(pin_x**2 + pin_y**2)
            target_distance_avoid_threshold = distance_to_pin - 0.15

            # Move the robot towards the pin
            self.move_forward(target_distance_avoid_threshold,STD_MOVING_SPEED)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Error looking up transform for pin {pin_id}: {e}")

        #raise NotImplementedError

if __name__ == '__main__':
    rospy.init_node('nav_real')

    nav = NavReal()
    nav.scan_for_fids()
    # Change the pin_id based on what is in the Lab
    target_pin_ids = [101, 106, 107, 109]
    for pin_id in target_pin_ids:
        nav.match_pin_rotation(pin_id)
        nav.face_pin(pin_id)
        nav.move_to_pin(pin_id)