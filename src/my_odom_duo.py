#!/usr/bin/env python3

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point 
from tf.transformations import euler_from_quaternion

class MyOdom:
    def __init__(self):
        # Subscribe to both robot odometries
        self.roba_odom_sub = rospy.Subscriber('/roba/odom', Odometry, self.roba_odom_cb)
        self.robb_odom_sub = rospy.Subscriber('/robb/odom', Odometry, self.robb_odom_cb)
        
        # Single publisher for Point[] message
        self.my_odom_pub = rospy.Publisher('my_odom', PointArray, queue_size=1)

        # Tracking variables for each robot
        self.roba_old_pose = None 
        self.roba_dist = 0.0
        self.roba_yaw = 0.0

        self.robb_old_pose = None 
        self.robb_dist = 0.0
        self.robb_yaw = 0.0
        
        # Flag to ensure we have data from both robots before publishing
        self.roba_updated = False
        self.robb_updated = False
                
    def roba_odom_cb(self, msg):
        """Callback function for roba odometry."""
        cur_pose = msg.pose.pose
        self.update_dist('roba', cur_pose) 
        self.update_yaw('roba', cur_pose.orientation)
        
    def robb_odom_cb(self, msg):
        """Callback function for robb odometry."""
        cur_pose = msg.pose.pose
        self.update_dist('robb', cur_pose) 
        self.update_yaw('robb', cur_pose.orientation)
        
    def update_dist(self, robot, cur_pose):
        """
        Updates distance for the specified robot.
        """
        if robot == 'roba':
            old_pose = self.roba_old_pose
            if old_pose is not None:
                x_diff = cur_pose.position.x - old_pose.position.x
                y_diff = cur_pose.position.y - old_pose.position.y
                self.roba_dist += math.sqrt(x_diff ** 2 + y_diff ** 2)
            self.roba_old_pose = cur_pose
            self.roba_updated = True
        
        elif robot == 'robb':
            old_pose = self.robb_old_pose
            if old_pose is not None:
                x_diff = cur_pose.position.x - old_pose.position.x
                y_diff = cur_pose.position.y - old_pose.position.y
                self.robb_dist += math.sqrt(x_diff ** 2 + y_diff ** 2)
            self.robb_old_pose = cur_pose
            self.robb_updated = True
        
        # Publish if we have updates from both robots
        if self.roba_updated and self.robb_updated:
            self.publish_data()

    def update_yaw(self, robot, cur_orientation):
        """
        Updates yaw for the specified robot.
        """
        orientations = [
            cur_orientation.x,
            cur_orientation.y,
            cur_orientation.z,
            cur_orientation.w
        ]
        (roll, pitch, yaw) = euler_from_quaternion(orientations)
        
        # Convert negative yaw to [0, 2π) range
        yaw = yaw if yaw >= 0 else 2 * math.pi + yaw
        
        if robot == 'roba':
            self.roba_yaw = yaw
        elif robot == 'robb':
            self.robb_yaw = yaw

    def publish_data(self):
        """
        Publish data for both robots as a Point[] message.
        """
        # Create a list of Points
        data = [
            Point(x=self.roba_dist, y=self.roba_yaw, z=0),  # roba point
            Point(x=self.robb_dist, y=self.robb_yaw, z=0)   # robb point
        ]
        
        # Publish the Point[] message
        self.my_odom_pub.publish(data)
        
        # Reset update flags
        self.roba_updated = False
        self.robb_updated = False

def main():
    rospy.init_node('my_odom')
    MyOdom()
    rospy.spin()

if __name__ == '__main__':
    main()