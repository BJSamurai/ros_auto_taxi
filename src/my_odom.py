#!/usr/bin/env python3

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point 
from tf.transformations import euler_from_quaternion

class MyOdom:
    def __init__(self):
        self.roba_odom_sub = rospy.Subscriber('/roba/odom', Odometry, self.roba_odom_cb)
        self.roba_my_odom_pub = rospy.Publisher('/roba/my_odom', Point, queue_size=1)
        self.robb_odom_sub = rospy.Subscriber('/robb/odom', Odometry, self.robb_odom_cb)
        self.robb_my_odom_pub = rospy.Publisher('/robb/my_odom', Point, queue_size=1)
        self.roba_old_pose = None 
        self.roba_dist = 0.0
        self.roba_yaw = 0.0

        self.robb_old_pose = None 
        self.robb_dist = 0.0
        self.robb_yaw = 0.0
                
    def roba_odom_cb(self, msg):
        """Callback function for `odom_sub`."""
        cur_pose = msg.pose.pose
        self.roba_update_dist(cur_pose) 
        self.roba_update_yaw(cur_pose.orientation)
        self.roba_publish_data()

    def roba_update_dist(self, cur_pose):
        """
        Helper to `odom_cb`.
        Updates `self.dist` to the distance between `self.old_pose` and
        `cur_pose`.
        """
        if self.roba_old_pose is not None:
            x_diff = cur_pose.position.x - self.roba_old_pose.position.x
            y_diff = cur_pose.position.y - self.roba_old_pose.position.y
            self.roba_dist = math.sqrt(x_diff ** 2 + y_diff ** 2)
        self.roba_old_pose = cur_pose

    def roba_update_yaw(self, cur_orientation):
        """
        Helper to `odom_cb`.
        Updates `self.yaw` to current heading of robot.
        """
        orientations = [cur_orientation.x,
                cur_orientation.y,
                cur_orientation.z,
                cur_orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientations)
        # From the robot's point of view, `odom` publishes values in: (i) [0, pi)
        # for the range [12, 6) o'clock counter-clockwise, but (ii) (0, -pi] for 
        # the range (12, 6] clockwise. The line below converts values in (ii) to
        # be in the range [pi, 2 * pi), so that we have values in [0, 2 * pi -1) 
        # for the entire [12, 12) o'clock counter-clockwise range.
        self.roba_yaw = yaw if yaw >= 0 else 2 * math.pi + yaw

    def roba_publish_data(self):
        """
        Publish `self.dist` and `self.yaw` on the `my_odom` topic.
        """
        # The `Point` object we create below is not used as a geometric point,
        # but simply as a data container for `self.dist` and `self.yaw` so we can
        # publish it on `my_odom`.
        data = Point()
        data.x = self.roba_dist
        data.y = self.roba_yaw
        self.roba_my_odom_pub.publish(data)

    def robb_odom_cb(self, msg):
        """Callback function for `odom_sub`."""
        cur_pose = msg.pose.pose
        self.robb_update_dist(cur_pose) 
        self.robb_update_yaw(cur_pose.orientation)
        self.robb_publish_data()

    def robb_update_dist(self, cur_pose):
        """
        Helper to `odom_cb`.
        Updates `self.dist` to the distance between `self.old_pose` and
        `cur_pose`.
        """
        if self.robb_old_pose is not None:
            x_diff = cur_pose.position.x - self.robb_old_pose.position.x
            y_diff = cur_pose.position.y - self.robb_old_pose.position.y
            self.robb_dist = math.sqrt(x_diff ** 2 + y_diff ** 2)
        self.robb_old_pose = cur_pose

    def robb_update_yaw(self, cur_orientation):
        """
        Helper to `odom_cb`.
        Updates `self.yaw` to current heading of robot.
        """
        orientations = [cur_orientation.x,
                cur_orientation.y,
                cur_orientation.z,
                cur_orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientations)
        # From the robot's point of view, `odom` publishes values in: (i) [0, pi)
        # for the range [12, 6) o'clock counter-clockwise, but (ii) (0, -pi] for 
        # the range (12, 6] clockwise. The line below converts values in (ii) to
        # be in the range [pi, 2 * pi), so that we have values in [0, 2 * pi -1) 
        # for the entire [12, 12) o'clock counter-clockwise range.
        self.robb_yaw = yaw if yaw >= 0 else 2 * math.pi + yaw

    def robb_publish_data(self):
        """
        Publish `self.dist` and `self.yaw` on the `my_odom` topic.
        """
        # The `Point` object we create below is not used as a geometric point,
        # but simply as a data container for `self.dist` and `self.yaw` so we can
        # publish it on `my_odom`.
        data = Point()
        data.x = self.robb_dist
        data.y = self.robb_yaw
        self.robb_my_odom_pub.publish(data)

if __name__ == '__main__':
    rospy.init_node('/roba/my_odom')
    rospy.init_node('/robb/my_odom')

    MyOdom()
    rospy.spin()
