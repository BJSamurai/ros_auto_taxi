#!/usr/bin/env python

import rospy
import math
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion

class PathOrientationTracker:
    def __init__(self):
        rospy.init_node('path_orientation_tracker', anonymous=True)
        
        # Subscribe to both path and goal
        self.path_sub = rospy.Subscriber('/move_base/NavfnROS/plan', 
                                       Path, 
                                       self.path_callback)
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal',
                                        PoseStamped,
                                        self.goal_callback)
        
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.tf_listener = tf.TransformListener()
        
        self.target_yaw = None
        self.is_turning = False
        self.last_goal = None
        self.rate = rospy.Rate(10)
        
    def get_current_yaw(self):
        try:
            (_, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            euler = euler_from_quaternion(rot)
            return euler[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None
            
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def calculate_heading(self, start_point, end_point):
        dx = end_point.x - start_point.x
        dy = end_point.y - start_point.y
        return math.atan2(dy, dx)
        
    def turn_to_target(self):
        current_yaw = self.get_current_yaw()
        if current_yaw is None or self.target_yaw is None:
            return False
            
        angle_diff = self.normalize_angle(self.target_yaw - current_yaw)
        
        cmd = Twist()
        # Explicitly set all linear velocities to 0
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        
        if abs(angle_diff) < 0.1:
            # When stopping, make sure all velocities are zero
            cmd.angular.x = 0.0
            cmd.angular.y = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            return True
            
        # Set only angular z velocity for rotation, keep others at 0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = max(min(angle_diff, 0.5), -0.5)
        self.cmd_vel_pub.publish(cmd)
        return False

    def goal_callback(self, goal_msg):
        if self.last_goal is None or \
           abs(goal_msg.pose.position.x - self.last_goal.pose.position.x) > 0.1 or \
           abs(goal_msg.pose.position.y - self.last_goal.pose.position.y) > 0.1:
            
            self.last_goal = goal_msg
            self.is_turning = True
            print("\nNew goal received, initiating turn")
        
    def path_callback(self, path_msg):
        if not self.is_turning or len(path_msg.poses) < 2:
            return
            
        start_point = path_msg.poses[0].pose.position
        next_point = path_msg.poses[1].pose.position
        
        self.target_yaw = self.calculate_heading(start_point, next_point)
        
        print("Target Yaw: {:.2f} radians ({:.2f} degrees)".format(
            self.target_yaw, math.degrees(self.target_yaw)))
        
        while self.is_turning and not rospy.is_shutdown():
            if self.turn_to_target():
                self.is_turning = False
                print("Turning complete")
            self.rate.sleep()

def main():
    tracker = PathOrientationTracker()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()