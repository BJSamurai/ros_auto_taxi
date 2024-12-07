#!/usr/bin/env python3
import math
import rospy
import tf2_ros
from enum import Enum
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Int32
from actionlib_msgs.msg import GoalStatusArray, GoalID
from move_base_msgs.msg import MoveBaseActionGoal

class RobotState(Enum):
    NAVIGATING = 1
    AT_SIGNAL = 2
    AT_STOP_SIGN = 3
    IDLE = 4

class NavigationController:
    def __init__(self):
        self.move_base_status_sub = rospy.Subscriber('move_base/status', GoalStatusArray, self.move_base_status_cb)
        self.move_base_goal_sub = rospy.Subscriber('move_base/goal', MoveBaseActionGoal, self.move_base_goal_cb)
        self.move_base_goal_pub = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=1)
        self.move_base_cancel_pub = rospy.Publisher('move_base/cancel', GoalID, queue_size=1)
        
        self.signal_sub = rospy.Subscriber('signal_sim', Bool, self.signal_cb)
        self.stop_sign_sub = rospy.Subscriber('stop_sign_sim', Int32, self.stop_sign_cb)

        self.state = RobotState.IDLE
        self.current_goal = None
        self.saved_goal = None
        self.current_signal_state = False
        self.stop_sign_queue = []
        self.at_traffic_control = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.signal_distance_threshold = 0.5
        self.signal_angle_threshold = math.pi/3
        self.stop_sign_distance_threshold = 0.2
        self.stop_sign_angle_threshold = math.pi/4

    def move_base_status_cb(self, msg):
        if not msg.status_list:
            return
        latest_status = msg.status_list[-1]
        if latest_status.status == 1:
            self.state = RobotState.NAVIGATING
        elif latest_status.status == 3:
            self.state = RobotState.IDLE
            self.at_traffic_control = False

    def move_base_goal_cb(self, msg):
        self.current_goal = msg

    def signal_cb(self, msg):
        self.current_signal_state = msg.data
        if self.state == RobotState.AT_SIGNAL and self.current_signal_state:
            self.resume_navigation()

    def stop_sign_cb(self, msg):
        if self.state == RobotState.AT_STOP_SIGN:
            if 'roba' not in self.stop_sign_queue:
                self.stop_sign_queue.append('roba')
            if self.stop_sign_queue[0] == 'roba':
                rospy.sleep(1.5)
                self.stop_sign_queue.remove('roba')
                self.resume_navigation()

    def check_traffic_controls(self):
        if self.state != RobotState.NAVIGATING:
            return

        signal_pos = self.get_pin_position(101)
        if signal_pos:
            x, y = signal_pos
            signal_distance = math.sqrt(x**2 + y**2)
            signal_angle = abs(math.atan2(y, x))
            rospy.loginfo(f"Signal dist: {signal_distance:.2f}, angle: {signal_angle:.2f}, signal: {self.current_signal_state}")
            
            if signal_distance < self.signal_distance_threshold and signal_angle < self.signal_angle_threshold:
                if not self.current_signal_state:  # Red light
                    rospy.loginfo("Stopping at red signal")
                    self.at_traffic_control = True
                    self.pause_navigation(RobotState.AT_SIGNAL)

        stop_pos = self.get_pin_position(100)
        if stop_pos:
            x, y = stop_pos
            stop_distance = math.sqrt(x**2 + y**2)
            stop_angle = abs(math.atan2(y, x))
            rospy.loginfo(f"Stop dist: {stop_distance:.2f}, angle: {stop_angle:.2f}")

            if stop_distance < self.stop_sign_distance_threshold and stop_angle < self.stop_sign_angle_threshold:
                rospy.loginfo("Stopping at stop sign")
                self.at_traffic_control = True
                self.pause_navigation(RobotState.AT_STOP_SIGN)

    def get_pin_position(self, pin_id):
        try:
            transform = self.tf_buffer.lookup_transform('base_link', f'pin_{pin_id}', rospy.Time())
            return transform.transform.translation.x, transform.transform.translation.y
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None

    def pause_navigation(self, new_state):
        if self.current_goal:
            self.saved_goal = self.current_goal
            cancel_msg = GoalID()
            cancel_msg.id = self.current_goal.goal_id.id
            self.move_base_cancel_pub.publish(cancel_msg)
            self.state = new_state

    def resume_navigation(self):
        if self.saved_goal:
            self.move_base_goal_pub.publish(self.saved_goal)
            self.saved_goal = None
            self.state = RobotState.NAVIGATING
            self.at_traffic_control = False

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.check_traffic_controls()
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('navigation_controller')
    NavigationController().run()