#!/usr/bin/env python3
import math
import rospy
import tf2_ros
from enum import Enum
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Int32
from actionlib_msgs.msg import GoalStatusArray, GoalID
from move_base_msgs.msg import MoveBaseActionGoal

SIGNAL_FID_STRAIGHT_FRONT = 110
SIGNAL_FID_STRAIGHT_BACK = 107
SIGNAL_FID_CORNER = 105
STOP_FID = 101

class RobotState(Enum):
    NAVIGATING = 1
    AT_SIGNAL = 2
    AT_STOP_SIGN = 3
    IDLE = 4

class NavigationController:
    def __init__(self):
        rospy.loginfo("Initializing NavigationController...")
        # move_base
        self.move_base_status_sub = rospy.Subscriber('move_base/status', GoalStatusArray, self.move_base_status_cb)
        self.move_base_goal_sub = rospy.Subscriber('move_base/goal', MoveBaseActionGoal, self.move_base_goal_cb)
        self.move_base_goal_pub = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=1)
        self.move_base_cancel_pub = rospy.Publisher('move_base/cancel', GoalID, queue_size=1)
        
        # signal & stop sign
        self.signal_sub = rospy.Subscriber('traffic_signal', Bool, self.signal_cb)
        self.stop_sign_sub = rospy.Subscriber('stop_sign', Int32, self.stop_sign_cb)
        self.stop_sign_pub = rospy.Publisher('stop_sign', Int32, queue_size = 1)

        # move_base related variable
        self.state = RobotState.IDLE
        self.current_goal = None
        self.saved_goal = None
        self.final_destination = None

        # signal & stop sign related variable
        self.current_signal_state = False
        self.stop_sign_queue = []
        self.stop_sign_count = 0
        # signal & stop sign related indicator
        self.stop_sign_used_recently = False
        self.at_traffic_control = False
        self.at_which_signal = 0

        # tf
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # signal & stop sign related threshold
        self.signal_distance_threshold = 0.3
        self.signal_angle_threshold = math.pi/6
        self.stop_sign_distance_threshold = 0.3
        self.stop_sign_angle_threshold = math.pi/3
        

    # move_base_status_cb
    # The callback of move_base/status, it help reset variables to IDLE.
    def move_base_status_cb(self, msg):
        if not msg.status_list:
            return
        latest_status = msg.status_list[-1]                
        if latest_status.status == 1:
            self.state = RobotState.NAVIGATING
        elif latest_status.status == 3:
            self.state = RobotState.IDLE
            self.at_traffic_control = False
            self.current_goal = None

    # move_base_goal_cb
    # The callback of move_base/goal, it record the new goal that is set. The new goal doesn't contain path goal on the way.
    def move_base_goal_cb(self, msg):
        rospy.loginfo("Received new navigation goal")
        self.current_goal = msg
        self.final_destination = msg

    # signal_cb 
    # The callback of signal rostopic. It resume the robot's move_base when signal turns 'Green' for certain signal light.
    def signal_cb(self, msg):
        self.current_signal_state = msg.data
        if self.state == RobotState.AT_SIGNAL and self.current_signal_state and self.at_which_signal == 1: # STRAIGHT SIGNAL
            rospy.loginfo("Signal turned green, resuming navigation")
            self.at_which_signal = 0
            self.resume_navigation()
        elif self.state == RobotState.AT_SIGNAL and not self.current_signal_state and self.at_which_signal == 2: # CORNER SIGNAL
            rospy.loginfo("Signal turned green, resuming navigation")
            self.at_which_signal = 0
            self.resume_navigation()

    # stop_sign_cb
    # This callback of stop sign rostopic. It add robot to a queue that will pop the first robot after a 3s stop.
    def stop_sign_cb(self, msg):
        if self.state == RobotState.AT_STOP_SIGN:
            if 'roba' not in self.stop_sign_queue:
                self.stop_sign_queue.append('roba')
                rospy.loginfo("ADD roba to stop sign queue")
            if self.stop_sign_queue[0] == 'roba':
                rospy.loginfo("Stop sign wait complete, resuming navigation")
                self.stop_sign_queue.pop(0)
                self.stop_sign_count -= 1
                self.stop_sign_pub.publish(self.stop_sign_count)
                self.stop_sign_used_recently = True
                self.resume_navigation()

    # check_traffic_controls
    # This function detects whether the robot in stop sign / signal or not.
    # If it is, pause robot's move_base
    def check_traffic_controls(self):
        if self.state != RobotState.NAVIGATING:
            return

        # Signal on straight path
        signal_pos_straight_front = self.get_pin_position(SIGNAL_FID_STRAIGHT_FRONT)
        if signal_pos_straight_front:
            x, y = signal_pos_straight_front
            signal_distance = math.sqrt(x**2 + y**2)
            signal_angle = abs(math.atan2(y, x))
            
            if signal_distance < self.signal_distance_threshold and signal_angle < self.signal_angle_threshold:
                if not self.current_signal_state:  # Red light
                    rospy.loginfo("Stopping at red signal for STRAIGHT")
                    self.at_which_signal = 1
                    self.at_traffic_control = True
                    self.pause_navigation(RobotState.AT_SIGNAL)

        # Signal on corner
        signal_pos_corner = self.get_pin_position(SIGNAL_FID_CORNER)
        if signal_pos_corner:
            x, y = signal_pos_corner
            signal_distance = math.sqrt(x**2 + y**2)
            signal_angle = abs(math.atan2(y, x))
            
            if signal_distance < self.signal_distance_threshold and signal_angle < self.signal_angle_threshold:
                if self.current_signal_state:  # Red light for corner
                    rospy.loginfo("Stopping at red signal for CORNER")
                    self.at_which_signal = 2
                    self.at_traffic_control = True
                    self.pause_navigation(RobotState.AT_SIGNAL)

        # Stop sign
        stop_pos = self.get_pin_position(STOP_FID)
        if stop_pos:
            x, y = stop_pos
            stop_distance = math.sqrt(x**2 + y**2)
            stop_angle = abs(math.atan2(y, x))

            if stop_distance < self.stop_sign_distance_threshold and stop_angle < self.stop_sign_angle_threshold:
                if self.stop_sign_used_recently:
                    return
                rospy.loginfo("Stopping at stop sign")
                self.stop_sign_count += 1
                self.stop_sign_pub.publish(self.stop_sign_count)
                self.at_traffic_control = True
                self.pause_navigation(RobotState.AT_STOP_SIGN)
                rospy.sleep(3.0)

                if self.stop_sign_count <= 1:  # If queue only have this robot
                    rospy.loginfo("First at stop sign, resuming after wait")
                    self.stop_sign_used_recently = True
                    self.stop_sign_count -= 1
                    self.resume_navigation()
            else:
                if self.stop_sign_used_recently:
                    self.stop_sign_used_recently = False

    # get_pin_position
    # This function provides robot's 'base_link' relative postion to fiducial pin.
    def get_pin_position(self, pin_id):
        try:
            transform = self.tf_buffer.lookup_transform('base_link', f'pin_{pin_id}', rospy.Time())
            return transform.transform.translation.x, transform.transform.translation.y
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logdebug(f"Could not get pin position: {e}")
            return None

    # pause_navigation
    # This function store robot's current move_base goal, and cancel its move_base to pause moving.
    def pause_navigation(self, new_state):
        if self.current_goal:
            rospy.loginfo(f"Pausing navigation, transitioning from {self.state} to {new_state}")
            self.saved_goal = self.current_goal
            cancel_msg = GoalID()
            cancel_msg.id = self.current_goal.goal_id.id
            self.move_base_cancel_pub.publish(cancel_msg)
            self.state = new_state
            rospy.loginfo("Navigation paused")
        else:
            rospy.logwarn("Cannot pause navigation: no active goal")

    # resume_navigation
    # This function publish a move_base goal that is stored in previous pause.
    def resume_navigation(self):
        if self.saved_goal:
            self.move_base_goal_pub.publish(self.saved_goal)
            self.saved_goal = None
            self.state = RobotState.NAVIGATING
            self.at_traffic_control = False

    # check_final_goal_completion
    # This function check robot's location to the final destination. If it is close to it, cancel current move_base.
    def check_final_goal_completion(self):
        if not self.final_destination:  # No final goal to check
            return
        
        try:
            # Get transform from base_link to map (robot's current pose in map frame)
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rospy.Time()
            )

            # Extract current robot position and orientation
            current_x = transform.transform.translation.x
            current_y = transform.transform.translation.y
            current_orientation = transform.transform.rotation

            # Get final goal position and orientation
            goal_pose = self.final_destination.goal.target_pose.pose
            goal_x = goal_pose.position.x
            goal_y = goal_pose.position.y
            goal_orientation = goal_pose.orientation

            # Calculate distance to final goal
            distance_to_goal = math.sqrt(
                (goal_x - current_x)**2 + 
                (goal_y - current_y)**2
            )

            from tf.transformations import euler_from_quaternion
            current_angles = euler_from_quaternion([
                current_orientation.x,
                current_orientation.y,
                current_orientation.z,
                current_orientation.w
            ])
            goal_angles = euler_from_quaternion([
                goal_orientation.x,
                goal_orientation.y,
                goal_orientation.z,
                goal_orientation.w
            ])

            # check yaw difference
            angle_diff = abs(current_angles[2] - goal_angles[2])
            angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

            # Check if robot is close enough to final goal
            if distance_to_goal < 0.02 and abs(angle_diff) < math.radians(20):
                rospy.loginfo("Close enough to final goal! Marking as complete.")
                self.state = RobotState.IDLE
                self.at_traffic_control = False
                
                # Cancel any ongoing navigation
                cancel_msg = GoalID()
                if hasattr(self.final_destination, 'goal_id'):
                    cancel_msg.id = self.final_destination.goal_id.id
                self.move_base_cancel_pub.publish(cancel_msg)
                
                # Clear all goals
                self.saved_goal = None
                self.current_goal = None
                self.final_destination = None

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Could not check goal completion: {e}")

    def run(self):
        rate = rospy.Rate(10)
        rospy.loginfo("NavigationController running...")
        while not rospy.is_shutdown():
            self.check_final_goal_completion()
            self.check_traffic_controls()
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('taxi_controller_solo')
    NavigationController().run()