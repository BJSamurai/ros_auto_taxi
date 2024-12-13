#!/usr/bin/env python3
import math
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalID

class WaypointManager:
    def __init__(self):
        rospy.init_node('waypoint_manager')
        
        # Publishers
        self.move_base_goal_pub = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=1)
        self.move_base_cancel_pub = rospy.Publisher('move_base/cancel', GoalID, queue_size=1)
        
        # TF setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Waypoint management
        self.waypoints = []
        self.current_waypoint_index = 0
        self.is_active = False
        self.completion_threshold = 0.15
        
        rospy.loginfo("WaypointManager initialized")

    def set_waypoints(self, waypoint_list):
        """
        Set a list of waypoints for the robot to visit
        waypoint_list: list of [x, y, orientation_z, orientation_w]
        Example: [[1.0, 0.0, 0.0, 1.0], [2.0, 2.0, 0.707, 0.707]]
        """
        self.waypoints = []
        self.current_waypoint_index = 0
        
        for wp in waypoint_list:
            goal = MoveBaseActionGoal()
            goal.goal.target_pose.header.frame_id = "map"
            goal.goal.target_pose.pose.position.x = wp[0]
            goal.goal.target_pose.pose.position.y = wp[1]
            goal.goal.target_pose.pose.orientation.z = wp[2]
            goal.goal.target_pose.pose.orientation.w = wp[3]
            self.waypoints.append(goal)
        
        rospy.loginfo(f"Loaded {len(self.waypoints)} waypoints")
        self.is_active = True
        self.send_next_waypoint()

    def send_next_waypoint(self):
        """Send the next waypoint if available"""
        if not self.is_active:
            return
            
        if self.current_waypoint_index < len(self.waypoints):
            next_goal = self.waypoints[self.current_waypoint_index]
            next_goal.goal.target_pose.header.stamp = rospy.Time.now()
            self.move_base_goal_pub.publish(next_goal)
            rospy.loginfo(f"Navigating to waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}")
        else:
            rospy.loginfo("Completed all waypoints!")
            self.is_active = False

    def check_waypoint_progress(self):
        """Check if current waypoint has been reached"""
        if not self.is_active or self.current_waypoint_index >= len(self.waypoints):
            return
            
        try:
            # Get current robot position
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rospy.Time()
            )

            # Get current waypoint
            current_waypoint = self.waypoints[self.current_waypoint_index]
            
            # Calculate distance to waypoint
            current_x = transform.transform.translation.x
            current_y = transform.transform.translation.y
            goal_x = current_waypoint.goal.target_pose.pose.position.x
            goal_y = current_waypoint.goal.target_pose.pose.position.y
            
            distance = math.sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2)
            
            if distance < self.completion_threshold:
                rospy.loginfo(f"Reached waypoint {self.current_waypoint_index + 1}")
                
                # Cancel current goal
                cancel_msg = GoalID()
                self.move_base_cancel_pub.publish(cancel_msg)

                rospy.sleep(0.5)
                
                # Move to next waypoint
                self.current_waypoint_index += 1
                self.send_next_waypoint()

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Could not check waypoint progress: {e}")

    def run(self):
        """Main run loop"""
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            self.check_waypoint_progress()
            rate.sleep()

if __name__ == '__main__':
    try:
        manager = WaypointManager()
        
        waypoints = [
            [1.0, 0.0, 0.707, 0.707],  # x, y, orientation_z, orientation_w
            [1.0, -1.5, 0.707, 0.707],
            [1.0, -1.8, 1.0, 0.0],
            [1.0, -2.2, 1.0, 0.0],
            [0.0, -2.2, 1.0, 0.0],
            [0.0, -1.1, 0.0, 1.0],
            [0.6, -1.1, 0.0, 1.0],
            [1.0, -1.14, -0.707, 0.707],
            [1.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0]
        ]
        manager.set_waypoints(waypoints)
        
        manager.run()
    except rospy.ROSInterruptException:
        pass