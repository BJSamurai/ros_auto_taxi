# ROS_Auto_Taxi

To start the navigation:
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map_final.yaml

To start the road system:
roslaunch ros_auto_taxi ros_auto_taxi.launch

To start demo waypoint:
rosrun ros_auto_taxi way_point.py