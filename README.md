---
title: Auto Taxi Simulation
author: Chao An
date: Dec 10 2024
---
## Introduction
As autonomous driving technology rapidly advances and is being adopted by major electric vehicle companies, the future of transportation stands at a fascinating crossroads. This project aims to simulate aspects of autonomous driving systems using the Robot Operating System (ROS), drawing inspiration from real-world implementations and innovations.
The project was particularly motivated by the growing presence of autonomous taxis in Seattle and mainland China, where companies are successfully deploying self-driving vehicles in complex urban environments. These real-world applications demonstrate the practical viability of autonomous systems and their potential to revolutionize urban mobility.
A key inspiration for this project comes from Beijing's sophisticated traffic management system, which integrates real-time signal light information into navigation applications. This system provides drivers with precise timing data about traffic signals, including countdown information for red lights. This integration closely parallels ROS's topic-based communication architecture, making it an ideal candidate for simulation in a ROS environment.
This project focuses on implementing three crucial features of autonomous driving systems:

Autonomous navigation within a pre-mapped environment, emphasizing efficient path planning and execution
Intelligent interaction with traffic control elements, including stop signs and traffic signals
A passenger service system that handles customer pickup and dropoff operations

## What was created
### Technical descriptions, illustrations

The environment mapping utilizes the gmapping package, implementing Simultaneous Localization and Mapping (SLAM) techniques. This approach combines laser scan data with the robot's odometric information to construct a detailed 2D occupancy grid map of the environment. The resulting map serves as the foundation for autonomous navigation and path planning.

__Navigation System__
Navigation is handled by the move_base framework, which provides:

Global path planning using Dijkstra's algorithm to determine optimal routes
Local path planning for real-time obstacle avoidance
Dynamic trajectory adjustment based on sensor feedback
Integration with the robot's velocity controllers for smooth motion execution

__Traffic Control Recognition__
The system employs the aruco_detect library for fiducial marker detection and interpretation. This component:

Processes video feed from the robot's camera in real-time
Identifies and decodes fiducial markers representing traffic elements
Stores marker positions in the robot's odometry system for future reference

__Obstacle Detection and Avoidance__
Safety and collision avoidance are managed through a LIDAR-based perception system that:

Continuously scans the environment for dynamic obstacles and pedestrians
Creates point cloud representations of detected objects
Integrates with the navigation stack to enable real-time path adjustments
Implements configurable safety zones and stopping distances

The system maintains a hierarchical control structure where high-level navigation goals can be modified based on real-time sensor data and traffic control information, ensuring safe and efficient autonomous operation in dynamic environments.

### Discussion of interesting algorithms, modules, techniques

1. Basic Mapping and Navigation
For this project, I integrated three key components to achieve autonomous navigation: gmapping for environment mapping, move_base for path planning, and Lidar sensing for real-time perception. The gmapping package implements SLAM techniques to construct a global map as the robot explores it prior to navigation. Using LIDAR sensor data, the system captures any components that weren't scanned during the mapping phase and uses it to create a local map to avoid unmapped obstacles.
With the environment mapped, the move_base package takes charge of navigation tasks. It processes the map data to generate a cost map, which helps the robot understand where it can safely travel. Before initiating any forward movement, the robot first rotates to align itself with its planned path direction, ensuring smooth and efficient navigation. This navigation system not only plans efficient routes but also ensures compliance with traffic rules detected through fiducial markers. The robot's LIDAR sensor continuously monitors its surroundings, enabling real-time detection and avoidance of dynamic obstacles that weren't present during the initial mapping phase, such as crossing pedestrians.

2. Dealing with Signals & Stop Signs
The traffic control system uses fiducial markers to represent signal lights and stop signs. When the robot approaches a signal light fiducial, it subscribes to a ROS topic that updates the light status every 30 seconds. Upon detecting a red light, the system stores the current move_base goal and temporarily halts navigation. Navigation automatically resumes when the signal turns green.
For stop sign management, the system maintains an array of vehicles waiting at the intersection. When a robot encounters a stop sign, it adds itself to this queue and comes to a complete stop. After a mandatory 3-second wait, the first robot in the queue is permitted to proceed, and its entry is removed from the array. Subsequent robots must remain stopped until they reach the front of the queue, implementing a first-come-first-serve traffic management system.

3. Pickup passengers
For passenger management, the system maintains a queue of pickup and dropoff locations read from a text file, each stored as XY coordinates. The robot processes these locations sequentially, navigating to each pickup/drop off point in order. Once a passenger location is reached, that destination is removed from the queue, and the robot proceeds to the next location. This simple yet effective system ensures orderly passenger service on a first-come-first-served basis.

### Demo Video for Real Life demo

https://drive.google.com/file/d/1nfp36Xyu62P1gIS2hcw-TmGPWURKG40f/view?usp=sharing
I also explained my program logic in the video.
This first half of demo video contain green light scenario for traffic signal. The later half contain red light scenario.

### Guide on how to use the code written in real life

To map the environment, use command
```
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```
And you may also use the teletop function to navigate robot around while mapping.
```
roslaunch turtlebot3_teletop turtlebot3_teletop_keys.launch
```

To run the program, use command
```
roslaunch ros_auto_taxi ros_auto_taxi.launch
```

To run the auto navigation, use command
```
roslaunch turtlebot3_navigation turtlebot3_navigation map_file:= map_file_name_here.yaml
```

To run the program by itself with demo passenger locations, use this command after run
```
rosrun ros_auto_taxi way_point.py
```

It will lead the robot to passenger position and take it back to start postition.


### Clear description and tables of source files, nodes, messages, actions and so on

| File name | Description |
|----------|----------|
| taxi_solo.py | include reaction towards signal/stop sign |
| mapper.py | mapping fiducial location |
| my_odom_solo.py | retrieve useful odom information |
| traffic_signal.py | publish signal lights shift |
| stop_sign.py| publish stop sign information |
| way_point.py | publish passenger location and destination |

| Message Name | Description | Message Type |
|-------------|-------------|--------------|
| /scan | LIDAR scan data for obstacle detection and mapping | sensor_msgs/LaserScan |
| /move_base/status | Navigation status updates from move_base | actionlib_msgs/GoalStatusArray |
| /move_base/goal | Target pose for navigation | move_base_msgs/MoveBaseActionGoal |
| /move_base/cancel | Cancel current navigation goal | actionlib_msgs/GoalID |
| /cmd_vel | Robot velocity commands | geometry_msgs/Twist |
| /traffic_signal | Traffic light status updates | std_msgs/Bool |
| /stop_sign | Stop sign detection and management | std_msgs/Int32 |
| /odom | Robot odometry data (position/velocity) | nav_msgs/Odometry |
| /my_odom | A simplified odom topic | std_msgs/Float32MultiArray |

## Story of the project. 
My initial vision for this project was ambitious - I wanted to create a multi-robot system operating in a mapped road network. I imagined two robots coordinating to serve passengers, with the nearest robot responding to pickup requests. These robots would navigate while following traffic rules by recognizing Fiducial markers representing traffic signals and stop signs. They would also handle real-world scenarios like pedestrian crossings and yielding to emergency vehicles. However, as I delved deeper into the implementation, I realized that developing and coordinating two robots would be too complex for a solo project. This led me to strategically narrow my focus to perfecting a single robot's operation within the traffic system.
For the navigation system, I considered two potential approaches. My primary plan utilized move_base with gmapping, which offered realistic navigation and flexibility in handling new destinations. I also kept a simpler backup plan using line or wall following algorithms in case the primary approach proved too challenging.
As I progressed with the implementation, I encountered several technical hurdles. During the mapping phase, I discovered that gmapping's default parameters weren't well-suited for small environments. The system would scan distant objects too early, and noise significantly impacted the mapping quality. This led me to develop optimized parameters and compile troubleshooting tips, which I later documented in the FAQ section.
The move_base implementation presented its own set of challenges, particularly in confined spaces like narrow mazes with approximately 20cm wide paths. The default path-finding algorithms, while effective in larger spaces like rooms, performed poorly in these tight conditions. I spent considerable time fine-tuning the system, which required understanding the complex relationships between costmap parameters, DWA planner settings, and move_base configurations. While these components operate independently, their effectiveness depends on careful synchronization - a challenge that proved both frustrating and enlightening as I worked to optimize the system's performance.

### Your own assessment
The project successfully implements basic autonomous taxi functionality, demonstrating effective integration of mapping, navigation, and traffic rule compliance. The system achieves its core objectives of autonomous movement, traffic sign response, and passenger management.

There are two main areas for potential improvement: implementing Computer Vision for more realistic traffic sign detection beyond fiducial markers, and expanding to a multi-robot system as originally envisioned. These enhancements would bring the system closer to real-world autonomous taxi operations.

Also, it would be better to develop my own version of path finding. The current move_base package works poorly on small environment. Besides, move_base is a black box that the internal algorithm is not clearly showed. The only way to improve its effeciency is to tune it through changing parameters. For future improvement, a self designed path finding algorithm is needed.