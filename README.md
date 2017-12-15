# Self driving car system integration project
## Implemented by Team Rushers

#### Dec 15, 2017

# Objective

The objective of the project is to control CARLAs throttle, steering and brakes to navigate map waypoints to successfully drive the car on a highway. The car is expected to follow lanes, traffic lights and objects and plan a trajectory to follow based on the waypoints. The car should be able to classify “Red”, “Green”, and “Yellow” lights and be able to stop/start/slow down based on the traffic signal.
Before running our code on CARLA we developed the code to work in a simulator. 
The simulator works in a very similar way to CARLA as all ROS nodes and topics are same. So if our system works in the simulator, we expect that it should also work on CARLA. 

# The system architecture and principles behind it

The system architecture consists of the following modules:

## Perception

Traffic light detection - We used a deep neural net to detect if the upcoming traffic light is red or not. We trained the classifier once with images from the simulator and once with real images from the ROS bag. A detailed description of the architecture and training parameters can be found in the respective section below (under additional resources implemented) 
We employed the MobileNet architecture to efficiently detect / classify traffic lights. We applied transfer learning … and implemented on two modes as follows: 
•	Simulator mode: classifies whole images as either red/green/yellow. The model was trained with several datasets using the Tensorflow Image Retraining Example 
•	Test-site mode: we employed <> framework to locate a traffic light…

## Planning
The waypoint updater node publishes a queue of n waypoints ahead of the vehicle position, each with a target velocity. For the simulator, n=100 is sufficient. For the site (the real-world test track), we reduce to n=20. We dequeued traversed waypoints and enqueued new points, preserving and reusing those in the middle. When a light-state changes, the entire queue is updated. The vehicle stops at the final base waypoint. This module is performed using the ROS Package Waypoint updater which is explained as below:
o	Waypoint Updater - sets target velocity for each waypoint based on upcoming traffic lights and obstacles.  This node subscribed to the nodes /base_waypoints, /current_pose, /obstacle_waypoint, and /traffic_waypoint topics, and published a list of waypoints ahead of the car with target velocities to the /final_waypoints topic.

## Control subsystems

The control subsystem is implemented using the ROS Package drive-by-wire which adjusts throttle and brakes according to the velocity targets published by the waypoint follower (which is informed by the waypoint updater node). If the list of waypoints contains a series of descending velocity targets, the PID velocity controller (in the twist controller component of DBW) will attempt to match the target velocity
o	DBW (Drive by Wire) - takes target trajectory information as input and sends control commands to navigate the vehicle.  The dbw_node subscribes to the /current_velocity topic along with the /twist_cmd topic to receive target linear and angular velocities. Additionally, this node subscribes to /vehicle/dbw_enabled, which indicates if the car is under dbw or driver control. This node will publish throttle, brake, and steering commands to the /vehicle/throttle_cmd, /vehicle/brake_cmd, and /vehicle/steering_cmd topics.

![alt text](https://github.com/ayanangshu/CarND-Capstone/blob/master/imgs/architecture.png)

## Implementation Nodes

The diagram below illustrates the system architecture. The autonomous vehicle controller is composed of three major units: perception, planning, and control.

![alt text](https://github.com/ayanangshu/CarND-Capstone/blob/master/imgs/Implementation%20Node.png)
  
  a: /camera/image_raw
  b: /current_pose
  c: /current_velocity
  d: /vehicle/dbw_enabled
  e: /traffic_waypoint
  f: /base_waypoints
  g: /final_waypoints
  h: /twist_cmd
  i: /vehicle/throttle_cmd
  j: /vehicle/brake_cmd
  k: /vehicle/steering_cmd

# Operation

There are three modes in which the controller operates:
•	site: When at the test site, this mode is launched. This mode can be run simultaneously with a rosbag to test the traffic light     classifier
•	sitesim: emulates the test site in the simulator at the first traffic light
•	styx: When using the term3 simulator, this mode is launched. The simulator communicates through server.py and bridge.py
These modes are started by roslaunch. For example, to run the styx (simulator) version we run:
roslaunch launch/styx.launch

# Team

•	Boris Dayma, Lead
•	Chris Ferone
•	Taiki Nishime
•	Pabasara Karunanayake
•	Ayanangshu Das

# Results

•	Video from the dash camera onboard the test vehicle:
•	Point cloud visualization:  
•	A map of the test run can be found at <>
•	Log file, ROS bag, and feedback:  
  Below is a visualization of the LIDAR point cloud from the team's test run on the autonomous car. <>

# Additional Resources Implemented

## Traffic Light Image Classification

The perception subsystem dynamically classifies the color of traffic lights in front of the vehicle. In the given simulator and test site environment, the car faces a single traffic light or a set of 3 traffic lights in the same state (green, yellow, red). We assume it is not possible to have multiple traffic lights in the different states at the same time.
We have considered classifying the entire image using CNN approach to solve the traffic light classification task. We used different set of models and got the following results

-Small mobilenet (2Mo): 79.2% test accuracy (664 test samples)
Our program did an exponential moving average on the probability of seeing a red light on each frame and it seemed to be working while it had a test accuracy of about 74%. We decided to use the larger model if it was fast enough; otherwise we just used the smaller model
- Large mobilenet (17Mo): 90.4% test accuracy (664 test samples)

![alt text](https://github.com/ayanangshu/CarND-Capstone/blob/master/imgs/training.jpg)
