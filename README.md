# Introduction
The goal of this project is to enable Karla (the Udacity’ car) to drive around the test track using waypoint navigation.  I'll be implementing components of the perception,planning, and control subsystems.

* In the perception subsystem, I'll implement traffic light detection and obstacle detection.

* In the planning subsystem, I'll implement a node called the waypoint updater. This node sets the target velocity for each waypoint based on the upcoming traffic lights and obstacles. For example, if the car sees a red traffic light on the horizon will to set decelerating velocities at the nodes leading up to that traffic light.

* In the control subsystem, I'll implement a drive by wire ROS node that takes target trajectory information is input, in sense, control commands to navigate the vehicle.

The project will require the use of Ubuntu Linux (the operating system of Carla) and a ROS framework for these nodes, as well as a version of the simulator that includes traffic lights and obstacles. 

### follow the steps below to get set up:

1. Because ROS is used, you will need to use Ubuntu to develop and test this project code. You may use

          Ubuntu 14.04 with ROS Indigo 
          Ubuntu 16.04 with ROS Kinetic
     
    Note: Please use the VM provided in **the [Introduction to ROS](https://github.com/A2Amir/Introduction-to-ROS--Robot-Operating-System) lesson**. The provided VM has ROS and Dataspeed DBW installed already.

2. The project repo can be found [here](https://github.com/udacity/CarND-Capstone). In the README, you should be able find any additional dependencies needed for the project.
3. From the the VM provided in **the [Introduction to ROS](https://github.com/A2Amir/Introduction-to-ROS--Robot-Operating-System) lesson** you can download the linux  version of the simulator [here](https://github.com/udacity/CarND-Capstone/releases) and install it. 
    
    Note that the latest version of the simulator has two test tracks: 
    
          A highway test track with traffic lights
          A testing lot test track similar to where the Udacity team will run Carla
    
    To use the second test lot, you will need to update your code to specify a new set of waypoints. I'll discuss how to do this in a later lesson. Additionally, the first track has a toggle button for camera data. Finally, the simulator displays vehicle velocity in units of mph. However, all values used within the project code use the metric system (m or m/s), including current velocity data coming from the simulator.
    
# 2. Project Overview

For this project, I will be writing ROS nodes to implement core functionality of the autonomous vehicle system, including traffic light detection, control, and waypoint following and I will test my code using the simulator.

### System Architecture Diagram

The following is a system architecture diagram showing the ROS nodes and topics used in the project. (I refer to the diagram throughout the project). The ROS nodes and topics shown in the diagram are described briefly in the **Code Structure** section below and more detail is provided for each node later.

<p align="center">
<img src="./img/1.png" alt="System Architecture Diagram" />
<p align="center">
          
          
### Code Structure
Below is a brief overview of the repo structure, along with descriptions of the ROS nodes. Within this directory /ros/src/, you will find the following ROS packages:


#### 1. /ros/src/tl_detector/

This package contains the traffic light detection node: **tl_detector.py**. This node takes in data from the **/image_color, /current_pose, and /base_waypoints topics** and publishes the locations to stop for red traffic lights to the **/traffic_waypoint topic**.
The **/current_pose** topic provides the vehicle's current position, and **/base_waypoints** provides a complete list of waypoints the car will be following.

l build both a traffic light detection node and a traffic light classification node. Traffic light detection takes place within **tl_detector.py**, whereas traffic light classification takes place within **../tl_detector/light_classification_model/tl_classfier.py**


<p align="center">
<img src="./img/2.png" alt="traffic light detection node" />
<p align="center">
          
          
          
#### 2. /ros/src/waypoint_updater/
This package contains the waypoint updater node: **waypoint_updater.py**. The purpose of this node is to update the target velocity property of each waypoint based on traffic light and obstacle detection data. This node will subscribe to **the /base_waypoints, /current_pose, /obstacle_waypoint, and /traffic_waypoint topics**, and publish a list of waypoints ahead of the car with target velocities to **the /final_waypoints** topic.


<p align="center">
<img src="./img/3.png" alt="waypoint_updater node" />
<p align="center">
          
Note: Waypoints are simply an ordered set of coordinates that Karla uses to plan a path around the track. Each of these waypoints also has an associated target velocity. Karla's planning subsystem updates the target velocity for the waypoints ahead of the vehicle depending on the desired vehicle behavior.


#### 3. /ros/src/twist_controller/

Carla is equipped with a drive-by-wire (dbw) system, meaning the throttle, brake and steering have electronic control. This package contains the files that are responsible for control of the vehicle: the node dbw_node.py and the file twist_controller.py, along with a pid and lowpass filter that I can use in my implementation. 

The dbw_node subscribes to the **/current_velocity topic** along with the **/twist_cmd topic** to receive target linear and angular velocities. Additionally, this node will subscribe to **/vehicle/dbw_enabled**, which indicates if the car is under dbw or driver control. This node will publish throttle, brake, and steering commands to the **/vehicle/throttle_cmd, /vehicle/brake_cmd and /vehicle/steering_cmd topics**.

<p align="center">
<img src="./img/4.png" alt="twist_controller" />
<p align="center">

#### In addition to these packages will find in this directory the following:

* /ros/src/styx/: A package that contains a server for communicating with the simulator, and a bridge to translate and publish simulator messages to ROS topics.

* /ros/src/styx_msgs/:A package which includes definitions of the custom ROS message types used in the project.

* /ros/src/waypoint_loader/:A package which loads the static waypoint data and publishes to /base_waypoints.

* /ros/src/waypoint_follower/:A package containing code from Autoware which subscribes to /final_waypoints and publishes target vehicle linear and angular velocities in the form of twist commands to the /twist_cmd topic. 
