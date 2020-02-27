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
