# HomeServiceRobot
[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)


https://github.com/Mennatallah98/HomeServiceRobot/assets/45118345/5940e532-9aae-4eb3-8605-de85114e4b3d


## Overview

This project is the fifth and final project in Udacity Robotics Software Engineer nano degree where the world was mapped using [gmmaping] which produces 2D-maps and[joy] package was used to move the rbot during the mapping process and it can also be replaced by [teleop_twist_keyboard].After mapping,2 boxes are placed and the robot is given their places as goals to pick and palce them.

**Keywords:** ROS,  mapping, navigation, pathplanning, gmapping, amcl.

**Author: Mennatallah Aly<br />**

The MapMyWorld package has been tested under [ROS] Melodic on Ubuntu 18.04. and Gazebo 9.0.0

## Installation

### Installation from Packages

To install all packages from the this repository as Debian packages use

    sudo apt-get update && sudo apt-get upgrade -y
    sudo apt-get install ros-melodic-navigation
    sudo apt-get install ros-melodic-map-server
    sudo apt-get install ros-melodic-move-base
    sudo apt-get install ros-melodic-amcl
    
To navigate with keyboard 

	sudo apt-get install ros-melodic-teleop-twist-keyboard
	
To navigate with joystick

	sudo apt-get install ros-melodic-joy ros-melodic-joystick-drivers

    
Or better, use `rosdep`:

	sudo rosdep install --from-paths src

## Building from Source

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_ws/src
	sudo apt update
	git clone https://github.com/Mennatallah98/HomeServiceRobot.git
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin_make
	
Source the workspace by adding this line .bashrc

	source ~/catkin_ws/devel/setup.bash

## Usage



### Mapping

In new window open test_slam bash file

	cd catkin_ws/src/HomeServiceRobot/scripts
	./test_slam.sh
	
If keyboard teleop is to be used instead of joy open the launch file and replace

	roslaunch my_robot joy.launch
	
with

	rosrun teleop_twist_keyboard teleop_twist_keyboard.py

### Navigation to pick and place

In new window open home_service bash file

	cd catkin_ws/src/HomeServiceRobot/scripts
	./home_service.sh

## Config files

Config file folder/config

* **base_local_planner_params.yaml:** contains the parameters for [base_local_planner] which is  responsible for computing velocity commands to send to the mobile base. 

* **costmap_common_params.yaml:** contains the [common] parameters between the global and local [costmap].

* **global_costmap_params.yaml:** contains the parameters for [global] [costmap].

* **local_costmap_params.yaml:** contains the parameters for [local] [costmap].

## Bash files

* **add_markers.sh:** runs world, and amcl launch files also add_markers node.

* **home_service.sh:** runs world, and amcl launch files also add_markers, and pick_objects nodes.

* **pick_objects.sh:** runs world, and amcl launch files also pick_objects node.

* **test_navigation.sh:** runs world, and amcl launch files.

* **test_slam.sh:** runs world, gmapping, and joy launch files.


## Launch files

* **robot_description.launch:** Runs the robot file and starts the joint publisher robot state publisher.

* **world.launch:** Starts [rviz] customized configuration and gazebo with the customized world , spawns the robot and launches robot_description.

* **joy.launch:** Runs [joy] node with customized joy script.

* **gmapping.launch:** Runs [gmapping] to generate map for the world.

* **amcl.launch:** Runs [amcl], [move_base], and [map_server] and sets the initial position of the robot in the map.


## Packages

* **my_robot:** Contains the URDF of r 4-wheeled under the name of my_robot with the attached sensors including depth camera in addition to the world with robot embbeded in aldo the modified configuartion files and the world map.

## Nodes

### joy_teleop

Sends velocity commands to the robot according to the buttons pressed in the joystick


#### Subscribed Topics

* **`/cmd_vel`** ([geometry_msgs/Twist])

	The speed to move the robot.


#### Published Topics

* **`/joy`** ([sensor_msgs/Joy Message])

	The buttons pressed in the joystick.

## Structure

	└── HomeServiceRobot                                    # Home Service project
	    ├── add_markers                                     # add markers package
	    │   ├── CMakeLists.txt                              # compiler instructions
	    │   ├── include                       
	    │   │   └── add_markers
	    │   ├── package.xml                                 # package info
	    │   └── src                                         # source folder for C++ scripts
	    │       ├── add_markers.cpp
	    │       └── add_markers_time.cpp
	    ├── my_robot                                        # my robot package
	    │   ├── CMakeLists.txt                              # compiler instructions
	    │   ├── launch                                      # launch folder for launch files
	    │   │   ├── amcl.launch
	    │   │   ├── joy.launch
	    │   │   ├── robot_description.launch
	    │   │   └── world.launch
	    │   ├── maps                                        # maps folder for maps
	    │   │   ├── myMap.pgm
	    │   │   └── myMap.yaml
	    │   ├── meshes                                      # meshes folder for sensors
	    │   │   └── hokuyo.dae
	    │   ├── package.xml                                 # package info                 
	    │   ├── rviz                                        # rviz folder for rviz configuration files
	    │   │   └── myworld.rviz
	    │   ├── scripts                                     # scripts folder for python scripts
	    │   │   └── joy_teleop
	    │   ├── urdf                                        # urdf folder for xarco files
	    │   │   ├── my_robot2.gazebo
	    │   │   ├── my_robot2.xacro
	    │   │   ├── my_robot.gazebo
	    │   │   └── my_robot.xacro
	    │   └── worlds                                      # world folder for world files
	    │       └── myworld.world
	    ├── pick_objects                                    # pick objects package
	    │   ├── CMakeLists.txt                              # compiler instructions
	    │   ├── include
	    │   │   └── pick_objects
	    │   ├── package.xml                                 # package info
	    │   └── src                                         # source folder for C++ scripts
	    │       └── pick_objects.cpp
	    ├── scripts                                         # scripts folder for bash scripts
	    │   ├── add_markers.sh
	    │   ├── home_service.sh
	    │   ├── pick_objects.sh
	    │   ├── test_navigation.sh
	    │   └── test_slam.sh
	    └── slam_gmapping                                   # slam gmapping package
		├── gmapping
		│   ├── CHANGELOG.rst
		│   ├── CMakeLists.txt
		│   ├── launch                                  # launch folder for launch files 
		│   │   └── gmapping.launch
		│   ├── nodelet_plugins.xml
		│   ├── package.xml                             # package info
		│   ├── src                                     # source folder for C++ scripts
		│   │   ├── main.cpp
		│   │   ├── nodelet.cpp
		│   │   ├── replay.cpp
		│   │   ├── slam_gmapping.cpp
		│   │   └── slam_gmapping.h
		│   └── test
		│       ├── basic_localization_laser_different_beamcount.test
		│       ├── basic_localization_stage.launch
		│       ├── basic_localization_stage_replay2.launch
		│       ├── basic_localization_stage_replay.launch
		│       ├── basic_localization_symmetry.launch
		│       ├── basic_localization_upside_down.launch
		│       ├── rtest.cpp
		│       └── test_map.py
		├── README.md
		└── slam_gmapping
		    ├── CHANGELOG.rst
		    ├── CMakeLists.txt
		    └── package.xml

    

[ROS]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[joy]: http://wiki.ros.org/joy
[teleop_twist_keyboard]: http://wiki.ros.org/teleop_twist_keyboard
[gmapping]: http://wiki.ros.org/gmapping
[amcl]: http://wiki.ros.org/amcl
[navigation_stack]: http://wiki.ros.org/navigation/Tutorials/RobotSetup
[base_local_planner]: http://wiki.ros.org/base_local_planner
[costmap]: http://wiki.ros.org/costmap_2d
[global]: http://wiki.ros.org/navigation/Tutorials/RobotSetup#Global_Configuration:~:text=Global%20Configuration%20(global_costmap)
[local]: http://wiki.ros.org/navigation/Tutorials/RobotSetup#Local_Configuration:~:text=Local%20Configuration%20(local_costmap)
[common]: http://wiki.ros.org/navigation/Tutorials/RobotSetup#Global_Configuration:~:text=Common%20Configuration%20(local_costmap)%20%26%20(global_costmap)
[move_base]: http://wiki.ros.org/move_base
[geometry_msgs/Twist]: http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html
[sensor_msgs/Joy Message]: http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Joy.html
[map_server]: http://wiki.ros.org/map_server
