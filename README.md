# Walker Robot
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
---

It is a simple walker robot much like a Roomba robot vacuum cleaner.

## Overview

This program is a basic implementation of a walker algorithm on a turtlebot. The robot moves forward until it senses an obstacle within a paticular range. If it detects an obstacle, it rotates until the obstacle goes out of range and then the robot moves forward and repeats the same. This algorithm is similar to the working of a Roomba robot vacuum cleaner.

The program also includes the feature of recording a bag file which when enabled can record all topics except /camera/*. The rosbag file is generated in the results directory by default. This rosbag file can be played back later.

## Dependencies

This program works on a device running Ubuntu 16.04 and ROS Kinetic.

To install ROS Kinetic in Ubuntu 16.04, follow the steps in this [link](http://wiki.ros.org/kinetic/Installation/Ubuntu).

To install catkin, follow the installation steps in this [link](http://wiki.ros.org/catkin).

The program also depends on the turtlebot_gazebo simulation. To install turtlebot_gazebo, run the following command in a terminal:
```
sudo apt-get install ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
```
To check the installation, you can launch a simple world with a Turtlebot using the following command in a terminal:
```
roslaunch turtlebot_gazebo turtlebot_world.launch
```

## Build Instructions

To build this code in a catkin workspace:
```
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone https://github.com/Ghost1995/walkerbot.git
cd ..
catkin_make
```
Note, that if you do not have a catkin workspace, then to build this code use the following commands:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone https://github.com/Ghost1995/walkerbot.git
cd ..
catkin_make
```

## Run Instructions

After following the build instructions, launch the code using the following commands:
```
source ~/catkin_ws/devel/setup.bash
roslaunch walkerbot walkerbot.launch
```

## Record bag File

A ros bag file records all the topic and messages being published in the terminal except /camera/*. To record a bag file, just enable recording in the launch file using the following commands:
```
source ~/catkin_ws/devel/setup.bash
roslaunch walkerbot walkerbot.launch record:=enable
```
Note that, the bag file is saved in the results folder by default.

#### Inspecting the bag File Generated

To get more information about the generated rosbag file, such as the data being recorded, the time duration, the size of the bag file, and so on, use the following commands:
```
cd <path to directory>/results
rosbag info simulation.bag
```

#### Playing the bag File Generated

The rosbag file records all the messages being published on to the terminal except /camera/*. To check if the messages were recorded properly, you can playback the recorded rosbag file.

Ina terminal, run "roscore" by using the following command:
```
roscore
```
Now, open a new terminal and use the following commands:
```
cd <path to directory>/results
source ~/catkin_ws/devel/setup.bash
rosbag play simulation.bag
```
Note that, while playing the rosbag file, make sure that Gazebo is not running.

## Plugins
##### CppChEclipse
To run cppcheck in Terminal
```
cd <path to directory>
cppcheck --enable=all --std=c++11 -I include/ --suppress=missingIncludeSystem $(find . -name \*.cpp -or -name \*.hpp | grep -vE -e "^./launch/" -e "^./results/" -e "./world/")
```
##### Google C++ Sytle
To check Google C++ Style formatting in Terminal
```
cd <path to directory>
cpplint $(find . -name \*.hpp -or -name \*.cpp | grep -vE -e "^./launch/" -e "^./world/" -e "^./results")
```
