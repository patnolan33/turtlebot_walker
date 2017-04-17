# turtlebot_walker

## Table of Contents
- [Overview](#overview)
- [Prerequisites / Dependencies](#prerequisites-dependencies)
- [Build Steps](#build-steps)
- [Run using roslaunch](#run-roslaunch)
- [Recording using rosbag](#recording-rosbag)
- [Playback using rosbag](#playback-rosbag)

## Overview
This is a ROS package that uses the [Turtlbot](http://wiki.ros.org/Robots/TurtleBot) platform and its sensors to create a "walker" style motion. This is similar to how an iRobot Roomba works - i.e. drive forward until an obstacle is detected. If an obstacle is detected, stop, turn in place until free from the obstacle, then continue to drive forward. 

## <a name="prerequisites-dependencies"></a> Prerequisites / Dependencies
This package requires that [ROS](http://wiki.ros.org/indigo/Installation) is installed as well as [catkin](http://wiki.ros.org/catkin?distro=indigo#Installing_catkin). This was tested using ROS Indigo, however any subsequent versions of ROS should still work. This package also requires [Gazebo](http://gazebosim.org/) and [Turtlebot_Gazebo](http://wiki.ros.org/turtlebot_gazebo) to be installed. Gazebo should already be installed as part of the ROS distro, however Turtlebot_Gazebo may need to be installed. To do so, open a terminal and run the following command:
```
sudo apt-get install ros-indigo-turtlebot-gazebo ros-indigo-turtlebot-apps ros-indigo-turtlebot-rviz-launchers
```
This package also depends on:
-roscpp
-geometry_msgs
-move_base_msgs
-sensor_msgs

## <a name="build-steps"></a> Build Steps
To use this package, a catkin workspace must be setup first. Assuming catkin has been installed, run the following steps in the directory of your choice (a common one is ~/catkin_ws)
```
$ cd <PATH_TO_YOUR_DIRECTORY>
$ mkdir -p catkin_ws/src
$ cd catkin_ws
$ catkin_make
$ source devel/setup.bash
```
Your workspace should now be setup and you should be able to use ros commands like `roscd` and `rosls`. Note that if you cannot use these commands or can't find ROS packages, try running `source devel/setup.bash` again in your catkin workspace.

To build the turtlebot_walker ROS package in this repository, first clone the repository into the catkin `src` directory:
```
$ cd <PATH_TO_YOUR_DIRECTORY>/catkin_ws/src
$ git clone https://github.com/patnolan33/turtlebot_walker
```
Now simply run catkin_make to build the ROS package.
```
$ cd ..
$ catkin_make
```
You should now see a turtlebot_walker directory in `catkin_ws/build`. 

## <a name="run-roslaunch"></a> Run Gazebo and ROS walker node using roslaunch
Open a separate terminal and change directories into your catkin workspace. Source the directory and run the launch file as follows:
```
$ cd <PATH_TO_YOUR_DIRECTORY>/catkin_ws
$ source devel/setup.bash
$ roslaunch turtlebot_walker turtlebot_walker.launch
```
You should see a Gazebo window open and spawn the demo world (located in the `/gazebo_world` directory of this repository) with jersey walls and other obstacles as well as the turtlebot robot. In addition, the turtlebot should start moving forward towards a wall. As it drives, should it encounter an obstacle, it will stop, turn in place to free itself, then continue driving.

## <a name="recording-rosbag"></a> Recording using rosbag
The package `rosbag` is a common ROS tool that is used to record and playback ROS topic messages. The launch file for this project accepts a boolean flag called `record_walker` that toggles rosbag recording if included (true for record, false for do not record). To run both the publisher and subscriber node and record the published topics, run:
```
$ cd <PATH_TO_YOUR_DIRECTORY>/catkin_ws
$ source devel/setup.bash
$ roslaunch turtlebot_walker turtlebot_walker.launch record_walker:=true
```
rosbag will save a file named `walker.bag` in the `~/.ros/` directory. To inspect it, simply change into the directory with `cd ~/.ros/` and run `rosbag info walker.bag`. This will output various information about the file, such as how long the file recorded, the start and end times, file size, the number of messages, and the topics recorded. 

## <a name="playback-rosbag"></a> Playback using rosbag
We can playback this recorded data to recreate a recorded scenario. Assuming a rosbag recording has taken place according to the above process, playback the rosbag file by executing:
```
$ cd ~/.ros/
$ rosbag play walker.bag
```
In the rosbag terminal, you will see an indication that the rosbag file is running. 

*NOTE: Gazebo should not be running when playing back with rosbag.*





