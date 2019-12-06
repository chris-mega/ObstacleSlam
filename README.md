# ObstacleSlam
*Developed by Christian Melendez*

Piano player program for a Robotis-Op3 robot

This was originally an assignment for COMP4180-"Intelligent Mobile Robotics" at University of Manitoba.

Please see `Report.pdf` for a detailed description of the project's development

## Dependencies
Your system must contain the following programs/libraries installed
- [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
- [Robotis OP3 Framework](http://emanual.robotis.com/docs/en/platform/op3/recovery/#op3-manual-installation)
- [Opencv 3](https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html)
- [Python 2.7](https://www.python.org/downloads/)
- [numpy](https://www.scipy.org/install.html)
- [pyyaml](https://pypi.org/project/PyYAML/)
- Unified-ROS-Platform, a framework adapted from OP3 Framework by the [Autonomous Agents Lab](http://aalab.cs.umanitoba.ca/) at University of Manitoba (not public, but some workarounds can be made for the missing files in this repository)

## Build
Copy the packages inside `src` into your workspace and type `catkin_make`

## Running
- Run `roslaunch events obstacle_run_slam.launch size:=kid` for the event
- Run `roslaunch iros_vision piano_detector.launch` for only vision
