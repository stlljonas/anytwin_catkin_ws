\page page_slam_loggers SLAM Loggers

# SLAM Loggers

## Overview

This package contains tools to log the output of SLAM nodes during execution, with the end goal of evaluating their performance.

It currently supports logging of SLAM poses in two formats: KITTI and TUM.

## Installation

### Install Debian Packages

To install the package and its dependencies using Debian packages run

    sudo apt install ros-${ROS_DISTRO}-slam-loggers

### Build from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org)

#### Build

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

    cd ~/catkin_ws/src
    git clone git@git.anybotics.com:anybotics/anybotics.git
    catkin build slam_loggers


## Usage

The package can be used to log the output of SLAM nodes. There are example launch files in the `launch` folder.

### Configuration

Two types of nodes can be run with this package:

* `gazebo_ground_truth_pose_publisher`: a node that request the model state of a link from Gazebo, publishes is as a ROS topic, and broadcasts it to the Tf tree.
* `pose_logger`: a node that listens to the output poses of a localization system, can republish them if desired, and logs them to '.csv' files in a format and location chosen by the user.

Example configuration files, with full description of the parameters are provided in the folder `config/`.

#### Launch

##### Standalone nodes

To launch a standalone Gazebo ground truth pose publisher, run:

```
roslaunch slam_loggers gazebo_ground_truth_pose_publisher.launch
```

To launch a standalone pose logger, run:

```
roslaunch slam_loggers pose logger.launch
```

##### Application example

To track the pose of a SLAM system that publishes a pose in the frame `map`, launch a pose_logger with the following parameters:

```


```