\page page_average_calculator_ros Average Calculator ROS

# Average Calculator ROS

## Overview

This packages implements a ROS interface for the average calculator.

## Installation

### Install Debian Packages

To install all packages from the this repository as Debian packages use

    sudo apt install ros-${ROS_DISTRO}-...

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics)

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

    cd ~/catkin_ws/src
    git clone git@code.anymal.com:anymal-research/anymal_research.git
    catkin build average_calculator_ros

### Unit Tests

Run the unit tests with

    catkin run_tests average_calculator_ros

## Usage

Launch an average calculator ROS node with

    roslaunch average_calculator_ros average_calculator_ros.launch

Print a list of all launch file arguments with

    roslaunch average_calculator_ros average_calculator_ros.launch --ros-args

Print a list of all included launch files with

    roslaunch average_calculator_ros average_calculator_ros.launch --files

## Configuration files

The configuration files are stored in `./config`.

* **default.yaml** Default parameters for the node.

* **...**

## Launch files

The launch files are stored in `./launch`.

* **average_calculator_ros.launch** Launch the ROS node with default parameters.

    Argument set 1

    * **`overlying_param_file`** Parameter file overlying parts or all parameters. Default: `default_param_file`.

    Argument set 2

    * **`...`**

* **...**

## Nodes

### average_calculator

Subscribes to values and computes their average.

#### Subscribed Topics

* **`value`** (`[average_calculator_msgs/Value]`)

  The data points from which the average is computed.

#### Published Topics

...

#### Services

* **`get_average_value`** (`[average_calculator_msgs/GetAverageValue]`)

  Returns information about the current average. For example, you can trigger the computation from the console with

    rosservice call /average_calculator/get_average_value

### NODE_B_NAME

...
