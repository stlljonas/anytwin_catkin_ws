# Actuated Lidar
-----------------------------------

## Overview

The actuated lidar package manages the control of the Dynamixel servo motor, the communication with the Hokuyo laser scanner and the laser assembler.

The package has been tested under ROS Melodic and Ubuntu 18.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

**Author:** Jérôme Maye, Remo Diethelm, Christian Gehring
**Contact:** Remo Diethelm, rdiethelm@anybotics.com


### Dependencies

- [Laser filters](http://wiki.ros.org/laser_filters) (ROS laser filters)

		sudo apt-get install ros-indigo-laser-filters

- [Laser assembler](http://wiki.ros.org/laser_assembler) (ROS laser assembler)

		sudo apt-get install ros-indigo-laser-assembler

- [Dynamixel ROS](https://bitbucket.org/leggedrobotics/dynamixel_ros) (ROS wrapper for dynamixel library)

		git clone git@bitbucket.org:leggedrobotics/dynamixel_ros.git



### Building

In order to install, clone the latest version from this repository into your catkin workspace and compile the packages.


## Troubleshooting

If your graphics card does not support hardware acceleration (e.g. for Intel GPUs, inside a VM) it can cause problems. To get around this add
```
#!c++

export LIBGL_ALWAYS_SOFTWARE=1
```
before launching gazebo or add it to your ~/.bashrc file.

## Unit Tests

No unit tests available yet.
