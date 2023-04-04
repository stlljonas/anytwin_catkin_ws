# RObot MOdel

## Overview
**romo** is the definition of a common interface for robot models.

The software has been tested under ROS Melodic and Ubuntu 18.04.

**Author(s):** Dario Bellicoso, Christian Gehring

Contact: Dario Bellicoso, bellicoso@mavt.ethz.ch

## Building

In order to install, clone the latest version from this repository into your catkin workspace and compile the packages.

## Usage

### Packages

* **romo:** Interface of a robot model
* **romo_rbdl:** Robot model which uses the Rigid Body Dynamics Library
* **romo_std:** Standard robot model
* **kindr_rbdl:** Rotation conversion test between kindr and rbdl.

## Dependencies

- [Eigen](http://eigen.tuxfamily.org)
- [Kindr](https://github.com/anybotics/kindr)
- [any_rbdl](https://bitbucket.org/leggedrobotics/rbdl)
- [message_logger](https://github.com/anybotics/message_logger)
- [robot_utils](https://bitbucket.org/leggedrobotics/robot_utils)

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://bitbucket.org/leggedrobotics/romo/issues).
