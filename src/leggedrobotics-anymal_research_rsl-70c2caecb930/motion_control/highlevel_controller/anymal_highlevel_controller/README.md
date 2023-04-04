# High-level Controller for ANYmal

## Overview

This software package provides a controller manager which enables the user to switch between different controllers for ANYmal. The manager is built on the
 [rocoma framework](https://github.com/anybotics/rocoma/).

The software has been tested under ROS Melodic and Ubuntu 18.04.

**Author(s):** Christian Gehring


## Building

In order to install, clone the latest version from this repository into your catkin workspace and compile the packages.

## Usage

The controller mananger runs as ROS node and uses [COSMO](https://bitbucket.org/leggedrobotics/cosmo) for fast and reliable data transmission.

### Packages
* **anymal_emcy_ctrl_freeze:** Emergency controller which freezes the robot
* **anymal_highlevel_controller:** High-level controller
* **rqt_highlevel_controlmanager:** RQT plugin which provides a GUI for the high-level controller


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://bitbucket.org/leggedrobotics/anymal_highlevel_controller/issues).
