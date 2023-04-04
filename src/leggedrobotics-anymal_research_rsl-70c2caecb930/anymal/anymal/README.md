# ANYmal

## Overview

Common code, simulation and msg/srv definitions for a ANYmal.

The software has been tested under ROS Melodic and Ubuntu 18.04.

[Documentation](https://anybotics-anymal-sim.readthedocs-hosted.com/)

## Building

In order to install, clone the latest version from this repository into your catkin workspace and compile the packages.

## Usage

### Packages

#### anymal_description

* **anymal_description:** Urdf, meshes, robot description and containers which describe ANYmal.

#### anymal_drivers

* **anymal_flir:** Package containing ANYmal specific launch and configuration files for Pointgrey Flir cameras.
* **anymal_lowlevel_controller:** Lowlevel controller for ANYmal, communicating with actuators.
* **anymal_piksi_rtk_gps:** Package containing ANYmal specific launch and configuration files for running the Piksi Multi RTK GPS.
* **anymal_realsense:** Package containing ANYmal specific launch and configuration files for Intel RealSense depth cameras.
* **anymal_velodyne:** Package containing ANYmal specific launch and configuration files for Velodyne 3D LIDARs.
* **power_distribution_board:** Drivers for ANYmal's power distribution board.
* **rpsm_software:** Drivers for ANYmal's robot power and system management.
* **status_light:** Drivers for ANYmal's status light.

#### anymal_model

* **anymal_model:** ANYmal model.
* **anymal_model_ros:** Ros types and conversion traits used for the ANYmal model.

#### anymal_simulation

* **anymal_gazebo:** Gazebo plugin for ANYmal. Derives from *anybot_gazebo_plugin* in *any_gazebo*. The included launch file does not load anything else than the bare simulation. To run the user interface and a controller, use *anymal_sim*.

#### anymal_system

* **anymal_logging:** Launch and config files to log and inspect ANYmal data.
* **anymal_msgs:** Commonly used ROS msgs for ANYmal.
* **anymal_roco:** ANYmal specialization of the *roco* library.

#### anymal_user_interface

* **hri_user_interface:** ROS node to run the HRI joystick.
* **joy_interface_anymal:** ANYmal specific bindings to control the robot with a generic joystick.
