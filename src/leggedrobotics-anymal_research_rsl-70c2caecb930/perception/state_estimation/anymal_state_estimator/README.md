# ANYmal State Estimator

## Overview

This package provides a ROS interface for a state estimator in addition to estimator implementations based on the [light_weight_filtering](https://bitbucket.org/anybotics/lightweight_filtering) and [two_state_information_filter](https://github.com/anybotics/two_state_information_filter) frameworks.

The software has been tested under ROS Melodic and Ubuntu 18.04.

**Author(s):** Christian Gehring, Michael Bloesch, Fabian Tresoldi

## Building

In order to install, clone the latest version from this repository into your catkin workspace and compile the packages.

## Usage

### Subscribed Topics


* **`imu`** ([sensor_msgs/Imu])

	The data from the IMU.

* **`actuator_readings`** ([series_elastic_actuator_msgs/SeActuatorReadings])

	The states of the leg joints.

* **`contact_force_*_foot`** ([geometry_msgs/WrenchStamped])

	The contact forces of four feet.

* **`pose`** ([geometry_msgs/PoseWithCovarianceStamped])

	The corrected pose of the torso from the localization.

### Published Topics

* **`anymal_state`** ([anymal_msgs/AnymalState])

	The estimated robot state.

* **`pose_in_*`** ([geometry_msgs/PoseWithCovarianceStamped])

	The estimated pose of the torso with covariance in odometry and map frames.

* **`twist`** ([geometry_msgs/TwistWithCovarianceStamped])

	The estimated twist of the torso with covariance.

### Advertised Services

* **`reset`** ([anymal_msgs/ResetStateEstimator])

    Resets the state estimator.

* **`toggle_pose_measurements`** ([any_msgs/Toggle])

    Activates and deactivates processing of absolute pose measurements.
