# anymal_gazebo

## Overview

Gazebo plugin for ANYmal. Derived from *seabot_gazebo_plugin*.

The included launch file does not load anything else than the bare simulation. To load the robot description, run the user interface and a controller, use *anymal_b_sim*.

## ROS Interface

*anymal_gazebo* adds the following topics to the standard Gazebo ROS interface.

### Subscribed Topics

* **`/anymal_highlevel_controller/actuator_commands`** ([series_elastic_actuator_msgs/SeActuatorCommands])

    ANYdrive commands intended for control.


### Published Topics

* **`/anydrive/readings`** ([series_elastic_actuator_msgs/SeActuatorReadings])

    ANYdrive readings published at the rate of the simulation, intended for control.

* **`/anymal_lowlevel_controller/actuator_readings_extended_throttle`** ([series_elastic_actuator_msgs/SeActuatorReadingsExtended])

    Throttled ANYdrive readings with extended information intended for GUIs or other nodes which do not need high information rate.

* **`/state_estimator/pose_in_odom`** ([geometry_msgs/PoseWithCovarianceStamped])

    Pose of the robot's main body expressed in the state estimator odometry frame.

* **`/state_estimator/anymal_state`** ([geometry_msgs/AnymalState])

    Output of the state estimator with complete information about the robot's state and transformations to various frames.

* **`/state_estimator/anymal_state_throttle`** ([geometry_msgs/AnymalState])

    Throttled version of the anymal state topic.

* **`/state_estimator/twist`** ([geometry_msgs/TwistWithCovarianceStamped])

    Twist of the robot's main body expressed in the main body frame.

* **`/state_estimator/joint_states`** ([any_msgs/ExtendedJointState])

    Joint states information.

* **`/diagnostics`** ([diagnostic_msgs/DiagnosticArray])

    ROS diagnostics.

* **`/sensors/contact_force_XY_foot`** ([geometry_msgs/WrenchStamped])

    Contact force acting at foot XY.

* **`/sensors/imu`** ([sensor_msgs/Imu])

    IMU information.
