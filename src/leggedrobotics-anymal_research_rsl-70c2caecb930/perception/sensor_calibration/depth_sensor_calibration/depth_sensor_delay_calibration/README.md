# Depth Sensor Calibration

Contains the following tools for calibrating depth sensors:

## Depth Sensor Delay Calibration

### Overview

Peripheral sensors often need to be calibrated for a time delay.
This package contains a tool for finding the optimal constant time stamp offset for sensors which deliver point clouds.

**Author:** Remo Diethelm, rdiethelm@anybotics.com

### Usage

1. Build the package:

    ```
    catkin build depth_sensor_delay_calibration
    ```

2. Launch the calibration tool (e.g. for Depth Camera using odom as fixed frame):

    ```
    roslaunch depth_sensor_delay_calibration depth_sensor_delay_calibration.launch point_cloud_topic:="/depth_camera/depth/color/points" fixed_frame_id:="odom"
    ```

3. Start recording point clouds:

    ```
    rosservice call /depth_sensor_delay_calibration/toggle_recording "enable: true"
    ```

4. Move your robot, such that the calibration fixed frame does not move, but the point cloud changes in its local frame. For ANYmal, one can use the *tilt_motion* from the *anymal_calibration_actions* collection.

5. Stop recording and start the optimization:

    ```
    rosservice call /depth_sensor_delay_calibration/toggle_recording "enable: false"
    ```

6. Wait until the optimization finds and prints the optimal time stamp delay.

### How it works

The goal of the algorithm is to find the optimal time delay for the recorded point cloud messages such that the point clouds match each other after transforming them to the fixed frame.
The costs which are being minimized is a function of the matching error.

It consists of the following steps:

1. Filter all input point clouds using the given filter configuration.
2. The optimizer tries to find the best time stamp delay by running the following substeps:
    1. Get a new time step offset using the nelder mead optimizer.
    2. Copy all original point clouds and add a time stamp offset.
    3. Transform the delayed point clouds to the fixed frame.
    4. Run ICP between all subsequent point clouds (1&2, 2&3, ...).
    5. The costs to minimize is the mean of all transformation errors. A single transformation error is the disparity angle between the z-axis of the two subsequent point clouds.

Conditions for the algorithm to work properly:

- At least 2 point clouds have been recorded with sufficient size after filtering.
- The robot's motion induces a motion of the point cloud frame.
- The motion of the point cloud frame has to be perceivable by the point cloud sensor (e.g. no orthogonal motions to the perceived surface etc.).
