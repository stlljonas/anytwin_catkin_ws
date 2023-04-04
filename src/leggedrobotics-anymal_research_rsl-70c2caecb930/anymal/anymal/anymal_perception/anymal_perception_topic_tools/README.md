\page page_anymal_perception_topic_tools ANYmal Perception Topic Tools

# ANYmal Perception Topic Tools

## Overview

This package contains tools to remap and rename topics and frames of ANYmal Perception sensors.

## Installation

### Install Debian Packages

To install the package and its dependencyes as Debian packages run

    sudo apt install ros-${ROS_DISTRO}-anymal-perception-topic-tools

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics)

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

    cd ~/catkin_ws/src
    git clone git@code.anymal.com:anymal-research/anymal_research.git
    catkin build anymal_perception_topic_tools


## Usage

### Offline tools

#### Fix timestamps of ROS bags when NPC or APC are out of sync with LPC

Example command to fix bags. It must be run in the Linux terminal.

```
rosrun anymal_perception_topic_tools fix_bag_timestamps_pc_clocks_out_of_sync.py <input_bag> <output_bag>
```

#### Rename topics and frames inside a BAG file to generic names

Example command to rename topics and frames inside a bag file, concerning Perception sensors (LIDAR, Depth Cameras and Wide Angle Cameras). It must be run in the Linux terminal.

```
rosrun anymal_perception_topic_tools rename_bag_sensor_topics_to_generic_names.py <input_bag> <output_bag>
```

### Online tools

ROS node to remap topics and frames inside a bag file, concerning Perception sensors (LIDAR, Depth Cameras and Wide Angle Cameras).

Example implementations provided in the launch/ folder, for each one of ANYmal Perception sensors.

For an ANYMal C, a launch file with the following parameters would suffice:

```
<?xml version="1.0" encoding="utf-8"?>

<launch>

  <!-- LIDAR -->
  <!-- Whether topic remapping for LIDAR is enabled -->
  <arg name="remap_lidar"                                       default="true"/>
  <!-- Remapping of LIDAR -->
  <arg name="lidar_original_name"                               default="velodyne"/>
  <arg name="lidar_new_name"                                    default="lidar"/>
  <arg name="lidar_frame_name"                                  default="$(arg lidar_new_name)"/>

  <!-- Depth Cameras -->
  <!-- Whether topic remapping for Depth Cameras is enabled -->
  <arg name="remap_depth_cameras"                               default="true"/>
  <!-- Remapping of front Depth Camera -->
  <arg name="depth_camera_front_original_name"                  default="realsense_d435_front"/>
  <arg name="depth_camera_front_new_name"                       default="depth_camera_front"/>
  <arg name="depth_camera_front_frame_prefix"                   default="$(arg depth_camera_front_new_name)"/>
  <!-- Remapping of rear Depth Camera -->
  <arg name="depth_camera_rear_original_name"                   default="realsense_d435_rear"/>
  <arg name="depth_camera_rear_new_name"                        default="depth_camera_rear"/>
  <arg name="depth_camera_rear_frame_prefix"                    default="$(arg depth_camera_rear_new_name)"/>
  <!-- Remapping of left Depth Camera -->
  <arg name="depth_camera_left_original_name"                   default="realsense_d435_left"/>
  <arg name="depth_camera_left_new_name"                        default="depth_camera_left"/>
  <arg name="depth_camera_left_frame_prefix"                    default="$(arg depth_camera_left_new_name)"/>
  <!-- Remapping of right Depth Camera -->
  <arg name="depth_camera_right_original_name"                  default="realsense_d435_right"/>
  <arg name="depth_camera_right_new_name"                       default="depth_camera_right"/>
  <arg name="depth_camera_right_frame_prefix"                   default="$(arg depth_camera_right_new_name)"/>

  <!-- Wide Angle Cameras -->
  <!-- Whether topic remapping for Wide Angle Cameras is enabled -->
  <arg name="remap_wide_angle_cameras"                          default="true"/>
  <!-- Remapping of front Wide Angle Camera -->
  <arg name="wide_angle_camera_front_original_name"             default="blackfly_front"/>
  <arg name="wide_angle_camera_front_new_name"                  default="wide_angle_camera_front"/>
  <arg name="wide_angle_camera_front_frame_name"                default="$(arg wide_angle_camera_front_new_name)"/>
  <!-- Remapping of rear Wide Angle Camera -->
  <arg name="wide_angle_camera_rear_original_name"              default="blackfly_rear"/>
  <arg name="wide_angle_camera_rear_new_name"                   default="wide_angle_camera_rear"/>
  <arg name="wide_angle_camera_rear_frame_name"                 default="$(arg wide_angle_camera_rear_new_name)"/>

  <!-- Depth Cameras -->
  <group ns="depth_camera" if="$(eval remap_depth_cameras)">
    <!-- Front Depth Camera -->
    <include file="$(find anymal_perception_topic_tools)/launch/depth_camera_remapping_realsense_d435.launch">
      <arg name="original_sensor_name"                  value="$(arg depth_camera_front_original_name)"/>
      <arg name="new_sensor_name"                       value="$(arg depth_camera_front_new_name)"/>
      <arg name="new_frame_prefix"                      value="$(arg depth_camera_front_frame_prefix)"/>
    </include>

    <!-- Rear Depth Camera -->
    <include file="$(find anymal_perception_topic_tools)/launch/depth_camera_remapping_realsense_d435.launch">
      <arg name="original_sensor_name"                  value="$(arg depth_camera_rear_original_name)"/>
      <arg name="new_sensor_name"                       value="$(arg depth_camera_rear_new_name)"/>
      <arg name="new_frame_prefix"                      value="$(arg depth_camera_rear_frame_prefix)"/>
    </include>

    <!-- Left Depth Camera -->
    <include file="$(find anymal_perception_topic_tools)/launch/depth_camera_remapping_realsense_d435.launch">
      <arg name="original_sensor_name"                  value="$(arg depth_camera_left_original_name)"/>
      <arg name="new_sensor_name"                       value="$(arg depth_camera_left_new_name)"/>
      <arg name="new_frame_prefix"                      value="$(arg depth_camera_left_frame_prefix)"/>
    </include>

    <!-- Right Depth Camera -->
    <include file="$(find anymal_perception_topic_tools)/launch/depth_camera_remapping_realsense_d435.launch">
      <arg name="original_sensor_name"                  value="$(arg depth_camera_right_original_name)"/>
      <arg name="new_sensor_name"                       value="$(arg depth_camera_right_new_name)"/>
      <arg name="new_frame_prefix"                      value="$(arg depth_camera_right_frame_prefix)"/>
    </include>
  </group>

  <!-- LIDAR -->
  <group ns="lidar" if="$(arg remap_lidar)">
    <include file="$(find anymal_perception_topic_tools)/launch/lidar_remapping.launch">
      <arg name="original_sensor_name"                  value="$(arg lidar_original_name)"/>
      <arg name="new_sensor_name"                       value="$(arg lidar_new_name)"/>
      <arg name="new_frame_name"                        value="$(arg lidar_frame_name)"/>
    </include>
  </group>

  <!-- Wide-Angle front camera -->
  <group ns="wide_angle_camera" if="$(arg remap_wide_angle_cameras)">
    <!-- Wide Angle Camera front -->
    <include file="$(find anymal_perception_topic_tools)/launch/wide_angle_camera_remapping.launch">
      <arg name="original_sensor_name"                  value="$(arg wide_angle_camera_front_original_name)"/>
      <arg name="new_sensor_name"                       value="$(arg wide_angle_camera_front_new_name)"/>
      <arg name="new_frame_name"                        value="$(arg wide_angle_camera_front_frame_name)"/>
    </include>

    <!-- Wide Angle Camera rear -->
    <include file="$(find anymal_perception_topic_tools)/launch/wide_angle_camera_remapping.launch">
      <arg name="original_sensor_name"                  value="$(arg wide_angle_camera_rear_original_name)"/>
      <arg name="new_sensor_name"                       value="$(arg wide_angle_camera_rear_new_name)"/>
      <arg name="new_frame_name"                        value="$(arg wide_angle_camera_rear_frame_name)"/>
    </include>
  </group>

</launch>
```