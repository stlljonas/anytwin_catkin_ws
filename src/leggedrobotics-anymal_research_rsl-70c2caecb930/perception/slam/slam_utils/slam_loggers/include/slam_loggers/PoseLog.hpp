/*
 * PoseLog.hpp
 *
 *  Author: Yoshua Nava
 *  Organization: ANYbotics AG
 *  License: Proprietary
 */
#pragma once

// C++ standard library
#include <vector>

// eigen
#include <Eigen/Dense>

// ros
#include <ros/time.h>

namespace slam_loggers {

// Struct for saving pose data with timestamps.
struct PoseLog {
  using RobotPoseList = std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>;

  //! Timestamps of poses received.
  std::vector<ros::Time> timestamps_;

  //! Poses in tracked frame. (e.g. "map")
  RobotPoseList robotInTrackedFramePoses_;

  //! Poses in ground truth frame. (e.g. "world")
  RobotPoseList robotInWorldFramePoses_;

  //! Ground truth poses, coming from Gazebo or another source.
  RobotPoseList groundTruthPoses_;

  void clear() {
    timestamps_.clear();
    robotInTrackedFramePoses_.clear();
    robotInWorldFramePoses_.clear();
    groundTruthPoses_.clear();
  }
};

}  // namespace slam_loggers