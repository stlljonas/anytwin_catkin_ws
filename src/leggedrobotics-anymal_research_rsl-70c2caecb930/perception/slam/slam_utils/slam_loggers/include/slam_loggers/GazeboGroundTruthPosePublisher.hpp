/*
 * GazeboGroundTruthPosePublisher.hpp
 *
 *  Author: Yoshua Nava
 *  Organization: ANYbotics AG
 *  License: Proprietary
 */
#pragma once

// standard library
#include <vector>

// eigen
#include <Eigen/Dense>

// ros
#include <ros/ros.h>

// ros msgs
#include <geometry_msgs/PoseStamped.h>

// tf
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

// slam loggers
#include "slam_loggers/CoordinateFrameIds.hpp"
#include "slam_loggers/PoseLog.hpp"

namespace slam_loggers {

/*
 * @brief Class for publishing Gazebo's absolute pose reference. Adapts to the system's tf tree.
 */
class GazeboGroundTruthPosePublisher {
 public:
  /**
   * @brief Constructor
   *
   * @param nodeHandle A ROS NodeHandle.
   */
  explicit GazeboGroundTruthPosePublisher(ros::NodeHandle& nodeHandle);

  /**
   * @brief Initialize this node. Calls methods to fetch parameters and init ROS transport layer.
   *
   */
  void init();

  /**
   * @brief Gets parameters from the ROS parameters server.
   *
   */
  void getParameters();

  /**
   * @brief Initializes publishers, subscribers and services.
   *
   */
  void initRosTransport();

  /**
   * @brief ROS timed callback. Queries Gazebo about robot model state. Publishes robot pose in world frame.
   *
   */
  void gazeboTransformCallback(const ros::TimerEvent&);

  /**
   * @brief Adapts Gazebo absolute pose reference to the system's Tf tree.
   *
   * @param robotInWorld Robot pose in world frame, given by Gazebo.
   * @param worldToIntermediateGtf Transform between world and intermediate frames.
   * @return True if successful.
   */
  bool getWorldToIntermediateTransform(const geometry_msgs::PoseStamped& robotInWorld,
                                       geometry_msgs::TransformStamped& worldToIntermediateGtf);

 protected:
  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! Coordinate frames information.
  CoordinateFrameIds coordinateFrameIds_;

  //! Time period for sampling poses from Gazebo.
  double samplingTime_;

  //! Max time to wait for tf transforms.
  double waitTimeTf_;

  //! Tf.
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  tf2_ros::TransformBroadcaster tfBroadcaster_;

  //! ROS timers.
  ros::Timer transformTimer_;

  //! ROS publishers.
  ros::Publisher posePub_;

  //! ROS services.
  ros::ServiceClient gazeboTransformService_;
};

}  // namespace slam_loggers