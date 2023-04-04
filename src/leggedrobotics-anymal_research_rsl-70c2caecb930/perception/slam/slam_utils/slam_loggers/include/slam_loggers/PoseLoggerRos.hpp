/*
 * PoseLoggerRos.hpp
 *
 *  Author: Yoshua Nava
 *  Organization: ANYbotics AG
 *  License: Proprietary
 */
#pragma once

// standard library
#include <mutex>
#include <vector>

// eigen
#include <Eigen/Dense>

// ros
#include <ros/ros.h>

// ros msgs
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>

// std srvs
#include <std_srvs/Empty.h>

// tf
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

// slam loggers
#include "slam_loggers/BaseLogger.hpp"
#include "slam_loggers/CoordinateFrameIds.hpp"
#include "slam_loggers/PoseLog.hpp"

namespace slam_loggers {

struct PoseLoggerRosParameters {
  //! Absolute path of the directory where log files will be saved.
  std::string outputDirectory_ = "";

  //! Prefix that will be added to the log files (e.g. 'vio', 'slam')
  std::string filePrefix_ = "";

  //! Session info suffix.
  std::string sessionInfo_ = "";

  //! Whether the ground truth poses should be fetched from the simulator (if available).
  bool isGroundTruthFetchedFromSimulator_ = false;

  //! Whether the tracked pose should be transformed to the ground truth frame.
  bool isTrackedPoseTransformedToWorldFrame_ = false;

  //! Type of topic to which the logger will listen.
  std::string topicType_ = "";

  //! Max time to wait for tf transforms.
  double waitTimeTf_ = 0.05;
};

/*
 * @brief Class for logging output from localization modules, in different formats.
 */
class PoseLoggerRos {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Parameters = PoseLoggerRosParameters;

  /**
   * @brief Constructor
   *
   * @param nodeHandle A ROS NodeHandle.
   */
  explicit PoseLoggerRos(ros::NodeHandle& nodeHandle);

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
   * @brief Resets the logger, discarding all logged trajectories and starting over.
   *
   */
  void reset();

  /**
   * @brief Requests that the logger is reset.
   *
   * @param request   ROS service request.
   * @param response  Response to service request.
   * @return true     If successful, false otherwise.
   */
  bool resetCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  /**
   * @brief Callback of ROS Pose subscriber
   *
   * @param poseStampedMsg A pose stamped message.
   */
  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& poseStampedMsg);

  /**
   * @brief Callback of ROS Pose subscriber
   *
   * @param poseStampedMsg A pose with covariance stamped message.
   */
  void poseWithCovarianceCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& poseWithCovarianceMsg);

  /**
   * @brief Log pose. Tries to fetch ground truth from Gazebo. Saves and publishes path msg.
   *
   * @param poseStamped A pose stamped message.
   */
  void logPose(const geometry_msgs::PoseStamped& poseStamped);

  /**
   * @brief Calls loggers and saves logs to filesystem.
   *
   */
  void saveLogs();

 protected:
  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! Node parameters.
  Parameters parameters_;

  //! Coordinate frames information.
  CoordinateFrameIds coordinateFrameIds_;

  //! Mutex to lock the pose log.
  mutable std::mutex mutex_;
  //! Logs.
  PoseLog poseLog_;

  //! Loggers.
  std::vector<BaseLogger::UniquePtr> loggers_;

  //! Tf.
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;

  //! Paths composed of the received robot poses.
  nav_msgs::Path trackedPath_;
  nav_msgs::Path trackedPathInWorldFrame_;
  nav_msgs::Path groundTruthPath_;

  //! ROS publishers.
  ros::Publisher trackedPosePub_;
  ros::Publisher trackedPathPub_;
  ros::Publisher trackedPoseInWorldFramePub_;
  ros::Publisher trackedPathInWorldFramePub_;
  ros::Publisher groundTruthPosePub_;
  ros::Publisher groundTruthPathPub_;

  //! ROS subscribers.
  ros::Subscriber robotPoseSub_;

  //! ROS services.
  ros::ServiceClient groundTruthTransformService_;
  ros::ServiceServer resetService_;
};

}  // namespace slam_loggers