/*
 * PoseLoggerRos.cpp
 *
 *  Author: Yoshua Nava
 *  Organization: ANYbotics AG
 *  License: Proprietary
 */

#include "slam_loggers/PoseLoggerRos.hpp"

// C standard library
#include <csignal>

// gazebo srvs
#include <gazebo_msgs/GetModelState.h>

// ros msgs
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TransformStamped.h>

// tf
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// slam common ros message conversions
#include <slam_common_ros/message_conversions/poses.hpp>

// slam loggers
#include "slam_loggers/KittiLogger.hpp"
#include "slam_loggers/TumLogger.hpp"

// message logger
#include <message_logger/message_logger.hpp>

namespace slam_loggers {

PoseLoggerRos::PoseLoggerRos(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle), tfListener_(tfBuffer_) {
  init();
}

void PoseLoggerRos::init() {
  getParameters();
  initRosTransport();

  MELO_DEBUG("Transform between frames <%s> and <%s> for robot '%s' will be logged.", coordinateFrameIds_.trackedFrameId().c_str(),
             coordinateFrameIds_.robotFrameId().c_str(), coordinateFrameIds_.robotName().c_str());

  MELO_DEBUG(
      "Ground truth between frames <%s> and <%s> for robot '%s' will be sampled sampled every time a localization pose is "
      "received",
      coordinateFrameIds_.worldFrameId().c_str(), coordinateFrameIds_.robotFrameId().c_str(), coordinateFrameIds_.robotName().c_str());

  MELO_DEBUG("Log files of <%s> during session '%s' will be saved to: %s", parameters_.filePrefix_.c_str(),
             parameters_.sessionInfo_.c_str(), parameters_.outputDirectory_.c_str());
}

void PoseLoggerRos::getParameters() {
  // Coordinate frames.
  nodeHandle_.param("coordinate_frames/robot_name", coordinateFrameIds_.robotNameMutable(), std::string("anymal"));
  nodeHandle_.param("coordinate_frames/world_frame", coordinateFrameIds_.worldFrameIdMutable(), std::string("world"));
  nodeHandle_.param("coordinate_frames/intermediate_frame", coordinateFrameIds_.intermediateFrameIdMutable(), std::string("odom"));
  nodeHandle_.param("coordinate_frames/robot_frame", coordinateFrameIds_.robotFrameIdMutable(), std::string("bla"));

  // Tracked frame.
  nodeHandle_.param("tracked_frame", coordinateFrameIds_.trackedFrameIdMutable(), std::string("map"));

  // Topic type for listening to poses.
  nodeHandle_.param("topic_type", parameters_.topicType_, std::string("PoseStamped"));

  // Whether ground truth will be queried from simulator.
  nodeHandle_.param("is_ground_truth_fetched_from_simulator", parameters_.isGroundTruthFetchedFromSimulator_, false);
  // Whether tracked pose will be transformed to the world frame.
  nodeHandle_.param("is_tracked_pose_transformed_to_world_frame", parameters_.isTrackedPoseTransformedToWorldFrame_, false);

  // Filesystem setup.
  nodeHandle_.param("output_directory", parameters_.outputDirectory_, std::string("/tmp/slam_loggers/"));
  nodeHandle_.param("file_prefix", parameters_.filePrefix_, std::string("slam"));
  nodeHandle_.param("session_info", parameters_.sessionInfo_, std::string("AnyMission"));

  // Loggers config.
  std::vector<std::string> loggingFormats;
  if (nodeHandle_.getParam("logging_formats", loggingFormats)) {
    for (const auto& loggingFormat : loggingFormats) {
      MELO_DEBUG_STREAM("Logging format " << loggingFormat);

      if (loggingFormat == "tum") {
        FileSystemConfig tumFileSystemConfig(parameters_.outputDirectory_, parameters_.filePrefix_, parameters_.sessionInfo_, "tum");
        loggers_.emplace_back(std::unique_ptr<TumLogger>(new TumLogger(tumFileSystemConfig)));
      }
      if (loggingFormat == "kitti") {
        FileSystemConfig kittiFileSystemConfig(parameters_.outputDirectory_, parameters_.filePrefix_, parameters_.sessionInfo_, "kitti");
        loggers_.emplace_back(std::unique_ptr<KittiLogger>(new KittiLogger(kittiFileSystemConfig)));
      }
    }
  } else {
    MELO_WARN("No logging formats found in the Parameter Server. Logger will not save trajectory files.");
  }

  for (auto& logger : loggers_) {
    logger->initLogFile();
  }
}

void PoseLoggerRos::initRosTransport() {
  // Services.
  resetService_ = nodeHandle_.advertiseService("reset", &PoseLoggerRos::resetCallback, this);
  groundTruthTransformService_ = nodeHandle_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

  // Publishers.
  trackedPosePub_ = nodeHandle_.advertise<geometry_msgs::PoseStamped>("tracked_pose", 1, true);
  trackedPathPub_ = nodeHandle_.advertise<nav_msgs::Path>("tracked_path", 1, true);

  if (parameters_.isTrackedPoseTransformedToWorldFrame_) {
    trackedPoseInWorldFramePub_ = nodeHandle_.advertise<geometry_msgs::PoseStamped>("tracked_pose_in_world_frame", 1, true);
    trackedPathInWorldFramePub_ = nodeHandle_.advertise<nav_msgs::Path>("tracked_path_in_world_frame", 1, true);
  }

  if (parameters_.isGroundTruthFetchedFromSimulator_) {
    groundTruthPosePub_ = nodeHandle_.advertise<geometry_msgs::PoseStamped>("gt_pose", 1, true);
    groundTruthPathPub_ = nodeHandle_.advertise<nav_msgs::Path>("gt_path", 1, true);
  }

  // Subscribers.
  if (parameters_.topicType_ == "PoseStamped") {
    robotPoseSub_ = nodeHandle_.subscribe<geometry_msgs::PoseStamped>("robot_pose", 1, &PoseLoggerRos::poseCallback, this,
                                                                      ros::TransportHints().tcpNoDelay());
  } else if (parameters_.topicType_ == "PoseWithCovarianceStamped") {
    robotPoseSub_ = nodeHandle_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
        "robot_pose", 1, &PoseLoggerRos::poseWithCovarianceCallback, this, ros::TransportHints().tcpNoDelay());
  } else {
    MELO_FATAL_STREAM("Invalid topic type " << parameters_.topicType_);
  }
}

void PoseLoggerRos::reset() {
  std::lock_guard<std::mutex> lock(mutex_);

  // Save logs.
  saveLogs();

  // Clear log of poses.
  poseLog_.clear();

  // Clear ROS containers.
  trackedPath_ = nav_msgs::Path();
  trackedPathInWorldFrame_ = nav_msgs::Path();
  groundTruthPath_ = nav_msgs::Path();
}

bool PoseLoggerRos::resetCallback(std_srvs::Empty::Request& /*request*/, std_srvs::Empty::Response& /*response*/) {
  MELO_DEBUG("Reset request");
  reset();
  return true;
}

void PoseLoggerRos::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& poseStampedMsg) {
  MELO_DEBUG("Pose callback");

  logPose(*poseStampedMsg);
}

void PoseLoggerRos::poseWithCovarianceCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& poseWithCovarianceMsg) {
  MELO_DEBUG("Pose with covariance callback");

  // Convert to PoseStamped and add to the logs.
  geometry_msgs::PoseStamped poseStamped;
  poseStamped.header = poseWithCovarianceMsg->header;
  poseStamped.pose = poseWithCovarianceMsg->pose.pose;

  logPose(poseStamped);
}

void PoseLoggerRos::logPose(const geometry_msgs::PoseStamped& poseStamped) {
  const auto stamp = poseStamped.header.stamp;
  const auto poseFrame = poseStamped.header.frame_id;

  if (poseFrame != coordinateFrameIds_.trackedFrameId()) {
    MELO_ERROR("Frame of received pose message is '%s'. Expected '%s'", poseFrame.c_str(), coordinateFrameIds_.trackedFrameId().c_str());
    return;
  }

  // Lock mutex from here on.
  std::lock_guard<std::mutex> lock(mutex_);

  // Create service call.
  gazebo_msgs::GetModelState srv;
  srv.request.model_name = coordinateFrameIds_.robotName();

  // Get gazebo ground truth.
  if (parameters_.isGroundTruthFetchedFromSimulator_) {
    if (groundTruthTransformService_.call(srv)) {
      MELO_DEBUG("Gazebo-sim Ground Truth received");
      geometry_msgs::PoseStamped robotInWorld;
      robotInWorld.header.stamp = stamp;
      robotInWorld.header.frame_id = coordinateFrameIds_.worldFrameId();
      robotInWorld.pose = srv.response.pose;

      // Transform ground truth pose to Eigen isometry.
      const Eigen::Isometry3d groundTruthPose = slam_common::convertPoseStampedMsgToEigenIsometry(robotInWorld);
      poseLog_.groundTruthPoses_.emplace_back(groundTruthPose);

      // Publish ground truth pose.
      groundTruthPath_.header = robotInWorld.header;
      groundTruthPath_.poses.push_back(robotInWorld);
      if (groundTruthPosePub_.getNumSubscribers() > 0u || groundTruthPosePub_.isLatched()) {
        groundTruthPosePub_.publish(robotInWorld);
      }
      if (groundTruthPathPub_.getNumSubscribers() > 0u || groundTruthPathPub_.isLatched()) {
        groundTruthPathPub_.publish(groundTruthPath_);
      }
    } else {
      MELO_WARN_THROTTLE(10.0, "Ground truth between frames <%s> and <%s> could not be sampled. (Warning is throttled: 10s)",
                         coordinateFrameIds_.worldFrameId().c_str(), coordinateFrameIds_.robotFrameId().c_str());
    }
  }

  // Transform SLAM pose to world frame.
  if (parameters_.isTrackedPoseTransformedToWorldFrame_ &&
      tfBuffer_.canTransform(coordinateFrameIds_.worldFrameId(), poseFrame, stamp, ros::Duration(parameters_.waitTimeTf_))) {
    // Transform received pose to world frame.
    const geometry_msgs::TransformStamped mapToWorldGtf =
        tfBuffer_.lookupTransform(coordinateFrameIds_.worldFrameId(), poseFrame, stamp, ros::Duration(parameters_.waitTimeTf_));
    geometry_msgs::PoseStamped robotPoseInWorldMsg;
    tf2::doTransform(poseStamped, robotPoseInWorldMsg, mapToWorldGtf);

    // Transform robot pose in world frame to Eigen isometry.
    const Eigen::Isometry3d robotInWorldFramePose = slam_common::convertPoseMsgToEigenIsometry(robotPoseInWorldMsg.pose);
    poseLog_.robotInWorldFramePoses_.emplace_back(robotInWorldFramePose);

    // Publish robot pose in world frame.
    robotPoseInWorldMsg.header.stamp = stamp;
    robotPoseInWorldMsg.header.frame_id = coordinateFrameIds_.worldFrameId();
    trackedPathInWorldFrame_.header.stamp = stamp;
    trackedPathInWorldFrame_.header.frame_id = poseFrame;
    trackedPathInWorldFrame_.poses.push_back(robotPoseInWorldMsg);
    if (trackedPoseInWorldFramePub_.getNumSubscribers() > 0u || trackedPoseInWorldFramePub_.isLatched()) {
      trackedPoseInWorldFramePub_.publish(robotPoseInWorldMsg);
    }
    if (trackedPathInWorldFramePub_.getNumSubscribers() > 0u || trackedPathInWorldFramePub_.isLatched()) {
      trackedPathInWorldFramePub_.publish(trackedPathInWorldFrame_);
    }
  }

  // Publish robot pose in tracked frame.
  trackedPath_.header.stamp = stamp;
  trackedPath_.header.frame_id = poseFrame;
  trackedPath_.poses.push_back(poseStamped);
  if (trackedPosePub_.getNumSubscribers() > 0u || trackedPosePub_.isLatched()) {
    trackedPosePub_.publish(poseStamped);
  }
  if (trackedPathPub_.getNumSubscribers() > 0u || trackedPathPub_.isLatched()) {
    trackedPathPub_.publish(trackedPath_);
  }

  // Transform robot pose in tracked frame to Eigen isometry.
  const Eigen::Isometry3d robotPose = slam_common::convertPoseMsgToEigenIsometry(poseStamped.pose);

  poseLog_.robotInTrackedFramePoses_.emplace_back(robotPose);
  poseLog_.timestamps_.emplace_back(stamp);
  MELO_DEBUG("%lu poses logged", poseLog_.timestamps_.size());
}  // namespace slam_loggers

void PoseLoggerRos::saveLogs() {
  for (auto& logger : loggers_) {
    logger->saveLogToFilesystem(poseLog_);
  }
}

}  // namespace slam_loggers