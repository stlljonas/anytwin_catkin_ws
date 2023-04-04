/*
 * GazeboGroundTruthPosePublisher.cpp
 *
 *  Author: Yoshua Nava
 *  Organization: ANYbotics AG
 *  License: Proprietary
 */

#include "slam_loggers/GazeboGroundTruthPosePublisher.hpp"

// gazebo msgs
#include <gazebo_msgs/GetModelState.h>

// ros msgs
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>

// tf
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// message logger
#include <message_logger/message_logger.hpp>

namespace slam_loggers {

GazeboGroundTruthPosePublisher::GazeboGroundTruthPosePublisher(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle), samplingTime_(0.05), waitTimeTf_(0.05), tfListener_(tfBuffer_) {
  init();
}

void GazeboGroundTruthPosePublisher::init() {
  getParameters();
  initRosTransport();

  MELO_DEBUG("Transform between frames <%s> and <%s> for robot '%s', sampled every %f seconds from Gazebo",
             coordinateFrameIds_.worldFrameId().c_str(), coordinateFrameIds_.intermediateFrameId().c_str(),
             coordinateFrameIds_.robotName().c_str(), samplingTime_);
}

void GazeboGroundTruthPosePublisher::getParameters() {
  // Parameters for sampling the robot state via Gazebo.
  nodeHandle_.param("sampling_time", samplingTime_, 0.05);
  nodeHandle_.param("wait_time_tf", waitTimeTf_, 0.05);

  // Coordinate frames.
  nodeHandle_.param("coordinate_frames/robot_name", coordinateFrameIds_.robotNameMutable(), std::string("anymal"));
  nodeHandle_.param("coordinate_frames/world_frame", coordinateFrameIds_.worldFrameIdMutable(), std::string("world"));
  nodeHandle_.param("coordinate_frames/intermediate_frame", coordinateFrameIds_.intermediateFrameIdMutable(), std::string("odom"));
  nodeHandle_.param("coordinate_frames/robot_frame", coordinateFrameIds_.robotFrameIdMutable(), std::string("base"));
}

void GazeboGroundTruthPosePublisher::initRosTransport() {
  // Services.
  gazeboTransformService_ = nodeHandle_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

  // Publishers.
  posePub_ = nodeHandle_.advertise<geometry_msgs::PoseStamped>("robot_pose", 1, false);

  // Timers.
  transformTimer_ = nodeHandle_.createTimer(ros::Duration(samplingTime_), &GazeboGroundTruthPosePublisher::gazeboTransformCallback, this);
}

void GazeboGroundTruthPosePublisher::gazeboTransformCallback(const ros::TimerEvent&) {
  MELO_DEBUG("Gazebo ground truth extraction callback.");
  const auto stamp = ros::Time::now();

  gazebo_msgs::GetModelState srv;
  srv.request.model_name = coordinateFrameIds_.robotName();

  // Request the absolute pose of the robot model from Gazebo.
  if (gazeboTransformService_.call(srv)) {
    geometry_msgs::PoseStamped robotInWorld;
    robotInWorld.header.stamp = stamp;
    robotInWorld.header.frame_id = coordinateFrameIds_.worldFrameId();
    robotInWorld.pose = srv.response.pose;

    // Broadcast world->intermediate transform to the Tf tree.
    geometry_msgs::TransformStamped worldToIntermediateGtf;
    if (getWorldToIntermediateTransform(robotInWorld, worldToIntermediateGtf)) {
      worldToIntermediateGtf.header.stamp = stamp;
      worldToIntermediateGtf.header.frame_id = coordinateFrameIds_.worldFrameId();
      worldToIntermediateGtf.child_frame_id = coordinateFrameIds_.intermediateFrameId();
      tfBroadcaster_.sendTransform(worldToIntermediateGtf);
    }

    // Publish pose.
    if (posePub_.getNumSubscribers() > 0u || posePub_.isLatched()) {
      posePub_.publish(robotInWorld);
    }
  } else {
    MELO_ERROR_THROTTLE(10, "Ground truth from Gazebo simulator could not be fetched. (Warning is throttled: 10s)");
  }
}

bool GazeboGroundTruthPosePublisher::getWorldToIntermediateTransform(const geometry_msgs::PoseStamped& robotInWorld,
                                                                     geometry_msgs::TransformStamped& worldToIntermediateGtf) {
  // We start with world->robot transform
  const auto stamp = robotInWorld.header.stamp;
  tf2::Stamped<tf2::Transform> intermediateToWorldTf;

  if (tfBuffer_.canTransform(coordinateFrameIds_.robotFrameId(), coordinateFrameIds_.intermediateFrameId(), stamp,
                             ros::Duration(waitTimeTf_))) {
    try {
      // Get robot->intermediate transform
      geometry_msgs::PoseStamped worldInIntermediate;
      geometry_msgs::TransformStamped robotToIntermediateGtf = tfBuffer_.lookupTransform(
          coordinateFrameIds_.robotFrameId(), coordinateFrameIds_.intermediateFrameId(), stamp, ros::Duration(waitTimeTf_));

      // Estimate intermediate->world transform
      tf2::doTransform(robotInWorld, worldInIntermediate, robotToIntermediateGtf);

      // Get world-intermediate transform
      tf2::fromMsg(worldInIntermediate, intermediateToWorldTf);
      tf2::Stamped<tf2::Transform> worldToIntermediate =
          tf2::Stamped<tf2::Transform>(intermediateToWorldTf.inverse(), stamp, coordinateFrameIds_.worldFrameId());
      worldToIntermediateGtf = tf2::toMsg(worldToIntermediate);

      return true;
    } catch (tf2::TransformException& ex) {
      MELO_WARN("%s", ex.what());
      return false;
    }
  }

  return false;
}

}  // namespace slam_loggers