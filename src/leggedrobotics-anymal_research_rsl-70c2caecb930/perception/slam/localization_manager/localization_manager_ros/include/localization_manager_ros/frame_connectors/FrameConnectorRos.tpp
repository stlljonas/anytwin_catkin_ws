#pragma once

// param io
#include <param_io/get_param.hpp>

// logging
#include <message_logger/message_logger.hpp>

// tf
#include <tf/exceptions.h>

// localization manager ros
#include <localization_manager_ros/frame_connectors/FrameConnectorRos.hpp>

namespace localization_manager_ros {

template <typename FrameConnectorType>
FrameConnectorRos<FrameConnectorType>::FrameConnectorRos(ros::NodeHandle& nodeHandle, const std::string& connectorTypeNamespace,
                                                         const std::string& name)
    : Base() {
  // Fetch general frame connector params.
  config_.odomFrameId_ = param_io::param<std::string>(nodeHandle, "frame_connectors/odom_frame_id", std::string{"odom"});
  config_.trackedFrameIdInOdom_ =
      param_io::param<std::string>(nodeHandle, "frame_connectors/tracked_frame_id_in_odom", std::string{"base"});
  config_.tfLookupTimeout_ = param_io::param<double>(nodeHandle, "frame_connectors/tf_lookup_timeout", 2.);
  config_.externalTransformTimeout_ = param_io::param<double>(nodeHandle, "frame_connectors/external_transform_timeout", 5.);

  // Build full namespace of the frame connector params.
  const auto id = connectorTypeNamespace + std::string{"/"} + name;

  // Fetch params for this connector.
  config_.name_ = name;
  config_.extFrameIdIn_ = param_io::param<std::string>(nodeHandle, id + std::string{"/ext_frame_id_in"}, std::string{"ext"});
  if (param_io::param<bool>(nodeHandle, id + std::string{"/remap_ext_frame_id"}, false)) {
    config_.extFrameIdOut_ = param_io::param<std::string>(nodeHandle, id + std::string{"/ext_frame_id_out"}, std::string{"ext"});
  } else {
    config_.extFrameIdOut_ = config_.extFrameIdIn_;
  }

  config_.broadcastZeroInitTf_ = param_io::param<bool>(nodeHandle, id + std::string{"/broadcast_zero_init_tf"}, false);
  if (frameConnector_.isFeedthrough()) {
    poseInOdomBufferPtr_.reset(new PoseBuffer(0.));
  } else {
    config_.trackedFrameIdInExt_ =
        param_io::param<std::string>(nodeHandle, id + std::string{"/tracked_frame_id_in_ext"}, std::string{"base"});
    poseInOdomBufferPtr_.reset(new PoseBuffer(param_io::param<double>(nodeHandle, id + std::string{"/pose_in_odom_buffer_size"}, 1.)));
  }

  config_.inputMsgType_ =
      static_cast<typename Config::InputMsgType>(param_io::param<unsigned int>(nodeHandle, id + std::string{"/input_msg_type"}, 0u));

  // Initialize the pose of the external frame in odom.
  extInOdomTf_.header.frame_id = config_.odomFrameId_;
  extInOdomTf_.child_frame_id = config_.extFrameIdOut_;
  poseToTransformRos(Pose(), extInOdomTf_);

  // Initialize subscriber to the pose measurement.
  const auto inputTopicName = param_io::param<std::string>(nodeHandle, id + std::string{"/input_topic"}, std::string{"default"});
  switch (config_.inputMsgType_) {
    case Config::InputMsgType::POSE_WITH_COVARIANCE_STAMPED:
      measurementSubscriber_ =
          nodeHandle.subscribe(inputTopicName, 1u, &FrameConnectorRos<FrameConnectorType>::poseMeasurementCallback, this);
      break;
    case Config::InputMsgType::ODOMETRY:
      measurementSubscriber_ =
          nodeHandle.subscribe(inputTopicName, 1u, &FrameConnectorRos<FrameConnectorType>::odometryMeasurementCallback, this);
      break;
    default:
      MELO_WARN("Unknown input msg type requested, using PoseWithCovarianceStamped.");
      measurementSubscriber_ =
          nodeHandle.subscribe(inputTopicName, 1u, &FrameConnectorRos<FrameConnectorType>::poseMeasurementCallback, this);
      break;
  }

  // Initialize service to reset this frame connector.
  resetServer_ =
      nodeHandle.advertiseService(name + std::string{"/reset"}, &FrameConnectorRos<FrameConnectorType>::resetServiceCallback, this);
}

template <typename FrameConnectorType>
const typename FrameConnectorRos<FrameConnectorType>::TransformRos& FrameConnectorRos<FrameConnectorType>::getTfExtInOdom() const {
  std::lock_guard<std::mutex> lock(mutexFrameConnectorRos_);
  return extInOdomTf_;
}

template <typename FrameConnectorType>
void FrameConnectorRos<FrameConnectorType>::updatePoseInOdom(const PoseRosPtr& poseInOdom) {
  std::lock_guard<std::mutex> lock(mutexFrameConnectorRos_);
  if (poseInOdom->header.frame_id != config_.odomFrameId_) {
    MELO_WARN_THROTTLE_STREAM(3, "\"" << config_.name_ << "\" received inconsistent frame id in pose in odom: "
                                      << poseInOdom->header.frame_id << " != " << config_.odomFrameId_ << ".");
    return;
  }
  poseInOdomBufferPtr_->add(any_measurements_ros::fromRos(poseInOdom->header.stamp), any_measurements_ros::fromRos(*poseInOdom).pose_);
  // Advance extInOdomTf_ time using the odometry stamp if it's not a feedthrough connector.
  if (!frameConnector_.isFeedthrough()) {
    extInOdomTf_.header.stamp = poseInOdom->header.stamp;
  }
}

template <typename FrameConnectorType>
void FrameConnectorRos<FrameConnectorType>::configureImpl(std::shared_ptr<ImplConfigBase> configPtr) {
  std::lock_guard<std::mutex> lock(mutexFrameConnectorRos_);
  frameConnector_.configure(configPtr);
}

template <typename FrameConnectorType>
bool FrameConnectorRos<FrameConnectorType>::hasTfExtInOdom() {
  // if this is a feedthrough connector, we don't need poses in odom
  const bool hasPoseInOdom = (!poseInOdomBufferPtr_->isEmpty()) || frameConnector_.isFeedthrough();
  const bool hasValidTfExtInOdom = (extInOdomTf_.header.stamp.toSec() > 0.);
  const bool hasRecentTfExtInOdom = ((ros::Time::now() - extInOdomTf_.header.stamp).toSec() < config_.externalTransformTimeout_);
  if (!hasRecentTfExtInOdom) {
    MELO_DEBUG_THROTTLE_STREAM(
        3, "Frame connector timeout has been triggered by the absence of robot poses in frame '" << config_.extFrameIdIn_ << "'")
  }

  return (hasPoseInOdom && hasValidTfExtInOdom && hasRecentTfExtInOdom) || config_.broadcastZeroInitTf_;
}

template <typename FrameConnectorType>
bool FrameConnectorRos<FrameConnectorType>::hasToBroadcastZeroInitialTf() {
  return config_.broadcastZeroInitTf_;
}

template <typename FrameConnectorType>
void FrameConnectorRos<FrameConnectorType>::reset() {
  std::lock_guard<std::mutex> lock(mutexFrameConnectorRos_);
  extInOdomTf_.header.stamp = ros::Time(0);
  poseInOdomBufferPtr_->reset();
  frameConnector_.reset();
}

template <typename FrameConnectorType>
void FrameConnectorRos<FrameConnectorType>::getPoseOdomTrackInExtTrack() {
  if (!frameConnector_.isFeedthrough() && !poseOdomTrackInExtTrackInited_) {
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    try {
      const TransformRos odomTrackInExtTrackTransform = tfBuffer.lookupTransform(
          config_.trackedFrameIdInExt_, config_.trackedFrameIdInOdom_, ros::Time::now(), ros::Duration(config_.tfLookupTimeout_));
      transformRosToPose(odomTrackInExtTrackTransform, poseOdomTrackInExtTrack_);
      poseOdomTrackInExtTrackInited_ = true;
    } catch (const tf::TransformException& exception) {
      MELO_WARN_THROTTLE_STREAM(
          3, "\"" << config_.name_
                  << "\" caught a tf exception while getting the tracked frame offsets, will use identity for now and retry: "
                  << exception.what());
      poseOdomTrackInExtTrack_ = Pose();
    }
  }
}

template <typename FrameConnectorType>
void FrameConnectorRos<FrameConnectorType>::processRosPoseMeasurement(const PoseRosPtr& msg) {
  std::lock_guard<std::mutex> lock(mutexFrameConnectorRos_);

  // Frame semantics checks.
  if (frameConnector_.isFeedthrough()) {
    if ((msg->header.frame_id != config_.odomFrameId_)) {
      MELO_WARN_THROTTLE_STREAM(3, "\"" << config_.name_ << "\" received inconsistent frame id in pose measurement: "
                                        << msg->header.frame_id << " != " << config_.odomFrameId_ << ".");
      return;
    }
  } else {
    if ((msg->header.frame_id != config_.extFrameIdIn_)) {
      MELO_WARN_THROTTLE_STREAM(3, "\"" << config_.name_ << "\" received inconsistent frame id in pose measurement: "
                                        << msg->header.frame_id << " != " << config_.extFrameIdIn_ << ".");
      return;
    }
  }
  if (msg->header.stamp.toSec() <= 0.) {
    return;
  }

  getPoseOdomTrackInExtTrack();

  auto poseInOdom = Pose();
  // Get pose in odom for non-feedthrough frame connectors.
  if (!frameConnector_.isFeedthrough()) {
    if (!poseInOdomBufferPtr_->find(any_measurements_ros::fromRos(msg->header.stamp), poseInOdom)) {
      MELO_WARN_THROTTLE_STREAM(
          3, "\"" << config_.name_
                  << "\" attempted to update the frame connector unsuccessfully due to failed pose buffer lookup (buffer size/miss count = "
                  << poseInOdomBufferPtr_->getSize() << "/" << poseInOdomBufferPtr_->getMissCount() << ").");
      return;
    }
  }

  // Update pose of external frame in odom (orientation and translation).
  poseToTransformRos(
      frameConnector_.update(poseInOdom, any_measurements_ros::fromRos(*msg).pose_ * poseOdomTrackInExtTrack_, msg->header.stamp.toSec()),
      extInOdomTf_);

  // If this is a a feedthrough connector, use the pose measurement stamp to advance extInOdomTf_ time.
  if (frameConnector_.isFeedthrough()) {
    extInOdomTf_.header.stamp = msg->header.stamp;
  }
}

template <typename FrameConnectorType>
void FrameConnectorRos<FrameConnectorType>::odometryMeasurementCallback(const OdometryRosPtr& msg) {
  // Extract the pose from the odometry msg and process it
  PoseRosPtr poseRosPtr(new PoseRos());
  poseRosPtr->header = msg->header;
  poseRosPtr->pose = msg->pose;
  processRosPoseMeasurement(poseRosPtr);
}

template <typename FrameConnectorType>
void FrameConnectorRos<FrameConnectorType>::poseMeasurementCallback(const PoseRosPtr& msg) {
  processRosPoseMeasurement(msg);
}

template <typename FrameConnectorType>
bool FrameConnectorRos<FrameConnectorType>::resetServiceCallback(std_srvs::Trigger::Request& /*req*/, std_srvs::Trigger::Response& res) {
  reset();

  // Build reply message and return.
  res.success = static_cast<unsigned char>(true);
  res.message = config_.name_ + std::string{" has been reset."};
  MELO_INFO_STREAM(res.message)
  return true;
}

}  // namespace localization_manager_ros
