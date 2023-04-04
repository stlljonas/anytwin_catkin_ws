/*!
* @file     AnymalTfPublisher.cpp
* @author   Christian Gehring, Dario Bellicoso
* @date     Sep 21, 2015
* @brief
*/

// anymal tf publisher
#include "anymal_tf_publisher/AnymalTfPublisher.hpp"

// anymal model ros
#include <anymal_model_ros/initializations.hpp>

// anymal model
#include <anymal_model/StateStatus.hpp>

// any measurements ros
#include <any_measurements_ros/any_measurements_ros.hpp>

// kindr ros
#include <kindr_ros/kindr_ros.hpp>

// urdf
#include <urdf/model.h>

// kdl
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace anymal_tf_publisher {

AnymalTfPublisher::AnymalTfPublisher(any_node::Node::NodeHandlePtr nh) :
any_node::Node(nh),
newState_(false)
{

}

bool AnymalTfPublisher::init() {
  const double publishFrequency = param<double>("publish_frequency", 100);

  tfPrefix_ = param<std::string>("tf_prefix", "");
  baseFrameId_ = param<std::string>("base_frame_id", "base");
  odomFrameId_ = param<std::string>("odom_frame_id", "odom");

  auto qsOptions = std::make_shared<cosmo_ros::SubscriberRosOptions<AnymalStateShm>>(
      "anymal_state", std::bind(&AnymalTfPublisher::anymalStateCallback, this, std::placeholders::_1), getNodeHandle());
  qsOptions->autoSubscribe_ = false;
  qsOptions->rosTransportHints_ = ros::TransportHints().tcpNoDelay();
  qsOptions->tryRosResubscribing_ = true;

  stateSubscriber_ = cosmo_ros::subscribeShmRos<AnymalStateShm,
                                                AnymalStateRos,
                                                anymal_model_ros::conversion_traits::ConversionTraits>("anymal_state", qsOptions);

  ignoreState_ = param<bool>("ignore_state", false);

  baseTransform_.header.frame_id = odomFrameId_;
  baseTransform_.child_frame_id = baseFrameId_;
  addTfPrefix(baseTransform_.child_frame_id);

  anymal_model_ros::initialize(frameTransforms_);
  // TODO: Here we know that the header.frame is always odom and only the children have to be prefixed.
  //       This could be solved nicer by having support for tf prefixes in the anymal_model_ros functions.
  addTfPrefix(frameTransforms_[0].child_frame_id);
  addTfPrefix(frameTransforms_[1].child_frame_id);

  for (const auto& jointKey : anymal_description::AnymalDescription::getJointKeys()) {
    jointPositions_.insert(std::pair<std::string, double>(jointKey.getName(), 0.0));
  }

  // Initialize robot state publisher
  std::string urdfName = param<std::string>("robot_description", "");
  urdf::Model model;
  if (!model.initParam(urdfName)) {
    MELO_WARN("URDF model load was NOT successful");
    return false;
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree)) {
    ROS_ERROR("Failed to extract kdl tree from xml robot description");
    return false;
  }

  robotStatePublisher_ = std::shared_ptr<robot_state_publisher::RobotStatePublisher>(new robot_state_publisher::RobotStatePublisher(tree));
  robotStatePublisher_->publishFixedTransforms(tfPrefix_, true);

  // Update worker
  any_worker::WorkerOptions workerOptions;
  workerOptions.name_ = ros::this_node::getName() + std::string{"_broadcast"};
  workerOptions.callback_ = boost::bind(&AnymalTfPublisher::update, this, _1);
  workerOptions.timeStep_ = 1.0/publishFrequency;
  workerOptions.defaultPriority_ = 0;

  if (!addWorker(workerOptions)) {
    ROS_ERROR_STREAM("Could not add worker: " << workerOptions.name_);
    return false;
  }

  return true;
}

void AnymalTfPublisher::cleanup() {
  stateSubscriber_->stop();
}

void AnymalTfPublisher::anymalStateCallback(const AnymalStateShm& state) {
  {
    std::lock_guard<std::mutex> lockModel(mutexAnymalState_);
    anymalState_ = state;
    newState_ = true;
  }
}

bool AnymalTfPublisher::update(const any_worker::WorkerEvent& event) {
  using AT = anymal_description::AnymalTopology;

  stateSubscriber_->receive();
  if(newState_)
  {
    std::lock_guard<std::mutex> lockModel(mutexAnymalState_);
    ros::Time timeMsg = any_measurements_ros::toRos(anymalState_.time_);

    if (!anymalState_.time_.isZero() &&
        (ignoreState_ || anymalState_.status_ >= anymal_model::StateStatus::STATUS_OK)) {
      // Broadcast transformation from anymal state frame to robot base.
      baseTransform_.header.stamp = timeMsg;
      kindr_ros::convertToRosGeometryMsg(anymalState_.anymalState_.getPositionWorldToBaseInWorldFrame(), baseTransform_.transform.translation);
      kindr_ros::convertToRosGeometryMsg(anymalState_.anymalState_.getOrientationBaseToWorld(), baseTransform_.transform.rotation);
      tfBroadcaster_.sendTransform(baseTransform_);
      try {
        frameTransforms_[0].header.stamp = timeMsg;
        kindr_ros::convertToRosGeometryMsg(anymalState_.anymalState_.getFrameTransform(AT::FrameTransformEnum::FootprintToOdom), frameTransforms_[0].transform);
        tfBroadcaster_.sendTransform(frameTransforms_[0]);
      } catch (...) {
        ROS_ERROR("AnymalTfPublisher: Could not get frame footprint");
      }
      try {
        frameTransforms_[1].header.stamp = timeMsg;
        kindr_ros::convertToRosGeometryMsg(anymalState_.anymalState_.getFrameTransform(AT::FrameTransformEnum::FeetcenterToOdom), frameTransforms_[1].transform);
        tfBroadcaster_.sendTransform(frameTransforms_[1]);
      } catch (...) {
        ROS_ERROR("AnymalTfPublisher: Could not get frame feetcenter");
      }
    }

    for (const auto& jointKey : anymal_description::AnymalDescription::getJointKeys()) {
      jointPositions_[jointKey.getName()] = anymalState_.anymalState_.getJointPositions()(jointKey.getId());
    }

    robotStatePublisher_->publishTransforms(jointPositions_, timeMsg, tfPrefix_);
    newState_ = false;
  }

  return true;
}

void AnymalTfPublisher::addTfPrefix(std::string& frameId) {
  frameId = tf::resolve(tfPrefix_, frameId);
}

} /* namespace anymal_tf_publisher */
