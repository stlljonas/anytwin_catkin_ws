/*!
* @file     AnymalTfPublisher.hpp
* @author   Christian Gehring
* @date     Sep 21, 2015
* @brief
*/
#pragma once

// ros
#include <ros/ros.h>

// tf
#include <tf/transform_broadcaster.h>

// anymal model ros
#include <anymal_model_ros/conversions.hpp>

// anymal model
#include <anymal_model/ExtendedAnymalState.hpp>

// robot state publisher
#include <robot_state_publisher/robot_state_publisher.h>

// any node
#include <any_node/any_node.hpp>

// anymal msgs
#include <anymal_msgs/AnymalState.h>

// stl
#include <mutex>

// cosmo ros
#include <cosmo_ros/cosmo_ros.hpp>

namespace anymal_tf_publisher {

//! This currently only re-publishes the robot state to two separate topics: pose and joint state.
class AnymalTfPublisher : public any_node::Node {
 public:
  typedef anymal_model::ExtendedAnymalState AnymalStateShm;
  typedef anymal_msgs::AnymalState AnymalStateRos;

  AnymalTfPublisher(any_node::Node::NodeHandlePtr nh);
  ~AnymalTfPublisher() override = default;

  bool init() override;
  void cleanup() override;
  void anymalStateCallback(const AnymalStateShm& msg);
  bool update(const any_worker::WorkerEvent& event);

 protected:
  cosmo_ros::SubscriberRosPtr<AnymalStateShm, AnymalStateRos,
                              anymal_model_ros::conversion_traits::ConversionTraits> stateSubscriber_;
  AnymalStateShm anymalState_;
  anymal_msgs::AnymalState anymalStateMsg_;
  std::mutex mutexAnymalState_;
  tf::TransformBroadcaster tfBroadcaster_;

  std::map<std::string, double> jointPositions_;
  std::shared_ptr<robot_state_publisher::RobotStatePublisher> robotStatePublisher_;

  std::string tfPrefix_;
  std::string baseFrameId_;
  std::string odomFrameId_;

  bool ignoreState_ = false;
  std::atomic<bool> newState_;
  geometry_msgs::TransformStamped baseTransform_;
  std::vector<geometry_msgs::TransformStamped> frameTransforms_;

  void addTfPrefix(std::string& frameId);
};

} /* namespace anymal_tf_publisher */
