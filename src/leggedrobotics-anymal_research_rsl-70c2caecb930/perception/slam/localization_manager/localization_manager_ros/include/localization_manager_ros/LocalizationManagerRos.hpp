#pragma once

// std
#include <string>

// ros
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

// any node
#include <any_node/Node.hpp>

// localization manager
#include <localization_manager/LocalizationManager.hpp>

// frame connectors
#include <localization_manager/frame_connectors/DifferenceFrameConnector.hpp>
#include <localization_manager/frame_connectors/FeedthroughFrameConnector.hpp>
#include <localization_manager_ros/frame_connectors/FrameConnectorRos.hpp>

namespace localization_manager_ros {

class LocalizationManagerRos : public any_node::Node {
 public:
  using FeedthroughFrameConnectorRos = FrameConnectorRos<localization_manager::FeedthroughFrameConnector>;
  using DifferenceFrameConnectorRos = FrameConnectorRos<localization_manager::DifferenceFrameConnector>;

  using PoseRosPtr = geometry_msgs::PoseWithCovarianceStampedPtr;

  /* Constructor and destructor */
  LocalizationManagerRos() = delete;
  explicit LocalizationManagerRos(any_node::Node::NodeHandlePtr nh);
  ~LocalizationManagerRos() final = default;

  /* ANYnode methods */
  bool init() final { return true; };
  void preCleanup() final{};
  void cleanup() final{};

 protected:
  // Subscriber to pose in odom.
  ros::Subscriber poseInOdomSubscriber_;
  void poseInOdomCallback(const PoseRosPtr& msg);

  // Tf Transform broadcasting callback.
  bool broadcast(const any_worker::WorkerEvent& event);
  tf2_ros::TransformBroadcaster tfBroadcaster_;

  // Collection of frame connectors.
  std::vector<std::unique_ptr<FrameConnectorRosBase>> frameConnectors_;

  // helper functions for the frame connectors

  /**
   * @brief Initialize frame connectors.
   *
   */
  void initFrameConnectors();

  /**
   * @brief Broadcast the transforms from the frame connectors to the ROS tf server.
   *
   */
  void broadcastFrameConnectors();

  /**
   * @brief Update the pose in odom of the frame connectors.
   *
   * @param poseInOdom  Latest pose in odom.
   */
  void updateFrameConnectorsPoseInOdom(const PoseRosPtr& poseInOdom);
};

}  // namespace localization_manager_ros
