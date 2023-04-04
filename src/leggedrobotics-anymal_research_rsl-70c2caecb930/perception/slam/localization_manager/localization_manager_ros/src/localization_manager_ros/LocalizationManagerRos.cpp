// std utils
#include <std_utils/std_utils.hpp>

// localization manager
#include "localization_manager_ros/LocalizationManagerRos.hpp"

// param io
#include <param_io/get_param.hpp>

// logging
#include <message_logger/message_logger.hpp>

namespace localization_manager_ros {

LocalizationManagerRos::LocalizationManagerRos(any_node::Node::NodeHandlePtr nh) : any_node::Node(nh) {
  initFrameConnectors();

  // add a subscriber for the pose in odom
  poseInOdomSubscriber_ = getNodeHandle().subscribe(param_io::param<std::string>(getNodeHandle(), "pose_in_odom_topic", "default"), 1u,
                                                    &LocalizationManagerRos::poseInOdomCallback, this);

  // add aworker for broadcasting
  any_worker::WorkerOptions workerOptions;
  workerOptions.callback_ = std::bind(&LocalizationManagerRos::broadcast, this, std::placeholders::_1);
  workerOptions.defaultPriority_ = 59;
  workerOptions.name_ = ros::this_node::getName() + "/broadcastWorker";
  workerOptions.timeStep_ = param_io::param<double>(getNodeHandle(), "broadcasting_time_step", 0.01);
  addWorker(workerOptions);

  // broadcast initial tfs for all frame connectors to allow modules that rely on those tfs to bootstrap
  broadcastFrameConnectors();
}

void LocalizationManagerRos::poseInOdomCallback(const PoseRosPtr& msg) {
  // do some sanity checks on the measurement
  if (msg->header.stamp.toSec() <= 0.) {
    return;
  }
  updateFrameConnectorsPoseInOdom(msg);
}

bool LocalizationManagerRos::broadcast(const any_worker::WorkerEvent& /*event*/) {
  broadcastFrameConnectors();
  return true;
}

void LocalizationManagerRos::initFrameConnectors() {
  frameConnectors_.clear();
  if (param_io::param<bool>(getNodeHandle(), "frame_connectors/feedthrough/enabled", false)) {
    const std::vector<std::string> feedthroughFrameConnectorList =
        param_io::param<std::vector<std::string>>(getNodeHandle(), "frame_connectors/feedthrough/list", std::vector<std::string>());
    for (const auto& connectorName : feedthroughFrameConnectorList) {
      MELO_INFO_STREAM("Setting up feedthrough frame connector with name \"" << connectorName << "\".");
      frameConnectors_.emplace_back(std_utils::make_unique<FeedthroughFrameConnectorRos>(
          getNodeHandle(), std::string{"frame_connectors/feedthrough"}, connectorName));
    }
  }
  if (param_io::param<bool>(getNodeHandle(), "frame_connectors/difference/enabled", false)) {
    const std::vector<std::string> differenceFrameConnectorList =
        param_io::param<std::vector<std::string>>(getNodeHandle(), "frame_connectors/difference/list", std::vector<std::string>());
    for (const auto& connectorName : differenceFrameConnectorList) {
      MELO_INFO_STREAM("Setting up difference frame connector with name \"" << connectorName << "\".");
      frameConnectors_.emplace_back(
          std_utils::make_unique<DifferenceFrameConnectorRos>(getNodeHandle(), std::string{"frame_connectors/difference"}, connectorName));
    }
  }
}

void LocalizationManagerRos::broadcastFrameConnectors() {
  for (const auto& connector : frameConnectors_) {
    if (connector->hasTfExtInOdom() || connector->hasToBroadcastZeroInitialTf()) {
      geometry_msgs::TransformStamped transform = connector->getTfExtInOdom();
      transform.header.stamp = ros::Time::now();
      tfBroadcaster_.sendTransform(transform);
    }
  }
}

void LocalizationManagerRos::updateFrameConnectorsPoseInOdom(const PoseRosPtr& poseInOdom) {
  for (const auto& connector : frameConnectors_) {
    connector->updatePoseInOdom(poseInOdom);
  }
}

}  // namespace localization_manager_ros
