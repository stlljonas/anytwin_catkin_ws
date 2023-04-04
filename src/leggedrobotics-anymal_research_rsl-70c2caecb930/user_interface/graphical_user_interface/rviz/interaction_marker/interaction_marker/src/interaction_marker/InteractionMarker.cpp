// interaction marker
#include "interaction_marker/InteractionMarker.hpp"

namespace interaction_marker {

InteractionMarker::InteractionMarker(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      tfListener_(nodeHandle_),
      classLoader_("interaction_marker", "interaction_marker::InteractionMarkerPluginBase") {
  // Instantiate parameters.
  std::string description;
  std::string meshResource;
  double initialResetTimeout = 0.0;
  XmlRpc::XmlRpcValue pluginSetup;

  // Get parameters.
  param_io::getParam(nodeHandle_, "name", name_);
  param_io::getParam(nodeHandle_, "description", description);
  param_io::getParam(nodeHandle_, "mesh_resource", meshResource);
  param_io::getParam(nodeHandle_, "global_frame_id", globalFrameId_);
  param_io::getParam(nodeHandle_, "marker_frame_id", markerFrameId_);
  param_io::getParam(nodeHandle_, "initial_reset_timeout", initialResetTimeout);
  param_io::getParam(nodeHandle_, "plugin_setup", pluginSetup);

  // Service servers.
  resetMarkerServer_ = nodeHandle_.advertiseService("reset", &InteractionMarker::resetMarkerServiceCb, this);

  // Create an interactive marker.
  visualization_msgs::InteractiveMarker interactiveMarker;
  interactiveMarker.header.frame_id = globalFrameId_;
  interactiveMarker.name = name_;
  interactiveMarker.description = description;

  // Create a marker for the mesh and the menu.
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.mesh_resource = meshResource;
  marker.mesh_use_embedded_materials = static_cast<unsigned char>(true);
  marker.scale.x = 0.001;
  marker.scale.y = 0.001;
  marker.scale.z = 0.001;
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;

  // Create a non-interactive control which contains the mesh.
  visualization_msgs::InteractiveMarkerControl controlMesh;
  controlMesh.always_visible = static_cast<unsigned char>(true);
  controlMesh.markers.push_back(marker);
  interactiveMarker.controls.push_back(controlMesh);

  // Create an interactive control which contains the menu.
  visualization_msgs::InteractiveMarkerControl controlMenu;
  controlMenu.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
  controlMenu.always_visible = static_cast<unsigned char>(true);
  controlMenu.markers.push_back(marker);
  interactiveMarker.controls.push_back(controlMenu);

  // Create an interactive marker server.
  server_.reset(new interactive_markers::InteractiveMarkerServer(name_));

  // Create an interactive marker control.
  visualization_msgs::InteractiveMarkerControl controlMove;
  controlMove.orientation.w = 0.5 * std::sqrt(2.0);
  controlMove.orientation.x = 0;
  controlMove.orientation.y = 0.5 * std::sqrt(2.0);
  controlMove.orientation.z = 0;
  // Move and rotate within the x-y-plane.
  controlMove.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE;
  interactiveMarker.controls.push_back(controlMove);
  // Move along the x-axis.
  controlMove.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  interactiveMarker.controls.push_back(controlMove);

  // Add the interactive marker to the server.
  server_->insert(interactiveMarker);

  // Create dropdown menu.
  menuHandler_.reset(new interactive_markers::MenuHandler());

  // Add default menu entry.
  menuHandler_->insert("Reset marker", boost::bind(&InteractionMarker::resetMarkerMarkerCb, this, _1));

  // Load all specified modules.
  const auto& plugins = pluginSetup["plugins"];
  for (const auto& plugin : plugins) {
    const std::string type = plugin.second["type"];
    try {
      // Create the module and initialize it.
      Module module = classLoader_.createInstance(type);
      module->initializeBase(this);
      module->initializePlugin(plugin.second["parameters"]);
      modules_.push_back(module);
    } catch (const pluginlib::PluginlibException& exception) {
      ROS_ERROR_STREAM("InteractionMarker: The plugin of type '" << type << "' failed to load. Pluginlib Exception: " << exception.what());
    } catch (const XmlRpc::XmlRpcException& exception) {
      ROS_ERROR_STREAM("InteractionMarker: The plugin of type '" << type
                                                                 << "' failed to load. XmlRpc Exception: " << exception.getMessage());
    }
  }

  // Add the menu to the marker.
  menuHandler_->apply(*server_, interactiveMarker.name);

  // Apply all changes and send the to the clients.
  server_->applyChanges();

  // Try to reset the marker if the transformation is available.
  resetMarker(ros::Duration(initialResetTimeout), true);
}

InteractionMarker::~InteractionMarker() = default;

ros::NodeHandle& InteractionMarker::getNodeHandle() {
  return nodeHandle_;
}

boost::shared_ptr<interactive_markers::MenuHandler>& InteractionMarker::getMenuHandler() {
  return menuHandler_;
}

tf::TransformListener& InteractionMarker::getTfListener() {
  return tfListener_;
}

bool InteractionMarker::transformPose(geometry_msgs::PoseStamped& pose, const std::string& targetFrameId) {
  if (pose.header.frame_id.empty()) {
    ROS_ERROR_STREAM("InteractionMarker: The pose frame id is empty.");
    return false;
  }
  if (targetFrameId.empty()) {
    ROS_ERROR_STREAM("InteractionMarker: The target frame id is empty.");
    return false;
  }

  if (pose.header.frame_id == targetFrameId) {
    return true;
  }

  try {
    const geometry_msgs::PoseStamped poseIn(pose);
    if (tfListener_.waitForTransform(targetFrameId, poseIn.header.frame_id, poseIn.header.stamp, ros::Duration(1.0))) {
      tfListener_.transformPose(targetFrameId, poseIn, pose);
      return true;
    } else {
      ROS_ERROR_STREAM("InteractionMarker: Wait for transform timeout from " << poseIn.header.frame_id << " to " << targetFrameId << " at "
                                                                             << poseIn.header.stamp << ".");
      return false;
    }
  } catch (tf::TransformException& exception) {
    ROS_ERROR_STREAM("InteractionMarker: Pose transform exception:" << exception.what());
    return false;
  }
}

bool InteractionMarker::resetMarker(const ros::Duration& timeout, bool force) {
  // Lookup the transformation.
  tf::StampedTransform stampedTransform;
  try {
    if (!tfListener_.waitForTransform(globalFrameId_, markerFrameId_, ros::Time(0), timeout)) {
      if (force) {
        ROS_ERROR_STREAM("InteractionMarker: No transformation from '" << markerFrameId_ << "' to '" << globalFrameId_
                                                                       << "' found (Timeout: " << timeout.toSec() << "s).");
      }
      return false;
    }
    tfListener_.lookupTransform(globalFrameId_, markerFrameId_, ros::Time(0), stampedTransform);
  } catch (tf::TransformException& exception) {
    ROS_ERROR_STREAM("InteractionMarker: Caught a transform exception:" << exception.what());
    return false;
  }

  // Convert the transformation.
  geometry_msgs::TransformStamped transformStampedMsg;
  tf::transformStampedTFToMsg(stampedTransform, transformStampedMsg);

  // Convert to a pose message ignoring roll and pitch.
  geometry_msgs::Pose pose4dMsg;
  pose4dMsg.position.x = transformStampedMsg.transform.translation.x;
  pose4dMsg.position.y = transformStampedMsg.transform.translation.y;
  pose4dMsg.position.z = transformStampedMsg.transform.translation.z;
  tf::Quaternion quaternion3d;
  tf::quaternionMsgToTF(transformStampedMsg.transform.rotation, quaternion3d);
  tf::Quaternion quaternion1d;
  quaternion1d.setRPY(0, 0, tf::getYaw(quaternion3d));
  geometry_msgs::Quaternion quaternion2dMsg;
  tf::quaternionTFToMsg(quaternion1d, quaternion2dMsg);
  pose4dMsg.orientation = quaternion2dMsg;

  // Set the pose of the marker.
  server_->setPose(name_, pose4dMsg);

  // Apply all changes and send the to the clients.
  server_->applyChanges();
  return true;
}

bool InteractionMarker::resetMarkerServiceCb(std_srvs::Empty::Request& /*req*/, std_srvs::Empty::Response& /*res*/) {
  return resetMarker();
}

void InteractionMarker::resetMarkerMarkerCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& /*feedback*/) {
  resetMarker();
}

}  // namespace interaction_marker
