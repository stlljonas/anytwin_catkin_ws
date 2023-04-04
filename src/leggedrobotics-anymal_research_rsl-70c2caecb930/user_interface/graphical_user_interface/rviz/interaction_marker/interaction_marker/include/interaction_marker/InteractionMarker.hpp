#pragma once

// c++
#include <memory>
#include <vector>

// ros
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <std_srvs/Empty.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

// interaction marker
#include "interaction_marker/InteractionMarkerPluginBase.hpp"

namespace interaction_marker {

/*!
 * This class implements the interaction marker and loads all plugins.
 */
class InteractionMarker {
 public:
  using Server = boost::shared_ptr<interactive_markers::InteractiveMarkerServer>;
  using MenuHandler = boost::shared_ptr<interactive_markers::MenuHandler>;
  using ClassLoader = pluginlib::ClassLoader<InteractionMarkerPluginBase>;
  using Module = boost::shared_ptr<InteractionMarkerPluginBase>;
  using Modules = std::vector<Module>;

 protected:
  ros::NodeHandle& nodeHandle_;
  tf::TransformListener tfListener_;
  ros::ServiceServer resetMarkerServer_;

  Server server_;
  MenuHandler menuHandler_;

  std::string name_;
  std::string globalFrameId_;
  std::string markerFrameId_;

  ClassLoader classLoader_;
  Modules modules_;

 public:
  explicit InteractionMarker(ros::NodeHandle& nodeHandle);
  virtual ~InteractionMarker();

  ros::NodeHandle& getNodeHandle();
  boost::shared_ptr<interactive_markers::MenuHandler>& getMenuHandler();

  tf::TransformListener& getTfListener();
  bool transformPose(geometry_msgs::PoseStamped& pose, const std::string& targetFrameId);

 protected:
  bool resetMarker(const ros::Duration& timeout = ros::Duration(), bool force = true);
  bool resetMarkerServiceCb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  void resetMarkerMarkerCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
};

}  // namespace interaction_marker
