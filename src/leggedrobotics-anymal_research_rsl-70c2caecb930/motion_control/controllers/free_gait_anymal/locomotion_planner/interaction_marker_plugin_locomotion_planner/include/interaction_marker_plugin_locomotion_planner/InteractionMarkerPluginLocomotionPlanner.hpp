#pragma once

// actionlib
#include <actionlib/client/simple_action_client.h>

// locomotion_planner
#include <locomotion_planner_msgs/NavigateToGoalPoseAction.h>

// interaction marker
#include <interaction_marker/InteractionMarker.hpp>
#include <interaction_marker/InteractionMarkerPluginBase.hpp>

namespace interaction_marker_plugin_locomotion_planner {

using ActionClientPtr = std::shared_ptr<actionlib::SimpleActionClient<locomotion_planner_msgs::NavigateToGoalPoseAction>>;

class InteractionMarkerPluginLocomotionPlanner : public interaction_marker::InteractionMarkerPluginBase {
 protected:
  ActionClientPtr actionClientPtr_;

  std::string poseFrameId_;

  const bool spinOnStart_ = true;

 public:
  InteractionMarkerPluginLocomotionPlanner();
  ~InteractionMarkerPluginLocomotionPlanner() override = default;

  void initializePlugin(XmlRpc::XmlRpcValue parameters) override;

 protected:
  void publishPose(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
};

}  // namespace interaction_marker_plugin_locomotion_planner
