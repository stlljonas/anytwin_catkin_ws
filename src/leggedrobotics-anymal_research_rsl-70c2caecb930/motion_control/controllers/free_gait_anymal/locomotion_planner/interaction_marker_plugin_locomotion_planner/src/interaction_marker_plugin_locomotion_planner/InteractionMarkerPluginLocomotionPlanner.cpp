// interaction_marker_plugin_locomotion_planner
#include "interaction_marker_plugin_locomotion_planner/InteractionMarkerPluginLocomotionPlanner.hpp"

// message logger
#include <message_logger/message_logger.hpp>

namespace interaction_marker_plugin_locomotion_planner {

InteractionMarkerPluginLocomotionPlanner::InteractionMarkerPluginLocomotionPlanner() = default;

void InteractionMarkerPluginLocomotionPlanner::initializePlugin(XmlRpc::XmlRpcValue parameters) {
  // Get parameters.
  poseFrameId_ = static_cast<std::string>(getParameter(parameters, "pose_frame_id"));
  const auto entryText = static_cast<std::string>(getParameter(parameters, "entry_text"));
  const auto serverName = static_cast<std::string>(getParameter(parameters, "server_name"));

  // Create action client
  actionClientPtr_ =
      std::make_shared<actionlib::SimpleActionClient<locomotion_planner_msgs::NavigateToGoalPoseAction>>(serverName, spinOnStart_);

  // Add module menu entry.
  interactionMarker_->getMenuHandler()->insert(entryText, boost::bind(&InteractionMarkerPluginLocomotionPlanner::publishPose, this, _1));
}

void InteractionMarkerPluginLocomotionPlanner::publishPose(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {
  if (!actionClientPtr_) {
    MELO_ERROR("Action Client Pointer is empty.")
    return;
  }

  if (!actionClientPtr_->isServerConnected()) {
    MELO_WARN("Action Server is not connected yet. Make sure to select [walk_to_pose] motion state.")
    return;
  }

  locomotion_planner_msgs::NavigateToGoalPoseGoal goalPose;
  goalPose.goal_pose.header.stamp = ros::Time::now();
  goalPose.goal_pose.header.frame_id = feedback->header.frame_id;
  goalPose.goal_pose.pose = feedback->pose;
  if (!interactionMarker_->transformPose(goalPose.goal_pose, poseFrameId_)) {
    return;
  }

  actionClientPtr_->sendGoal(goalPose);
}

}  // namespace interaction_marker_plugin_locomotion_planner

PLUGINLIB_EXPORT_CLASS(interaction_marker_plugin_locomotion_planner::InteractionMarkerPluginLocomotionPlanner,
                       interaction_marker::InteractionMarkerPluginBase)
