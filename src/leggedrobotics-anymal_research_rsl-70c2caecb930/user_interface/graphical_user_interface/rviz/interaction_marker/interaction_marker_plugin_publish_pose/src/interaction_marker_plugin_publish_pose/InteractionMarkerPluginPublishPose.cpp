// interaction marker
#include "interaction_marker/InteractionMarker.hpp"

// interaction marker icp
#include "interaction_marker_plugin_publish_pose/InteractionMarkerPluginPublishPose.hpp"


namespace interaction_marker_plugin_publish_pose {


InteractionMarkerPluginPublishPose::InteractionMarkerPluginPublishPose()
{

}

InteractionMarkerPluginPublishPose::~InteractionMarkerPluginPublishPose()
{

}

void InteractionMarkerPluginPublishPose::initializePlugin(XmlRpc::XmlRpcValue parameters)
{
  // Get parameters.
  poseFrameId_ = static_cast<std::string>(getParameter(parameters, "pose_frame_id"));
  const auto entryText = static_cast<std::string>(getParameter(parameters, "entry_text"));

  // Publishers.
  auto posePublisherParameters = getParameter(getParameter(parameters, "publishers"), "pose");
  posePublisher_ = interactionMarker_->getNodeHandle().advertise<geometry_msgs::PoseStamped>(
      static_cast<std::string>(getParameter(posePublisherParameters, "topic")),
      static_cast<int>(getParameter(posePublisherParameters, "queue_size")),
      static_cast<bool>(getParameter(posePublisherParameters, "latch")));

  // Add module menu entry.
  interactionMarker_->getMenuHandler()->insert(entryText, boost::bind(&InteractionMarkerPluginPublishPose::publishPose, this, _1));
}

void InteractionMarkerPluginPublishPose::publishPose(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  if (posePublisher_.getNumSubscribers() == 0 &&
      !posePublisher_.isLatched())
    return;

  geometry_msgs::PoseStamped poseStamped;
  poseStamped.header.stamp = ros::Time::now();
  poseStamped.header.frame_id = feedback->header.frame_id;
  poseStamped.pose = feedback->pose;
  if (!interactionMarker_->transformPose(poseStamped, poseFrameId_))
    return;

  posePublisher_.publish(poseStamped);
}


} // interaction_marker_plugin_publish_pose


PLUGINLIB_EXPORT_CLASS(interaction_marker_plugin_publish_pose::InteractionMarkerPluginPublishPose, interaction_marker::InteractionMarkerPluginBase)

