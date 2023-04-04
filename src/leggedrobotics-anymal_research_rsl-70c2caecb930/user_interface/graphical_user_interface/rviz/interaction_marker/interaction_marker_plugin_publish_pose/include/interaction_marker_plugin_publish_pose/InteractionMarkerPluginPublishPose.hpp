#pragma once

// ros
#include <geometry_msgs/PoseStamped.h>

// interaction marker
#include <interaction_marker/InteractionMarkerPluginBase.hpp>


namespace interaction_marker_plugin_publish_pose {


class InteractionMarkerPluginPublishPose : public interaction_marker::InteractionMarkerPluginBase
{
protected:
  ros::Publisher posePublisher_;

  std::string poseFrameId_;

public:
  InteractionMarkerPluginPublishPose();
  virtual ~InteractionMarkerPluginPublishPose();

  void initializePlugin(XmlRpc::XmlRpcValue parameters);

protected:
  void publishPose(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
};


} // interaction_marker_plugin_publish_pose

