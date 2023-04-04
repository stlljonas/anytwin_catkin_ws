#pragma once


// interaction marker
#include <interaction_marker/InteractionMarkerPluginBase.hpp>


namespace interaction_marker_plugin_example {


class InteractionMarkerPluginExample : public interaction_marker::InteractionMarkerPluginBase
{
protected:
  ros::Publisher chatterPublisher_;

  std::string examplePerson_ = "Sam";

public:
  InteractionMarkerPluginExample();
  virtual ~InteractionMarkerPluginExample();

  void initializePlugin(XmlRpc::XmlRpcValue parameters);

protected:
  void chatter(const std::string& message);

  void publishPosition(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void sayHello(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void sayGoodbye(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
};


} // interaction_marker_plugin_example

