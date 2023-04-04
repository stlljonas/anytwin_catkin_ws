// ros
#include <std_msgs/String.h>

// interaction marker
#include "interaction_marker/InteractionMarker.hpp"

// interaction marker example
#include "interaction_marker_plugin_example/InteractionMarkerPluginExample.hpp"


namespace interaction_marker_plugin_example {


InteractionMarkerPluginExample::InteractionMarkerPluginExample()
{

}

InteractionMarkerPluginExample::~InteractionMarkerPluginExample()
{

}

void InteractionMarkerPluginExample::initializePlugin(XmlRpc::XmlRpcValue parameters)
{
  // Get parameters.
  examplePerson_ = static_cast<std::string>(getParameter(parameters, "example_person"));

  // Publishers.
  auto chatterPublisherParameters = getParameter(getParameter(parameters, "publishers"), "chatter");
  chatterPublisher_ = interactionMarker_->getNodeHandle().advertise<std_msgs::String>(
      static_cast<std::string>(getParameter(chatterPublisherParameters, "topic")),
      static_cast<int>(getParameter(chatterPublisherParameters, "queue_size")),
      static_cast<bool>(getParameter(chatterPublisherParameters, "latch")));

  // Add module menu entries.
  interactionMarker_->getMenuHandler()->insert("Publish position", boost::bind(&InteractionMarkerPluginExample::publishPosition, this, _1));
  interactive_markers::MenuHandler::EntryHandle advancedEntry = interactionMarker_->getMenuHandler()->insert("Talk ...");
  interactionMarker_->getMenuHandler()->insert(advancedEntry, "Say hello", boost::bind(&InteractionMarkerPluginExample::sayHello, this, _1));
  interactionMarker_->getMenuHandler()->insert(advancedEntry, "Say goodbye", boost::bind(&InteractionMarkerPluginExample::sayGoodbye, this, _1));
}

void InteractionMarkerPluginExample::chatter(const std::string& message)
{
  // Publish and print the message.
  std_msgs::String stringMsg;
  stringMsg.data = message;
  chatterPublisher_.publish(stringMsg);
  ROS_INFO_STREAM(message);
}

void InteractionMarkerPluginExample::publishPosition(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  std::stringstream message;
  message << "I am at [" << feedback->pose.position.x << ", " << feedback->pose.position.y << ", " << feedback->pose.position.z << "].";
  chatter(message.str());
}

void InteractionMarkerPluginExample::sayHello(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  std::stringstream message;
  message << "Hello " << examplePerson_ << ".";
  chatter(message.str());
}

void InteractionMarkerPluginExample::sayGoodbye(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  std::stringstream message;
  message << "Goodbye " << examplePerson_ << ".";
  chatter(message.str());
}


} // interaction_marker_plugin_example


PLUGINLIB_EXPORT_CLASS(interaction_marker_plugin_example::InteractionMarkerPluginExample, interaction_marker::InteractionMarkerPluginBase)

