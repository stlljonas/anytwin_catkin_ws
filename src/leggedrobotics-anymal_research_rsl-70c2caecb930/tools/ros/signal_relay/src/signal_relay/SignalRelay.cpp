#include "signal_relay/SignalRelay.hpp"

// message logger
#include <message_logger/message_logger.hpp>

namespace signal_relay {

SignalRelay::SignalRelay(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle) {
  // TODO(ynava) Add signal handler.

#ifndef NDEBUG
  MELO_WARN_STREAM("Node is built in Debug mode. For improved performance, change to Release mode.");
#endif
  readParameters();
}

bool SignalRelay::readParameters() {
  // Get the root parameter of the setup tree
  XmlRpc::XmlRpcValue routeList;
  if (!nodeHandle_.getParam("reset_routes", routeList)) {
    MELO_ERROR_STREAM("Parameters for reset routes not loaded");
    return false;
  }

  bool success = true;

  // Iterate over all provided routes
  for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator routePair = routeList.begin(); routePair != routeList.end(); ++routePair) {
    // The name for each route is only to separate routes, so ignore the value of route->first
    XmlRpc::XmlRpcValue route = routePair->second;

    // Check and show errors in the route configuration
    if (route["input"].getType() != XmlRpc::XmlRpcValue::TypeString) {
      MELO_WARN_STREAM("Route '" << routePair->first << " input topic invalid or does not exist");
      success = false;
      continue;
    }
    const bool outputTopicsExists = route["output_topics"].getType() == XmlRpc::XmlRpcValue::TypeArray;
    const bool outputServicesExists = route["output_services"].getType() == XmlRpc::XmlRpcValue::TypeArray;
    if (!outputTopicsExists && !outputServicesExists) {
      MELO_WARN_STREAM("Route '" << routePair->first << " output topics/services invalid or do not exist");
      success = false;
      continue;
    }

    std::string input_topic = static_cast<std::string>(route["input"]);
    MELO_DEBUG_STREAM("Route Reset topic from: " << input_topic << " to:");

    // Setup the output topics
    if (outputTopicsExists) {
      // Setup the publishers for all the output topics
      XmlRpc::XmlRpcValue output_topics_list = route["output_topics"];
      for (int i = 0; i < output_topics_list.size(); i++) {  // NOLINT
        std::string output_topic = static_cast<std::string>(output_topics_list[i]);
        MELO_DEBUG_STREAM("    Topic: " << output_topic);

        // Create a publisher if one does not already exist
        if (publishers_.find(output_topic) == publishers_.end()) {
          publishers_[output_topic] = nodeHandle_.advertise<std_msgs::Empty>(output_topic, kQueueSize_);
        }

        // Add the publisher to the list for this input topic
        topicRoutes_[input_topic].emplace_back(output_topic);
      }
    }
    // Setup the output services
    if (outputServicesExists) {
      // Setup the publishers for all the output topics
      XmlRpc::XmlRpcValue output_services_list = route["output_services"];
      for (int i = 0; i < output_services_list.size(); i++) {  // NOLINT
        std::string output_service = static_cast<std::string>(output_services_list[i]);
        MELO_DEBUG_STREAM("    Service: " << output_service);

        // Create a publisher if one does not already exist
        if (serviceClients_.find(output_service) == serviceClients_.end()) {
          serviceClients_[output_service] = nodeHandle_.serviceClient<std_srvs::Empty>(output_service);
        }

        // Add the publisher to the list for this input topic
        serviceRoutes_[input_topic].emplace_back(output_service);
      }
    }

    // Setup the subscriber for the input topic
    ros::Subscriber sub =
        nodeHandle_.subscribe<std_msgs::Empty>(input_topic, kQueueSize_, boost::bind(&SignalRelay::resetMessageCb, this, _1, input_topic));
    subscribers_.emplace_back(sub);
  }

  // TODO(ynava) Detect loops
  // TODO(ynava) Detect publishers and subscribers on the same topic (e.g. need to have separate reset in and out topics)

  return success;
}

void SignalRelay::resetMessageCb(const std_msgs::Empty::ConstPtr& msg, std::string topic) {
  MELO_INFO_STREAM("Received Reset notification from topic: " << topic);
  for (auto publisherKey : topicRoutes_[topic]) {
    publishers_[publisherKey].publish(*msg);

    MELO_INFO_STREAM("Relayed Reset notification to topic: " << publishers_[publisherKey].getTopic());
  }

  for (auto serviceClientKey : serviceRoutes_[topic]) {
    // Skip calling the service if it is not available
    if (!serviceClients_[serviceClientKey].exists()) {
      MELO_DEBUG_STREAM("Service '" << serviceClientKey << "' does not exist. Skipping service call");
      continue;
    }

    std_srvs::EmptyRequest req;
    std_srvs::EmptyResponse res;
    serviceClients_[serviceClientKey].call(req, res);

    MELO_INFO_STREAM("Relayed Reset via service call: " << serviceClients_[serviceClientKey].getService());
  }
}

}  // namespace signal_relay
