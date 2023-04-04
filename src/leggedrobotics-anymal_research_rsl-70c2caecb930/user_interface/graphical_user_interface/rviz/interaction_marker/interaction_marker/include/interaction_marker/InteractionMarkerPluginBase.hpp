#pragma once

// ros
#include <ros/ros.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

// pluginlib
#include <pluginlib/class_list_macros.h>
#include <pluginlib/class_loader.h>

// param io
#include <param_io/get_param.hpp>

// xml rpc
#include <XmlRpc.h>

namespace interaction_marker {

// Forward declaration of the InteractionMarker.
class InteractionMarker;

/*!
 * This class is the base class of all interaction marker plugins.
 */
class InteractionMarkerPluginBase {
 protected:
  InteractionMarker* interactionMarker_ = nullptr;

  /*!
   * Access a parameter in a set of parameters.
   * Throws and XmlRpcException in case parameter does not exist.
   * @param parameters Set of parameters.
   * @param key Key of the parameter to access.
   * @return Parameter.
   */
  XmlRpc::XmlRpcValue getParameter(XmlRpc::XmlRpcValue parameters, const std::string& key);

 public:
  InteractionMarkerPluginBase();
  virtual ~InteractionMarkerPluginBase();

  /*!
   * This function will be called after the constructor.
   * @param interactionMarker  pointer to the interaction marker this plugin belongs to.
   */
  void initializeBase(InteractionMarker* interactionMarker);

  /*!
   * This function will be called after initializeBase(..).
   * @param parameters         Plugin specific parameters.
   */
  virtual void initializePlugin(XmlRpc::XmlRpcValue parameters) = 0;
};

}  // namespace interaction_marker
