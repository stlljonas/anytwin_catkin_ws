// interaction marker
#include "interaction_marker/InteractionMarkerPluginBase.hpp"
#include "interaction_marker/InteractionMarker.hpp"

namespace interaction_marker {

XmlRpc::XmlRpcValue InteractionMarkerPluginBase::getParameter(XmlRpc::XmlRpcValue parameters, const std::string& key) {
  if (!parameters.hasMember(key)) {
    throw XmlRpc::XmlRpcException(std::string("Failed to find parameter at key '") + key + std::string("'."));
  }
  return parameters[key];
}

InteractionMarkerPluginBase::InteractionMarkerPluginBase() = default;

InteractionMarkerPluginBase::~InteractionMarkerPluginBase() = default;

void InteractionMarkerPluginBase::initializeBase(InteractionMarker* interactionMarker) {
  interactionMarker_ = interactionMarker;
}

}  // namespace interaction_marker
