#include <anydrive_ethercat/setup/SetupEthercat.hpp>

#include "anydrive_ethercat/AnydriveEthercatSlave.hpp"
#include "anydrive_ethercat_ros/AnydriveManagerEthercatRos.hpp"

namespace anydrive_ethercat_ros {

AnydriveManagerEthercatRos::AnydriveManagerEthercatRos(const bool standalone, const bool installSignalHandler, const double timeStep,
                                                       ros::NodeHandle& nh, const std::string& rosPrefix)
    : anydrive_ros::AnydriveManagerRos(standalone, installSignalHandler, timeStep, nh, rosPrefix) {
  setCommunicationManager(anydrive::communication::CommunicationManagerBasePtr(new anydrive_ethercat::AnydriveEthercatBusManager()));
}

anydrive::setup::SetupPtr AnydriveManagerEthercatRos::createSetup() const {
  return anydrive::setup::SetupPtr(new anydrive_ethercat::setup::SetupEthercat());
}

bool AnydriveManagerEthercatRos::sendSdoReadCb(anydrive_msgs::SendSdoReadRequest& req, anydrive_msgs::SendSdoReadResponse& res) {
  // Send the SDO to one drive.
  anydrive::AnydrivePtr anydrive = getAnydrive(req.device_name);
  if (!anydrive) {
    return false;
  }

  return std::dynamic_pointer_cast<anydrive_ethercat::AnydriveEthercatSlave>(anydrive->getCommunicationInterface())
      ->sendSdoReadGeneric(req.index, req.subindex, req.value_type, res.value);
}

bool AnydriveManagerEthercatRos::sendSdoWriteCb(anydrive_msgs::SendSdoWriteRequest& req, anydrive_msgs::SendSdoWriteResponse& /*res*/) {
  // Send the SDO to one drive.
  anydrive::AnydrivePtr anydrive = getAnydrive(req.device_name);
  if (!anydrive) {
    return false;
  }

  return std::dynamic_pointer_cast<anydrive_ethercat::AnydriveEthercatSlave>(anydrive->getCommunicationInterface())
      ->sendSdoWriteGeneric(req.index, req.subindex, req.value_type, req.value);
}

}  // namespace anydrive_ethercat_ros
