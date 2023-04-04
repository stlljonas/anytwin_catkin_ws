#include <algorithm>
#include <string>

#include <param_io/get_param.hpp>

#include <anydrive/Exception.hpp>
#include <anydrive/common/sorting.hpp>
#include <anydrive_ethercat/setup/AnydriveEthercat.hpp>
#include <anydrive_ethercat/setup/SetupEthercat.hpp>
#include <anydrive_ros/conversions.hpp>

#include "anydrive_ethercat_ros/conversions.hpp"

namespace anydrive_ethercat_ros {

bool readCommunicationParameters(XmlRpc::XmlRpcValue& params, anydrive::setup::AnydrivePtr& anydrivePtr) {
  auto anydriveEthercatPtr = std::dynamic_pointer_cast<anydrive_ethercat::setup::AnydriveEthercat>(anydrivePtr);
  if (anydriveEthercatPtr == nullptr) {
    return false;
  }

  if (params.hasMember("bus")) {
    anydriveEthercatPtr->ethercatBus_ = param_io::getMember<std::string>(params, "bus");
  } else {
    throw anydrive::Exception("The EtherCAT bus is not defined.");
  }
  if (params.hasMember("address")) {
    const auto ethercatAddress = param_io::getMember<int>(params, "address");
    if (ethercatAddress == 0) {
      throw anydrive::Exception("The EtherCAT address is 0.");
    }
    anydriveEthercatPtr->ethercatAddress_ = ethercatAddress;
  }
  if (params.hasMember("pdo_type")) {
    const std::string pdoType = param_io::getMember<std::string>(params, "pdo_type");
    if (pdoType == "A") {
      anydriveEthercatPtr->ethercatPdoTypeEnum_ = anydrive_ethercat::PdoTypeEnum::A;
    } else if (pdoType == "B") {
      anydriveEthercatPtr->ethercatPdoTypeEnum_ = anydrive_ethercat::PdoTypeEnum::B;
    } else if (pdoType == "C") {
      anydriveEthercatPtr->ethercatPdoTypeEnum_ = anydrive_ethercat::PdoTypeEnum::C;
    } else if (pdoType == "D") {
      anydriveEthercatPtr->ethercatPdoTypeEnum_ = anydrive_ethercat::PdoTypeEnum::D;
    } else {
      throw anydrive::Exception("EtherCAT PDO Type '" + pdoType + "' could not be parsed.");
    }
  }
  return true;
}

bool readSetupParameters(XmlRpc::XmlRpcValue& params, anydrive::setup::SetupPtr& setupPtr) {
  // Cast to Ethercat setup pointer and return false if this is not possible.
  auto setupEthercatPtr = std::dynamic_pointer_cast<anydrive_ethercat::setup::SetupEthercat>(setupPtr);
  if (setupEthercatPtr == nullptr) {
    return false;
  }

  // Read max joint velocity for joint configuration manager.
  if (params.hasMember("joint_position_configuration_manager")) {
    XmlRpc::XmlRpcValue p = param_io::getMember<XmlRpc::XmlRpcValue>(params, "joint_position_configuration_manager");
    if (p.hasMember("max_joint_velocity")) {
      setupPtr->jointPositionConfigurationManager_.maxJointVelocity_ = param_io::getMember<double>(p, "max_joint_velocity");
    }
  }

  // Get ANYdrives container.
  if (!params.hasMember("anydrives")) {
    ANYDRIVE_ERROR("Container 'anydrives' does not exist.")
    return false;
  }
  XmlRpc::XmlRpcValue anydriveParams = param_io::getMember<XmlRpc::XmlRpcValue>(params, "anydrives");

  // Go through all ANYdrives.
  std::vector<int> order;
  setupPtr->anydrives_.clear();
  for (auto& param : anydriveParams) {
    auto anydrive = setupEthercatPtr->createAnydrive();
    ANYDRIVE_DEBUG("Read communication and configuration parameters for: " << param.first)
    anydrive->name_ = param.first;

    // Read id.
    if (param.second.hasMember("id")) {
      order.push_back(param_io::getMember<int>(param.second, "id"));
    } else {
      ANYDRIVE_ERROR("Every ANYdrive needs an id.")
      return false;
    }

    // Read communication parameters.
    if (param.second.hasMember("communication")) {
      XmlRpc::XmlRpcValue communicationParams = param_io::getMember<XmlRpc::XmlRpcValue>(param.second, "communication");
      readCommunicationParameters(communicationParams, anydrive);
    }

    // Read configurations.
    if (param.second.hasMember("configuration")) {
      XmlRpc::XmlRpcValue configurationParams = param_io::getMember<XmlRpc::XmlRpcValue>(param.second, "configuration");
      anydrive_ros::readAnydriveSetupParameters(configurationParams, anydrive);
    }

    setupPtr->anydrives_.push_back(anydrive);
  }

  // Order the ANYdrives according to the ids.
  auto permutation = anydrive::common::sortPermutation(order, [](int const& a, int const& b) { return a < b; });
  order = anydrive::common::applyPermutation(order, permutation);
  auto adjacent = std::adjacent_find(order.begin(), order.end());
  if (adjacent != order.end()) {
    ANYDRIVE_ERROR("Duplicated ids (" << *adjacent << ") are not allowed.")
    return false;
  }
  setupPtr->anydrives_ = anydrive::common::applyPermutation(setupPtr->anydrives_, permutation);

  return true;
}

}  // namespace anydrive_ethercat_ros
