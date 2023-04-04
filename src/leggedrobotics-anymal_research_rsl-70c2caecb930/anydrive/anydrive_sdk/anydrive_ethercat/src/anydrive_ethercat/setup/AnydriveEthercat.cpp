#include <anydrive/Exception.hpp>

#include "anydrive_ethercat/setup/AnydriveEthercat.hpp"

namespace anydrive_ethercat {
namespace setup {

void AnydriveEthercat::fromFile(const yaml_tools::YamlNode& yamlNode, const std::string& setupFile) {
  Base::fromFile(yamlNode, setupFile);
  if (yamlNode.hasKey("ethercat_bus")) {
    ethercatBus_ = yamlNode["ethercat_bus"].as<std::string>();
  }
  if (yamlNode.hasKey("ethercat_address")) {
    const auto ethercatAddress = yamlNode["ethercat_address"].as<uint32_t>();
    if (ethercatAddress == 0) {
      throw anydrive::Exception("The EtherCAT address is 0.");
    }
    ethercatAddress_ = ethercatAddress;
  }
  if (yamlNode.hasKey("ethercat_pdo_type")) {
    const std::string pdoType = yamlNode["ethercat_pdo_type"].as<std::string>();
    if (pdoType == "A") {
      ethercatPdoTypeEnum_ = PdoTypeEnum::A;
    } else if (pdoType == "B") {
      ethercatPdoTypeEnum_ = PdoTypeEnum::B;
    } else if (pdoType == "C") {
      ethercatPdoTypeEnum_ = PdoTypeEnum::C;
    } else if (pdoType == "D") {
      ethercatPdoTypeEnum_ = PdoTypeEnum::D;
    } else {
      throw anydrive::Exception("EtherCAT PDO Type '" + pdoType + "' could not be parsed.");
    }
  }
}

}  // namespace setup
}  // namespace anydrive_ethercat
