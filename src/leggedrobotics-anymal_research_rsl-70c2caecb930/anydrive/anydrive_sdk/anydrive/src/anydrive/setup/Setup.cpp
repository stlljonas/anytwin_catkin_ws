#include "anydrive/setup/Setup.hpp"

#include <memory>

namespace anydrive {
namespace setup {

Setup::Setup() {
  // Create the default setup.
  anydrives_.push_back(this->createAnydrive());
}

void Setup::fromFile(const std::string& setupFile) {
  // Load the setup from the file.
  yaml_tools::YamlNode yamlNode = yaml_tools::YamlNode::fromFile(setupFile);

  if (yamlNode.hasKey("anydrives")) {
    // Clear the anydrives first.
    anydrives_.clear();

    const yaml_tools::YamlNode anydrives = yamlNode["anydrives"];
    for (size_t i = 0; i < anydrives.size(); i++) {  // NOLINT
      AnydrivePtr anydrive = this->createAnydrive();
      anydrive->fromFile(anydrives[i], setupFile);
      anydrives_.push_back(anydrive);
    }
  }

  if (yamlNode.hasKey("joint_position_configuration_manager")) {
    jointPositionConfigurationManager_.fromFile(yamlNode["joint_position_configuration_manager"]);
  }
}

AnydrivePtr Setup::createAnydrive() const {
  return std::make_shared<Anydrive>();
}

}  // namespace setup
}  // namespace anydrive
