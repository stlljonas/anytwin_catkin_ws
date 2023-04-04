#include "anydrive/setup/JointPositionConfigurationManager.hpp"
#include "anydrive/Exception.hpp"

namespace anydrive {
namespace setup {

void JointPositionConfigurationManager::fromFile(const yaml_tools::YamlNode& yamlNode) {
  if (yamlNode.hasKey("max_joint_velocity")) {
    const auto maxJointVelocity = yamlNode["max_joint_velocity"].as<double>();
    if (maxJointVelocity <= 0.0) {
      throw Exception("Max joint velocity is smaller or equal to zero.");
    }
    maxJointVelocity_ = maxJointVelocity;
  }
}

}  // namespace setup
}  // namespace anydrive
