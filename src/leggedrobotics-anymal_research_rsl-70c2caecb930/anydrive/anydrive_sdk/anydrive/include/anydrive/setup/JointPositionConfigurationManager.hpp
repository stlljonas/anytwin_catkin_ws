#pragma once

#include <string>
#include <vector>

#include <yaml_tools/YamlNode.hpp>

namespace anydrive {
namespace setup {

/*!
 * Class containing a joint position configuration manager setup.
 * Used when loading a setup.
 */
class JointPositionConfigurationManager {
 public:
  /*!
   * The maximal joint velocity used for the ANYdrive
   * with the biggest position error.
   * Unit    rad/s
   * Range   (0.0, .inf)
   * Default 1.0
   */
  double maxJointVelocity_ = 1.0;

 public:
  /*!
   * Constructor, setting all members to default.
   */
  JointPositionConfigurationManager() = default;

  /*!
   * Destructor.
   */
  ~JointPositionConfigurationManager() = default;

  /*!
   * Load the setup from a file.
   * Can throw an anydrive::Exception.
   * Called by Setup::fromFile(..).
   * @param yamlNode YAML node containing setup data.
   */
  void fromFile(const yaml_tools::YamlNode& yamlNode);
};

}  // namespace setup
}  // namespace anydrive
