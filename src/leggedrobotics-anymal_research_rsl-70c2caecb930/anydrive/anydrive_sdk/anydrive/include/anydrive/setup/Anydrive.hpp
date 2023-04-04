#pragma once

#include <memory>
#include <string>

#include <yaml_tools/YamlNode.hpp>

#include "anydrive/configuration/Configuration.hpp"

namespace anydrive {
namespace setup {

/*!
 * Class containing an ANYdrive setup.
 * Used when loading a setup.
 */
class Anydrive {
 public:
  /*!
   * Unique name of the ANYdrive.
   * Default: "anydrive"
   */
  std::string name_ = "anydrive";

  /*!
   * Configuration of the device.
   * Default: Set in the empty constructor.
   */
  configuration::Configuration configuration_;

 public:
  /*!
   * Constructor, setting all members to default.
   */
  Anydrive() = default;

  /*!
   * Destructor.
   */
  virtual ~Anydrive() = default;

  /*!
   * Load the setup from a file.
   * Can throw an anydrive::Exception.
   * Called by Setup::fromFile(..).
   * @param yamlNode  YAML node containing setup data.
   * @param setupFile Path to the setup file.
   */
  virtual void fromFile(const yaml_tools::YamlNode& yamlNode, const std::string& setupFile);
};

using AnydrivePtr = std::shared_ptr<Anydrive>;

}  // namespace setup
}  // namespace anydrive
