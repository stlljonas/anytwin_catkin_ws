#pragma once

#include <memory>
#include <vector>

#include <yaml_tools/YamlNode.hpp>

#include "anydrive/configuration/Configuration.hpp"
#include "anydrive/setup/Anydrive.hpp"
#include "anydrive/setup/JointPositionConfigurationManager.hpp"

namespace anydrive {
namespace setup {

/*!
 * Class containing a setup.
 * Contains equivalent data to the setup file.
 * Can be filled manually or by loading the data from a setup file.
 */
class Setup {
 public:
  using SetupPtr = std::shared_ptr<Setup>;

  /*!
   * A list of your ANYdrives.
   * Default: Single ANYdrive with default setup.
   */
  std::vector<AnydrivePtr> anydrives_;

  /*!
   * Setup of the Joint Position Configuration Manager.
   * Default: Set in the empty constructor.
   */
  JointPositionConfigurationManager jointPositionConfigurationManager_;

 public:
  /*!
   * Constructor, setting all members to default.
   */
  Setup();

  /*!
   * Destructor.
   */
  virtual ~Setup() = default;

  /*!
   * Load the setup from a file.
   * Can throw an anydrive::Exception.
   * Called by Setup::fromFile(..).
   * @param setupFile Path to the setup file.
   */
  void fromFile(const std::string& setupFile);

  /*!
   * Create a pointer containing a new ANYdrive setup.
   * @return Pointer to a new ANYdrive setup.
   */
  virtual AnydrivePtr createAnydrive() const;
};

using SetupPtr = std::shared_ptr<Setup>;

}  // namespace setup
}  // namespace anydrive
