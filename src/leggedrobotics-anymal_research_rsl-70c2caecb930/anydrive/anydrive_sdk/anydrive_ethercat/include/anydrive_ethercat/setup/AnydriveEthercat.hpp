#pragma once

#include <cstdint>

#include <anydrive/common/Macros.hpp>
#include <anydrive/setup/Anydrive.hpp>

#include "anydrive_ethercat/PdoTypeEnum.hpp"

namespace anydrive_ethercat {
namespace setup {

/*!
 * Class containing an ANYdrive EtherCAT setup.
 * Used when loading a setup.
 */
class AnydriveEthercat : public anydrive::setup::Anydrive {
 public:
  using Base = anydrive::setup::Anydrive;

  /*!
   * The EtherCAT bus where the ANYdrive is connected to.
   * Default: "eth0"
   */
  std::string ethercatBus_ = "eth0";

  /*!
   * The EtherCAT slave address of the ANYdrive.
   * The auto-increment method is used, so the slaves are
   * enumerated starting from 1, every time the bus is started.
   * Default: 1
   */
  uint32_t ethercatAddress_ = 1;

  /*!
   * The PDO type the slave should use.
   * Default: A
   */
  PdoTypeEnum ethercatPdoTypeEnum_ = PdoTypeEnum::A;

 public:
  /*!
   * Constructor, setting all members to default.
   */
  AnydriveEthercat() = default;

  /*!
   * Destructor.
   */
  ~AnydriveEthercat() override = default;

  /*!
   * Load the setup from a file.
   * Can throw an anydrive::Exception.
   * Called by Setup::fromFile(..).
   * @param yamlNode  YAML node containing setup data.
   * @param setupFile Path to the setup file.
   */
  void fromFile(const yaml_tools::YamlNode& yamlNode, const std::string& setupFile) override;
};

using AnydriveEthercatPtr = std::shared_ptr<AnydriveEthercat>;

}  // namespace setup
}  // namespace anydrive_ethercat
