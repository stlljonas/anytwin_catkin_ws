#pragma once

#include <anydrive/setup/Setup.hpp>

#include "anydrive_ethercat/setup/AnydriveEthercat.hpp"

namespace anydrive_ethercat {
namespace setup {

/*!
 * Class containing an EtherCAT setup.
 * Contains equivalent data to the setup file.
 * Can be filled manually or by loading the data from a setup file.
 */
class SetupEthercat : public anydrive::setup::Setup {
 public:
  /*!
   * Constructor, setting all members to default.
   */
  SetupEthercat();

  /*!
   * Destructor.
   */
  ~SetupEthercat() override = default;

  /*!
   * Create a pointer containing a new ANYdrive setup.
   * @return Pointer to a new ANYdrive setup.
   */
  anydrive::setup::AnydrivePtr createAnydrive() const override;
};

using SetupEthercatPtr = std::shared_ptr<SetupEthercat>;

}  // namespace setup
}  // namespace anydrive_ethercat
