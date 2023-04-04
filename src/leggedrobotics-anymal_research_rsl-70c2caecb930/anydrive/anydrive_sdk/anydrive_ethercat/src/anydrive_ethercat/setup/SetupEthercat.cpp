#include "anydrive_ethercat/setup/SetupEthercat.hpp"

namespace anydrive_ethercat {
namespace setup {

SetupEthercat::SetupEthercat() {
  // Create the default setup again.
  // The reason for this is that Setup() is called before SetupEthercat(),
  // but cannot resolve the virtual method createAnydrive() yet.
  anydrives_.clear();
  anydrives_.push_back(this->createAnydrive());
}

anydrive::setup::AnydrivePtr SetupEthercat::createAnydrive() const {
  return anydrive::setup::AnydrivePtr(new AnydriveEthercat());
}

}  // namespace setup
}  // namespace anydrive_ethercat
