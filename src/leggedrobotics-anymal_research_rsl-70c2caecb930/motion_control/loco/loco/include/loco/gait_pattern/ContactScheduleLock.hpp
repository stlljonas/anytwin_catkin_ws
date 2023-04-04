/*
 * ContactScheduleLock.hpp
 *
 *  Created on: April 27, 2018
 *      Author: Fabian Jenelten
 */

#pragma once

// loco
#include "loco/common/ModuleBase.hpp"

namespace loco {

class ContactScheduleLock : public ModuleBase {
 public:
  ContactScheduleLock() : ModuleBase(), isLocked_(true) {}

  ~ContactScheduleLock() override = default;

  //! True if the gait is locked.
  bool isLocked() const noexcept { return isLocked_; }

  //! Lock/Unlock the gait.
  void lock(bool isLocked = true) noexcept { isLocked_ = isLocked; }

 protected:
  //! True if the gait is locked.
  bool isLocked_;
};

}  // namespace loco
