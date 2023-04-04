/*
 * ContactDetectorBase.hpp
 *
 *  Created on: May 26, 2014
 *      Author: Christian Gehring
 */

#pragma once

// loco
#include "loco/common/ModuleBase.hpp"

// Tiny XML
#include <tinyxml.h>

namespace loco {

class ContactDetectorBase : public ModuleBase {
 public:
  ContactDetectorBase();
  ~ContactDetectorBase() override = default;
};

} /* namespace loco */
