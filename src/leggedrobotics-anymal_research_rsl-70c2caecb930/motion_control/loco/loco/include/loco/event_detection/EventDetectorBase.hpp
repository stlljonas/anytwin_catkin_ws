/*!
 * @file     EventDetectorBase.hpp
 * @author   C. Dario Bellicoso, Christian Gehring
 * @date     Sep 5, 2014
 * @brief
 */

#pragma once

// loco
#include "loco/common/ModuleBase.hpp"
#include "loco/common/legs/Legs.hpp"

namespace loco {

class EventDetectorBase : public ModuleBase {
 public:
  EventDetectorBase() = default;
  ~EventDetectorBase() override = default;
};

} /* namespace loco */
