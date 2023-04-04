/*
 * ContactDetectorConstantDuringStance.hpp
 *
 *  Created on: May 26, 2014
 *      Author: Christian Gehring, Dario Bellicoso
 */

#pragma once

// loco
#include "loco/common/legs/Legs.hpp"
#include "loco/contact_detection/ContactDetectorBase.hpp"

// stl
#include <vector>

namespace loco {

class ContactDetectorConstantDuringStance : public ContactDetectorBase {
 public:
  explicit ContactDetectorConstantDuringStance(Legs& legs);
  ~ContactDetectorConstantDuringStance() override = default;
  bool initialize(double dt) override;
  bool advance(double dt) override;
  bool loadParameters(const TiXmlHandle& handle) override;

 protected:
  Legs& legs_;
  std::vector<bool> registeredContact_;
};

} /* namespace loco */
