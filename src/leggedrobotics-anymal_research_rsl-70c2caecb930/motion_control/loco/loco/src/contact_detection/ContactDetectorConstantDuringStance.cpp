/*
 * ContactDetectorConstantDuringStance.cpp
 *
 *  Created on: May 26, 2014
 *      Author: Christian Gehring, Dario Bellicoso
 */

#include "loco/contact_detection/ContactDetectorConstantDuringStance.hpp"

namespace loco {

ContactDetectorConstantDuringStance::ContactDetectorConstantDuringStance(Legs& legs)
    : ContactDetectorBase(), legs_(legs), registeredContact_(legs.size(), false) {}

bool ContactDetectorConstantDuringStance::initialize(double) {
  for (auto leg : legs_) {
    registeredContact_[leg->getId()] = leg->getContactSchedulePtr()->isGrounded();
  }

  return true;
}

bool ContactDetectorConstantDuringStance::advance(double) {
  for (auto leg : legs_) {
    unsigned int i = leg->getId();
    registeredContact_[i] = (leg->getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::Support) ||
                            (leg->getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::ContactInvariant);

    leg->getContactSchedulePtr()->setIsGrounded(registeredContact_[i] || leg->getContactSchedulePtr()->isGrounded());
  }
  return true;
}

bool ContactDetectorConstantDuringStance::loadParameters(const TiXmlHandle& handle) {
  return true;
}

} /* namespace loco */
