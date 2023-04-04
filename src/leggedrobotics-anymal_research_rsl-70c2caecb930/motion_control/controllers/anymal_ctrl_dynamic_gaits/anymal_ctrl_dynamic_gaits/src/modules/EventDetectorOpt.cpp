/*
 * EventDetectorOpt.cpp
 *
 *  Created on: Nov 14, 2018
 *      Author: Fabian Jenelten
 */

// anymal_description
#include <anymal_description/LegEnum.hpp>

// loco.
#include "anymal_ctrl_dynamic_gaits/modules/EventDetectorOpt.hpp"

// tinyxml tools.
#include "tinyxml_tools/tinyxml_tools.hpp"


namespace loco {
using namespace message_logger::color;

EventDetectorOpt::EventDetectorOpt(WholeBody& wholeBody):
    EventDetectorBase(),
    legs_(*wholeBody.getLegsPtr()),
    timeSinceInit_(0.0),
    enableFrictionModulation_(true),
    feedbackModulationWhileSlipping_(0.5),
    frictionModulation_(1.0),
    wasLegSlipping_(false),
    timeSpendInStance_(0.0),
    timeSpendInStanceNoSlip_(0.0)
{

}

bool EventDetectorOpt::addVariablesToLog(const std::string & ns) const {
  for (const auto& legEnum : anymal_description::LegEnumIterator()) {
    signal_logger::add(frictionModulation_[legEnum], "/event_detector/" + anymal_description::mapLegEnumToString[legEnum] + "/friction_modulation", "/loco", "-");
  }
  return true;
}


bool EventDetectorOpt::initialize(double dt) {
  timeSinceInit_ = 0.0;

  for (const auto& legEnum : anymal_description::LegEnumIterator()) {
    frictionModulation_[legEnum] = 1.0;
    wasLegSlipping_[legEnum] = false;
    timeSpendInStance_[legEnum] = 0.0;
    timeSpendInStanceNoSlip_[legEnum] = 0.0;
  }


  return true;
}

bool EventDetectorOpt::loadParameters(const TiXmlHandle& handle) {
  MELO_DEBUG_STREAM(magenta << "[EventDetectorOpt] " << blue << "Load parameters." << def)

  TiXmlHandle eventDetectorHandle = handle;
  if(!tinyxml_tools::getChildHandle(eventDetectorHandle, handle, "EventDetector")) { return false; }

  TiXmlHandle modulationHandle = handle;
  if(!tinyxml_tools::getChildHandle(modulationHandle, eventDetectorHandle, "FrictionModulation")) { return false; }
  if(!tinyxml_tools::loadParameter(enableFrictionModulation_, modulationHandle, "enable")) { return false; }
  if(!tinyxml_tools::loadParameter(feedbackModulationWhileSlipping_, modulationHandle, "value_while_slipping")) { return false; }

  return true;
}


bool EventDetectorOpt::advance(double dt) {
  timeSinceInit_ += dt;

  for (auto leg : legs_) {
    // Lift-off detection.
    if (leg->getContactSchedule().wasGrounded() && !leg->getContactSchedule().isGrounded()) {
      leg->getStateLiftOff()->setStateChangedAtTime(timeSinceInit_);
    }

    // Touch-down detection.
    if (!leg->getContactSchedule().wasGrounded() && leg->getContactSchedule().isGrounded()) {
      leg->getStateTouchDown()->setPositionWorldToFootInWorldFrame(leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame());
      leg->getStateTouchDown()->setStateChangedAtTime(timeSinceInit_);
    }
  }

  if (!computeFrictionModulation(dt)) { return false; }

  return true;

}

bool EventDetectorOpt::computeFrictionModulation(double dt) {
  if (!enableFrictionModulation_) { return true; }

  for (auto leg : legs_) {
    const auto legEnum = contact_schedule::LegEnumAnymal(leg->getId());

    // Time since last touch-down.
    if (leg->getContactSchedule().isGrounded()) {
      timeSpendInStance_[legEnum] += dt;
    } else {
      timeSpendInStance_[legEnum] = 0.0;
    }

    // Update time spend in stance and no-slip.
    if (wasLegSlipping_[legEnum] && leg->getContactSchedule().isGrounded()) {
      if (!leg->getContactSchedule().isSlipping()) {
        timeSpendInStanceNoSlip_[legEnum] += dt;
      } else {
        timeSpendInStanceNoSlip_[legEnum] = 0.0;
      }
    }

    // Leg is slipping.
    if (leg->getContactSchedule().isSlipping()) {
      if (timeSpendInStance_[legEnum]>0.03) { // make sure leg is really slipping
        frictionModulation_[legEnum] = feedbackModulationWhileSlipping_;
        wasLegSlipping_[legEnum] = true;
      }
    }

    // Leg is not slipping.
    else {

      // We set the default friction modulation value only for stance legs.
      if (leg->getContactSchedule().isGrounded() && wasLegSlipping_[legEnum]) {
        if (timeSpendInStanceNoSlip_[legEnum]>0.5) { // make sure leg is no more slipping.
          frictionModulation_[legEnum] = 1.0;
          wasLegSlipping_[legEnum] = false;
          timeSpendInStanceNoSlip_[legEnum] = 0.0;
        }
      }
    }

    // Set modulation value.
    leg->setFrictionModulation(frictionModulation_[legEnum]);
  }

  return true;
}


} /* namespace loco */
