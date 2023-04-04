/*
 * EventDetector.cpp
 *
 *  Created on: Sep 5, 2014
 *      Author: C. Dario Bellicoso
 */

// loco
#include "loco/event_detection/EventDetectorGaitPattern.hpp"

namespace loco {

EventDetectorGaitPattern::EventDetectorGaitPattern(Legs& legs, const GaitPatternBase* gaitPattern)
    : EventDetector(legs), gaitPattern_(gaitPattern) {}

double EventDetectorGaitPattern::getStridePhase() const {
  if (gaitPattern_ != nullptr) {
    return gaitPattern_->getStridePhase();
  }
  return 0.0;
}

} /* namespace loco */
