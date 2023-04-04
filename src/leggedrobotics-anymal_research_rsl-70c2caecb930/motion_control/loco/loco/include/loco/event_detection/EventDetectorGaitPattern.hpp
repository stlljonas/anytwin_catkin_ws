/*
 * EventDetectorGaitPattern.hpp
 *
 *  Created on: Sep 5, 2014
 *      Author: C. Dario Bellicoso
 */

#pragma once

// loco
#include "loco/common/legs/Legs.hpp"
#include "loco/event_detection/EventDetector.hpp"
#include "loco/gait_pattern/GaitPatternBase.hpp"

namespace loco {

class EventDetectorGaitPattern : public EventDetector {
 public:
  EventDetectorGaitPattern(Legs& legs, const GaitPatternBase* gaitPattern);
  ~EventDetectorGaitPattern() override = default;

 protected:
  double getStridePhase() const override;

  const GaitPatternBase* gaitPattern_;
};

} /* namespace loco */
