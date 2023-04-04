/*
 * EventDetector.hpp
 *
 *  Created on: Sep 5, 2014
 *      Author: C. Dario Bellicoso
 */

#pragma once

// loco
#include "loco/common/legs/Legs.hpp"
#include "loco/event_detection/EventDetectorBase.hpp"
#include "loco/gait_pattern/GaitPatternBase.hpp"

namespace loco {

class EventDetector : public EventDetectorBase {
 public:
  explicit EventDetector(Legs& legs);
  ~EventDetector() override = default;

  bool initialize(double dt) override;
  bool advance(double dt) override;

 protected:
  virtual double getStridePhase() const;

  Legs& legs_;
  double toleratedDelay_;
  double minimumDistanceForSlipDetection_;
  double minimumSpeedForSlipDetection_;
  double timeSinceInit_;
};

} /* namespace loco */
