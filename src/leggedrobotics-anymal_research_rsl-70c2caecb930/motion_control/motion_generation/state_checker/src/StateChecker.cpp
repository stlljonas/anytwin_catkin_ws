/*
 * StateChecker.cpp
 *
 *  Created on: Aug 08, 2019
 *      Author: Fabian Jenelten
 */

// state checker.
#include "state_checker/StateChecker.hpp"
#include "state_checker/indicators/IndicatorContactDetection.hpp"
#include "state_checker/indicators/IndicatorTerrain.hpp"

namespace loco {
namespace state_checker {

StateChecker::StateChecker(WholeBody& wholeBody, TerrainModelBase& terrain)
    : StateCheckerBase(),
      wholeBody_(wholeBody),
      terrain_(terrain),
      indicatorContainer_(),
      timer_(),
      didPinTimer_(false),
      indicatorThreshold_(0.5),
      timeThreshold_(3.0) {
  // Add indicators.
  indicatorContainer_.clear();
  indicatorContainer_.addIndicator(new IndicatorTerrain());
  indicatorContainer_.addIndicator(new IndicatorContactDetection());
}

bool StateChecker::initialize(double dt) {
  if (!StateCheckerBase::initialize(dt)) {
    return false;
  }
  if (!indicatorContainer_.initialize(dt, wholeBody_, terrain_)) {
    return false;
  }
  didPinTimer_ = false;
  return true;
}

bool StateChecker::loadParameters(const TiXmlHandle& handle) {
  return indicatorContainer_.loadParameters(handle);
}

bool StateChecker::advance(double dt) {
  indicatorValue_ = indicatorContainer_.computeIndicatorValue(dt, wholeBody_, terrain_);

  // If indicator value is bigger than some threshold -> start timer.
  if (indicatorValue_ > indicatorThreshold_) {
    didPinTimer_ = true;
    timer_.pinTime();
  }

  // If timer is started, set unsafe mode until some time threshold is reached.
  if (didPinTimer_ && timer_.getElapsedTimeSec() < timeThreshold_) {
    isSafe_ = false;
  } else {
    isSafe_ = true;
  }

  return true;
}

} /* namespace state_checker */
} /* namespace loco */
