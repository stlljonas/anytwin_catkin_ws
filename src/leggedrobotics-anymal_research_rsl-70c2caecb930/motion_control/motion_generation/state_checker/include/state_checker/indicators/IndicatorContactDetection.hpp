/*
 * IndicatorContactDetection.hpp
 *
 *  Created on: Aug 08, 2019
 *      Author: Fabian Jenelten
 */

#pragma once

#include <vector>

// state checker.
#include "state_checker/IndicatorBase.hpp"

// std utils.
#include "std_utils/std_utils.hpp"

// robot utils.
#include "robot_utils/math/math.hpp"

class TiXmlHandle;

namespace loco {
namespace state_checker {

class IndicatorContactDetection : public IndicatorBase {
 public:
  IndicatorContactDetection() : IndicatorBase(), timer_(), didPinTimer_(1, false) { name_ = "contact detection"; }

  ~IndicatorContactDetection() override = default;

  bool initialize(double /*dt*/, WholeBody& wholeBody, TerrainModelBase& /*terrain*/) override {
    const auto& legs = wholeBody.getLegs();

    timer_.resize(legs.size());
    didPinTimer_.resize(legs.size());

    for (auto leg : legs) {
      const auto legId = leg->getId();
      didPinTimer_[legId] = false;
    }
    return true;
  }

  bool loadParameters(const TiXmlHandle& /*handle*/) override { return true; };

  double computeIndicatorValue(double /*dt*/, WholeBody& wholeBody, TerrainModelBase& /*terrain*/) override {
    double indicatorValue = 0.0;
    const auto& legs = wholeBody.getLegs();

    for (auto leg : legs) {
      const auto legId = leg->getId();

      // Check if state switcher entered a dangerous state for some time.
      bool isUndesiredStateSwitcherState = false;
      double timerThreshold = 0.0;
      double increaseInIndicatorValue = 0.0;
      switch (leg->getStateSwitcher().getState()) {
        case StateSwitcher::States::SwingLateLiftOff: {
          isUndesiredStateSwitcherState = true;
          timerThreshold = 0.20 * leg->getContactSchedule().getSwingDuration();
          increaseInIndicatorValue = 0.9;
        } break;

        case StateSwitcher::States::StanceSlipping: {
          isUndesiredStateSwitcherState = true;
          timerThreshold = 0.15 * leg->getContactSchedule().getStanceDuration();
          increaseInIndicatorValue = 0.5;
        } break;

        case StateSwitcher::States::SwingExpectingContact: {
          isUndesiredStateSwitcherState = true;
          timerThreshold = 0.10 * leg->getContactSchedule().getSwingDuration();
          increaseInIndicatorValue = 0.8;
        } break;

        default:
          break;
      }

      // Set condition number.
      if (isUndesiredStateSwitcherState) {
        if (!didPinTimer_[legId]) {
          timer_[legId].pinTime();
          didPinTimer_[legId] = true;
        }

        if (didPinTimer_[legId] && timer_[legId].getElapsedTimeSec() >= timerThreshold) {
          indicatorValue += increaseInIndicatorValue;
        }
      } else {
        didPinTimer_[legId] = false;
      }
    }

    return std::fmin(indicatorValue, 1.0);
  }

 protected:
  //! A watchdog timer to detect dangerous state switcher states.
  std::vector<std_utils::HighResolutionClockTimer> timer_;

  //! Helper boolean to trigger timers.
  std::vector<bool> didPinTimer_;
};

} /* namespace state_checker */
} /* namespace loco */
