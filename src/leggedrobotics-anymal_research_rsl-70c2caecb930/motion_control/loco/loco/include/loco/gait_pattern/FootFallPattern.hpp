/*!
 * @file 	FootFallPattern.h
 * @author 	Christian Gehring, Stelian Coros
 * @date		Jun 14, 2012
 * @version 	1.0
 * @ingroup 	robotController
 * @brief
 */

#pragma once

#include <cassert>
#include <robot_utils/math/math.hpp>

namespace loco {

class FootFallPattern {
 public:
  //! Identifier of the leg this pattern belongs to
  unsigned int legId_;
  // keep track of the relative phase when the foot lifts off the ground...
  double liftOffPhase_;
  // and the relative phase when the foot strikes the ground...
  double strikePhase_;

  FootFallPattern() : legId_(0), liftOffPhase_(0.0), strikePhase_(0.0) {}

  FootFallPattern(unsigned int legId, double fLiftOff, double fStrike) : legId_(legId), liftOffPhase_(fLiftOff), strikePhase_(fStrike) {}

  FootFallPattern(const FootFallPattern&) = default;
  FootFallPattern& operator=(const FootFallPattern&) = default;
  FootFallPattern(FootFallPattern&&) = default;
  FootFallPattern& operator=(FootFallPattern&&) = default;

  /*!
   * @param absolutePhase   current state of the stride phase in [0,1]
   * @return phase left until foot lift off (which can happen in next stride)
   */
  double getPhaseLeftUntilLiftOff(double absolutePhase) const {
    assert(!(absolutePhase < 0.0 || absolutePhase > 1.0));  //"no reason why absolute phase should be outside the range [0, 1]");
    if (liftOffPhase_ == strikePhase_) {                    // added (Christian)
      return 1.0;
    }

    double start = liftOffPhase_;
    // We need the line below twice: if footLiftOff is -0.1, and phase = 0.95 for instance...
    while (start < absolutePhase) {
      start += 1.0;
    }

    double result = start - absolutePhase;
    assert(!(result < 0.0 || result > 1.0));  //"Bug in getPhaseLeftUntilFootLiftOff!!!";
    return result;
  }

  double getPhaseLeftUntilStrike(double absolutePhase) const {
    assert(!(absolutePhase < 0.0 || absolutePhase > 1.0));  //"no reason why absolute phase should be outside the range [0, 1]");
    if (liftOffPhase_ == strikePhase_) {                    // added (Christian)
      return 0.0;
    }

    double end = strikePhase_;
    while (end < absolutePhase) {
      end += 1.0;
    }

    double result = end - absolutePhase;
    while (result > 1.0) {
      result -= 1.0;
    }
    assert(!(result < 0.0 || result > 1.0));  //"Bug in getPhaseLeftUntilFootStrike!!!"
    return result;
  }
};

}  // namespace loco
