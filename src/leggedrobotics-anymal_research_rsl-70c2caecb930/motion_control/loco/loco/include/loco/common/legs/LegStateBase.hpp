/*
 * LegStateBase.hpp
 *
 *  Created on: Feb 26, 2014
 *      Author: Christian Gehring, Dario Bellicoso
 */

#pragma once

namespace loco {

//! Base class for state of the Leg
class LegStateBase {
 public:
  LegStateBase();
  virtual ~LegStateBase() = default;

  bool isNow() const;
  void setIsNow(bool isNow);

  bool lastStateWasEarly() const;
  void setLastStateWasEarly(bool wasEarly);

  bool lastStateWasLate() const;
  void setLastStateWasLate(bool wasLate);

  double stateChangedAtTime() const;
  void setStateChangedAtTime(double time);

  double getPhase() const;
  void setPhase(double swingPhase);

 protected:
  bool isNow_;
  bool lastStateWasEarly_;
  bool lastStateWasLate_;
  double stateChangedAtTime_;
  double phase_;
};

} /* namespace loco */
