/*
 * StateSwitcher.hpp
 *
 *  Created on: Oct 5, 2014
 *      Author: Dario Bellicoso
 */

#pragma once

// loco
#include "loco/state_switcher/StateSwitcherBase.hpp"

// stl
#include <map>
#include <memory>
#include <string>

namespace loco {

class StateSwitcher : public StateSwitcherBase {
 public:
  /* Limb state:
   *  1: stance phase normal condition
   *  2: swing phase normal condition
   * -1: stance, but slipping
   * -2: stance, but lost contact / not yet touch-down
   *  3: swing, but late lift-off
   *  4: late swing, but early touch-down
   * -3: middle swing, but bumped into obstacle while swinging
   *  5: swing, expecting contact
   */
  enum class States : int {
    Init = 0,

    StanceNormal = 1,
    StanceSlipping = -1,
    StanceLostContact = -2,

    SwingNormal = 2,
    SwingLateLiftOff = 3,
    SwingEarlyTouchDown = 4,

    SwingExpectingContact = 5,

    SwingBumpedIntoObstacle = -3,
  };

  StateSwitcher();
  ~StateSwitcher() override = default;

  bool initialize(double dt) override;

  virtual const std::string& getStateNameFromEnum(States state) const;
  virtual const std::string& getCurrentStateName() const;

  virtual bool addVariablesToLog(const std::string& ns) const;

  virtual void setState(States state);
  virtual States getState() const;

 protected:
  int limbState_;
  std::map<States, std::string> statesEnumNameMap_;
};

using StateSwitcherPtr = std::unique_ptr<StateSwitcher>;

} /* namespace loco */
