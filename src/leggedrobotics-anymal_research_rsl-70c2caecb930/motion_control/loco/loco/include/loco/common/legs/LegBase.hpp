/*
 * LegBase.hpp
 *
 *  Created on: Feb 25, 2014
 *      Author: Christian Gehring, Dario Bellicoso
 */

#pragma once

// loco
#include <loco/common/legs/LegProperties.hpp>
#include "loco/common/end_effectors/FootBase.hpp"
#include "loco/common/legs/ContactSchedule.hpp"
#include "loco/common/legs/LegStateLiftOff.hpp"
#include "loco/common/legs/LegStateTouchDown.hpp"
#include "loco/common/limbs/LimbBase.hpp"
#include "loco/common/typedefs.hpp"
#include "loco/state_switcher/StateSwitcher.hpp"

// kindr
#include <kindr/Core>

// eigen
#include <Eigen/Core>

// stl
#include <cstdint>
#include <iostream>
#include <string>

namespace loco {

//! Base class for a leg
/*! This should be used only as a data container
 *
 */
class LegBase : public LimbBase {
 public:
  LegBase(const std::string& name, const unsigned int numDofLimb, LegPropertiesPtr&& properties, FootBasePtr&& foot);
  ~LegBase() override = default;

  LegStateTouchDown* getStateTouchDown();
  const LegStateTouchDown& getStateTouchDown() const;

  LegStateLiftOff* getStateLiftOff();
  const LegStateLiftOff& getStateLiftOff() const;

  LegProperties* getLegPropertiesPtr();
  const LegProperties& getLegProperties() const;

  FootBase* getFootPtr();
  const FootBase& getFoot() const;

  // todo: remove virtual?
  virtual ContactSchedule* getContactSchedulePtr();
  virtual const ContactSchedule& getContactSchedule() const;

  StateSwitcher* getStateSwitcherPtr();
  const StateSwitcher& getStateSwitcher() const;

  virtual void setDesiredJointControlModeToLeg(loco::ControlMode controlMode);
  virtual bool isAnyJointInControlMode(loco::ControlMode controlMode) const;

  virtual bool didTouchDownAtLeastOnceDuringStance() const;
  virtual void setDidTouchDownAtLeastOnceDuringStance(bool didTouchDownAtLeastOnceDuringStance);

  bool didLiftOffAtLeastOnceDuringSwing() const;
  void setDidLiftOffAtLeastOnceDuringSwing(bool didLiftOffAtLeastOnceDuringSwing);

  virtual void setPositionWorldToLostContactPositionInWorldFrame(const Position& positionWorldToLostContactPositionInWorldFrame);
  virtual const Position& getPositionWorldToLostContactPositionInWorldFrame() const;

  virtual const Position& getPositionWorldToLastOrCurrentContactInWorldFrame() const;
  virtual void setPositionWorldToLastOrCurrentContactInWorldFrame(const Position& positionWorldToLastOrCurrentContactInWorldFrame);

  virtual void setDidSetLostContactPositionForPhase(bool didSetLostContactPositionForPhase);
  virtual bool didSetLostContactPositionForPhase() const;

  void print(std::ostream& out) const override;
  bool addVariablesToLog(const std::string& ns) const override;

 protected:
  //! Switches the logical state of the leg as a function of event detection.
  StateSwitcherPtr stateSwitcher_;

  //! Defines the contact schedule state.
  ContactSchedulePtr contactSchedule_;

  //! This is the state of the leg at touch-down event.
  LegStateTouchDown stateTouchDown_;

  //! This is the state of the leg at lift-off event.
  LegStateLiftOff stateLiftOff_;

  //*! Namespace for logging.
  std::string logNameSpace_;

  //! Indicates if the leg touched-down at least once during the current stance phase.
  bool didTouchDownAtLeastOnceDuringStance_;

  //! Indicates if the leg lifted-off at least once during the current swing phase.
  bool didLiftOffAtLeastOnceDuringSwing_;

  //! Indicates if the lost contact position was stored during the current phase.
  bool didSetLostContactPositionForPhase_;

  //! The position vector of contact position of this leg when contact was lost.
  Position positionWorldToLostContactPositionInWorldFrame_;

  //! The position vector of the most recent contact position of this leg.
  Position positionWorldToLastOrCurrentContactInWorldFrame_;
};

} /* namespace loco */
