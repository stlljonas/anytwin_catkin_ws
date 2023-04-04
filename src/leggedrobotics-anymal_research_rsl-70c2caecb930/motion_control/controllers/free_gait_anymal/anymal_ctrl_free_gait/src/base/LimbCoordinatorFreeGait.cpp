/*
 * LimbCoordinatorFreeGait.cpp
 *
 *  Created on: Jun 17, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "loco/state_switcher/StateSwitcher.hpp"

// FreeGait
#include <anymal_ctrl_free_gait/base/LimbCoordinatorFreeGait.hpp>
#include <anymal_ctrl_free_gait/base/StateLoco.hpp>

namespace loco {

using AD = anymal_description::AnymalDescription;

LimbCoordinatorFreeGait::LimbCoordinatorFreeGait(loco_anymal::LegsAnymal& legs, TorsoBase& torso, free_gait::Executor& executor,
                                                 GaitPatternBase& gaitPattern)
    : LimbCoordinatorBase(), legs_(legs), torso_(torso), executor_(executor), gaitPattern_(gaitPattern) {}

bool LimbCoordinatorFreeGait::initialize(double dt) {
  return advance(0.0);
}

bool LimbCoordinatorFreeGait::advance(double dt) {
  /* state_
   * 0: stance phase normal condition
   * 1: swing phase normal condition
   * 2: stance, but slipping
   * 3: stance, but lost contact / not yet touch-down
   * 4: swing, but late lift-off
   * 5: late swing, but early touch-down
   * 6: middle swing, but bumped into obstacle while swinging
   */

  StateSwitcher* stateSwitcher;

  for (auto leg : legs_) {
    stateSwitcher = leg->getStateSwitcherPtr();
    const auto& limbEnum = AD::mapKeyIdToKeyEnum<AD::LimbEnum>(leg->getLimbUInt());

    //--- Decide if a leg is supporting or not
    // Check timing
    if (leg->getContactSchedule().shouldBeGrounded()) {
      // stance mode according to plan
      if (leg->getContactSchedule().isGrounded()) {
        if (leg->getContactSchedule().isSlipping()) {
          // not safe to use this leg as support leg
          stateSwitcher->setState(StateSwitcher::States::StanceSlipping);
          leg->getLimbStrategyPtr()->setLimbStrategyEnum(LimbStrategyEnum::ContactRecovery);

          // todo think harder about this
        } else {
          // safe to use this leg as support leg
          stateSwitcher->setState(StateSwitcher::States::StanceNormal);
          leg->getLimbStrategyPtr()->setLimbStrategyEnum(LimbStrategyEnum::Support);
        }
        // save current position
        leg->getStateLiftOff()->setPositionWorldToFootInWorldFrame(
            leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame());

      } else {
        // leg is not a support leg. Now determine if it is expecting a contact or if it lost it.
        if (leg->didTouchDownAtLeastOnceDuringStance()) {
          stateSwitcher->setState(StateSwitcher::States::StanceLostContact);
          leg->getLimbStrategyPtr()->setLimbStrategyEnum(LimbStrategyEnum::ContactInvariant);
        } else {
          stateSwitcher->setState(StateSwitcher::States::SwingExpectingContact);
          leg->getLimbStrategyPtr()->setLimbStrategyEnum(LimbStrategyEnum::ContactRecovery);
        }
        // save current position
        if (!leg->didSetLostContactPositionForPhase()) {
          leg->setDidSetLostContactPositionForPhase(true);
          leg->setPositionWorldToLostContactPositionInWorldFrame(
              leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame());
        }
      }

    } else {
      // Swing mode according to plan.
      if (leg->getContactSchedule().isGrounded()) {
        bool isFootstepType = false;
        if (executor_.isInitialized()) {
          if (executor_.getQueue().active()) {
            const auto& step = executor_.getQueue().getCurrentStep();
            if (step.hasLegMotion(limbEnum)) {
              if (step.getLegMotion(limbEnum).hasContactAtStart()) {
                isFootstepType = true;
              }
            }
          }
        }

        if (isFootstepType) {
          // For footstep leg motions.
          if (leg->getContactSchedule().getSwingPhase() <= 0.3) {
            // leg should lift-off (late lift-off)
            stateSwitcher->setState(StateSwitcher::States::SwingLateLiftOff);
            leg->getLimbStrategyPtr()->setLimbStrategyEnum(LimbStrategyEnum::Motion);
          } else if (leg->getContactSchedule().getSwingPhase() > 0.6) {
            // early touch-down
            stateSwitcher->setState(StateSwitcher::States::SwingEarlyTouchDown);
            leg->getLimbStrategyPtr()->setLimbStrategyEnum(LimbStrategyEnum::Support);
          } else {
            // leg bumped into obstacle
            stateSwitcher->setState(StateSwitcher::States::SwingBumpedIntoObstacle);
            leg->getLimbStrategyPtr()->setLimbStrategyEnum(LimbStrategyEnum::Motion);
          }
        } else {
          // For other leg motions.
          stateSwitcher->setState(StateSwitcher::States::SwingEarlyTouchDown);
          leg->getLimbStrategyPtr()->setLimbStrategyEnum(LimbStrategyEnum::Support);
        }
      } else {
        // leg is on track
        stateSwitcher->setState(StateSwitcher::States::SwingNormal);
        leg->getLimbStrategyPtr()->setLimbStrategyEnum(LimbStrategyEnum::Motion);
      }
    }

    // Override support leg flag if no touchdown is expected.
    if (executor_.isInitialized()) {
      if (executor_.getState().isIgnoreContact(AD::mapKeyIdToKeyEnum<AD::LimbEnum>(leg->getLimbUInt()))) {
        stateSwitcher->setState(StateSwitcher::States::SwingNormal);
        leg->getLimbStrategyPtr()->setLimbStrategyEnum(LimbStrategyEnum::Motion);
      }
    }
  }

  return true;
}

GaitPatternBase* LimbCoordinatorFreeGait::getGaitPatternPtr() {
  return &gaitPattern_;
}

const GaitPatternBase& LimbCoordinatorFreeGait::getGaitPattern() const {
  return gaitPattern_;
}

bool LimbCoordinatorFreeGait::loadParameters(const TiXmlHandle& handle) {
  return true;
}

bool LimbCoordinatorFreeGait::setToInterpolated(const LimbCoordinatorBase& limbCoordinator1, const LimbCoordinatorBase& limbCoordinator2,
                                                double t) {
  return false;
}

} /* namespace loco */
