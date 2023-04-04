/*!
* @file     LimbCoordinatorOpt.cpp
* @author   Dario Bellicoso
* @date     Feb, 2014
* @version  1.0
* @ingroup
* @brief
*/

// loco
#include "anymal_ctrl_dynamic_gaits/modules/LimbCoordinatorOpt.hpp"

namespace loco {

LimbCoordinatorOpt::LimbCoordinatorOpt(WholeBody& wholeBody) :
    LimbCoordinatorBase(),
    legs_(*wholeBody.getLegsPtr()),
    verbose_(false),
    logRate_(1.0),
    positionWorldToLastOrCurrentContactInWorldFrame_(Position::Zero())
{

}

bool LimbCoordinatorOpt::addVariablesToLog(const std::string & ns) const {
  return true;
}

bool LimbCoordinatorOpt::initialize(double dt) {
  for (auto leg : legs_) {
    const auto legEnum = contact_schedule::LegEnumAnymal(leg->getId());
    positionWorldToLastOrCurrentContactInWorldFrame_[legEnum] = leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();
  }

  return advance(dt);
}

bool LimbCoordinatorOpt::loadParameters(const TiXmlHandle& handle) {
  return true;
}

bool LimbCoordinatorOpt::advance(double dt) {
  for (auto leg : legs_) {
    auto stateSwitcher = leg->getStateSwitcherPtr();
    auto limbStrategy = leg->getLimbStrategyPtr();
    const auto legEnum = contact_schedule::LegEnumAnymal(leg->getId());

    if (leg->getContactSchedule().shouldBeGrounded()) {
      // (1) Should be grounded and is grounded.
      if (leg->getContactSchedule().isGrounded()) {
        if (leg->getContactSchedule().isSlipping()) {
          stateSwitcher->setState(StateSwitcher::States::StanceSlipping);
          limbStrategy->setLimbStrategyEnum(LimbStrategyEnum::ContactInvariant);
        } else {
          stateSwitcher->setState(StateSwitcher::States::StanceNormal);
          limbStrategy->setLimbStrategyEnum(LimbStrategyEnum::Support);
          positionWorldToLastOrCurrentContactInWorldFrame_[legEnum] = leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();
        }
        leg->setDidTouchDownAtLeastOnceDuringStance(true);
        leg->setDidLiftOffAtLeastOnceDuringSwing(false);
      }
      // (2) Should be grounded and is swinging.
      else {
        if (leg->didTouchDownAtLeastOnceDuringStance()) {
          stateSwitcher->setState(StateSwitcher::States::StanceLostContact);
          // contact recovery: Slip down from edge -> regain contact.
          // other limb state: contact detection noise -> use leg as support.
          if (limbStrategy->getLimbStrategyEnum() != LimbStrategyEnum::ContactRecovery) {
            limbStrategy->setLimbStrategyEnum(LimbStrategyEnum::ContactInvariant);
          }
          positionWorldToLastOrCurrentContactInWorldFrame_[legEnum] = leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();
        } else {
          stateSwitcher->setState(StateSwitcher::States::SwingExpectingContact);
          limbStrategy->setLimbStrategyEnum(LimbStrategyEnum::ContactRecovery);
        }
      }
    } else {
      // (3) Should be swinging and is ground.
      if (leg->getContactSchedule().isGrounded()) {
        if (!leg->didLiftOffAtLeastOnceDuringSwing()) {
          stateSwitcher->setState(StateSwitcher::States::SwingLateLiftOff);
          limbStrategy->setLimbStrategyEnum(LimbStrategyEnum::Motion);
        } else {
          if (leg->getContactSchedule().getSwingPhase() > 0.75) {
            stateSwitcher->setState(StateSwitcher::States::SwingEarlyTouchDown);
            limbStrategy->setLimbStrategyEnum(LimbStrategyEnum::Support);
            positionWorldToLastOrCurrentContactInWorldFrame_[legEnum] = leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();
            leg->setDidTouchDownAtLeastOnceDuringStance(true);
          } else {
            stateSwitcher->setState(StateSwitcher::States::SwingBumpedIntoObstacle);
            limbStrategy->setLimbStrategyEnum(LimbStrategyEnum::Motion);
          }
        }
      }

      // (4) Should be swinging and is swinging.
      else {
        // Slip down from edge after early touch-down -> regain contact.
        if (stateSwitcher->getState()==StateSwitcher::States::SwingEarlyTouchDown || stateSwitcher->getState()==StateSwitcher::States::StanceLostContact) {
          stateSwitcher->setState(StateSwitcher::States::StanceLostContact);
          limbStrategy->setLimbStrategyEnum(LimbStrategyEnum::ContactRecovery);
        }

        // Normal swing phase.
        else {
          stateSwitcher->setState(StateSwitcher::States::SwingNormal);
          limbStrategy->setLimbStrategyEnum(LimbStrategyEnum::Motion);
          leg->setDidLiftOffAtLeastOnceDuringSwing(true);
          leg->setDidTouchDownAtLeastOnceDuringStance(false);
        }
      }
    }

    // Remember position where contact was lost.
    if (stateSwitcher->getState() == StateSwitcher::States::StanceLostContact) {
      if (!leg->getContactSchedule().isLosingContact()) {
        leg->getContactSchedulePtr()->setIsLosingContact(true);
        leg->setPositionWorldToLostContactPositionInWorldFrame(
            leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame());
      }
    } else {
      leg->getContactSchedulePtr()->setIsLosingContact(false);
    }

    // Set previous stance foot location.
    leg->setPositionWorldToLastOrCurrentContactInWorldFrame(positionWorldToLastOrCurrentContactInWorldFrame_[legEnum]);

    // Log the limb coordinator state change.
    if (verbose_) {
      const auto limbStrategyEnum = leg->getLimbStrategy().getLimbStrategyEnum();
      const auto limbState = stateSwitcher->getState();

      if (limbStrategyEnum != leg->getLimbStrategy().getLimbStrategyEnum()) {
        std::string msg = "";
        msg.append(message_logger::color::magenta + "[LimbCoordinatorOpt::advance]" + message_logger::color::def + " Leg " + leg->getName() + " has strategy: " + leg->getLimbStrategy().getLimbStrategyName() + "\n");
        MELO_INFO_STREAM(msg);
      }

      if (limbState != stateSwitcher->getState()) {
        std::string msg = "";
        msg.append(message_logger::color::magenta + "[LimbCoordinatorOpt::advance]" + message_logger::color::def + " Leg " + leg->getName() + " has state: " + stateSwitcher->getCurrentStateName() + "\n");
        msg.append("Leg " + leg->getName() + " is grounded:        " + std::to_string(leg->getContactSchedule().isGrounded()) + "\n");
        msg.append("Leg " + leg->getName() + " should be grounded: " + std::to_string(leg->getContactSchedule().shouldBeGrounded()) + "\n");
        msg.append("Leg " + leg->getName() + " stance phase:  " + std::to_string(leg->getContactSchedule().getStancePhase()) + "\n");
        msg.append("Leg " + leg->getName() + " swing phase:   " + std::to_string(leg->getContactSchedule().getSwingPhase()) + "\n");
        MELO_INFO_THROTTLE_STREAM(logRate_, msg);
      }

      MELO_INFO_THROTTLE_STREAM(logRate_, getContactScheduleInfo().str());
    }
  }

  return true;
}


GaitPatternBase* LimbCoordinatorOpt::getGaitPatternPtr() {
  MELO_WARN_STREAM("[LimbCoordinatorOpt::getGaitPatternPtr] Function is not implemented!");
  return nullptr;
}


std::ostringstream LimbCoordinatorOpt::getContactScheduleInfo() {
  constexpr auto streamWidth = 20;
  std::ostringstream stream;
  stream << "----------------" << std::endl;
  stream << std::setw(streamWidth) << " "
         << std::setw(streamWidth) << legs_.get(0).getName()
         << std::setw(streamWidth) << legs_.get(1).getName()
         << std::setw(streamWidth) << legs_.get(2).getName()
         << std::setw(streamWidth) << legs_.get(3).getName() << std::endl;
  stream << std::setw(streamWidth) << "st. ph."
         << std::setw(streamWidth) << legs_.get(0).getContactSchedule().getStancePhase()
         << std::setw(streamWidth) << legs_.get(1).getContactSchedule().getStancePhase()
         << std::setw(streamWidth) << legs_.get(2).getContactSchedule().getStancePhase()
         << std::setw(streamWidth) << legs_.get(3).getContactSchedule().getStancePhase() << std::endl;
  stream << std::setw(streamWidth) << "sw. ph."
         << std::setw(streamWidth) << legs_.get(0).getContactSchedule().getSwingPhase()
         << std::setw(streamWidth) << legs_.get(1).getContactSchedule().getSwingPhase()
         << std::setw(streamWidth) << legs_.get(2).getContactSchedule().getSwingPhase()
         << std::setw(streamWidth) << legs_.get(3).getContactSchedule().getSwingPhase() << std::endl;
  stream << std::setw(streamWidth) << "st. d."
         << std::setw(streamWidth) << legs_.get(0).getContactSchedule().getStanceDuration()
         << std::setw(streamWidth) << legs_.get(1).getContactSchedule().getStanceDuration()
         << std::setw(streamWidth) << legs_.get(2).getContactSchedule().getStanceDuration()
         << std::setw(streamWidth) << legs_.get(3).getContactSchedule().getStanceDuration() << std::endl;
  stream << std::setw(streamWidth) << "sw. d."
         << std::setw(streamWidth) << legs_.get(0).getContactSchedule().getSwingDuration()
         << std::setw(streamWidth) << legs_.get(1).getContactSchedule().getSwingDuration()
         << std::setw(streamWidth) << legs_.get(2).getContactSchedule().getSwingDuration()
         << std::setw(streamWidth) << legs_.get(3).getContactSchedule().getSwingDuration() << std::endl;
  stream << std::setw(streamWidth) << "should st."
         << std::setw(streamWidth) << legs_.get(0).getContactSchedule().shouldBeGrounded()
         << std::setw(streamWidth) << legs_.get(1).getContactSchedule().shouldBeGrounded()
         << std::setw(streamWidth) << legs_.get(2).getContactSchedule().shouldBeGrounded()
         << std::setw(streamWidth) << legs_.get(3).getContactSchedule().shouldBeGrounded() << std::endl;
  stream << std::setw(streamWidth) << "strategy"
         << std::setw(streamWidth) << legs_.get(0).getLimbStrategy().getLimbStrategyName()
         << std::setw(streamWidth) << legs_.get(1).getLimbStrategy().getLimbStrategyName()
         << std::setw(streamWidth) << legs_.get(2).getLimbStrategy().getLimbStrategyName()
         << std::setw(streamWidth) << legs_.get(3).getLimbStrategy().getLimbStrategyName() << std::endl;

  stream << std::setw(streamWidth) << "state"
         << std::setw(streamWidth) << legs_.get(0).getStateSwitcher().getCurrentStateName()
         << std::setw(streamWidth) << legs_.get(1).getStateSwitcher().getCurrentStateName()
         << std::setw(streamWidth) << legs_.get(2).getStateSwitcher().getCurrentStateName()
         << std::setw(streamWidth) << legs_.get(3).getStateSwitcher().getCurrentStateName() << std::endl;

  stream << "----------------" << std::endl;

  return stream;
}


} /* namespace loco */
