/*!
 * @file     LimbCoordinatorDeprecated.cpp
 * @author   Christian Gehring, Dario Bellicoso
 * @date     Feb, 2014
 * @version  1.0
 * @ingroup
 * @brief
 */

// loco
#include <loco/limb_coordinator/LimbCoordinatorDeprecated.hpp>
#include <loco/state_switcher/StateSwitcher.hpp>

// message logger
#include <message_logger/message_logger.hpp>

namespace loco {

LimbCoordinatorDeprecated::LimbCoordinatorDeprecated(WholeBody& wholeBody, GaitPatternBase& gaitPattern, bool isUpdatingStridePhase)
    : LimbCoordinatorBase(),
      wholeBody_(wholeBody),
      gaitPattern_(gaitPattern),
      isUpdatingStridePhase_(isUpdatingStridePhase),
      logLimbCoordinatorState_(false) {}

void LimbCoordinatorDeprecated::setIsUpdatingStridePhase(bool isUpdatingStridePhase) {
  isUpdatingStridePhase_ = isUpdatingStridePhase;
}

bool LimbCoordinatorDeprecated::isUpdatingStridePhase() const {
  return isUpdatingStridePhase_;
}

bool LimbCoordinatorDeprecated::initialize(double dt) {
  return advance(dt);
}

bool LimbCoordinatorDeprecated::advance(double /*dt*/) {
  StateSwitcher* stateSwitcher;

  for (auto arm : *wholeBody_.getArmsPtr()) {
    arm->getLimbStrategyPtr()->setLimbStrategyEnum(LimbStrategyEnum::Motion);
  }

  for (auto leg : *wholeBody_.getLegsPtr()) {
    stateSwitcher = leg->getStateSwitcherPtr();
    const LimbStrategyEnum limbstrategy = leg->getLimbStrategy().getLimbStrategyEnum();
    const StateSwitcher::States limbState = stateSwitcher->getState();

    leg->setFrictionModulation(1.0);

    // @fixme: Hack (Christian)
    leg->getContactSchedulePtr()->setIsSlipping(false);

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
      } else {
        // leg is not a support leg. Now determine if it is expecting a contact or if it lost it.
        if (leg->didTouchDownAtLeastOnceDuringStance()) {
          stateSwitcher->setState(StateSwitcher::States::StanceLostContact);
          leg->getLimbStrategyPtr()->setLimbStrategyEnum(LimbStrategyEnum::ContactInvariant);
          //          MELO_INFO_STREAM(leg->getName() << ": StanceLostContact");
        } else {
          stateSwitcher->setState(StateSwitcher::States::SwingExpectingContact);
          leg->getLimbStrategyPtr()->setLimbStrategyEnum(LimbStrategyEnum::ContactRecovery);
          //          MELO_INFO_STREAM(leg->getName() << ": SwingExpectingContact");
        }

        // save current position
        if (!leg->didSetLostContactPositionForPhase()) {
          leg->setDidSetLostContactPositionForPhase(true);
          leg->setPositionWorldToLostContactPositionInWorldFrame(
              leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame());
        }
      }
    } else {
      // swing mode according to plan
      if (leg->getContactSchedule().isGrounded()) {
        if (leg->getContactSchedule().getSwingPhase() <= 0.5) {
          // leg should lift-off (late lift-off)
          stateSwitcher->setState(StateSwitcher::States::SwingLateLiftOff);
          leg->getLimbStrategyPtr()->setLimbStrategyEnum(LimbStrategyEnum::Motion);
        } else if (leg->getContactSchedule().getSwingPhase() > 0.8) {
          // early touch-down
          stateSwitcher->setState(StateSwitcher::States::SwingEarlyTouchDown);
          leg->getLimbStrategyPtr()->setLimbStrategyEnum(LimbStrategyEnum::Support);
          leg->setDidTouchDownAtLeastOnceDuringStance(true);
        } else {
          // leg bumped into obstacle
          stateSwitcher->setState(StateSwitcher::States::SwingBumpedIntoObstacle);
          leg->getLimbStrategyPtr()->setLimbStrategyEnum(LimbStrategyEnum::Motion);
        }
      } else {
        stateSwitcher->setState(StateSwitcher::States::SwingNormal);
        leg->getLimbStrategyPtr()->setLimbStrategyEnum(LimbStrategyEnum::Motion);
      }
    }
    if ((leg->getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::Support) ||
        (leg->getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::ContactInvariant)) {
      // Update the most recent contact position of the leg.
      leg->setPositionWorldToLastOrCurrentContactInWorldFrame(
          leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame());
    }

    // Log the limb coordinator state change.
    if (logLimbCoordinatorState_) {
      if (limbstrategy != leg->getLimbStrategy().getLimbStrategyEnum()) {
        MELO_INFO_STREAM(message_logger::color::magenta << "[LimbCoordinatorDeprecated::advance]" << message_logger::color::def << " Leg "
                                                        << leg->getName()
                                                        << " has strategy: " << leg->getLimbStrategy().getLimbStrategyName() << std::endl)
      }

      if (limbState != stateSwitcher->getState()) {
        MELO_INFO_STREAM(message_logger::color::magenta
                         << "[LimbCoordinatorDeprecated::advance] " << message_logger::color::def << "Leg " << leg->getName()
                         << " has state: " << stateSwitcher->getCurrentStateName() << std::endl
                         << "Leg " << leg->getName() << " is grounded:        " << std::to_string(leg->getContactSchedule().isGrounded())
                         << std::endl
                         << "Leg " << leg->getName()
                         << " should be grounded: " << std::to_string(leg->getContactSchedule().shouldBeGrounded()) << std::endl
                         << "Leg " << leg->getName() << " stance phase:  " << std::to_string(leg->getContactSchedule().getStancePhase())
                         << std::endl
                         << "Leg " << leg->getName() << " swing phase:   " << std::to_string(leg->getContactSchedule().getSwingPhase())
                         << std::endl
                         << "Leg " << leg->getName()
                         << " interpolation: " << std::to_string(leg->getContactSchedule().getInterpolationParameter()) << std::endl)
      }
    }
  }

  return true;
}

void LimbCoordinatorDeprecated::setStridePhase(double stridePhase) {
  gaitPattern_.setStridePhase(stridePhase);
}

const GaitPatternBase& LimbCoordinatorDeprecated::getGaitPattern() const {
  return gaitPattern_;
}

GaitPatternBase* LimbCoordinatorDeprecated::getGaitPatternPtr() {
  return &gaitPattern_;
}

bool LimbCoordinatorDeprecated::loadParameters(const TiXmlHandle& handle) {
  return true;
}

bool LimbCoordinatorDeprecated::setToInterpolated(const LimbCoordinatorBase& limbCoordinator1, const LimbCoordinatorBase& limbCoordinator2,
                                                  double t) {
  const auto& coordinator1 = dynamic_cast<const LimbCoordinatorDeprecated&>(limbCoordinator1);
  const auto& coordinator2 = dynamic_cast<const LimbCoordinatorDeprecated&>(limbCoordinator2);
  return gaitPattern_.setToInterpolated(coordinator1.getGaitPattern(), coordinator2.getGaitPattern(), t);
}

void LimbCoordinatorDeprecated::printContactSchedule() {
  std::cout << "----------------" << std::endl;
  std::cout << std::setw(10) << " " << std::setw(10) << "LF" << std::setw(10) << " " << std::setw(10) << "RF" << std::setw(10) << " "
            << std::setw(10) << "LH" << std::setw(10) << " " << std::setw(10) << "RH" << std::endl;
  const auto legs = wholeBody_.getLegs();
  std::cout << std::setw(10) << "st. ph." << std::setw(10) << legs.get(0).getContactSchedule().getStancePhase() << std::setw(10) << " "
            << std::setw(10) << legs.get(1).getContactSchedule().getStancePhase() << std::setw(10) << " " << std::setw(10)
            << legs.get(2).getContactSchedule().getStancePhase() << std::setw(10) << " " << std::setw(10)
            << legs.get(3).getContactSchedule().getStancePhase() << std::setw(10) << std::endl;
  std::cout << std::setw(10) << "sw. ph." << std::setw(10) << legs.get(0).getContactSchedule().getSwingPhase() << std::setw(10) << " "
            << std::setw(10) << legs.get(1).getContactSchedule().getSwingPhase() << std::setw(10) << " " << std::setw(10)
            << legs.get(2).getContactSchedule().getSwingPhase() << std::setw(10) << " " << std::setw(10)
            << legs.get(3).getContactSchedule().getSwingPhase() << std::setw(10) << std::endl;
  std::cout << std::setw(10) << "st. d." << std::setw(10) << legs.get(0).getContactSchedule().getStanceDuration() << std::setw(10) << " "
            << std::setw(10) << legs.get(1).getContactSchedule().getStanceDuration() << std::setw(10) << " " << std::setw(10)
            << legs.get(2).getContactSchedule().getStanceDuration() << std::setw(10) << " " << std::setw(10)
            << legs.get(3).getContactSchedule().getStanceDuration() << std::setw(10) << std::endl;
  std::cout << std::setw(10) << "sw. d." << std::setw(10) << legs.get(0).getContactSchedule().getSwingDuration() << std::setw(10) << " "
            << std::setw(10) << legs.get(1).getContactSchedule().getSwingDuration() << std::setw(10) << " " << std::setw(10)
            << legs.get(2).getContactSchedule().getSwingDuration() << std::setw(10) << " " << std::setw(10)
            << legs.get(3).getContactSchedule().getSwingDuration() << std::setw(10) << std::endl;
  std::cout << std::setw(10) << "should st." << std::setw(10) << legs.get(0).getContactSchedule().shouldBeGrounded() << std::setw(10) << " "
            << std::setw(10) << legs.get(1).getContactSchedule().shouldBeGrounded() << std::setw(10) << " " << std::setw(10)
            << legs.get(2).getContactSchedule().shouldBeGrounded() << std::setw(10) << " " << std::setw(10)
            << legs.get(3).getContactSchedule().shouldBeGrounded() << std::setw(10) << std::endl;
  std::cout << std::setw(10) << "should st." << std::setw(10) << legs.get(0).getContactSchedule().shouldBeGrounded() << std::setw(10) << " "
            << std::setw(10) << legs.get(1).getContactSchedule().shouldBeGrounded() << std::setw(10) << " " << std::setw(10)
            << legs.get(2).getContactSchedule().shouldBeGrounded() << std::setw(10) << " " << std::setw(10)
            << legs.get(3).getContactSchedule().shouldBeGrounded() << std::setw(10) << std::endl;

  std::cout << "----------------" << std::endl;
}

} /* namespace loco */
