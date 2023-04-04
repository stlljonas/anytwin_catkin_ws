/*
 * EventDetector.cpp
 *
 *  Created on: Sep 5, 2014
 *      Author: C. Dario Bellicoso
 */

// loco
#include "loco/event_detection/EventDetector.hpp"

#define EVENT_DEBUG 0

namespace loco {

EventDetector::EventDetector(Legs& legs)
    : EventDetectorBase(),
      legs_(legs),
      toleratedDelay_(0.1),
      minimumDistanceForSlipDetection_(0.01),
      minimumSpeedForSlipDetection_(0.01),
      timeSinceInit_(0.0) {}

bool EventDetector::initialize(double dt) {
  toleratedDelay_ = 0.1;
  minimumDistanceForSlipDetection_ = 0.01;
  minimumSpeedForSlipDetection_ = 0.01;
  timeSinceInit_ = 0.0;

  return true;
}

double EventDetector::getStridePhase() const {
  return 0.0;
}

bool EventDetector::advance(double dt) {
  timeSinceInit_ += dt;

  for (auto leg : legs_) {
    const double swingPhase = leg->getContactSchedule().getSwingPhase();
    const double stancePhase = leg->getContactSchedule().getStancePhase();

    /*********************
     * Liftoff detection *
     *********************/
    if (leg->getContactSchedule().wasGrounded() && !leg->getContactSchedule().isGrounded()) {
      /*
       * Last time the leg was grounded, but now the leg is in the air.
       * This is a lift-off event.
       */
      leg->getStateLiftOff()->setIsNow(true);

      // reset the touch down flag for the next stance phase
      if (!leg->getContactSchedule().shouldBeGrounded() && leg->getContactSchedule().getSwingPhase() < 0.5) {
        // According to the plan the leg should be in swing phase.
        leg->setDidTouchDownAtLeastOnceDuringStance(false);
      }

      leg->getStateLiftOff()->setStateChangedAtTime(timeSinceInit_);

      // fixme: add logic to check if leg has ever touched down during swing phase
      // leg->getStateLiftOff()->setPhase(leg->getSwingPhase());
      // leg->getStateTouchDown()->setPhase(0.0);

      // A liftoff was detected, now check if it is earlier or later than expected
      if (leg->getContactSchedule().shouldBeGrounded() && (stancePhase < (1 - toleratedDelay_))) {
#if EVENT_DEBUG
        std::cout << "[eventDetector] EARLY liftoff on leg: " << leg->getName() << std::endl;
#endif
        leg->getStateLiftOff()->setLastStateWasEarly(true);
        leg->getStateLiftOff()->setLastStateWasLate(false);
      } else if (!leg->getContactSchedule().shouldBeGrounded() && (swingPhase > toleratedDelay_)) {
#if EVENT_DEBUG
        std::cout << "[eventDetector] LATE liftoff on leg: " << leg->getName() << std::endl;
#endif
        leg->getStateLiftOff()->setLastStateWasEarly(false);
        leg->getStateLiftOff()->setLastStateWasLate(true);
      } else {
#if EVENT_DEBUG
        std::cout << "[eventDetector] TOLERATED liftoff on leg: " << leg->getName() << std::endl;
#endif
        leg->getStateLiftOff()->setLastStateWasEarly(false);
        leg->getStateLiftOff()->setLastStateWasLate(false);
      }
    }  // if liftoff
    else {
      // this is not a lift-off event
      // reset liftoff state
      leg->getStateLiftOff()->setIsNow(false);
    }
    /*************************
     * End liftoff detection *
     *************************/

    /***********************
     * Touchdown detection *
     ***********************/
    if (!leg->getContactSchedule().wasGrounded() && leg->getContactSchedule().isGrounded()) {
      /*
       * Last time the leg was in the air, but now the leg is grounded.
       * This is a touch-down event.
       */
      leg->getStateTouchDown()->setIsNow(true);

      if (!leg->didTouchDownAtLeastOnceDuringStance() && leg->getContactSchedule().shouldBeGrounded()) {
        leg->setDidTouchDownAtLeastOnceDuringStance(true);
      }

      leg->getStateTouchDown()->setPositionWorldToFootInWorldFrame(
          leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame());
      leg->getStateTouchDown()->setStateChangedAtTime(timeSinceInit_);

      // fixme: add logic to check if leg has ever touched down during swing phase
      // leg->getStateTouchDown()->setPhase(leg->getStancePhase());
      // leg->getStateLiftOff()->setPhase(0.0);

      // A touchdown was detected, now check if it is earlier or later than expected
      if (!leg->getContactSchedule().shouldBeGrounded() && (swingPhase < (1 - toleratedDelay_))) {
#if EVENT_DEBUG
        std::cout << "[eventDetector] EARLY touchdown on leg: " << leg->getName() << std::endl;
#endif
        leg->getStateTouchDown()->setLastStateWasEarly(true);
        leg->getStateTouchDown()->setLastStateWasLate(false);
      } else if (leg->getContactSchedule().shouldBeGrounded() && (stancePhase > toleratedDelay_)) {
#if EVENT_DEBUG
        std::cout << "[eventDetector] LATE touchdown on leg: " << leg->getName() << std::endl;
#endif
        leg->getStateTouchDown()->setLastStateWasEarly(false);
        leg->getStateTouchDown()->setLastStateWasLate(true);
      } else {
#if EVENT_DEBUG
        std::cout << "[eventDetector] TOLERATED touchdown on leg: " << leg->getName() << std::endl;
#endif
        leg->getStateTouchDown()->setLastStateWasEarly(false);
        leg->getStateTouchDown()->setLastStateWasLate(false);
      }
    }  // if touchdown
    else {
      // this is not a touch-down
      // reset touchdown state
      leg->getStateTouchDown()->setIsNow(false);
    }
    /***************************
     * End touchdown detection *
     ***************************/

    /******************
     * Slip detection *
     ******************/
    if (leg->getContactSchedule().isGrounded()) {
      /* It is assumed that the foot can slip only if grounded. Check if the distance between
       * the current foot position and the touchdown foot position in world frame is greater
       * than a default minimum. Slipping stops when the linear velocity drops in norm under
       * a default minimum.
       */
      const loco::LinearVelocity& linearVelocityFootInWorldFrame =
          leg->getFoot().getStateMeasured().getLinearVelocityEndEffectorInWorldFrame();
      //        loco::Position distanceFromTouchdown = leg->getPositionWorldToFootInWorldFrame()
      //                                               - leg->getStateTouchDown()->getPositionWorldToFootInWorldFrame();

      // if ( (distanceFromTouchdown.norm() > minimumDistanceForSlipDetection_) ) {
      if ((linearVelocityFootInWorldFrame.norm() > minimumSpeedForSlipDetection_)) {
        // leg->setIsSlipping(true);
        leg->getContactSchedulePtr()->setIsSlipping(false);

#if EVENT_DEBUG
        std::cout << "[eventDetector] leg " << leg->getName() << " is slipping!" << std::endl;
        std::cout << "speed: " << linearVelocityFootInWorldFrame.norm() << std::endl;
        std::cout << "distance from touchdown: " << distanceFromTouchdown.norm() << std::endl;
#endif

      }  // if slipping

      /* Slipping state should be reset if the foot has not traveled for MINIMUM_DISTANCE_FOR_SLIP_DETECTION
       * or if, after slipping for some time, its speed drops below MINIMUM_SPEED_FOR_SLIP_DETECTION
       */
      if ((linearVelocityFootInWorldFrame.norm() < minimumSpeedForSlipDetection_)) {
        leg->getContactSchedulePtr()->setIsSlipping(false);
      }
    }  // if grounded
    else {
      // foot cannot be slipping if not grounded
      leg->getContactSchedulePtr()->setIsSlipping(false);
    }
    /**********************
     * End slip detection *
     **********************/

    /***************************
     * Check if losing contact during stance phase when leg is supposed to be grounded *
     ***************************/
    if (!leg->getContactSchedule().isGrounded() && leg->getContactSchedule().shouldBeGrounded()) {
      if (leg->didTouchDownAtLeastOnceDuringStance()) {
        leg->getContactSchedulePtr()->setIsLosingContact(true);
      }
    } else {
      leg->getContactSchedulePtr()->setIsLosingContact(false);
    }

    // reset the set lost contact flag while in swing mode
    if (!leg->getContactSchedule().shouldBeGrounded()) {
      leg->setDidSetLostContactPositionForPhase(false);
    }

    /*******************************
     * End check if losing contact *
     *******************************/

    if (leg->getContactSchedule().isGrounded()) {
      leg->getContactSchedulePtr()->setLastOrCurrentContactPhase(getStridePhase());
    }
  }

  return true;
}

} /* namespace loco */
