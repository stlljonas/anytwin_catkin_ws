/*
 * MissionControlStaticGait.cpp
 *
 *  Created on: Aug 22, 2014
 *      Author: C. Dario Bellicoso, Christian Gehring
 */

// loco
#include "loco/mission_control/MissionControlStaticGait.hpp"
#include "loco/foot_placement_strategy/FootPlacementStrategyStaticGait.hpp"
#include "loco/gait_pattern/GaitPatternStaticGait.hpp"
#include "loco/mission_control/MissionControlSpeedFilter.hpp"
#include "loco/torso_control/TorsoControlDynamicGaitFreePlane.hpp"
#include "loco/torso_control/TorsoControlStaticGait.hpp"

// message logger
#include "message_logger/message_logger.hpp"

// signal logger
#include <signal_logger/signal_logger.hpp>

namespace loco {

MissionControlStaticGait::MissionControlStaticGait(kindr::TwistLocalD* desRobotTwist, kindr::HomTransformQuatD* desRobotPose,
                                                   LocomotionController* locomotionController)
    : desRobotTwist_(desRobotTwist),
      desRobotPose_(desRobotPose),
      locomotionController_(locomotionController),
      speedFilter_(),
      locomotionState_(LocomotionState::Standing),
      locomotionMode_(LocomotionMode::ModeStanding),
      standingDidPinTime_(false),
      walkingFwDidPinTime_(false),
      walkingFwDidPinTimeHeading_(false),
      walkingFwShouldGoToStand_(false),
      walkingBwDidPinTime_(false),
      walkingBwDidPinTimeHeading_(false),
      walkingBwShouldGoToStand_(false),
      velocityHeadingThreshold_(0.005),
      velocityThreshold_(0.005),
      timeThresholdFwToBw_(2.0),
      timeThresholdStandToWalk_(1.0),
      timeThresholdWalkToStand_(2.0) {}

bool MissionControlStaticGait::initialize(double dt) {
  standingDidPinTime_ = false;

  walkingFwDidPinTime_ = false;
  walkingFwDidPinTimeHeading_ = false;
  walkingFwShouldGoToStand_ = false;

  walkingBwDidPinTime_ = false;
  walkingBwDidPinTimeHeading_ = false;
  walkingBwShouldGoToStand_ = false;

  locomotionState_ = LocomotionState::Standing;
  locomotionMode_ = LocomotionMode::ModeStanding;

  auto* contactSchedule = dynamic_cast<loco::GaitPatternStaticGait*>(locomotionController_->getContactSchedulePtr());
  contactSchedule->setStridePhase(0.0);
  contactSchedule->lock(true);

  modeNameMap_[LocomotionMode::ModeStanding] = "stand";
  modeNameMap_[LocomotionMode::ModeWalking] = "walk";

  signal_logger::add(positionOffset_, "desiredPositionOffset", "mission_control", "m");

  return true;
}

const std::string& MissionControlStaticGait::getLocomotionMode() const {
  return modeNameMap_.at(locomotionMode_);
}

bool MissionControlStaticGait::advance(double dt) {
  bool standing = true;
  for (auto leg : *locomotionController_->getLegsPtr()) {
    standing &= leg->getContactSchedule().isInStandConfiguration();
  }

  if (locomotionState_ == LocomotionState::Standing && standing) {
    auto* contactSchedule = dynamic_cast<loco::GaitPatternStaticGait*>(locomotionController_->getContactSchedulePtr());
    contactSchedule->setStridePhase(0.0);
    contactSchedule->lock(true);
  }

  /******************************************************************************************************
   * Based on the LocomotionMode value, read pose as orientation setpoints or twists as base twists *
   ******************************************************************************************************/
  switch (locomotionMode_) {
    case (LocomotionMode::ModeStanding): {
      speedFilter_.setUnfilteredDesiredPositionMiddleOfFeetToBaseInWorldFrame(desRobotPose_->getPosition());
      EulerAnglesZyx desRobotOrientation(desRobotPose_->getRotation());
      desRobotOrientation = EulerAnglesZyx(-desRobotOrientation.yaw(), desRobotOrientation.pitch(), desRobotOrientation.roll());
      speedFilter_.setUnfilteredDesiredOrientationOffset(RotationQuaternion(desRobotOrientation.getUnique()));
      speedFilter_.setUnfilteredDesiredBaseTwistInControlFrame(Twist());
    } break;

    case (LocomotionMode::ModeWalking): {
      desRobotTwist_->getRotationalVelocity().z() *= -1.0;
      speedFilter_.setUnfilteredDesiredOrientationOffset(RotationQuaternion());
      speedFilter_.setUnfilteredDesiredPositionMiddleOfFeetToBaseInWorldFrame(Position());
      speedFilter_.setUnfilteredDesiredBaseTwistInControlFrame(*desRobotTwist_);
    } break;

    default:
      break;
  }
  /******************************************************************************************************/

  // Filter the desired twist (and position)
  speedFilter_.advance(dt);
  const Twist& desiredTwist = speedFilter_.getDesiredBaseTwistInControlFrame();
  const Position& desiredFilteredPositionOffset = speedFilter_.getFilteredPositionOffsetInWorldFrame();

  // Set the desired twist to the torso module
  loco::LinearVelocity desLinearVelocityBaseInControlFrame = desiredTwist.getTranslationalVelocity();
  const loco::LocalAngularVelocity& desAngularVelocityBaseInControlFrame = desiredTwist.getRotationalVelocity();

  // Set position and orientation references to the main body
  locomotionController_->getTorsoPtr()->getDesiredStatePtr()->setDesiredPositionOffsetInWorldFrame(desiredFilteredPositionOffset);
  locomotionController_->getTorsoPtr()->getDesiredStatePtr()->setDesiredOrientationOffset(
      speedFilter_.getFilteredDesiredOrientationOffset());

  // the average velocity norm is used to decide if the state must be switched
  const double averageDesiredVelocityNorm =
      0.5 * (loco::Vector(desLinearVelocityBaseInControlFrame) + loco::Vector(desAngularVelocityBaseInControlFrame)).norm();

  /********************************************
   * Switch locomotion gait based on velocity *
   ********************************************/
  if (locomotionMode_ == LocomotionMode::ModeWalking) {
    switch (locomotionState_) {
      case (LocomotionState::Standing): {
        // check if 0.5*||linearVelocity+angularVelocity|| is bigger than the threshold and if the timer was not started
        if (averageDesiredVelocityNorm > velocityHeadingThreshold_ && !standingDidPinTime_) {
          chronoTimer_.pinTime();
          standingDidPinTime_ = true;
        }

        // check if 0.5*||linearVelocity+angularVelocity|| is lower than the threshold and if the timer was started
        if (averageDesiredVelocityNorm <= velocityHeadingThreshold_ && standingDidPinTime_) {
          standingDidPinTime_ = false;
          //          std::cout << message_logger::color::blue << "Standing: Timer flag reset" << message_logger::color::def << std::endl;
        }

        if (standingDidPinTime_ && chronoTimer_.getElapsedTimeSec() > timeThresholdStandToWalk_) {
          // reset flag
          standingDidPinTime_ = false;

          locomotionController_->getTorsoPtr()->getDesiredStatePtr()->setLinearVelocityTargetInControlFrame(
              desLinearVelocityBaseInControlFrame);
          locomotionController_->getTorsoPtr()->getDesiredStatePtr()->setAngularVelocityBaseInControlFrame(
              desAngularVelocityBaseInControlFrame);

          if (desLinearVelocityBaseInControlFrame.x() >= 0.0) {
            switchToForwardWalk();
          } else {
            switchToBackwardWalk();
          }
        }

      } break;

      case (LocomotionState::WalkingForwards): {
        double desHeadingVelocity = desLinearVelocityBaseInControlFrame.x();
        if (desHeadingVelocity < 0.0) {
          desLinearVelocityBaseInControlFrame.x() = 0.0;
        }

        if (desHeadingVelocity < 0.0 && !walkingFwDidPinTimeHeading_) {
          chronoTimerHeading_.pinTime();
          walkingFwDidPinTimeHeading_ = true;
        }
        if (desHeadingVelocity > velocityThreshold_ && walkingFwDidPinTimeHeading_) {
          walkingFwDidPinTimeHeading_ = false;
        }
        if (walkingFwDidPinTimeHeading_ && chronoTimerHeading_.getElapsedTimeSec() > timeThresholdFwToBw_) {
          walkingFwShouldGoToStand_ = true;
        }

        // check if velocity is lower than the threshold and if the timer was not started
        if (averageDesiredVelocityNorm <= (velocityHeadingThreshold_ * 0.9) && !walkingFwDidPinTime_) {
          chronoTimer_.pinTime();
          walkingFwDidPinTime_ = true;
          //        std::cout << message_logger::color::blue << "Walking fw: Timer started" << message_logger::color::def << std::endl;
        }

        // check if velocity is greater than the threshold and if the timer was started
        if (averageDesiredVelocityNorm > (velocityHeadingThreshold_ * 0.9) && walkingFwDidPinTime_) {
          walkingFwDidPinTime_ = false;
          //        std::cout << message_logger::color::blue << "Walking fw: Timer flag reset" << message_logger::color::def << std::endl;
        }

        // switch back to stand
        if ((walkingFwDidPinTime_ && chronoTimer_.getElapsedTimeSec() > timeThresholdWalkToStand_) || walkingFwShouldGoToStand_) {
          // wait for a stand phase to go to stand
          if (dynamic_cast<const loco::GaitPatternStaticGait&>(locomotionController_->getContactSchedule()).isFullStancePhase()) {
            // reset flags for next iteration
            walkingFwDidPinTime_ = false;
            walkingFwDidPinTimeHeading_ = false;
            walkingFwShouldGoToStand_ = false;

            // reset planned footholds
            for (auto leg : *locomotionController_->getLegsPtr()) {
              dynamic_cast<loco::ComSupportControlStaticGait*>(locomotionController_->getTorsoControllerPtr()->getComSupportControlPtr())
                  ->setFootHold(leg->getId(), leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame());
            }

            // decide if the state must switch to stand or to backwards walk
            if (desHeadingVelocity < 0.0) {
              switchToStand();
              switchToBackwardWalk(false);
            } else {
              switchToStand();
            }

            switchToStand();
          }
        }

      } break;

      case (LocomotionState::WalkingBackwards): {
        double desHeadingVelocity = desLinearVelocityBaseInControlFrame.x();
        if (desHeadingVelocity > 0.0) {
          desLinearVelocityBaseInControlFrame.x() = 0.0;
        }

        if (desHeadingVelocity > 0.0 && !walkingBwDidPinTimeHeading_) {
          //        std::cout << message_logger::color::blue << "Walking bw heading: start timer" << message_logger::color::def <<
          //        std::endl;
          chronoTimerHeading_.pinTime();
          walkingBwDidPinTimeHeading_ = true;
        }
        if (desHeadingVelocity < -velocityThreshold_ && walkingBwDidPinTimeHeading_) {
          //        std::cout << message_logger::color::blue << "Walking bw heading: reset flag" << message_logger::color::def << std::endl;
          walkingBwDidPinTimeHeading_ = false;
        }
        if (walkingBwDidPinTimeHeading_ && chronoTimerHeading_.getElapsedTimeSec() > timeThresholdFwToBw_) {
          //        std::cout << message_logger::color::blue << "Walking bw heading: should go to stand" << message_logger::color::def <<
          //        std::endl;
          walkingBwShouldGoToStand_ = true;
        }

        // check if velocity is lower than the threshold and if the timer was not started
        if (averageDesiredVelocityNorm <= (velocityHeadingThreshold_ * 0.9) && !walkingBwDidPinTime_) {
          chronoTimer_.pinTime();
          walkingBwDidPinTime_ = true;
          //        std::cout << message_logger::color::blue << "Timer started" << message_logger::color::def << std::endl;
        }

        // check if velocity is greater than the threshold and if the timer was started
        if (averageDesiredVelocityNorm > (velocityHeadingThreshold_ * 0.9) && walkingBwDidPinTime_) {
          walkingBwDidPinTime_ = false;
          //        std::cout << message_logger::color::blue << "Timer flag reset" << message_logger::color::def << std::endl;
        }

        if ((walkingBwDidPinTime_ && chronoTimer_.getElapsedTimeSec() > timeThresholdWalkToStand_) || walkingBwShouldGoToStand_) {
          // wait for a stand phase to go to stand
          if (dynamic_cast<const loco::GaitPatternStaticGait&>(locomotionController_->getContactSchedule()).isFullStancePhase()) {
            // reset flags for next iteration
            walkingBwDidPinTime_ = false;
            walkingBwDidPinTimeHeading_ = false;
            walkingBwShouldGoToStand_ = false;

            // reset planned footholds
            for (auto leg : *locomotionController_->getLegsPtr()) {
              dynamic_cast<loco::ComSupportControlStaticGait*>(locomotionController_->getTorsoControllerPtr()->getComSupportControlPtr())
                  ->setFootHold(leg->getId(), leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame());
            }

            // decide if the state must switch to stand or to forward walk
            if (desHeadingVelocity > 0.0) {
              switchToStand();
              switchToForwardWalk(false);
            } else {
              switchToStand();
            }

            switchToStand();
          }
        }

      } break;
    }
  }
  /********************************************/

  locomotionController_->getTorsoPtr()->getDesiredStatePtr()->setLinearVelocityTargetInControlFrame(desLinearVelocityBaseInControlFrame);
  locomotionController_->getTorsoPtr()->getDesiredStatePtr()->setAngularVelocityBaseInControlFrame(desAngularVelocityBaseInControlFrame);

  positionOffset_ = loco::Vector(desRobotPose_->getPosition());

  return true;
}

bool MissionControlStaticGait::loadParameters(const TiXmlHandle& handle) {
  return speedFilter_.loadParameters(handle);
}

const Twist& MissionControlStaticGait::getDesiredBaseTwistInControlFrame() const {}
//  return gaitSwitcher_->getLocomotionController()->getDesiredBaseTwistInHeadingFrame();

const Twist& MissionControlStaticGait::getMaximumBaseTwistInControlFrame() const {
  return speedFilter_.getMaximumBaseTwistInControlFrame();
}

const Pose& MissionControlStaticGait::getMinimalPoseOffset() const {
  return speedFilter_.getMinimalPoseOffset();
}

const Pose& MissionControlStaticGait::getMaximalPoseOffset() const {
  return speedFilter_.getMaximalPoseOffset();
}

double MissionControlStaticGait::interpolateJoystickAxis(double value, double minValue, double maxValue) {
  return 0.5 * (maxValue - minValue) * (value + 1.0) + minValue;
}

bool MissionControlStaticGait::switchToStand() {
  locomotionState_ = LocomotionState::Standing;

  dynamic_cast<loco::GaitPatternStaticGait*>(locomotionController_->getContactSchedulePtr())->setStridePhase(0.0);
  dynamic_cast<loco::GaitPatternStaticGait*>(locomotionController_->getContactSchedulePtr())->lock(true);

  dynamic_cast<loco::TorsoControlStaticGait*>(locomotionController_->getTorsoControllerPtr())->setIsInStandConfiguration(true);

  for (auto leg : *locomotionController_->getLegsPtr()) {
    leg->getContactSchedulePtr()->setIsInStandConfiguration(true);
  }

  return true;
}

bool MissionControlStaticGait::switchToForwardWalk(bool wasStanding) {
  locomotionState_ = LocomotionState::WalkingForwards;

  // update footfall pattern and turn gait pattern on
  dynamic_cast<loco::GaitPatternStaticGait*>(locomotionController_->getContactSchedulePtr())->setStridePhase(0.0);
  dynamic_cast<loco::GaitPatternStaticGait*>(locomotionController_->getContactSchedulePtr())->lock(false);
  dynamic_cast<loco::GaitPatternStaticGait*>(locomotionController_->getContactSchedulePtr())->setUseGaitPatternFw(true);
  dynamic_cast<loco::GaitPatternStaticGait*>(locomotionController_->getContactSchedulePtr())->updateFootfallPattern();
  dynamic_cast<loco::GaitPatternStaticGait*>(locomotionController_->getContactSchedulePtr())->resetSwingIndexes(wasStanding);

  if (wasStanding) {
    dynamic_cast<loco::FootPlacementStrategyStaticGait*>(locomotionController_->getFootPlacementStrategyPtr())->planFootholds();
    dynamic_cast<loco::TorsoControlStaticGait*>(locomotionController_->getTorsoControllerPtr())->setIsInStandConfiguration(false);

    for (auto leg : *locomotionController_->getLegsPtr()) {
      leg->getContactSchedulePtr()->setIsInStandConfiguration(false);
    }
  }

  return true;
}

bool MissionControlStaticGait::switchToBackwardWalk(bool wasStanding) {
  locomotionState_ = LocomotionState::WalkingBackwards;

  // update footfall pattern and turn gait pattern on
  dynamic_cast<loco::GaitPatternStaticGait*>(locomotionController_->getContactSchedulePtr())->setStridePhase(0.0);
  dynamic_cast<loco::GaitPatternStaticGait*>(locomotionController_->getContactSchedulePtr())->lock(false);
  dynamic_cast<loco::GaitPatternStaticGait*>(locomotionController_->getContactSchedulePtr())->setUseGaitPatternFw(false);
  dynamic_cast<loco::GaitPatternStaticGait*>(locomotionController_->getContactSchedulePtr())->updateFootfallPattern();
  dynamic_cast<loco::GaitPatternStaticGait*>(locomotionController_->getContactSchedulePtr())->resetSwingIndexes(wasStanding);

  if (wasStanding) {
    dynamic_cast<loco::FootPlacementStrategyStaticGait*>(locomotionController_->getFootPlacementStrategyPtr())->planFootholds();
    dynamic_cast<loco::TorsoControlStaticGait*>(locomotionController_->getTorsoControllerPtr())->setIsInStandConfiguration(false);

    for (auto leg : *locomotionController_->getLegsPtr()) {
      leg->getContactSchedulePtr()->setIsInStandConfiguration(false);
    }
  }

  return true;
}

bool MissionControlStaticGait::switchToWalkMode() {
  std::cout << message_logger::color::magenta << "[MissionController/advance] " << message_logger::color::blue << "Going to "
            << message_logger::color::red << "walk" << message_logger::color::blue << " configuration." << message_logger::color::def
            << std::endl;

  // todo: check if switchToStand is necessary

  locomotionMode_ = LocomotionMode::ModeWalking;
  locomotionState_ = LocomotionState::Standing;
  switchToStand();

  return true;
}

bool MissionControlStaticGait::switchToStandMode() {
  std::cout << message_logger::color::magenta << "[MissionController/advance] " << message_logger::color::blue << "Going to "
            << message_logger::color::red << "stand" << message_logger::color::blue << " configuration." << message_logger::color::def
            << std::endl;

  locomotionMode_ = LocomotionMode::ModeStanding;

  switchToStand();

  return true;
}

} /* namespace loco */
