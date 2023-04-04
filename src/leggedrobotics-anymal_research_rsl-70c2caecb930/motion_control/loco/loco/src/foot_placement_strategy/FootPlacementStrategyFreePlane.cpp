/*!
 * @file     FootPlacementStrategyFreePlane.hpp
 * @author   C. Dario Bellicoso, Christian Gehring
 * @date     Sep 16, 2014
 * @brief
 */

// loco
#include "loco/foot_placement_strategy/FootPlacementStrategyFreePlane.hpp"
#include "loco/state_switcher/StateSwitcher.hpp"
#include "loco/swing_trajectory_generation/SwingTrajectoryGeneratorLinearInterpolation.hpp"
#include "loco/swing_trajectory_generation/SwingTrajectoryGeneratorSpline.hpp"

// message logger
#include <message_logger/message_logger.hpp>

// robot utils
#include "robot_utils/math/LinearAlgebra.hpp"
#include "robot_utils/math/math.hpp"

namespace loco {

FootPlacementStrategyFreePlane::FootPlacementStrategyFreePlane(WholeBody& wholeBody, TerrainModelBase& terrain,
                                                               ContactScheduleLock& contactSchedule,
                                                               SwingTrajectoryGeneratorBase& swingTrajectoryGenerator,
                                                               FootholdGeneratorInvertedPendulumBase& footholdGenerator)
    : FootPlacementStrategyBase(),
      regainVelocity_(1.0),
      maxFootholdDisplacementVelocity_(0.01),
      torso_(*wholeBody.getTorsoPtr()),
      legs_(*wholeBody.getLegsPtr()),
      terrain_(terrain),
      contactSchedule_(contactSchedule),
      swingTrajectoryGenerator_(swingTrajectoryGenerator),
      footholdGenerator_(footholdGenerator),
      positionWorldToOldDesiredFootHoldInWorldFrame_(legs_.size(), Position()),
      positionWorldToOldInterpolatedFootHoldInWorldFrame_(legs_.size(), Position()),
      positionWorldToDesiredFootHoldInWorldFrame_(legs_.size(), Position()),
      positionWorldToDesiredFootInWorldFrame_(legs_.size(), Position()),
      positionWorldToHipOnPlaneAlongNormalInWorldFrame_(legs_.size()),
      positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInWorldFrame_(legs_.size(), Position()),
      positionDesiredFootOnTerrainToDesiredFootInWorldFrame_(legs_.size(), Position()),
      positionHipOnTerrainToDesiredFootHoldOnTerrainFeedForwardInControlFrame_(legs_.size(), Position()),
      positionHipOnTerrainToDesiredFootHoldOnTerrainFeedBackInControlFrame_(legs_.size(), Position()),
      positionWorldToHipOnTerrainAlongNormalAtLiftOffInWorldFrame_(legs_.size(), Position()),
      positionWorldToDefaultFootHoldInWorldFrame_(legs_.size(), Position()),
      positionWorldToFootHoldInvertedPendulumInWorldFrame_(legs_.size(), Position()) {}

bool FootPlacementStrategyFreePlane::initialize(double /*dt*/) {
  for (auto leg : legs_) {
    positionWorldToHipOnPlaneAlongNormalInWorldFrame_[leg->getId()] = leg->getLimbStateMeasured().getPositionWorldToLimbBaseInWorldFrame();
    positionWorldToHipOnPlaneAlongNormalInWorldFrame_[leg->getId()].z() = 0.0;

    positionWorldToDesiredFootHoldInWorldFrame_[leg->getId()] =
        leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();
    positionWorldToOldDesiredFootHoldInWorldFrame_[leg->getId()] =
        leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();
    positionWorldToOldInterpolatedFootHoldInWorldFrame_[leg->getId()] =
        leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();
    positionWorldToDesiredFootInWorldFrame_[leg->getId()] = leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();

    positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInWorldFrame_[leg->getId()] = Position();
    positionDesiredFootOnTerrainToDesiredFootInWorldFrame_[leg->getId()] = Position();
    positionHipOnTerrainToDesiredFootHoldOnTerrainFeedForwardInControlFrame_[leg->getId()] = Position();
    positionHipOnTerrainToDesiredFootHoldOnTerrainFeedBackInControlFrame_[leg->getId()] = Position();

    // save the hip position at lift off for trajectory generation
    const Position positionWorldToHipAtLiftOffInWorldFrame = leg->getStateLiftOff()->getPositionWorldToHipInWorldFrame();
    positionWorldToHipOnTerrainAlongNormalAtLiftOffInWorldFrame_[leg->getId()] =
        getPositionProjectedOnPlaneAlongSurfaceNormal(positionWorldToHipAtLiftOffInWorldFrame);
    const Position positionWorldToHipOnTerrainAlongNormalAtLiftOffInWorldFrame =
        positionWorldToHipOnTerrainAlongNormalAtLiftOffInWorldFrame_[leg->getId()];
    leg->getStateLiftOff()->setPositionWorldToHipOnTerrainAlongWorldZInWorldFrame(
        positionWorldToHipOnTerrainAlongNormalAtLiftOffInWorldFrame);
  }

  // Setup logger
  if (isFirstTimeInit_) {
    isFirstTimeInit_ = false;
  }

  return true;
}

bool FootPlacementStrategyFreePlane::loadParameters(const TiXmlHandle& handle) {
  if (!footholdGenerator_.loadParameters(handle)) {
    return false;
  }
  if (!swingTrajectoryGenerator_.loadParameters(handle)) {
    return false;
  }

  TiXmlHandle parameterHandle = handle;
  if (!tinyxml_tools::getChildHandle(parameterHandle, handle, "FootPlacementStrategy")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(regainVelocity_, parameterHandle, "RegainVelocity")) {
    return false;
  }
  return tinyxml_tools::loadParameter(maxFootholdDisplacementVelocity_, parameterHandle, "maxFootholdDisplacementVelocity");
}

bool FootPlacementStrategyFreePlane::advance(double dt) {
  contactSchedule_.lock(false);
  for (auto leg : legs_) {
    // save the hip position at lift off for trajectory generation
    if (leg->getContactSchedule().shouldBeGrounded() ||
        (!leg->getContactSchedule().shouldBeGrounded() && leg->getContactSchedule().isGrounded() &&
         leg->getContactSchedule().getSwingPhase() < 0.25)) {
      // Project the hip at lift off along the normal to the plane that models the ground
      Position positionWorldToHipAtLiftOffInWorldFrame = leg->getLimbStateMeasured().getPositionWorldToLimbBaseInWorldFrame();
      positionWorldToHipOnTerrainAlongNormalAtLiftOffInWorldFrame_[leg->getId()] =
          getPositionProjectedOnPlaneAlongSurfaceNormal(positionWorldToHipAtLiftOffInWorldFrame);
      Position positionWorldToHipOnTerrainAlongNormalAtLiftOffInWorldFrame =
          positionWorldToHipOnTerrainAlongNormalAtLiftOffInWorldFrame_[leg->getId()];

      // Project the hip at lift off along the z axis in world frame
      Position positionWorldToHipOnTerrainAlongWorldZInWorldFrame = positionWorldToHipAtLiftOffInWorldFrame;
      terrain_.getHeight(positionWorldToHipOnTerrainAlongWorldZInWorldFrame);

      leg->getStateLiftOff()->setPositionWorldToHipOnTerrainAlongWorldZInWorldFrame(positionWorldToHipOnTerrainAlongWorldZInWorldFrame);
      leg->getStateLiftOff()->setPositionWorldToFootInWorldFrame(
          leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame());
      leg->getStateLiftOff()->setPositionWorldToHipInWorldFrame(leg->getLimbStateMeasured().getPositionWorldToLimbBaseInWorldFrame());
    }

    // Reset foot position at lift off.
    if ((leg->getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::Support) ||
        (leg->getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::ContactInvariant)) {
      leg->getStateLiftOff()->setPositionWorldToFootInWorldFrame(
          leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame());
    } else {
      switch (leg->getStateSwitcher().getState()) {
        case (StateSwitcher::States::StanceSlipping): {
          MELO_DEBUG_THROTTLE_STREAM(0.5, "Regain. Slipping. " << leg->getName())
          regainContact(leg, dt);
        } break;
        case (StateSwitcher::States::StanceLostContact): {
          MELO_DEBUG_THROTTLE_STREAM(0.5, "Regain. Lost contact. " << leg->getName())
          regainContact(leg, dt);
        } break;
        case (StateSwitcher::States::SwingExpectingContact): {
          MELO_DEBUG_THROTTLE_STREAM(0.5, "Regain. Expecting. " << leg->getName())
          regainContact(leg, dt);
        } break;

        case (StateSwitcher::States::SwingNormal):
        case (StateSwitcher::States::SwingLateLiftOff): {
          //        case(StateSwitcher::States::SwingBumpedIntoObstacle):
          setFootTrajectory(leg, dt);
          break;
        }
        default:
          break;
      }
    }
  }  // for each

  return true;
}

void FootPlacementStrategyFreePlane::setFootTrajectory(LegBase* leg, double dt) {
  /*
   * 1) Compute the desired Cartesian foot position
   */

  Position positionWorldToFootInWorldFrame;
  LinearVelocity linearVelocityDesFootInWorldFrame;
  LinearAcceleration linearAccelerationDesFootInWorldFrame;
  computeDesiredFootState(positionWorldToFootInWorldFrame, linearVelocityDesFootInWorldFrame, linearAccelerationDesFootInWorldFrame, leg,
                          dt);

  leg->getFootPtr()->getStateDesiredPtr()->setPositionWorldToEndEffectorInWorldFrame(positionWorldToFootInWorldFrame);
  leg->getFootPtr()->getStateDesiredPtr()->setLinearVelocityEndEffectorInWorldFrame(linearVelocityDesFootInWorldFrame);
  leg->getFootPtr()->getStateDesiredPtr()->setLinearAccelerationEndEffectorInWorldFrame(linearAccelerationDesFootInWorldFrame);

  /*
   * 2) Compute the desired joint positions and velocities
   */

  const Position positionBaseToFootInWorldFrame =
      positionWorldToFootInWorldFrame - torso_.getMeasuredState().getPositionWorldToBaseInWorldFrame();
  const Position positionBaseToFootInBaseFrame =
      torso_.getMeasuredState().getOrientationWorldToBase().rotate(positionBaseToFootInWorldFrame);

  const LinearVelocity linearVelocityBaseInWorldFrame =
      torso_.getMeasuredState().getOrientationWorldToBase().inverseRotate(torso_.getMeasuredState().getLinearVelocityBaseInBaseFrame());

  const LinearVelocity desiredLinearVelocityBaseToFootInBaseFrame =
      torso_.getMeasuredState().getOrientationWorldToBase().rotate(linearVelocityDesFootInWorldFrame) -
      torso_.getMeasuredState().getLinearVelocityBaseInBaseFrame() -
      LinearVelocity(torso_.getMeasuredState().getAngularVelocityBaseInBaseFrame().toImplementation().cross(
          leg->getFoot().getStateMeasured().getPositionBaseToEndEffectorInBaseFrame().toImplementation()));

  //  const double kp = 100.0;
  //  const Position positionError = positionBaseToFootInBaseFrame - leg->getPositionBaseToFootInBaseFrame();
  //  const RotationQuaternion& orientationWorldToBase = torso_.getMeasuredState().getOrientationWorldToBase();
  //  const LegBase::JointVelocities
  //  desiredJointVelocities(pseudoInverseAdaptiveDls(leg->getTranslationJacobianFromBaseToFootInBaseFrame())*(kp*positionError.toImplementation()
  //  + desiredLinearVelocityBaseToFootInBaseFrame.toImplementation()));

  leg->getLimbStateDesiredPtr()->setJointPositions(
      leg->getEndEffectorPtr()->getJointPositionsFromPositionBaseToEndEffectorInBaseFrame(positionBaseToFootInBaseFrame));
  leg->getLimbStateDesiredPtr()->setJointVelocities(
      leg->getEndEffectorPtr()->getJointVelocitiesFromLinearVelocityBaseToEndEffectorInBaseFrame(
          desiredLinearVelocityBaseToFootInBaseFrame));
  auto torques = leg->getLimbStateDesiredPtr()->getJointTorques();
  torques.setZero();
  leg->getLimbStateDesiredPtr()->setJointTorques(torques);
  //  leg->setDesiredJointVelocities(desiredJointVelocities);
}

void FootPlacementStrategyFreePlane::regainContact(LegBase* leg, double dt) {
  contactSchedule_.lock(true);
  Position positionWorldToFootInWorldFrame = leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();
  //  double loweringSpeed = 0.05;

  loco::Vector normalInWorldFrame = loco::Vector::UnitZ();
  positionWorldToFootInWorldFrame -= regainVelocity_ * dt * static_cast<loco::Position>(normalInWorldFrame);
  //  if (terrain_->getNormal(positionWorldToFootInWorldFrame,normalInWorldFrame)) {
  //    positionWorldToFootInWorldFrame -= 0.01*(loco::Position)normalInWorldFrame;
  //    //positionWorldToFootInWorldFrame -= (loweringSpeed*dt) * (loco::Position)normalInWorldFrame;
  //  }
  //  else  {
  //    throw std::runtime_error("FootPlacementStrategyFreePlane::advance cannot get terrain normal.");
  //  }

  const LinearVelocity desiredLinearVelocityInWorldFrame((-regainVelocity_) * normalInWorldFrame);
  leg->getFootPtr()->getStateDesiredPtr()->setPositionWorldToEndEffectorInWorldFrame(positionWorldToFootInWorldFrame);
  leg->getFootPtr()->getStateDesiredPtr()->setLinearVelocityEndEffectorInWorldFrame(desiredLinearVelocityInWorldFrame);
  leg->getFootPtr()->getStateDesiredPtr()->setLinearAccelerationEndEffectorInWorldFrame(
      2.0 * dt * LinearAcceleration(desiredLinearVelocityInWorldFrame));

  const Position& positionWorldToBaseInWorldFrame = torso_.getMeasuredState().getPositionWorldToBaseInWorldFrame();
  const Position positionBaseToFootInWorldFrame = positionWorldToFootInWorldFrame - positionWorldToBaseInWorldFrame;
  const Position positionBaseToFootInBaseFrame =
      torso_.getMeasuredState().getOrientationWorldToBase().rotate(positionBaseToFootInWorldFrame);

  const LinearVelocity desiredLinearVelocityBaseToFootInBaseFrame =
      torso_.getMeasuredState().getOrientationWorldToBase().rotate(desiredLinearVelocityInWorldFrame) -
      torso_.getMeasuredState().getLinearVelocityBaseInBaseFrame() -
      LinearVelocity(torso_.getMeasuredState().getAngularVelocityBaseInBaseFrame().toImplementation().cross(
          leg->getFoot().getStateMeasured().getPositionBaseToEndEffectorInBaseFrame().toImplementation()));

  leg->getLimbStateDesiredPtr()->setJointPositions(
      leg->getEndEffectorPtr()->getJointPositionsFromPositionBaseToEndEffectorInBaseFrame(positionBaseToFootInBaseFrame));
  leg->getLimbStateDesiredPtr()->setJointVelocities(
      leg->getEndEffectorPtr()->getJointVelocitiesFromLinearVelocityBaseToEndEffectorInBaseFrame(
          desiredLinearVelocityBaseToFootInBaseFrame));
  auto torques = leg->getLimbStateDesiredPtr()->getJointTorques();
  torques.setZero();
  leg->getLimbStateDesiredPtr()->setJointTorques(torques);
  //  leg->setDesiredJointVelocities(LegBase::JointVelocities());
}

Position FootPlacementStrategyFreePlane::getPositionProjectedOnPlaneAlongSurfaceNormal(const Position& position) {
  return terrain_.getPositionProjectedOnPlaneAlongSurfaceNormalInWorldFrame(position);
}

void FootPlacementStrategyFreePlane::computeDesiredFootState(Position& positionWorldToDesiredFootInWorldFrame,
                                                             LinearVelocity& linearVelocityDesiredFootInWorldFrame,
                                                             LinearAcceleration& linearAccelerationDesiredFootInWorldFrame, LegBase* leg,
                                                             double dt) {
  // 1. Generate foothold
  // todo: update method to take a leg ref.
  Position positionWorldToFootHoldInWorldFrame = footholdGenerator_.computeWorldToFootholdInWorldFrame(static_cast<int>(leg->getId()));

  // save for logging (should not be used anywhere else)
  //  positionWorldToFootHoldInvertedPendulumInWorldFrame_[leg->getId()] = positionWorldToFootHoldInWorldFrame;
  positionWorldToDesiredFootHoldInWorldFrame_[leg->getId()] = positionWorldToFootHoldInWorldFrame;
  positionHipOnTerrainToDesiredFootHoldOnTerrainFeedForwardInControlFrame_[leg->getId()] =
      footholdGenerator_.getPositionHipOnTerrainToDesiredFootHoldOnTerrainFeedForwardInControlFrame(static_cast<int>(leg->getId()));
  positionHipOnTerrainToDesiredFootHoldOnTerrainFeedBackInControlFrame_[leg->getId()] =
      footholdGenerator_.getPositionHipOnTerrainToDesiredFootHoldOnTerrainFeedBackInControlFrame(static_cast<int>(leg->getId()));

  // not needed anymore:
  //  const Position positionHipOnTerrainToDesiredFootholdInControlFrame =
  //  footholdGenerator_->getPositionHipOnTerrainToDesiredFootHoldOnTerrainFeedBackInControlFrame(leg->getId())+
  //                                                                       footholdGenerator_->getPositionHipOnTerrainToDesiredFootHoldOnTerrainFeedForwardInControlFrame(leg->getId());

  // 2. Generate next desired foot position in swing trajectory
  //  positionWorldToDesiredFootInWorldFrame =
  //  swingTrajectoryGenerator_->getPositionWorldToDesiredFootInWorldFrame(positionHipOnTerrainToDesiredFootholdInControlFrame,
  //                                                                                                                         leg->getId());
  Position positionWorldToFootHoldInControlFrame =
      torso_.getMeasuredState().inControlFrame().getOrientationWorldToControl().rotate(positionWorldToFootHoldInWorldFrame);

  if (!swingTrajectoryGenerator_.getDesiredFootState(positionWorldToDesiredFootInWorldFrame, linearVelocityDesiredFootInWorldFrame,
                                                     linearAccelerationDesiredFootInWorldFrame, positionWorldToFootHoldInControlFrame, leg,
                                                     dt)) {
    MELO_FATAL("Could not get desired foot state from swing trajectory generator!")
  }

  // 3. Check distance between actual foot position and new trajectory (if to big, recalculate trajectory using the old foothold)
  const Position& actualPositionWorldToFootInWorldFrame = leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();
  Position distanceToNewTrajectoryVector = positionWorldToDesiredFootInWorldFrame - actualPositionWorldToFootInWorldFrame;
  double distanceToNewTrajectory = distanceToNewTrajectoryVector.squaredNorm();

  double swingPhase = leg->getContactSchedule().getSwingPhase();
  Position positionWorldToOldFootHoldInControlFrame = torso_.getMeasuredState().inControlFrame().getOrientationWorldToControl().rotate(
      positionWorldToOldDesiredFootHoldInWorldFrame_[leg->getId()]);
  Position positionWorldToOldInterpolatedFootHoldInControlFrame =
      torso_.getMeasuredState().inControlFrame().getOrientationWorldToControl().rotate(
          positionWorldToOldInterpolatedFootHoldInWorldFrame_[leg->getId()]);

  // todo Remove magic number
  if (distanceToNewTrajectory > 0.0010 && swingPhase > 0.5) {
    positionWorldToFootHoldInControlFrame = positionWorldToOldFootHoldInControlFrame;
    //    if (!swingTrajectoryGenerator_.getDesiredFootState(positionWorldToDesiredFootInWorldFrame,
    //                                                         linearVelocityDesiredFootInWorldFrame,
    //                                                         linearAccelerationDesiredFootInWorldFrame,
    //                                                         positionWorldToOldFootHoldInControlFrame,
    //                                                         leg,
    //                                                         dt)) {
    //        MELO_FATAL("Could not get desired foot state from swing trajectory generator with old foothold!");
    //      }
    //      std::cout<<"Old Foothold was taken\n";
    //      positionWorldToDesiredFootHoldInWorldFrame_[leg->getId()] = positionWorldToOldDesiredFootHoldInWorldFrame_[leg->getId()];
  }

  // 4. restrict max foothold displacement velocity

  Position displacementDirectionInControlFrame =
      positionWorldToFootHoldInControlFrame - positionWorldToOldInterpolatedFootHoldInControlFrame;
  double footholdDisplacement = displacementDirectionInControlFrame.vector().norm();
  if (footholdDisplacement > maxFootholdDisplacementVelocity_) {
    footholdDisplacement = maxFootholdDisplacementVelocity_;
  }

  if (displacementDirectionInControlFrame.vector().norm() != 0) {
    displacementDirectionInControlFrame.vector().normalize();
  }

  Position positionWorldToNewFootHoldInControlFrame =
      positionWorldToOldInterpolatedFootHoldInControlFrame + footholdDisplacement * displacementDirectionInControlFrame;

  if (!swingTrajectoryGenerator_.getDesiredFootState(positionWorldToDesiredFootInWorldFrame, linearVelocityDesiredFootInWorldFrame,
                                                     linearAccelerationDesiredFootInWorldFrame, positionWorldToNewFootHoldInControlFrame,
                                                     leg, dt)) {
    MELO_FATAL("Could not get desired foot state from swing trajectory generator!")
  }

  // Update Old footholds
  positionWorldToFootHoldInWorldFrame =
      torso_.getMeasuredState().inControlFrame().getOrientationWorldToControl().inverseRotate(positionWorldToFootHoldInControlFrame);
  positionWorldToOldDesiredFootHoldInWorldFrame_[leg->getId()] = positionWorldToFootHoldInWorldFrame;

  loco::Position positionWorldToNewFootHoldInWorldFrame =
      torso_.getMeasuredState().inControlFrame().getOrientationWorldToControl().inverseRotate(positionWorldToNewFootHoldInControlFrame);
  positionWorldToOldInterpolatedFootHoldInWorldFrame_[leg->getId()] = positionWorldToNewFootHoldInWorldFrame;

  positionWorldToDesiredFootInWorldFrame_[leg->getId()] = positionWorldToDesiredFootInWorldFrame;

  //---
}

bool FootPlacementStrategyFreePlane::setToInterpolated(const FootPlacementStrategyBase& footPlacementStrategy1,
                                                       const FootPlacementStrategyBase& footPlacementStrategy2, double t) {
  const auto& footPlacement1 = dynamic_cast<const FootPlacementStrategyFreePlane&>(footPlacementStrategy1);
  const auto& footPlacement2 = dynamic_cast<const FootPlacementStrategyFreePlane&>(footPlacementStrategy2);

  const double interpolatedFeedbackScale = robot_utils::linearlyInterpolate(
      footPlacement1.getFootholdGenerator().getFeedbackScale(), footPlacement2.getFootholdGenerator().getFeedbackScale(), 0.0, 1.0, t);
  this->getFootholdGeneratorPtr()->setFeedbackScale(interpolatedFeedbackScale);

  // fixme remove get
  int iLeg = 0;
  for (auto leg : legs_) {
    leg->getLegPropertiesPtr()->setDesiredDefaultSteppingPositionHipToFootInControlFrame(robot_utils::linearlyInterpolate(
        footPlacement1.getLegs().get(iLeg).getLegProperties().getDesiredDefaultSteppingPositionHipToFootInControlFrame(),
        footPlacement2.getLegs().get(iLeg).getLegProperties().getDesiredDefaultSteppingPositionHipToFootInControlFrame(), 0.0, 1.0, t));
    iLeg++;
  }

  return true;
}

const Legs& FootPlacementStrategyFreePlane::getLegs() const {
  return legs_;
}

const FootholdGeneratorInvertedPendulumBase& FootPlacementStrategyFreePlane::getFootholdGenerator() const {
  return dynamic_cast<FootholdGeneratorInvertedPendulumBase&>(footholdGenerator_);
}

FootholdGeneratorInvertedPendulumBase* FootPlacementStrategyFreePlane::getFootholdGeneratorPtr() {
  return dynamic_cast<FootholdGeneratorInvertedPendulumBase*>(&footholdGenerator_);
}

const SwingTrajectoryGeneratorBase& FootPlacementStrategyFreePlane::getSwingTrajectoryGenerator() const {
  return swingTrajectoryGenerator_;
}

SwingTrajectoryGeneratorBase* FootPlacementStrategyFreePlane::getSwingTrajectoryGeneratorPtr() {
  return &swingTrajectoryGenerator_;
}

const std::vector<Position>& FootPlacementStrategyFreePlane::getPositionWorldToDesiredFootHoldInWorldFrame() const {
  return positionWorldToDesiredFootHoldInWorldFrame_;
}
const std::vector<Position>& FootPlacementStrategyFreePlane::getPositionWorldToDesiredFootInWorldFrame() const {
  return positionWorldToDesiredFootInWorldFrame_;
}
const std::vector<Position>& FootPlacementStrategyFreePlane::getPositionWorldToHipOnPlaneAlongNormalInWorldFrame() const {
  return positionWorldToHipOnPlaneAlongNormalInWorldFrame_;
}
const std::vector<Position>& FootPlacementStrategyFreePlane::getPositionHipOnTerrainAlongNormalToDesiredFootOnTerrainInWorldFrame() const {
  return positionHipOnTerrainAlongNormalToDesiredFootOnTerrainInWorldFrame_;
}
const std::vector<Position>& FootPlacementStrategyFreePlane::getPositionDesiredFootOnTerrainToDesiredFootInWorldFrame() const {
  return positionDesiredFootOnTerrainToDesiredFootInWorldFrame_;
}
const std::vector<Position>& FootPlacementStrategyFreePlane::getPositionDesiredFootHoldOnTerrainFeedForwardInControlFrame() const {
  return positionHipOnTerrainToDesiredFootHoldOnTerrainFeedForwardInControlFrame_;
}
const std::vector<Position>& FootPlacementStrategyFreePlane::getPositionDesiredFootHoldOnTerrainFeedBackInControlFrame() const {
  return positionHipOnTerrainToDesiredFootHoldOnTerrainFeedBackInControlFrame_;
}
const std::vector<Position>& FootPlacementStrategyFreePlane::getPositionWorldToHipOnTerrainAlongNormalAtLiftOffInWorldFrame() const {
  return positionWorldToHipOnTerrainAlongNormalAtLiftOffInWorldFrame_;
}
const std::vector<Position>& FootPlacementStrategyFreePlane::getPositionWorldToDefaultFootHoldInWorldFrame() const {
  return positionWorldToDefaultFootHoldInWorldFrame_;
}
const std::vector<Position>& FootPlacementStrategyFreePlane::getPositionWorldToFootHoldInvertedPendulumInWorldFrame() const {
  return positionWorldToFootHoldInvertedPendulumInWorldFrame_;
}

} /* namespace loco */
