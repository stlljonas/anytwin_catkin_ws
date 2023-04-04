/*!
 * @file     FootPlacementStrategyStaticGait.cpp
 * @author   C. Dario Bellicoso, Christian Gehring
 * @date     Oct 6, 2014
 * @brief
 */

// loco
#include "loco/foot_placement_strategy/FootPlacementStrategyStaticGait.hpp"
#include "loco/swing_trajectory_generation/SwingTrajectoryGeneratorStaticGait.hpp"

// signal logger
#include "signal_logger/signal_logger.hpp"

// message logger
#include "message_logger/message_logger.hpp"

namespace color = message_logger::color;

namespace loco {

FootPlacementStrategyStaticGait::FootPlacementStrategyStaticGait(WholeBody& wholeBody, TerrainModelBase& terrain,
                                                                 GaitPatternStaticGait& gaitPattern,
                                                                 ComSupportControlStaticGait& comControl,
                                                                 SwingTrajectoryGeneratorBase& swingTrajectoryGenerator)
    : FootPlacementStrategyBase(),
      torso_(*wholeBody.getTorsoPtr()),
      legs_(*wholeBody.getLegsPtr()),
      terrain_(terrain),
      gaitPattern_(gaitPattern),
      comControl_(comControl),
      swingTrajectoryGenerator_(swingTrajectoryGenerator),
      footholdGenerator_(wholeBody, terrain_),
      didPlannedFootholdForNextPhase_(false),
      maxStepLengthHeading_(0.0),
      maxStepLengthLateral_(0.0),
      regainFactor_(legs_.size(), 0.0),
      rampingDown_(legs_.size(), false),
      dt_(0.0025),
      maxDistanceHipToFoot_(0.56),
      regainContactSpeed_(0.05),
      timeSinceSupport_(legs_.size(), 0.0),
      legLoad_(legs_.size(), 0.0),
      regainIndefinitely_(false) {
  for (auto leg : legs_) {
    leg->getFootPtr()->getStateDesiredPtr()->setPositionWorldToFootholdInWorldFrame(
        leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame());
  }
}

bool FootPlacementStrategyStaticGait::initialize(double dt) {
  footholdGenerator_.initialize(dt);

  for (auto& factor : regainFactor_) {
    factor = 0.0;
  }

  for (auto leg : legs_) {
    leg->getFootPtr()->getStateDesiredPtr()->setPositionWorldToFootholdInWorldFrame(
        leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame());
    footprintAtLastStance_.col(leg->getId()) =
        leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame().toImplementation();
  }

  for (auto&& k : rampingDown_) {
    k = false;
  }

  for (double& k : timeSinceSupport_) {
    k = 0.0;
  }

  for (double& k : legLoad_) {
    k = 1.0;
  }

  return true;
}

bool FootPlacementStrategyStaticGait::addVariablesToLog(bool updateLogger) {
  return true;
}

bool FootPlacementStrategyStaticGait::loadParameters(const TiXmlHandle& handle) {
  bool success = true;
  success &= footholdGenerator_.loadParameters(handle);
  success &= swingTrajectoryGenerator_.loadParameters(handle);

  TiXmlElement* pElem(handle.FirstChild("FootPlacementStrategy").FirstChild("Strategy").Element());

  if (pElem == nullptr) {
    std::cout << color::magenta << "[FootPlacementStrategyStaticGait/loadParameters] " << color::red << "***" << color::blue
              << "Could not find section 'FootPlacementStrategy::Strategy'." << color::red << "***" << color::def
              << " Regain contact will be active until hip to foot limit." << std::endl;
  } else {
    if (pElem->QueryBoolAttribute("regainIndefinitely", &regainIndefinitely_) != TIXML_SUCCESS) {
      regainIndefinitely_ = false;
      std::cout << color::magenta << "[FootPlacementStrategyStaticGait/loadParameters] " << color::red << "***" << color::blue
                << "Found section 'FootPlacementStrategy::Strategy::regainIndefinitely' but was not able to read from it." << color::red
                << "***" << color::def << " Regain contact will be active until hip to foot limit." << std::endl;
    } else {
      std::cout << color::magenta << "[FootPlacementStrategyStaticGait/loadParameters] " << color::red << "***" << color::blue
                << "Found section 'FootPlacementStrategy::Strategy::regainIndefinitely'." << color::red << "***" << color::def
                << " Regain contact will be active until contact." << std::endl;
    }
  }

  return success;
}

void FootPlacementStrategyStaticGait::computeLegLoad(LegBase& leg, const double startRampingDownAtStancePhase,
                                                     const double endRampingUpAtStancePhase, const double minLegLoad) {
  const unsigned int legId = leg.getId();
  rampingDown_[leg.getId()] = false;
  legLoad_[legId] = 1.0;

  if ((leg.getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::Support) ||
      (leg.getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::ContactInvariant)) {
    timeSinceSupport_[legId] += dt_;

    if (timeSinceSupport_[legId] <= endRampingUpAtStancePhase * leg.getContactSchedule().getStanceDuration()) {
      // ramping up - leg has just landed
      legLoad_[legId] = minLegLoad + (1.0 - minLegLoad) * robot_utils::mapTo01Range(
                                                              timeSinceSupport_[leg.getId()], 0.0,
                                                              endRampingUpAtStancePhase * leg.getContactSchedule().getStanceDuration());
    } else if (leg.getContactSchedule().getStancePhase() >= startRampingDownAtStancePhase) {
      // ramping down - leg is going to take off
      legLoad_[legId] = minLegLoad + (1.0 - minLegLoad) * (1.0 - robot_utils::mapTo01Range(leg.getContactSchedule().getStancePhase(),
                                                                                           startRampingDownAtStancePhase, 1.0));
      rampingDown_[legId] = true;
    } else {
      // mid stance phase - reset leg load to the default value
      legLoad_[leg.getId()] = 1.0;
    }
  } else {
    // leg is not supporting the main body
    timeSinceSupport_[legId] = 0.0;
    legLoad_[legId] = minLegLoad;
  }

  leg.setLoadFactor(legLoad_[legId]);
}

void FootPlacementStrategyStaticGait::planFootholds() {
  for (auto leg : legs_) {
    const Position& positionWorldToFootInWorldFrame = leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();
    footprintAtLastStance_.col(leg->getId()) << positionWorldToFootInWorldFrame.x(), positionWorldToFootInWorldFrame.y(),
        positionWorldToFootInWorldFrame.z();
  }

  // get pointer to next swing leg
  const int currentSwingLegId = gaitPattern_.getNextSwingLeg();

  // generate a foothold, limit it and save it locally
  const Position positionWorldToDesiredFootholdInWorldFrame =
      footholdGenerator_.generateFootHold(currentSwingLegId, footprintAtLastStance_, false);
  // fixme: remove get
  legs_.getPtr(currentSwingLegId)
      ->getFootPtr()
      ->getStateDesiredPtr()
      ->setPositionWorldToFootholdInWorldFrame(positionWorldToDesiredFootholdInWorldFrame);

  comControl_.setFootHold(currentSwingLegId, positionWorldToDesiredFootholdInWorldFrame);

  didPlannedFootholdForNextPhase_ = true;
}

bool FootPlacementStrategyStaticGait::advance(double dt) {
  dt_ = dt;

  /*******************
   * Update leg data *
   *******************/
  for (auto leg : legs_) {
    // Store the foot at lift off for trajectory generation.
    if (leg->getContactSchedule().shouldBeGrounded() ||
        (!leg->getContactSchedule().shouldBeGrounded() && leg->getContactSchedule().isGrounded() &&
         leg->getContactSchedule().getSwingPhase() < 0.25)) {
      const Position& positionWorldToFootInWorldFrame = leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();
      leg->getStateLiftOff()->setPositionWorldToFootInWorldFrame(positionWorldToFootInWorldFrame);
    }
    // fixme: check this
    leg->getFootPtr()->getStateDesiredPtr()->setLinearVelocityEndEffectorInWorldFrame(LinearVelocity());
    leg->getFootPtr()->getStateDesiredPtr()->setLinearAccelerationEndEffectorInWorldFrame(LinearAcceleration());
  }
  /*******************/

  if (gaitPattern_.isFullStancePhase() && !didPlannedFootholdForNextPhase_) {
    planFootholds();
  }

  if (!gaitPattern_.isFullStancePhase()) {
    didPlannedFootholdForNextPhase_ = false;
  }

  /*********************************************
   * Decide what to do with leg based on state *
   *********************************************/
  gaitPattern_.lock(false);

  for (auto leg : legs_) {
    if (leg->getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::Motion) {
      // Unlock the gait pattern and update the motion reference.
      gaitPattern_.lock(false);
      setFootTrajectory(*leg, dt);
    } else if (leg->getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::ContactRecovery) {
      regainContact(*leg, dt);
    } else {
      // Control reference is not handled by foot placement strategy.
      // As a safety layer, update the desired state with the current state.
      const Position& positionWorldToFootInWorldFrame = leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();
      leg->getFootPtr()->getStateDesiredPtr()->setPositionWorldToEndEffectorInWorldFrame(positionWorldToFootInWorldFrame);
      leg->getFootPtr()->getStateDesiredPtr()->setLinearVelocityEndEffectorInWorldFrame(LinearVelocity());
      leg->getFootPtr()->getStateDesiredPtr()->setLinearAccelerationEndEffectorInWorldFrame(LinearAcceleration());

      leg->getLimbStateDesiredPtr()->setJointPositions(leg->getLimbStateMeasured().getJointPositions());
      leg->getLimbStateDesiredPtr()->setJointVelocities(leg->getLimbStateMeasured().getJointVelocities());
      auto torques = leg->getLimbStateDesiredPtr()->getJointTorques();
      torques.setZero();
      leg->getLimbStateDesiredPtr()->setJointTorques(torques);
    }

  }  // for auto leg
  /*********************************************/

  return true;
}

void FootPlacementStrategyStaticGait::regainContact(LegBase& leg, double dt) {
  const Position& startPosition = leg.getPositionWorldToLostContactPositionInWorldFrame();
  Position desiredPositionWorldToFootInWorldFrame = startPosition;
  const loco::Position positionHipToFootInWorldFrame =
      leg.getLimbStateMeasured().getPositionWorldToLimbBaseInWorldFrame() - desiredPositionWorldToFootInWorldFrame;

  double regainSpeed = 0.7;

  // lower foot if not too far from hip
  if ((positionHipToFootInWorldFrame.norm() < maxDistanceHipToFoot_) || regainIndefinitely_) {
    regainFactor_[leg.getId()] += regainSpeed * dt;
    desiredPositionWorldToFootInWorldFrame = startPosition - regainFactor_[leg.getId()] * loco::Position::UnitZ();

    const loco::LinearVelocity desiredLinearVelocityInWorldFrame(-regainSpeed * loco::Vector::UnitZ());

    // Set motion reference in operational space.
    leg.getFootPtr()->getStateDesiredPtr()->setPositionWorldToEndEffectorInWorldFrame(desiredPositionWorldToFootInWorldFrame);
    leg.getFootPtr()->getStateDesiredPtr()->setLinearVelocityEndEffectorInWorldFrame(desiredLinearVelocityInWorldFrame);
    leg.getFootPtr()->getStateDesiredPtr()->setLinearAccelerationEndEffectorInWorldFrame(
        2.0 * loco::LinearAcceleration(desiredLinearVelocityInWorldFrame) * dt);

    // Set motion reference in joint space.
    const Position& positionWorldToBaseInWorldFrame = torso_.getMeasuredState().getPositionWorldToBaseInWorldFrame();
    const Position positionBaseToFootInWorldFrame = desiredPositionWorldToFootInWorldFrame - positionWorldToBaseInWorldFrame;
    const Position positionBaseToFootInBaseFrame =
        torso_.getMeasuredState().getOrientationWorldToBase().rotate(positionBaseToFootInWorldFrame);

    const LinearVelocity desiredLinearVelocityBaseToFootInBaseFrame =
        torso_.getMeasuredState().getOrientationWorldToBase().rotate(desiredLinearVelocityInWorldFrame) -
        torso_.getMeasuredState().getLinearVelocityBaseInBaseFrame() -
        LinearVelocity(torso_.getMeasuredState().getAngularVelocityBaseInBaseFrame().toImplementation().cross(
            leg.getFoot().getStateMeasured().getPositionBaseToEndEffectorInBaseFrame().toImplementation()));

    leg.getLimbStateDesiredPtr()->setJointPositions(
        leg.getEndEffectorPtr()->getJointPositionsFromPositionBaseToEndEffectorInBaseFrame(positionBaseToFootInBaseFrame));
    leg.getLimbStateDesiredPtr()->setJointVelocities(
        leg.getEndEffectorPtr()->getJointVelocitiesFromLinearVelocityBaseToEndEffectorInBaseFrame(
            desiredLinearVelocityBaseToFootInBaseFrame));
    leg.getLimbStateDesiredPtr()->setJointTorques(leg.getInitializedJointTorques());
  } else {
    const loco::LinearVelocity desiredLinearVelocityInBaseFrame;

    // Set motion reference.
    leg.getFootPtr()->getStateDesiredPtr()->setLinearVelocityEndEffectorInWorldFrame(loco::LinearVelocity());
    leg.getFootPtr()->getStateDesiredPtr()->setLinearAccelerationEndEffectorInWorldFrame(loco::LinearAcceleration());
    leg.getLimbStateDesiredPtr()->setJointVelocities(
        leg.getEndEffectorPtr()->getJointVelocitiesFromLinearVelocityBaseToEndEffectorInBaseFrame(desiredLinearVelocityInBaseFrame));
    leg.getLimbStateDesiredPtr()->setJointTorques(leg.getInitializedJointTorques());
  }
}

void FootPlacementStrategyStaticGait::setFootTrajectory(LegBase& leg, double dt) {
  regainFactor_[leg.getId()] = 0.0;

  Position positionWorldToFootInWorldFrame;
  LinearVelocity linearVelocityFootInWorldFrame;
  LinearAcceleration linearAccelerationFootInWorldFrame;

  const Position positionWorldToDesiredFootholdInControlFrame =
      torso_.getMeasuredState().inControlFrame().getOrientationWorldToControl().rotate(
          leg.getFoot().getStateDesired().getPositionWorldToFootholdInWorldFrame());
  swingTrajectoryGenerator_.getDesiredFootState(positionWorldToFootInWorldFrame, linearVelocityFootInWorldFrame,
                                                linearAccelerationFootInWorldFrame, positionWorldToDesiredFootholdInControlFrame, &leg, dt);

  leg.getFootPtr()->getStateDesiredPtr()->setPositionWorldToEndEffectorInWorldFrame(positionWorldToFootInWorldFrame);
  leg.getFootPtr()->getStateDesiredPtr()->setLinearVelocityEndEffectorInWorldFrame(linearVelocityFootInWorldFrame);
  leg.getFootPtr()->getStateDesiredPtr()->setLinearAccelerationEndEffectorInWorldFrame(linearAccelerationFootInWorldFrame);

  const Position positionBaseToFootInWorldFrame =
      positionWorldToFootInWorldFrame - torso_.getMeasuredState().getPositionWorldToBaseInWorldFrame();
  const Position positionBaseToFootInBaseFrame =
      torso_.getMeasuredState().getOrientationWorldToBase().rotate(positionBaseToFootInWorldFrame);

  const LinearVelocity desiredLinearVelocityBaseToFootInBaseFrame =
      torso_.getMeasuredState().getOrientationWorldToBase().rotate(linearVelocityFootInWorldFrame) -
      torso_.getMeasuredState().getLinearVelocityBaseInBaseFrame() -
      LinearVelocity(torso_.getMeasuredState().getAngularVelocityBaseInBaseFrame().toImplementation().cross(
          leg.getFoot().getStateMeasured().getPositionBaseToEndEffectorInBaseFrame().toImplementation()));

  leg.getLimbStateDesiredPtr()->setJointPositions(
      leg.getEndEffectorPtr()->getJointPositionsFromPositionBaseToEndEffectorInBaseFrame(positionBaseToFootInBaseFrame));
  leg.getLimbStateDesiredPtr()->setJointVelocities(
      leg.getEndEffectorPtr()->getJointVelocitiesFromLinearVelocityBaseToEndEffectorInBaseFrame(
          desiredLinearVelocityBaseToFootInBaseFrame));
  leg.getLimbStateDesiredPtr()->setJointTorques(leg.getInitializedJointTorques());
}

bool FootPlacementStrategyStaticGait::setToInterpolated(const FootPlacementStrategyBase& footPlacementStrategy1,
                                                        const FootPlacementStrategyBase& footPlacementStrategy2, double /*t*/) {
  return false;
}

const SwingTrajectoryGeneratorBase& FootPlacementStrategyStaticGait::getSwingTrajectoryGenerator() const {
  return swingTrajectoryGenerator_;
}

SwingTrajectoryGeneratorBase* FootPlacementStrategyStaticGait::getSwingTrajectoryGeneratorPtr() {
  return &swingTrajectoryGenerator_;
}

} /* namespace loco */
