/*!
* @file     FootPlacementStrategyOptimized.tpp
* @author   Fabian Jenelten
* @date     Mar 20, 2018
* @brief
*/

// anymal_description
#include <anymal_description/LegEnum.hpp>

// loco
#include "loco/foot_placement_strategy/FootPlacementStrategyOptimized.hpp"

#include "loco/foothold_generation/FootholdPlanInvPend.hpp"
#include "loco/foothold_generation/FootholdGeneratorOptimizedInvPend.hpp"

// message logger
#include <message_logger/message_logger.hpp>

using namespace message_logger::color;

namespace loco {

template<typename FootholdOptimizer_, typename FootholdPlan_>
FootPlacementStrategyOptimized<FootholdOptimizer_, FootholdPlan_>::FootPlacementStrategyOptimized(
    WholeBody& wholeBody,
    TerrainModelBase& terrain,
    SwingTrajectoryGeneratorModule& swingTrajectoryGenerator,
    FootholdOptimizer_& footholdGenerator,
    ContactScheduleZmp& contactSchedule,
    HeadingGenerator& headingGenerator,
    const TerrainAdapter& terrainAdapter,
    FootholdGeneratorInvertedPendulumBase& referenceFootholdGenerator)
    : FootPlacementStrategyBase(),
      wholeBody_(wholeBody),
      torso_(*wholeBody.getTorsoPtr()),
      legs_(*wholeBody.getLegsPtr()),
      terrain_(terrain),
      referenceFootholdGenerator_(referenceFootholdGenerator),
      swingTrajectoryGenerator_(swingTrajectoryGenerator),
      footholdPlanner_(footholdGenerator),
      plan_(),
      contactSchedule_(contactSchedule),
      headingGenerator_(headingGenerator),
      terrainAdapter_(terrainAdapter),
      mutexFootholdPlan_(),
      regainPosition_(0.0),
      invPendParamHandler_(),
      singularityValue_(0.5),
      singularityValueAtStartOfContactRecovery_(0.5),
      slippageRecoveryStrategy_(fps::SlippageRecoveryStrategy::Undefined) {

}

template<typename FootholdOptimizer_, typename FootholdPlan_>
bool FootPlacementStrategyOptimized<FootholdOptimizer_, FootholdPlan_>::initialize(double dt) {

  for (auto leg : legs_) {
    const auto legId = contact_schedule::LegEnumAnymal(leg->getId());
    updateSingularityValue(leg);
    singularityValueAtStartOfContactRecovery_[legId] = singularityValue_[legId];
  }

  if(!plan_.initialize(wholeBody_)) {
    MELO_WARN_STREAM("[FootPlacementStrategyOptimized::initialize] Failed to initialize foothold plan.");
    return false;
  }

  footholdPlanner_.parallelize(true);

  if(!footholdPlanner_.stopPlanning()) {
    MELO_WARN_STREAM("[FootPlacementStrategyOptimized::initialize] Failed to stop foothold planner.");
    return false;
  }

  if(!getSwingTrajectoryGeneratorModulePtr()->stop()) {
    MELO_WARN_STREAM("[FootPlacementStrategyOptimized::initialize] Failed to stop swing trajectory generator.");
    return false;
  }

  if(!getSwingTrajectoryGeneratorModulePtr()->initialize(dt)) {
    MELO_WARN_STREAM("[FootPlacementStrategyOptimized::initialize] Failed to initialize swing trajectory module.");
    return false;
  }

  if(!setFootholdPlan()) {
    MELO_WARN_STREAM("[FootPlacementStrategyOptimized::initialize] Failed to set foothold plan.");
    return false;
  }


  footholdPlanner_.setModulePlan(plan_);

  if(!footholdPlanner_.initialize(dt, false)) {
    MELO_WARN_STREAM("[FootPlacementStrategyOptimized::initialize] Failed to initialize foothold planner.");
    return false;
  }

  return true;
}

template<typename FootholdOptimizer_, typename FootholdPlan_>
bool FootPlacementStrategyOptimized<FootholdOptimizer_, FootholdPlan_>::stop() {
  if(!footholdPlanner_.stopPlanning()) {
    MELO_WARN_STREAM("[FootPlacementStrategyOptimized] Failed to stop foothold planner");
  }

  if(!getSwingTrajectoryGeneratorModulePtr()->stop()) {
    MELO_WARN_STREAM("[FootPlacementStrategyOptimized] Failed to stop swing trajectory");
  }

  return true;
}

template<typename FootholdOptimizer_, typename FootholdPlan_>
bool FootPlacementStrategyOptimized<FootholdOptimizer_, FootholdPlan_>::loadParameters(const TiXmlHandle& handle) {
  MELO_DEBUG_STREAM(magenta << "[FootPlacementStrategyOptimized] " << blue << "Load parameters." << def)

  if (!footholdPlanner_.loadParameters(handle)) { return false; }
  if (!swingTrajectoryGenerator_.loadParameters(handle)) { return false; }
  if (!referenceFootholdGenerator_.loadParameters(handle)) { return false; }
  if (!plan_.loadParameters(handle)) { return false; }

  // Regain velocity.
  TiXmlHandle fpsHandle = handle;
  if (!tinyxml_tools::getChildHandle(fpsHandle, handle, "FootPlacementStrategy")) { return false; }
  if (!tinyxml_tools::loadParameter(regainPosition_, fpsHandle, "RegainPositionForStride")) { return false; }

  std::string slippageRecoveryStrategyStr;
  if (!tinyxml_tools::loadParameter(slippageRecoveryStrategyStr, fpsHandle, "slippageRecoveryStrategy")) { return false; }
  if (slippageRecoveryStrategyStr == fps::SlippageRecoveryStrategyMap[fps::SlippageRecoveryStrategy::ApproachToCurrentFoothold]) {
    slippageRecoveryStrategy_ = fps::SlippageRecoveryStrategy::ApproachToCurrentFoothold;
  } else if (slippageRecoveryStrategyStr == fps::SlippageRecoveryStrategyMap[fps::SlippageRecoveryStrategy::ApproachToPreviousFoothold]) {
    slippageRecoveryStrategy_ = fps::SlippageRecoveryStrategy::ApproachToPreviousFoothold;
  } else if (slippageRecoveryStrategyStr == fps::SlippageRecoveryStrategyMap[fps::SlippageRecoveryStrategy::ApproachToPreviousFootholdIteratively]) {
    slippageRecoveryStrategy_ = fps::SlippageRecoveryStrategy::ApproachToPreviousFootholdIteratively;
  } else {
    MELO_FATAL_STREAM("[FootPlacementStrategyOptimized::loadParameters] Slippage recovery strategy  must be a ApproachToCurrentFoothold, ApproachToPreviousFoothold or ApproachToPreviousFootholdIteratively.");
    return false;
  }

  // Load inverted pendulum parameters.
  const TiXmlHandle fgHandle          = tinyxml_tools::getChildHandle(fpsHandle, "FootholdGenerator");
  const TiXmlHandle fgOptimizedHandle = tinyxml_tools::getChildHandle(fgHandle, "OptimizedInvPend");
  std::vector<TiXmlElement*> gaitElements;
  if(!tinyxml_tools::getChildElements(gaitElements, fgOptimizedHandle, "Gait")) { return false; }
  if(!contact_schedule::loadGaitParameters(gaitElements, invPendParamHandler_, contactSchedule_.getMapGaitNameToId())) { return false; }

  return true;
}


template<typename FootholdOptimizer_, typename FootholdPlan_>
bool FootPlacementStrategyOptimized<FootholdOptimizer_, FootholdPlan_>::advance(double dt) {

  if (!getSwingTrajectoryGeneratorModulePtr()->advance(dt)) {
    MELO_WARN_STREAM("[FootPlacementStrategyOptimized<FootholdOptimizer_, FootholdPlan_>::advance] Failed to advance swing trajectory generator.");
    return false;
  }

  // Generate an optimized footprint.
  if (!footholdPlanner_.hasStartedPlanning() && footholdPlanner_.hasFinishedPlanning()) {

    if(!setFootholdPlan()) {
      MELO_WARN_STREAM("[FootPlacementStrategyOptimized::advance] Failed to set foothold plan.");
      return false;
    }

    // Copy the plan to the planner.
    {
      boost::shared_lock<boost::shared_mutex> lock(mutexFootholdPlan_);
      footholdPlanner_.setModulePlan(plan_);
    }

    // Run the optimization.
    footholdPlanner_.startPlanning();
  }

  // Optimization has been completed.
  if (footholdPlanner_.hasStartedPlanning() && footholdPlanner_.hasFinishedPlanning()) {
    footholdPlanner_.setHasStartedPlanning(false);

    // get foothold plan from the planner
    {
      boost::unique_lock<boost::shared_mutex> lock(mutexFootholdPlan_);
      footholdPlanner_.getModulePlan(plan_);
    }
  }

  // Copy desired footholds into leg-container.
  {
    boost::shared_lock<boost::shared_mutex> lock(mutexFootholdPlan_);
    for (const auto& legId : anymal_description::LegEnumIterator()) {
      const auto legIdInt = static_cast<unsigned int>(legId);

      // Don't update a foothold if leg is in contact recovery.
      if (legs_.get(legIdInt).getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::ContactRecovery) {
        continue;
      }

      // Do not update desired foothold if leg is close to touch-down.
      if((contactSchedule_.shouldBeLegGrounded(legId) && contactSchedule_.getTimeSpentInStance(legId)<0.2*contactSchedule_.getTimeLeftInStance(legId)) ||
          (contactSchedule_.shouldBeLegSwing(legId) && contactSchedule_.getTimeSpentInSwing(legId)>1.5*contactSchedule_.getTimeLeftInSwing(legId))) {
        continue;
      }

      // Extract foothold.
      const auto& foothold = plan_.getPositionWorldToFootholdInWorldFrame(legIdInt);

      // Set.
      legs_.getPtr(legIdInt)->getFootPtr()->getStateDesiredPtr()->setPositionWorldToFootholdInWorldFrame(foothold);
    }
  }

  for (auto leg : legs_) {
    const auto legId = contact_schedule::LegEnumAnymal(leg->getId());
    updateSingularityValue(leg);

    // Decide what to do based on the current state.
      switch(leg->getLimbStrategy().getLimbStrategyEnum()) {
        // Case 1: Regain contact.
        case(LimbStrategyEnum::ContactRecovery) : {
          regainContact(leg, dt);
        } break;

        // Case 2: Swing leg.
        case(LimbStrategyEnum::Motion) : {
          setFootTrajectory(leg, dt);
          singularityValueAtStartOfContactRecovery_[legId] = singularityValue_[legId];
        } break;

        // Case 3: Stance leg.
        case(LimbStrategyEnum::Support) :
        case(LimbStrategyEnum::ContactInvariant) : {
          setFootOnGround(leg, dt);
          singularityValueAtStartOfContactRecovery_[legId] = singularityValue_[legId];
        } break;

         default : {
           MELO_WARN_STREAM("[FootPlacementStrategyOptimized:advance] Unknown State.");
         } break;
      }
  }

  return true;
}

template<typename FootholdOptimizer_, typename FootholdPlan_>
void FootPlacementStrategyOptimized<FootholdOptimizer_, FootholdPlan_>::setFootTrajectory(LegBase* leg, double dt) {
  Position positionWorldToFootInWorldFrame;
  LinearVelocity linearVelocityDesFootInWorldFrame;
  LinearAcceleration linearAccelerationDesFootInWorldFrame;
  computeDesiredFootState(
      positionWorldToFootInWorldFrame,
      linearVelocityDesFootInWorldFrame,
      linearAccelerationDesFootInWorldFrame,
      leg, dt);

  setDesiredFootState(leg,
      positionWorldToFootInWorldFrame,
      linearVelocityDesFootInWorldFrame,
      linearAccelerationDesFootInWorldFrame);
}

template<typename FootholdOptimizer_, typename FootholdPlan_>
void FootPlacementStrategyOptimized<FootholdOptimizer_, FootholdPlan_>::setDesiredFootState(
    LegBase* leg,
    const Position& positionWorldToFootInWorldFrame,
    const LinearVelocity& linearVelocityDesFootInWorldFrame,
    const LinearAcceleration& linearAccelerationDesFootInWorldFrame) {

  auto footDesiredState = leg->getFootPtr()->getStateDesiredPtr();
  footDesiredState->setPositionWorldToEndEffectorInWorldFrame(positionWorldToFootInWorldFrame);
  footDesiredState->setLinearVelocityEndEffectorInWorldFrame(linearVelocityDesFootInWorldFrame);
  footDesiredState->setLinearAccelerationEndEffectorInWorldFrame(linearAccelerationDesFootInWorldFrame);
}


template<typename FootholdOptimizer_, typename FootholdPlan_>
void FootPlacementStrategyOptimized<FootholdOptimizer_, FootholdPlan_>::regainContact(LegBase* leg, double dt) {
  const auto legId = contact_schedule::LegEnumAnymal(leg->getId());

  // Compute velocity gain s.t. end-effectors get slower the closer they get to a singular configuration.
  // maxAlowedSingularityValue corresponds to that value at which the end-effector reaches zero velocity.
  constexpr double maxAlowedSingularityValue = 0.95;
  const double scaledSingularityValue = std::fmin((singularityValue_[legId]-singularityValueAtStartOfContactRecovery_[legId])/(maxAlowedSingularityValue-singularityValueAtStartOfContactRecovery_[legId]), 1.0);
  const double velocityGain = 1.0-scaledSingularityValue*scaledSingularityValue;

  // Compute regain trajectory.
  LinearVelocity desiredLinearVelocityInWorldFrame(0.0, 0.0, -computeRegainVelocity()*velocityGain);
  Position positionWorldToFootInWorldFrame =
      leg->getFoot().getStateDesired().getPositionWorldToEndEffectorInWorldFrame() +
      Position(desiredLinearVelocityInWorldFrame*dt);
  LinearAcceleration linearAccelerationDesiredFootInWorldFrame = LinearAcceleration::Zero();

  setDesiredFootState(leg,
      positionWorldToFootInWorldFrame,
      desiredLinearVelocityInWorldFrame,
      linearAccelerationDesiredFootInWorldFrame);
}

template<typename FootholdOptimizer_, typename FootholdPlan_>
void FootPlacementStrategyOptimized<FootholdOptimizer_, FootholdPlan_>::setFootOnGround(LegBase* leg, double dt) {
  Position positionWorldToFootInWorldFrame;
  LinearVelocity linearVelocityFootInWorldFrame;
  LinearAcceleration linearAccelerationFootInWorldFrame = LinearAcceleration::Zero();

  // Approach to previous stance foothold.
  if (leg->getContactSchedule().isSlipping()) {
    switch(slippageRecoveryStrategy_) {
      case fps::SlippageRecoveryStrategy::ApproachToCurrentFoothold : {
        positionWorldToFootInWorldFrame = leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();
        linearVelocityFootInWorldFrame.setZero();
      } break;

      case fps::SlippageRecoveryStrategy::ApproachToPreviousFoothold : {
        positionWorldToFootInWorldFrame = leg->getPositionWorldToLastOrCurrentContactInWorldFrame();
        linearVelocityFootInWorldFrame.setZero();
      } break;

      case fps::SlippageRecoveryStrategy::ApproachToPreviousFootholdIteratively : {
        const auto positionFootToPreviousStanceContactInWorldFrame =
            leg->getPositionWorldToLastOrCurrentContactInWorldFrame() -
            leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();
        const double slippingDistance = positionFootToPreviousStanceContactInWorldFrame.toImplementation().norm();

        if (slippingDistance<1.0e-6) {
          positionWorldToFootInWorldFrame = leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();
          linearVelocityFootInWorldFrame.setZero();
        }

        else {
          const Eigen::Vector3d axisFootToPreviousStanceContactInWorldFrame = positionFootToPreviousStanceContactInWorldFrame.toImplementation()/slippingDistance;
          linearVelocityFootInWorldFrame = LinearVelocity(axisFootToPreviousStanceContactInWorldFrame*computeRegainVelocity());
          positionWorldToFootInWorldFrame =
              leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame() +
              Position(linearVelocityFootInWorldFrame*dt);
        }
      } break;

      default : {
        MELO_FATAL_STREAM("[FootPlacementStrategyOptimized::setFootOnGround] Unknown slippage recovery strategy.");
      } break;
    }
  }
  else {
    positionWorldToFootInWorldFrame = leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();
    linearVelocityFootInWorldFrame.setZero();
  }

  setDesiredFootState(
      leg,
      positionWorldToFootInWorldFrame,
      linearVelocityFootInWorldFrame,
      linearAccelerationFootInWorldFrame);
}

template<typename FootholdOptimizer_, typename FootholdPlan_>
Position FootPlacementStrategyOptimized<FootholdOptimizer_, FootholdPlan_>::getPositionProjectedOnPlaneAlongSurfaceNormal(const Position& position) {
  return terrain_.getPositionProjectedOnPlaneAlongSurfaceNormalInWorldFrame(position);
}

template<typename FootholdOptimizer_, typename FootholdPlan_>
void FootPlacementStrategyOptimized<FootholdOptimizer_, FootholdPlan_>::computeDesiredFootState(
    Position& positionWorldToDesiredFootInWorldFrame,
    LinearVelocity& linearVelocityDesiredFootInWorldFrame,
    LinearAcceleration& linearAccelerationDesiredFootInWorldFrame,
    LegBase* leg, double dt)
 {
  const Position& positionWorldToFootHoldInWorldFrame = leg->getFoot().getStateDesired().getPositionWorldToFootholdInWorldFrame();

  if (!swingTrajectoryGenerator_.getDesiredFootState(
      positionWorldToDesiredFootInWorldFrame,
      linearVelocityDesiredFootInWorldFrame,
      linearAccelerationDesiredFootInWorldFrame,
      torso_.getMeasuredState().inControlFrame().getOrientationWorldToControl().rotate(positionWorldToFootHoldInWorldFrame),
      leg, dt)) {
    MELO_WARN("[FootPlacementStrategyOptimized::computeDesiredFootState] Could not get desired foot state from swing trajectory generator!");
    positionWorldToDesiredFootInWorldFrame = getPositionProjectedOnPlaneAlongSurfaceNormal(leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame());
    linearVelocityDesiredFootInWorldFrame.setZero();
    linearAccelerationDesiredFootInWorldFrame.setZero();
  }
}

template<typename FootholdOptimizer_, typename FootholdPlan_>
bool FootPlacementStrategyOptimized<FootholdOptimizer_, FootholdPlan_>::setToInterpolated(
    const FootPlacementStrategyBase& /* footPlacementStrategy1 */,
    const FootPlacementStrategyBase& /* footPlacementStrategy2 */, double /* t */) {
  return true;
}

template<typename FootholdOptimizer_, typename FootholdPlan_>
const Legs& FootPlacementStrategyOptimized<FootholdOptimizer_, FootholdPlan_>::getLegs() const {
  return legs_;
}

template<typename FootholdOptimizer_, typename FootholdPlan_>
const SwingTrajectoryGeneratorBase& FootPlacementStrategyOptimized<FootholdOptimizer_, FootholdPlan_>::getSwingTrajectoryGenerator() const {
  return swingTrajectoryGenerator_;
}

template<typename FootholdOptimizer_, typename FootholdPlan_>
SwingTrajectoryGeneratorBase* FootPlacementStrategyOptimized<FootholdOptimizer_, FootholdPlan_>::getSwingTrajectoryGeneratorPtr() {
  return &swingTrajectoryGenerator_;
}

template<typename FootholdOptimizer_, typename FootholdPlan_>
bool FootPlacementStrategyOptimized<FootholdOptimizer_, FootholdPlan_>::setFootholdPlan() {
  boost::unique_lock<boost::shared_mutex> lock(mutexFootholdPlan_);

  // Set up foothold plan.
  const Position& positionPlaneToDesiredTargetHeightInPlaneFrame = terrainAdapter_.getPositionPlaneToDesiredTargetHeightInPlaneFrame();
  if(!plan_.updateData(wholeBody_, terrain_, contactSchedule_, headingGenerator_, positionPlaneToDesiredTargetHeightInPlaneFrame)) {
    MELO_WARN_STREAM("[FootPlacementStrategyOptimized::setFootholdPlan] Failed to update data.");
    return false;
  }

  return true;
}

template<>
bool FootPlacementStrategyOptimized<FootholdGeneratorOptimizedInvPend, foothold_generator::FootholdPlanInvPend>::setFootholdPlan() {
  boost::unique_lock<boost::shared_mutex> lock(mutexFootholdPlan_);

  // Set up foothold plan.
  const Position& positionPlaneToDesiredTargetHeightInPlaneFrame = terrainAdapter_.getPositionPlaneToDesiredTargetHeightInPlaneFrame();
  if(!plan_.updateData(wholeBody_, terrain_, contactSchedule_, headingGenerator_, positionPlaneToDesiredTargetHeightInPlaneFrame)) {
    MELO_WARN_STREAM("[FootPlacementStrategyOptimized] Failed to update data!");
    return false;
  }

  /*
   * Get gait depending parameters. We switch the gait parameters if a new active gait
   * appears.
   */
  const auto& params = invPendParamHandler_.getParams(contactSchedule_.getActiveGaitName());

  // Set control weights.
  referenceFootholdGenerator_.setFeedbackScale(params.feedbackScale_);
  referenceFootholdGenerator_.setFeedforwardScale(params.feedforwardScale_);

  // Set default footholds.
  double distanceBaseToDefaultFootholdLateralHind;
  double distanceBaseToDefaultFootholdLateralFront;
  if (contactSchedule_.isForwardDirection()) {
    distanceBaseToDefaultFootholdLateralHind = params.distanceBaseToDefaultFootholdLateralHind_;
    distanceBaseToDefaultFootholdLateralFront = params.distanceBaseToDefaultFootholdLateralFront_;
  } else {
    distanceBaseToDefaultFootholdLateralHind = params.distanceBaseToDefaultFootholdLateralFront_;
    distanceBaseToDefaultFootholdLateralFront = params.distanceBaseToDefaultFootholdLateralHind_;
  }

  const auto& linearVelocityInControlFrame = wholeBody_.getTorso().getDesiredState().getLinearVelocityTargetInControlFrame();

  auto footholdLateralComponent = 0.0;
  // The offset to be added to the nominal stance foothold at a reference velocity.
  // The following parameters help create the scaling function between velocity error and foothold offset.
  // For example if nominal operation velocity = 0.4 and lateral offset at nominal velocity = 0.08
  // then, f(x) = 0.08 * (x / 0.4)^3 st. at x = 0.4, f(x) = 0.08
  // This function is used to attenuate small velocity errors.
  const auto& nominalOperationVelocity = params.relativeOffsetLateral_.nominalOperationVelocity;
  const auto& lateralWidthOffsetAtNominalOperationVelocity = params.relativeOffsetLateral_.lateralWidthOffsetAtNominalOperationVelocity;

  // Check we are balancing. In this case, we do not scale the velocity error.
  auto isZeroCommandedVelocity = wholeBody_.getTorso().getDesiredState().getLinearVelocityCommandedTargetInControlFrame().norm() < 1e-4 &&
      wholeBody_.getTorso().getDesiredState().getAngularVelocityCommandedBaseInControlFrame().norm() < 1e-4;

  // Compute the linear velocity error for each leg, scale and average it to compute the lateral offset for foothold.
  for (const auto& leg : wholeBody_.getLegs()) {
    const auto lateralVelocityErrorInControlFrame = referenceFootholdGenerator_.computeLinearVelocityErrorInControlFrame(*leg).y();
    const auto scaledLateralVelocityError = isZeroCommandedVelocity ? lateralVelocityErrorInControlFrame
                                                  : std::pow(lateralVelocityErrorInControlFrame / nominalOperationVelocity, 3) *
                                                        lateralWidthOffsetAtNominalOperationVelocity;
    footholdLateralComponent += std::fabs(referenceFootholdGenerator_.computeLinearVelocityProjectionInControlFrame(*leg).y()) +
                                std::fabs(scaledLateralVelocityError);
  }
  footholdLateralComponent /= wholeBody_.getLegs().size();

  // Clip the lateral offset to be below the maximum allowed lateral offset.
  footholdLateralComponent = std::min(footholdLateralComponent, params.relativeOffsetLateral_.maxOffset_);
  referenceFootholdGenerator_.setDefaultFootholds(
      params.distanceBaseToDefaultFootholdHeading_,
      distanceBaseToDefaultFootholdLateralHind + footholdLateralComponent ,
      distanceBaseToDefaultFootholdLateralFront + footholdLateralComponent ,
      linearVelocityInControlFrame.x()*params.relativeOffsetHeading_,
      linearVelocityInControlFrame.y()*params.relativeOffsetHeading_
  );

  // Set weights.
  plan_.setWeights(params.weightsDesiredFoothold_, params.weightsPreviousFoothold_);

  // Compute desired foothold.
  for (unsigned int legId=0u; legId<legs_.size(); ++legId) {
    plan_.setPositionWorldToVelocityProjectionInWorldFrame(referenceFootholdGenerator_.computeWorldToFootholdInWorldFrame(legId), legId);
  }

  return true;
}

template<typename FootholdOptimizer_, typename FootholdPlan_>
void FootPlacementStrategyOptimized<FootholdOptimizer_, FootholdPlan_>::getFootholdPlan(FootholdPlan_& plan) const {
  boost::shared_lock<boost::shared_mutex> lock(mutexFootholdPlan_);
  plan.copy(plan_);
}

template<typename FootholdOptimizer_, typename FootholdPlan_>
void FootPlacementStrategyOptimized<FootholdOptimizer_, FootholdPlan_>::updateSingularityValue(LegBase* leg) {
  const Position positionBaseToLinkBaseInWorldFrame =
      wholeBody_.getTorso().getMeasuredState().getOrientationWorldToBase().inverseRotate(
          leg->getLinks().get(1).getBaseToLinkPositionInBaseFrame());

  const Position positionWorldToThighInWorldFrame =
      wholeBody_.getTorso().getMeasuredState().getPositionWorldToBaseInWorldFrame() +
      positionBaseToLinkBaseInWorldFrame;

  const Position positionThighToDesiredFootholdInWorldFrame =
      leg->getFoot().getStateDesired().getPositionWorldToEndEffectorInWorldFrame() -
      positionWorldToThighInWorldFrame;

  // Avoid leg over extension.
  const double maxLegExtension = leg->getLegProperties().getMaximumLimbExtension();
  const double legExtension = positionThighToDesiredFootholdInWorldFrame.norm();
  singularityValue_[contact_schedule::LegEnumAnymal(leg->getId())] = legExtension/maxLegExtension;
}

template<typename FootholdOptimizer_, typename FootholdPlan_>
double FootPlacementStrategyOptimized<FootholdOptimizer_, FootholdPlan_>::computeRegainVelocity() const {
  return (contactSchedule_.getNominalStrideDuration() > 1.0e-3 ?
      regainPosition_/contactSchedule_.getNominalStrideDuration() :
      regainPosition_
  );
}


} /* namespace loco */
