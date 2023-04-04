/*!
* @file     FootPlacementStrategyOptimizedRegain.tpp
* @author   Fabian Jenelten
* @date     Mar 20, 2018
* @brief
*/

// anymal_description
#include <anymal_description/LegEnum.hpp>

// loco
#include "loco/foot_placement_strategy/FootPlacementStrategyOptimizedRegain.hpp"

#include "loco/foothold_generation/FootholdPlanInvPend.hpp"
#include "loco/foothold_generation/FootholdGeneratorOptimizedInvPend.hpp"

// message logger
#include <message_logger/message_logger.hpp>

using namespace message_logger::color;

namespace loco {

template<typename FootholdOptimizer_, typename FootholdPlan_>
FootPlacementStrategyOptimizedRegain<FootholdOptimizer_, FootholdPlan_>::FootPlacementStrategyOptimizedRegain(
    WholeBody& wholeBody,
    TerrainModelBase& terrain,
    SwingTrajectoryGeneratorModule& swingTrajectoryGenerator,
    FootholdOptimizer_& footholdGenerator,
    ContactScheduleZmp& contactSchedule,
    HeadingGenerator& headingGenerator,
    const TerrainAdapter& terrainAdapter,
    ComSupportControlZmp& comControlZmp,
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
      comControlZmp_(comControlZmp),
      mutexFootholdPlan_(),
      regainPosition_(0.0),
      invPendParamHandler_(),
      scaledSingularityValue_(0.5),
      positionThighToDesiredFootInWorldFrameNormalized_(),
      singularAvoidancePGain_(1.0),
      regainingFootholdPositionFeedback_(1.0),
      regainingFootholdVelocityFeedback_(0.3),
      maxAllowedSingularityValue_(0.8),
      slippageRecoveryStrategy_(fps::SlippageRecoveryStrategy::Undefined),
      regainVelocityInWorldFrame_(),
      regainVelocityFilterTimeConstant_(0.01),
      enableSwingLegOverExtensionAvoidance_(false),
      enableStanceLegOverExtensionAvoidance_(false) {

}

template<typename FootholdOptimizer_, typename FootholdPlan_>
bool FootPlacementStrategyOptimizedRegain<FootholdOptimizer_, FootholdPlan_>::initialize(double dt) {

  for (const auto leg : legs_) {
    const auto legId = static_cast<contact_schedule::LegEnumAnymal>(leg->getId());
    updateSingularityValue(leg);
    regainVelocityInWorldFrame_[legId].setFilterParameters(dt, regainVelocityFilterTimeConstant_, 1.0, LinearVelocity::Zero());
  }

  if(!plan_.initialize(wholeBody_)) {
    MELO_WARN_STREAM("[FootPlacementStrategyOptimizedRegain::initialize] Failed to initialize foothold plan.");
    return false;
  }

  footholdPlanner_.parallelize(true);

  if(!footholdPlanner_.stopPlanning()) {
    MELO_WARN_STREAM("[FootPlacementStrategyOptimizedRegain::initialize] Failed to stop foothold planner.");
    return false;
  }

  if(!getSwingTrajectoryGeneratorModulePtr()->stop()) {
    MELO_WARN_STREAM("[FootPlacementStrategyOptimizedRegain::initialize] Failed to stop swing trajectory generator.");
    return false;
  }

  if(!getSwingTrajectoryGeneratorModulePtr()->initialize(dt)) {
    MELO_WARN_STREAM("[FootPlacementStrategyOptimizedRegain::initialize] Failed to initialize swing trajectory module.");
    return false;
  }

  if(!setFootholdPlan()) {
    MELO_WARN_STREAM("[FootPlacementStrategyOptimizedRegain::initialize] Failed to set foothold plan.");
    return false;
  }


  footholdPlanner_.setModulePlan(plan_);

  if(!footholdPlanner_.initialize(dt, false)) {
    MELO_WARN_STREAM("[FootPlacementStrategyOptimizedRegain::initialize] Failed to initialize foothold planner.");
    return false;
  }

  return true;
}

template<typename FootholdOptimizer_, typename FootholdPlan_>
bool FootPlacementStrategyOptimizedRegain<FootholdOptimizer_, FootholdPlan_>::stop() {
  if(!footholdPlanner_.stopPlanning()) {
    MELO_WARN_STREAM("[FootPlacementStrategyOptimizedRegain] Failed to stop foothold planner");
  }

  if(!getSwingTrajectoryGeneratorModulePtr()->stop()) {
    MELO_WARN_STREAM("[FootPlacementStrategyOptimizedRegain] Failed to stop swing trajectory");
  }

  return true;
}

template<typename FootholdOptimizer_, typename FootholdPlan_>
bool FootPlacementStrategyOptimizedRegain<FootholdOptimizer_, FootholdPlan_>::loadParameters(const TiXmlHandle& handle) {
  std::cout << magenta << "[FootPlacementStrategyOptimizedRegain] " << blue << "Load parameters." << def << std::endl;

  if (!footholdPlanner_.loadParameters(handle)) { return false; }
  if (!swingTrajectoryGenerator_.loadParameters(handle)) { return false; }
  if (!referenceFootholdGenerator_.loadParameters(handle)) { return false; }
  if (!plan_.loadParameters(handle)) { return false; }

  // Regain velocity.
  TiXmlHandle fpsHandle = handle;
  if (!tinyxml_tools::getChildHandle(fpsHandle, handle, "FootPlacementStrategy")) { return false; }

  std::string slippageRecoveryStrategyStr;
  if (!tinyxml_tools::loadParameter(slippageRecoveryStrategyStr, fpsHandle, "slippageRecoveryStrategy")) { return false; }
  if (slippageRecoveryStrategyStr == fps::SlippageRecoveryStrategyMap[fps::SlippageRecoveryStrategy::ApproachToCurrentFoothold]) {
    slippageRecoveryStrategy_ = fps::SlippageRecoveryStrategy::ApproachToCurrentFoothold;
  } else if (slippageRecoveryStrategyStr == fps::SlippageRecoveryStrategyMap[fps::SlippageRecoveryStrategy::ApproachToPreviousFoothold]) {
    slippageRecoveryStrategy_ = fps::SlippageRecoveryStrategy::ApproachToPreviousFoothold;
  } else if (slippageRecoveryStrategyStr == fps::SlippageRecoveryStrategyMap[fps::SlippageRecoveryStrategy::ApproachToPreviousFootholdIteratively]) {
    slippageRecoveryStrategy_ = fps::SlippageRecoveryStrategy::ApproachToPreviousFootholdIteratively;
  } else {
    MELO_FATAL_STREAM("[FootPlacementStrategyOptimizedRegain::loadParameters] Slippage recovery strategy  must be a ApproachToCurrentFoothold, ApproachToPreviousFoothold or ApproachToPreviousFootholdIteratively.");
    return false;
  }

  // Load inverted pendulum parameters.
  const TiXmlHandle fgHandle          = tinyxml_tools::getChildHandle(fpsHandle, "FootholdGenerator");
  const TiXmlHandle fgOptimizedHandle = tinyxml_tools::getChildHandle(fgHandle, "OptimizedInvPend");
  std::vector<TiXmlElement*> gaitElements;
  if(!tinyxml_tools::getChildElements(gaitElements, fgOptimizedHandle, "Gait")) { return false; }
  if(!contact_schedule::loadGaitParameters(gaitElements, invPendParamHandler_, contactSchedule_.getMapGaitNameToId())) { return false; }

  // Regaining.
  TiXmlHandle regainingHandle = handle;
  if (!tinyxml_tools::getChildHandle(regainingHandle, fpsHandle, "Regaining")) { return false; }
  if (!tinyxml_tools::loadParameter(regainPosition_, regainingHandle, "regain_position_for_stride")) { return false; }
  if (!tinyxml_tools::loadParameter(regainingFootholdPositionFeedback_, regainingHandle, "p_gain")) { return false; }
  if (!tinyxml_tools::loadParameter(regainingFootholdVelocityFeedback_, regainingHandle, "d_gain")) { return false; }
  if (!tinyxml_tools::loadParameter(regainVelocityFilterTimeConstant_, regainingHandle, "filter_constant")) { return false; }

  // Singularity avoidance.
  TiXmlHandle singAvoidanceHandle = handle;
  if (!tinyxml_tools::getChildHandle(singAvoidanceHandle, fpsHandle, "SingularityAvoidance")) { return false; }
  if (!tinyxml_tools::loadParameter(enableSwingLegOverExtensionAvoidance_, singAvoidanceHandle, "enable_for_swing_phase")) { return false; }
  if (!tinyxml_tools::loadParameter(enableStanceLegOverExtensionAvoidance_, singAvoidanceHandle, "enable_for_stance_phase")) { return false; }
  if (!tinyxml_tools::loadParameter(maxAllowedSingularityValue_, singAvoidanceHandle, "max_sing_value")) { return false; }
  if (!tinyxml_tools::loadParameter(singularAvoidancePGain_, singAvoidanceHandle, "p_gain")) { return false; }

  return true;
}


template<typename FootholdOptimizer_, typename FootholdPlan_>
bool FootPlacementStrategyOptimizedRegain<FootholdOptimizer_, FootholdPlan_>::advance(double dt) {

  if (!getSwingTrajectoryGeneratorModulePtr()->advance(dt)) {
    MELO_WARN_STREAM("[FootPlacementStrategyOptimizedRegain<FootholdOptimizer_, FootholdPlan_>::advance] Failed to advance swing trajectory generator.");
    return false;
  }

  // Generate an optimized footprint.
  if (!footholdPlanner_.hasStartedPlanning() && footholdPlanner_.hasFinishedPlanning()) {

    if(!setFootholdPlan()) {
      MELO_WARN_STREAM("[FootPlacementStrategyOptimizedRegain::advance] Failed to set foothold plan.");
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

      // Do not update a foothold if leg is close to touch-down.
      // ToDo: use height over ground for this!!
      if(contactSchedule_.shouldBeLegSwing(legId) && legs_.get(legIdInt).getContactSchedule().getSwingPhase() > 0.85) {
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
          regainVelocityInWorldFrame_[legId].reset(leg->getFoot().getStateDesired().getLinearVelocityEndEffectorInWorldFrame());
        } break;

        // Case 3: Stance leg.
        case(LimbStrategyEnum::Support) :
        case(LimbStrategyEnum::ContactInvariant) : {
          setFootOnGround(leg, dt);
          regainVelocityInWorldFrame_[legId].reset(leg->getFoot().getStateDesired().getLinearVelocityEndEffectorInWorldFrame());
        } break;

         default : {
           MELO_WARN_STREAM("[FootPlacementStrategyOptimizedRegain:advance] Unknown State.");
         } break;
      }
  }

  return true;
}

template<typename FootholdOptimizer_, typename FootholdPlan_>
void FootPlacementStrategyOptimizedRegain<FootholdOptimizer_, FootholdPlan_>::setFootTrajectory(LegBase* leg, double dt) {
  const auto legId = static_cast<contact_schedule::LegEnumAnymal>(leg->getId());
  Position positionWorldToFootInWorldFrame;
  LinearVelocity linearVelocityDesFootInWorldFrame;
  LinearAcceleration linearAccelerationDesFootInWorldFrame;

  // Update swing leg trajectory and extract desired foot state.
  computeDesiredFootState(
      positionWorldToFootInWorldFrame,
      linearVelocityDesFootInWorldFrame,
      linearAccelerationDesFootInWorldFrame,
      leg, dt);

  if (enableSwingLegOverExtensionAvoidance_) {
    // Singularity dependent feedback gains.
    const double alpha = boost::math::pow<2>(1.0 - scaledSingularityValue_[legId]);
    const double beta = 1.0 - alpha;

    // Desired linear velocity part 1: Swing leg tracking (with reduce velocity tracking along singular axis).
    if (beta > 1.0e-4) {
      RotationQuaternion orientationWorldToSingularAxis;
      orientationWorldToSingularAxis.setFromVectors<Eigen::Vector3d>(
          -positionThighToDesiredFootInWorldFrameNormalized_[legId],
          Eigen::Vector3d::UnitX()
      );

      LinearVelocity linearVelocityDesFootInSingularFrame = orientationWorldToSingularAxis.rotate(linearVelocityDesFootInWorldFrame);

      // Check if velocity direction would increase leg extension.
      if (linearVelocityDesFootInSingularFrame.x() < 0.0) {
        linearVelocityDesFootInSingularFrame.x() *= alpha;
        linearVelocityDesFootInWorldFrame = orientationWorldToSingularAxis.inverseRotate(linearVelocityDesFootInSingularFrame);
      }
    }

    //  Desired linear velocity part 2: avoid leg over-extension (the closer to singular configuration, the larger the weight)
    if (singularAvoidancePGain_ > 0.0 && beta > 1.0e-4) {
      linearVelocityDesFootInWorldFrame -= singularAvoidancePGain_ * beta * LinearVelocity(positionThighToDesiredFootInWorldFrameNormalized_[legId]);
    }

    // Desired position: Integration of velocity.
    positionWorldToFootInWorldFrame =
        leg->getFoot().getStateDesired().getPositionWorldToEndEffectorInWorldFrame() +
        Position(linearVelocityDesFootInWorldFrame * dt);
  }

  setDesiredFootState(leg,
      positionWorldToFootInWorldFrame,
      linearVelocityDesFootInWorldFrame,
      linearAccelerationDesFootInWorldFrame);
}

template<typename FootholdOptimizer_, typename FootholdPlan_>
void FootPlacementStrategyOptimizedRegain<FootholdOptimizer_, FootholdPlan_>::setDesiredFootState(
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
void FootPlacementStrategyOptimizedRegain<FootholdOptimizer_, FootholdPlan_>::regainContact(LegBase* leg, double dt) {
  const auto legId = static_cast<contact_schedule::LegEnumAnymal>(leg->getId());
  const auto& orientationWorldToControl = torso_.getMeasuredState().inControlFrame().getOrientationWorldToControl();

  // Singularity dependent feedback gains.
  const double alpha = boost::math::pow<2>(1.0 - scaledSingularityValue_[legId]);
  const double beta = 1.0 - alpha;

  // Regain velocity part 1: regain towards ground (the closer to singular configuration, the smaller the regain velocity).
  LinearVelocity desiredLinearVelocityInWorldFrame(0.0, 0.0, -alpha * computeRegainVelocity());


  // Regain velocity part 2: avoid leg over-extension (the closer to singular configuration, the larger the weight)
  if (singularAvoidancePGain_ > 0.0 && beta > 1.0e-4) {
    desiredLinearVelocityInWorldFrame -= singularAvoidancePGain_ * beta * LinearVelocity(positionThighToDesiredFootInWorldFrameNormalized_[legId]);
  }

  // Regain velocity part 3: foot position close to gravity projection of nominal foothold.
  // If end-effector does not move (probably because it is stuck), this objective should  be disabled.
  if (regainingFootholdPositionFeedback_ > 0.0 && leg->getFoot().getStateMeasured().getLinearVelocityEndEffectorInWorldFrame().norm( ) > 0.01) {
    Position positionLimbBaseToNominalFootXYInControlFrame = leg->getLegProperties().getDesiredDefaultSteppingPositionHipToFootInControlFrame();
    positionLimbBaseToNominalFootXYInControlFrame.z() = 0.0;

    const Position positionWorldToNominalFootholdInWorldFrame =
        leg->getLimbStateMeasured().getPositionWorldToLimbBaseInWorldFrame() +
        orientationWorldToControl.inverseRotate(positionLimbBaseToNominalFootXYInControlFrame);
    const Position positionErrorInWorldFrame = positionWorldToNominalFootholdInWorldFrame - leg->getFoot().getStateDesired().getPositionWorldToEndEffectorInWorldFrame();

    desiredLinearVelocityInWorldFrame += regainingFootholdPositionFeedback_ * LinearVelocity(positionErrorInWorldFrame.x(), positionErrorInWorldFrame.y(), 0.0);
  }

  // Regain velocity part 4: compensation for torso velocity error.
  if (regainingFootholdVelocityFeedback_ > 0.0) {
    const auto linearVelocityDesiredInWorldFrame  = orientationWorldToControl.inverseRotate(torso_.getDesiredState().getLinearVelocityTargetInControlFrame());
    const auto& linearVelocityMeasuredInWorldFrame = leg->getLimbStateMeasured().getLinearVelocityLimbBaseInWorldFrame();
    auto linearVelocityErrorInWorldFrame = linearVelocityMeasuredInWorldFrame - linearVelocityDesiredInWorldFrame;

    desiredLinearVelocityInWorldFrame += regainingFootholdVelocityFeedback_ * LinearVelocity(linearVelocityErrorInWorldFrame.x(), linearVelocityErrorInWorldFrame.y(), 0.0);
  }

  // Regain velocity part 5 (Safety): Regain velocity needs to have a negative vertical velocity.
  desiredLinearVelocityInWorldFrame.z() = std::fmin(desiredLinearVelocityInWorldFrame.z(), -1.0e-4);

  // Filter regain velocity.
  const LinearVelocity desiredFilteredLinearVelocityInWorldFrame = (regainVelocityFilterTimeConstant_ > 0.0 ?
      regainVelocityInWorldFrame_[legId].advance(desiredLinearVelocityInWorldFrame) :
      desiredLinearVelocityInWorldFrame
  );

  // Regain position part 1: obtained by integrating velocity.
  Position positionWorldToFootInWorldFrame =
      leg->getFoot().getStateDesired().getPositionWorldToEndEffectorInWorldFrame() +
      Position(desiredFilteredLinearVelocityInWorldFrame * dt);

  // Regain position part 2 (safety): In case leg is stuck, make sure desired position does not diverge.
  const Position positionPreviousFootToNextFootInWorldFrame = positionWorldToFootInWorldFrame - leg->getFoot().getStateDesired().getPositionWorldToEndEffectorInWorldFrame();
  const double positionUpdateDistance = positionPreviousFootToNextFootInWorldFrame.norm();
  constexpr double maxAllowedEndEffectorDivergence = 0.1;
  if (positionUpdateDistance > maxAllowedEndEffectorDivergence) {
    positionWorldToFootInWorldFrame =
        leg->getFoot().getStateDesired().getPositionWorldToEndEffectorInWorldFrame() +
        positionPreviousFootToNextFootInWorldFrame / positionUpdateDistance * maxAllowedEndEffectorDivergence;
  }

  // Regain acceleration: Obtained by first order derivative of regain velocity.
  LinearAcceleration linearAccelerationDesiredFootInWorldFrame;
  if (regainVelocityFilterTimeConstant_ > 0.0) {
    linearAccelerationDesiredFootInWorldFrame = static_cast<LinearAcceleration>(
        desiredFilteredLinearVelocityInWorldFrame -
        leg->getFoot().getStateDesired().getLinearVelocityEndEffectorInWorldFrame()
    ) / dt;
  } else {
    linearAccelerationDesiredFootInWorldFrame.setZero();
  }

  setDesiredFootState(leg,
      positionWorldToFootInWorldFrame,
      desiredFilteredLinearVelocityInWorldFrame,
      linearAccelerationDesiredFootInWorldFrame);
}

template<typename FootholdOptimizer_, typename FootholdPlan_>
void FootPlacementStrategyOptimizedRegain<FootholdOptimizer_, FootholdPlan_>::setFootOnGround(LegBase* leg, double dt) {
  const auto legId = static_cast<contact_schedule::LegEnumAnymal>(leg->getId());
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
        MELO_FATAL_STREAM("[FootPlacementStrategyOptimizedRegain::setFootOnGround] Unknown slippage recovery strategy.");
      } break;
    }
  }
  else {
    positionWorldToFootInWorldFrame = leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();
    linearVelocityFootInWorldFrame.setZero();
  }


  const Position positionWorldToOriginalFootInWorldFrame = positionWorldToFootInWorldFrame;
  const LinearVelocity originalLinearVelocityFootInWorldFrame = linearVelocityFootInWorldFrame;
  if (enableStanceLegOverExtensionAvoidance_) {
    // Singularity dependent feedback gains.
    const double alpha = boost::math::pow<2>(1.0 - scaledSingularityValue_[legId]);
    const double beta = 1.0 - alpha;

    //  Desired linear velocity part 2: avoid leg over-extension (the closer to singular configuration, the larger the weight)
    if (singularAvoidancePGain_ > 0.0 && beta > 1.0e-4) {
      linearVelocityFootInWorldFrame -= singularAvoidancePGain_ * beta * LinearVelocity(positionThighToDesiredFootInWorldFrameNormalized_[legId]);
    }

    // Desired position: Integration of velocity.
    positionWorldToFootInWorldFrame =
        leg->getFoot().getStateDesired().getPositionWorldToEndEffectorInWorldFrame() +
        Position(linearVelocityFootInWorldFrame * dt);
  }

  setDesiredFootState(
      leg,
      positionWorldToFootInWorldFrame,
      linearVelocityFootInWorldFrame,
      linearAccelerationFootInWorldFrame);
}

template<typename FootholdOptimizer_, typename FootholdPlan_>
Position FootPlacementStrategyOptimizedRegain<FootholdOptimizer_, FootholdPlan_>::getPositionProjectedOnPlaneAlongSurfaceNormal(const Position& position) {
  return terrain_.getPositionProjectedOnPlaneAlongSurfaceNormalInWorldFrame(position);
}

template<typename FootholdOptimizer_, typename FootholdPlan_>
void FootPlacementStrategyOptimizedRegain<FootholdOptimizer_, FootholdPlan_>::computeDesiredFootState(
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
    MELO_WARN("[FootPlacementStrategyOptimizedRegain::computeDesiredFootState] Could not get desired foot state from swing trajectory generator!");
    positionWorldToDesiredFootInWorldFrame = getPositionProjectedOnPlaneAlongSurfaceNormal(leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame());
    linearVelocityDesiredFootInWorldFrame.setZero();
    linearAccelerationDesiredFootInWorldFrame.setZero();
  }
}

template<typename FootholdOptimizer_, typename FootholdPlan_>
bool FootPlacementStrategyOptimizedRegain<FootholdOptimizer_, FootholdPlan_>::setToInterpolated(
    const FootPlacementStrategyBase& footPlacementStrategy1,
    const FootPlacementStrategyBase& footPlacementStrategy2, double t) {
  return true;
}

template<typename FootholdOptimizer_, typename FootholdPlan_>
const Legs& FootPlacementStrategyOptimizedRegain<FootholdOptimizer_, FootholdPlan_>::getLegs() const {
  return legs_;
}

template<typename FootholdOptimizer_, typename FootholdPlan_>
const SwingTrajectoryGeneratorBase& FootPlacementStrategyOptimizedRegain<FootholdOptimizer_, FootholdPlan_>::getSwingTrajectoryGenerator() const {
  return swingTrajectoryGenerator_;
}

template<typename FootholdOptimizer_, typename FootholdPlan_>
SwingTrajectoryGeneratorBase* FootPlacementStrategyOptimizedRegain<FootholdOptimizer_, FootholdPlan_>::getSwingTrajectoryGeneratorPtr() {
  return &swingTrajectoryGenerator_;
}

template<typename FootholdOptimizer_, typename FootholdPlan_>
bool FootPlacementStrategyOptimizedRegain<FootholdOptimizer_, FootholdPlan_>::setFootholdPlan() {
  boost::unique_lock<boost::shared_mutex> lock(mutexFootholdPlan_);

  // Set up foothold plan.
  if(!plan_.updateData(wholeBody_, terrain_, contactSchedule_, headingGenerator_, comControlZmp_)) {
    MELO_WARN_STREAM("[FootPlacementStrategyOptimizedRegain::setFootholdPlan] Failed to update data.");
    return false;
  }

  return true;
}

template<>
bool FootPlacementStrategyOptimizedRegain<FootholdGeneratorOptimizedInvPend, foothold_generator::FootholdPlanInvPend>::setFootholdPlan() {
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
void FootPlacementStrategyOptimizedRegain<FootholdOptimizer_, FootholdPlan_>::getFootholdPlan(FootholdPlan_& plan) const {
  boost::shared_lock<boost::shared_mutex> lock(mutexFootholdPlan_);
  plan.copy(plan_);
}

template<typename FootholdOptimizer_, typename FootholdPlan_>
void FootPlacementStrategyOptimizedRegain<FootholdOptimizer_, FootholdPlan_>::updateSingularityValue(LegBase* leg) {
  const auto legId = static_cast<contact_schedule::LegEnumAnymal>(leg->getId());

  const Position positionBaseToThighInWorldFrame = torso_.getMeasuredState().getOrientationWorldToBase().inverseRotate(
      leg->getLimbStateMeasured().getPositionBaseToLimbBaseInBaseFrame()
  );

  const Position positionWorldToThighInWorldFrame =
      torso_.getMeasuredState().getPositionWorldToBaseInWorldFrame() +
      positionBaseToThighInWorldFrame;

  const Position  positionThighToDesiredFootInWorldFrame =
      leg->getFoot().getStateDesired().getPositionWorldToEndEffectorInWorldFrame() -
      positionWorldToThighInWorldFrame;

  /*
   * Compute singularity value:
   *   0 if extension reaches kinematic minimum,
   *   1 if extension reaches kinematic maximum.
   * Leg extension is the difference between the limb base and the previous desired foothold.
   */
  const double maxLegExtension = leg->getLegProperties().getMaximumLimbExtension();
  const double minLegExtension = leg->getLegProperties().getMinimumLimbExtension();
  const double legExtension = positionThighToDesiredFootInWorldFrame.norm();
  double singularityValue = (legExtension - minLegExtension) / (maxLegExtension - minLegExtension);
  robot_utils::boundToRange(&singularityValue, 0.0, 1.0);

  if (legExtension > 1.0e-4) {
    positionThighToDesiredFootInWorldFrameNormalized_[legId] = positionThighToDesiredFootInWorldFrame.toImplementation()/legExtension;
  }

  /*
   * Compute scaled singularity value:
   *  0 if singularity value < threshold (safe limb extension)
   *  1 if singularity value = 1 (max limb extension)
   */
  scaledSingularityValue_[legId] = (singularityValue - maxAllowedSingularityValue_) / (1.0 - maxAllowedSingularityValue_);
  robot_utils::boundToRange(&scaledSingularityValue_[legId], 0.0, 1.0);
}

template<typename FootholdOptimizer_, typename FootholdPlan_>
double FootPlacementStrategyOptimizedRegain<FootholdOptimizer_, FootholdPlan_>::computeRegainVelocity() const {
  return (contactSchedule_.getNominalStrideDuration() > 1.0e-3 ?
      regainPosition_/contactSchedule_.getNominalStrideDuration() :
      regainPosition_
  );
}


} /* namespace loco */
