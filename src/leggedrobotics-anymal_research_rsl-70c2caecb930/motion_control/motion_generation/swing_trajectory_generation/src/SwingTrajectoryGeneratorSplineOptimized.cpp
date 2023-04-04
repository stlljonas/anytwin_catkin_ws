/*
 * SwingTrajectoryGeneratorSplineOptimized.cpp
 *
 *  Created on: Jan. 15, 2017
 *      Author: Fabian Jenelten
 */

// anymal_description
#include <anymal_description/LegEnum.hpp>

// swing trajectory generation
#include "swing_trajectory_generation/SwingTrajectoryGeneratorSplineOptimized.hpp"

// robot utils
#include "robot_utils/math/math.hpp"

// message logger
#include <message_logger/message_logger.hpp>


namespace loco {

SwingTrajectoryGeneratorSplineOptimized::SwingTrajectoryGeneratorSplineOptimized(
    WholeBody& wholeBody,
    TerrainModelBase& terrain,
    contact_schedule::ContactScheduleAnymalBase& contactSchedule) :
  wholeBody_(wholeBody),
  terrain_(terrain),
  contactSchedule_(contactSchedule),
  swingTrajectoryOptimizer_(),
  motionPlan_(wholeBody_.getLegs().size()),
  mutexMotionPlan_(),
  finalHeightPositionOffset_(0.0),
  finalHeightVelocityOffset_(0.0),
  initWithMeasuredFootState_(false),
  verbose_(true),
  optStatus_(wholeBody_.getLegs().size(), stg::OptStatus::no_plan_available),
  upperSwingPhaseThreshold_(1.0),
  lowerSwingTimeThreshold_(0.0),
  positionWorldToDesiredFootInWorldFrame_(wholeBody_.getLegs().size()),
  linearVelocityDesiredFootInWorldFrame_(wholeBody_.getLegs().size()),
  linearAccelerationDesiredFootInWorldFrame_(wholeBody_.getLegs().size())
{

}

bool SwingTrajectoryGeneratorSplineOptimized::initialize(double dt) {
  // Initialize motion plan.
  for (unsigned int legId=0u; legId<wholeBody_.getLegs().size(); ++legId) {
    if(!motionPlan_[legId].initialize()) { return false; }
    optStatus_[legId] = stg::OptStatus::no_plan_available;
    linearVelocityDesiredFootInWorldFrame_[legId].setZero();
    linearAccelerationDesiredFootInWorldFrame_[legId].setZero();
    positionWorldToDesiredFootInWorldFrame_[legId] = wholeBody_.getLegs().get(legId).getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();
  }

  // Initialize optimizer.
  swingTrajectoryOptimizer_.parallelize(true);
  if(!swingTrajectoryOptimizer_.stopPlanning()) { return false; }
  if(!swingTrajectoryOptimizer_.initialize(dt, wholeBody_.getLegs().size(), false)) { return false; }

  // Timings.
  lowerSwingTimeThreshold_ = 1.0*dt;

  return true;
}

bool SwingTrajectoryGeneratorSplineOptimized::stop() {
  return swingTrajectoryOptimizer_.stopPlanning();
}

bool SwingTrajectoryGeneratorSplineOptimized::loadParameters(const TiXmlHandle& handle) {
  bool success = true;

  // Swing trajectory generator.
  const TiXmlHandle footPlacementStrategyHandle = tinyxml_tools::getChildHandle(handle, "FootPlacementStrategy");
  const TiXmlHandle swingTrajectoryHandle = tinyxml_tools::getChildHandle(footPlacementStrategyHandle, "SwingTrajectoryGenerator");
  success &= tinyxml_tools::loadParameter(verbose_, swingTrajectoryHandle, "verbose", false);

  const TiXmlHandle optimizedHandle = tinyxml_tools::getChildHandle(swingTrajectoryHandle, "Optimized");

  // Conditioning.
  const TiXmlHandle conditioningHandle = tinyxml_tools::getChildHandle(optimizedHandle, "Conditioning");
  success &= tinyxml_tools::loadParameter(upperSwingPhaseThreshold_, conditioningHandle, "skip_optimization_if_swing_phase_larger_than", 1.0);

  // Equality constraints
  const TiXmlHandle eqConstraintsHandle = tinyxml_tools::getChildHandle(optimizedHandle, "EqualityConstraints");
  success &= tinyxml_tools::loadParameter(finalHeightPositionOffset_, eqConstraintsHandle, "final_pos", 0.0);
  success &= tinyxml_tools::loadParameter(finalHeightVelocityOffset_, eqConstraintsHandle, "final_vel", 0.0);
  success &= tinyxml_tools::loadParameter(initWithMeasuredFootState_, eqConstraintsHandle, "init_with_foot_state", false);

  // Load parameters for trajectory optimizer.
  success &= swingTrajectoryOptimizer_.loadParameters(optimizedHandle);

  return success;
}


bool SwingTrajectoryGeneratorSplineOptimized::getDesiredFootState(
    Position& positionWorldToDesiredFootInWorldFrame,
    LinearVelocity& linearVelocityDesiredFootInWorldFrame,
    LinearAcceleration& linearAccelerationDesiredFootInWorldFrame,
    const Position& positionWorldToDesiredFootholdInControlFrame,
    LegBase* leg, double dt)
{
  positionWorldToDesiredFootInWorldFrame    = positionWorldToDesiredFootInWorldFrame_[leg->getId()];
  linearVelocityDesiredFootInWorldFrame     = linearVelocityDesiredFootInWorldFrame_[leg->getId()];
  linearAccelerationDesiredFootInWorldFrame = linearAccelerationDesiredFootInWorldFrame_[leg->getId()];
  return true;
}

bool SwingTrajectoryGeneratorSplineOptimized::advance(double dt) {

  // If the optimization has finished --> Read the solution
  if (swingTrajectoryOptimizer_.hasStartedPlanning() && swingTrajectoryOptimizer_.hasFinishedPlanning()) {
    swingTrajectoryOptimizer_.setHasStartedPlanning(false);

    if (swingTrajectoryOptimizer_.getDidOptimizationSuceeded()) {
      bool success = true;

      // Copy the motion plan.
      {
        boost::unique_lock<boost::shared_mutex> lock(mutexMotionPlan_);
        swingTrajectoryOptimizer_.getMotionPlans(motionPlan_);

        for (unsigned int legId=0u; legId<wholeBody_.getLegs().size(); ++legId) {
          // Check if we have received a new motion plan for any leg.
          if (swingTrajectoryOptimizer_.isOptimizedLegIndex(legId)) {
            optStatus_[legId] = stg::OptStatus::success;

            // Add final stance phase to optimized swing trajectory
            if(!motionPlan_[legId].addStancePhase()) {
              MELO_WARN_STREAM("[SwingTrajectoryGeneratorSplineOptimized::advance] Failed to add stance phase");
              success = false;
            }

            // Compute container time.

            //ToDO: is is real robot.
            double optimalContainerTime = 0.0;
            if (!initWithMeasuredFootState_) {
              optimalContainerTime = motionPlan_[legId].getTimeSpendInSwing() + swingTrajectoryOptimizer_.getLastComputationDuration();
            }
            else if (!wholeBody_.getLegs().get(legId).getContactSchedule().shouldBeGrounded()) {
              const Position positionPlaneToEndEffectorInPlaneFrame  =
                  motionPlan_[legId].getVirtualPlaneFrame().getPosePlaneToWorld().inverseTransform(
                      wholeBody_.getLegs().get(legId).getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame()
                  );
              optimalContainerTime = motionPlan_[legId].getSwingTrajectoryInPlaneFramePtr()->lineSearch(
                  swingTrajectoryOptimizer_.getLastComputationDuration(),
                  positionPlaneToEndEffectorInPlaneFrame, 0.0, 0.5*motionPlan_[legId].getOptimizationHorizon()
              );
            }

            // Set container time.
            if(!motionPlan_[legId].getSwingTrajectoryInPlaneFramePtr()->setContainerTime(optimalContainerTime)) {
              MELO_WARN_STREAM("[SwingTrajectoryGeneratorSplineOptimized::advance] Failed to set optimal container time");
              success = false;
              optStatus_[legId] = stg::OptStatus::init_failed;
            }
          }
        }
      }

      if (!success) { return false; } // return after mutex is locked!

      // Display computation duration.
      if (verbose_) {
        MELO_INFO_THROTTLE_STREAM(1.0, "[SwingTrajectoryGeneratorSplineOptimized::getDesiredFootState] Optimization time [ms] "
            << swingTrajectoryOptimizer_.getLastComputationDuration()*1000.0 << ".");
      }
    }
  }

  // If previous optimization has finished we are ready for a new update.
  if (!swingTrajectoryOptimizer_.hasStartedPlanning() && swingTrajectoryOptimizer_.hasFinishedPlanning()) {
    planTrajectory();
  }

  for (unsigned int legId=0u; legId<wholeBody_.getLegs().size(); ++legId) {
    if(!processReferenceSignalsForLeg(dt, legId)) {
      MELO_FATAL_STREAM("[SwingTrajectoryGeneratorSplineOptimized::advance] Failed to process reference signals.");
      return false;
    }
  }

  return true;
}

bool SwingTrajectoryGeneratorSplineOptimized::planTrajectory() {
  sto::MotionPlan motionPlan;
  swingTrajectoryOptimizer_.resetOptimizedLegIndexes();
  unsigned int numOfSuccessfullyAddedMotionPlans = 0u;

  for (const auto legId : anymal_description::LegEnumIterator()) {
    if (contactSchedule_.getEventHorizonStatus(legId) == contact_schedule::EventHorizonStatus::None ||
        contactSchedule_.getEventHorizonStatus(legId) == contact_schedule::EventHorizonStatus::LiftOff) {
      continue;
    }

    // Copy previous motion plan.
    {
      boost::shared_lock<boost::shared_mutex> lock(mutexMotionPlan_);
      motionPlan.copy(motionPlan_[static_cast<unsigned int>(legId)]);
    }

    // Initialize motion plan with measurements and copy it to the planner.
    if (setMotionPlan(motionPlan, static_cast<unsigned int>(legId))) {
      ++numOfSuccessfullyAddedMotionPlans;
      swingTrajectoryOptimizer_.setMotionPlan(motionPlan, static_cast<unsigned int>(legId));
    }
  }

  // Run the optimization
  if (numOfSuccessfullyAddedMotionPlans>0u) {
    swingTrajectoryOptimizer_.startPlanning();
  }

  return true;
}

void SwingTrajectoryGeneratorSplineOptimized::getMotionPlan(std::vector<sto::MotionPlan>& motionPlan) const {
  boost::shared_lock<boost::shared_mutex> lock(mutexMotionPlan_);
  motionPlan = motionPlan_;
}


bool SwingTrajectoryGeneratorSplineOptimized::setMotionPlan(sto::MotionPlan& motionPlan, unsigned int legId) {
  bool success = true;
  const auto& leg = wholeBody_.getLegs().get(legId);

  // Get timing informations.
  double phaseSpendInSwing    = leg.getContactSchedule().getSwingPhase();
  const double swingDuration  = leg.getContactSchedule().getSwingDuration();
  const double stanceDuration = std::fmax(leg.getContactSchedule().getStanceDuration(), 0.0);

  // Leg is still in stance mode.
  if (robot_utils::areNear(phaseSpendInSwing, -1.0)) {
    phaseSpendInSwing = 0.0;
  }

  // Incorrect times.
  if (phaseSpendInSwing<0.0 || swingDuration<0.0) {
    MELO_WARN_STREAM("[SwingTrajectoryGeneratorSplineOptimized::isFootLiftOff] Negative time!");
    return false;
  }

  const double timeSpendInSwing    = phaseSpendInSwing*swingDuration;
  const double trajectoryStartTime = (initWithMeasuredFootState_ ? timeSpendInSwing : 0.0);
  const double optimizationHorizon = swingDuration-trajectoryStartTime;


  /*
   * Don't optimize for a new trajectory if
   *  > foot is close to a touch down (by this we prevent issues based on wrong contact detection), or
   *  > Optimization horizon would be too small, or
   *  > robot is standing (swingDuration = 0)
   */
  if (phaseSpendInSwing>upperSwingPhaseThreshold_ ||
      optimizationHorizon<lowerSwingTimeThreshold_ ||
      robot_utils::areNear(swingDuration, 0.0)) {
    return false;
  }


  // Check if the foot has lift-off since the previous optimization.
  double previousTimeSpendInSwing = 0.0;
  {
    boost::shared_lock<boost::shared_mutex> lock(mutexMotionPlan_);
    previousTimeSpendInSwing = motionPlan_[legId].getTimeSpendInSwing();
  }

  const bool isFootLiftOff = (
      (previousTimeSpendInSwing > timeSpendInSwing && phaseSpendInSwing<0.5) ||
      (previousTimeSpendInSwing < 0.0)
  );


  // Get orientations.
  const RotationQuaternion& orientationWorldToBase    = wholeBody_.getTorso().getMeasuredState().getOrientationWorldToBase();
  const RotationQuaternion& orientationWorldToControl = wholeBody_.getTorso().getMeasuredState().inControlFrame().getOrientationWorldToControl();

  // Compute start and end positions.
  const Position positionWorldToDesiredFootholdInWorldFrame          = leg.getFoot().getStateDesired().getPositionWorldToFootholdInWorldFrame();
  const Position positionWorldToCorrectedDesiredFootholdInWorldFrame = positionWorldToDesiredFootholdInWorldFrame + Position::UnitZ()*finalHeightPositionOffset_;
  const Position& positionWorldToPreviousStanceFootholdInWorldFrame  = leg.getPositionWorldToLastOrCurrentContactInWorldFrame();
  const Position& positionWorldToEndEffectorInWorldFrame             = leg.getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();

  // Compute initial acceleration (Note: before resetting the virtual plane frame!).
  LinearAcceleration accelerationEndEffectorInWorldFrame;
  if (isFootLiftOff) { accelerationEndEffectorInWorldFrame.setZero(); }
  else { motionPlan.getLinearAccelerationDesiredFootInWorldFrame(accelerationEndEffectorInWorldFrame); }


  /**********************
   * Set up Motion plan *
   **********************/
  // Set virtual plane frame (after reading the acceleration!!).
  if(!motionPlan.setVirtualPlaneFrame(
      positionWorldToPreviousStanceFootholdInWorldFrame,
      wholeBody_.getTorso().getMeasuredState().inControlFrame().getOrientationWorldToControl())
  ) {
    return false;
  }

  const loco::Pose& posePlaneToWorld = motionPlan.getVirtualPlaneFrame().getPosePlaneToWorld();
  const loco::RotationQuaternion& orientationPlaneToWorld = posePlaneToWorld.getRotation();

  // Set initial conditions.
  if (initWithMeasuredFootState_) {

    LinearVelocity velocityEndEffectorInWorldFrame = orientationWorldToBase.inverseRotate(
          static_cast<LinearVelocity>(
              leg.getFoot().getStateMeasured().getTranslationJacobianBaseToEndEffectorInBaseFrame()*
              leg.getLimbStateMeasured().getJointVelocities()
          ) +
          wholeBody_.getTorso().getMeasuredState().getLinearVelocityBaseInBaseFrame()
    );

    // Add angular component
    const Position positionBaseToFoodInWorldFrame =
        positionWorldToEndEffectorInWorldFrame-
        wholeBody_.getTorso().getMeasuredState().getPositionWorldToBaseInWorldFrame();
    const LocalAngularVelocity angualarVelocityBaseInWorldFrame = orientationWorldToBase.inverseRotate(wholeBody_.getTorso().getMeasuredState().getAngularVelocityBaseInBaseFrame());
    velocityEndEffectorInWorldFrame += static_cast<LinearVelocity>(angualarVelocityBaseInWorldFrame.toImplementation().cross(positionBaseToFoodInWorldFrame.toImplementation()));

    motionPlan.setPlaneToInitialPositionInPlaneFrame(posePlaneToWorld.inverseTransform(positionWorldToEndEffectorInWorldFrame));
    motionPlan.setInitialVelocityInPlaneFrame(orientationPlaneToWorld.inverseRotate(velocityEndEffectorInWorldFrame));
    motionPlan.setInitialAccelerationInPlaneFrame(orientationPlaneToWorld.inverseRotate(accelerationEndEffectorInWorldFrame));
  } else {
    motionPlan.setPlaneToInitialPositionInPlaneFrame(posePlaneToWorld.inverseTransform(positionWorldToPreviousStanceFootholdInWorldFrame));
  }

  // Set final conditions.
  motionPlan.setPlaneToFinalPositionInPlaneFrame(posePlaneToWorld.inverseTransform(positionWorldToCorrectedDesiredFootholdInWorldFrame));
  motionPlan.setFinalVelocityInPlaneFrame(orientationPlaneToWorld.inverseRotate(LinearVelocity(0.0, 0.0, finalHeightVelocityOffset_)));

  // Set timing informations.
  const bool isPreviousOptimizationAvailable = !isFootLiftOff && optStatus_[legId]==stg::OptStatus::success;
  motionPlan.setTimingInformations(swingDuration, stanceDuration, timeSpendInSwing, trajectoryStartTime, isPreviousOptimizationAvailable);
  /**********************/

  return success;
}

bool SwingTrajectoryGeneratorSplineOptimized::processReferenceSignalsForLeg(double timeToAdvance, unsigned int legId) {
  if (optStatus_[legId] == stg::OptStatus::success && !wholeBody_.getLegs().get(legId).getContactSchedule().shouldBeGrounded()) {
    bool success = true;

    // Advance motion plan.
    {
      boost::unique_lock<boost::shared_mutex> lock(mutexMotionPlan_);
      if (!motionPlan_[legId].getSwingTrajectoryInPlaneFramePtr()->advance(timeToAdvance, false)) {
        optStatus_[legId] = stg::OptStatus::end_of_container;
        success = false;
      }
    }

    // Read reference signals.
    if (success) {
      boost::shared_lock<boost::shared_mutex> lock(mutexMotionPlan_);
      motionPlan_[legId].getPositionWorldToDesiredFootInWorldFrame(positionWorldToDesiredFootInWorldFrame_[legId]);
      motionPlan_[legId].getLinearVelocityDesiredFootInWorldFrame(linearVelocityDesiredFootInWorldFrame_[legId]);
      motionPlan_[legId].getLinearAccelerationDesiredFootInWorldFrame(linearAccelerationDesiredFootInWorldFrame_[legId]);
      return true;
    } else {
      MELO_WARN_STREAM("[SwingTrajectoryGeneratorSplineOptimized::processReferenceSignalsForLeg] Failed to advance on swing trajectory for leg " << legId << ".");
    }
  }

  //! We don't have any motion plan available (or we we should be grounded).
  const Position& positionWorldToEndEffectorInWorldFrame = wholeBody_.getLegs().get(legId).getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();
  positionWorldToDesiredFootInWorldFrame_[legId] = terrain_.getPositionProjectedOnPlaneAlongSurfaceNormalInWorldFrame(positionWorldToEndEffectorInWorldFrame);
  linearVelocityDesiredFootInWorldFrame_[legId].setZero();
  linearAccelerationDesiredFootInWorldFrame_[legId].setZero();
  return true;

}




} /* namespace loco */
