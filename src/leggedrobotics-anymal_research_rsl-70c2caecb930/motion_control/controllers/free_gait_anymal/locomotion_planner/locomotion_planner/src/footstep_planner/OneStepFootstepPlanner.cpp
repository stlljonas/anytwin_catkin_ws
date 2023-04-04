/*
 * OneStepFootstepPlanner.cpp
 *
 *  Created on: Mar 6, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include "locomotion_planner/footstep_planner/OneStepFootstepPlanner.hpp"
#include "locomotion_planner/common/type_prints.hpp"
#include "locomotion_planner/common/geometry.hpp"

#include <free_gait_core/free_gait_core.hpp>
#include <message_logger/message_logger.hpp>
#include <robot_utils/surfaces/Ellipsoid.hpp>

#include <cmath>

namespace locomotion_planner {

OneStepFootstepPlanner::OneStepFootstepPlanner(const free_gait::AdapterBase& adapter, Parameters& parameters, GaitPatternBase& gaitPattern)
    : FootstepPlannerBase(adapter),
      parameters_(parameters),
      speedFactor_(0.0),
      gaitPattern_(gaitPattern),
      lastCycleToGoalCounter_(0),
      nStepsPerCycle_(4) // TODO Read from parameters.
{
}

OneStepFootstepPlanner::~OneStepFootstepPlanner()
{
}

bool OneStepFootstepPlanner::plan(const Pose& goalPose, const Transform& goalToWorldFrameTransform, const double speedFactor, const bool isNewGoal,
                                  const bool allowIntermediatePoses, std::vector<Step>& plan, bool& lastStepToGoal)
{
  MELO_DEBUG("------------ Starting to plan footsteps ------------");
  goalToWorldFrameTransform_ = goalToWorldFrameTransform;
  plan.clear();

  if (isNewGoal) {
    goalPoses_.clear();
    goalPoses_.push_back(goalPose);
    goalPoses_.front().getRotation().setUnique();
    speedFactor_ = speedFactor;
    lastCycleToGoalCounter_ = 0;
  }

  determineStartFootprintAndPose();
  computePoseDifference();

  if (isNewGoal && allowIntermediatePoses) {
    computeIntermediateGoals();
    computePoseDifference();
  }

  if (lastCycleToGoalCounter_ > 0 && goalPoses_.size() > 1) {
    MELO_DEBUG("Approaching intermediate goal pose.");
    goalPoses_.erase(goalPoses_.begin());
    computePoseDifference();
  }

  bool directionChanged = determineDirection();
  MELO_DEBUG_STREAM("Direction changed: " << (directionChanged ? "True" : "False"));
  if (gaitPattern_.getDirection() == GaitPatternBase::Direction::UNDEFINED) {
    MELO_WARN_STREAM("Undefined gait pattern direction.");
    return false;
  }

  updateStepPose(false);
  if (!determineNextSwingLeg(directionChanged)) {
    MELO_WARN("Could not determine next swing leg.");
    return false;
  }

  std::unique_ptr<free_gait::Footstep> footstep;
  double stepLength = std::nan("");
  const auto previousLimb = gaitPattern_.getCurrentLimb();
  while (lastCycleToGoalCounter_ < nStepsPerCycle_ + 1) {
    updateStepPose(false);
    footstep.reset(new free_gait::Footstep(computeNextFootstep()));
    free_gait::StepQueue previousPlan;
    previousPlan.add(previousPlan_);
    const Vector stepVector(footstep->getTargetPosition()
            - getFootPositionAfterActiveStep(gaitPattern_.getCurrentLimb(), adapter_, previousPlan));
    stepLength = stepVector.vector().head(2).norm(); // We are only interested in xy-plane.
    if (stepLength < parameters_.getSkipStepThreshold()) {
      MELO_DEBUG_STREAM("Skipping leg: " << gaitPattern_.getCurrentLimb() << " (step length " << stepLength << " m).");
      gaitPattern_.advance();
      if (gaitPattern_.getCurrentLimb() == previousLimb) {
        MELO_DEBUG_STREAM("Breaking cycle.");
        gaitPattern_.advance();
        break;
      }
    } else {
      break;
    }
  }

  if (lastCycleToGoalCounter_ > nStepsPerCycle_) {
    MELO_DEBUG("Already at goal pose!");
    lastStepToGoal = true;
    return true;
  }

  MELO_DEBUG_STREAM("Next leg: " << gaitPattern_.getCurrentLimb());
  MELO_DEBUG_STREAM("Step length: " << stepLength);

  Step stepBaseAuto;
  free_gait::BaseAuto baseAuto;
  parameters_.populateBaseAutoParameters(speedFactor_, baseAuto);
  stepBaseAuto.addBaseMotion(baseAuto);
  plan.push_back(stepBaseAuto);
  Step step;
  step.addLegMotion(*footstep);
  step.addBaseMotion(baseAuto);
  plan.push_back(step);
  plan.push_back(stepBaseAuto);

  previousPlan_ = plan;
  if (nStepsPerCycle_ <= lastCycleToGoalCounter_) {
    MELO_DEBUG("Last step to goal computed.");
    lastStepToGoal = true;
  } else {
    lastStepToGoal = false;
  }
  MELO_DEBUG("Finished one step planner successfully.");
  return true;
}

double OneStepFootstepPlanner::getExpectedUpperPlanningTime() const
{
  return 0.1;
}

void OneStepFootstepPlanner::stop()
{
}

void OneStepFootstepPlanner::computeIntermediateGoals()
{
  RotationVector rotationToGoalInStartFrame;
  const Vector preferredDirection(Eigen::Vector3d::UnitX()); // TODO Make this a parameter.
  rotationToGoalInStartFrame.setFromVectors(
      preferredDirection.toImplementation(),
      Vector(goalPoseInStartFrame_.x(), goalPoseInStartFrame_.y(), 0.0).toImplementation());

  const double distanceToGoal = goalPoseInStartFrame_.matrix().head(2).norm();
  MELO_DEBUG_STREAM("Distance to goal: " << distanceToGoal << " m.");

  // Check if intermediate poses are needed.
  if (distanceToGoal <= parameters_.getMaxUnpreferredDirectionDistance()) return;
  const double rotationTolerance = 0.35;
  if (fabs(goalPoseInStartFrame_.z()) + fabs(rotationToGoalInStartFrame.z()) < rotationTolerance) {
    MELO_DEBUG_STREAM("Rotation is within the limits.");
    return;
  }

  // Compute intermediate poses.
  MELO_DEBUG_STREAM("Adding intermediate poses.");
  const PlanarPose goalPlanarPoseInGoalFrame = getPlanarPoseFromPose(goalPoses_.front());
  const Transform worldToGoalFrameTransform(
      -goalToWorldFrameTransform_.getRotation().inverseRotate(goalToWorldFrameTransform_.getPosition()),
      goalToWorldFrameTransform_.getRotation().inverted());
  const Pose startPoseInGoalFrame(worldToGoalFrameTransform * startPose_);
  const PlanarPose startPlanarPoseInGoalFrame = getPlanarPoseFromPose(startPoseInGoalFrame);
  const double yawAngleToGoalInGoalFrame = RotationVector(startPoseInGoalFrame.getRotation() * rotationToGoalInStartFrame).getUnique().z();
  const double distanceToIntermediateGoal =
      distanceToGoal > 2.0 * parameters_.getTurnAndWalkDistance() ?
          parameters_.getTurnAndWalkDistance() : 0.5 * distanceToGoal;
  MELO_DEBUG_STREAM("Distance to intermediate goal: " << distanceToIntermediateGoal << ".");
  Vector directionToGoal;
  directionToGoal.vector().head(2) = (goalPlanarPoseInGoalFrame.matrix().head(2) - startPlanarPoseInGoalFrame.matrix().head(2)).normalized();

  // First intermediate goal.
  PlanarPose firstIntermediateGoal;
  firstIntermediateGoal.matrix().head(2) = startPlanarPoseInGoalFrame.matrix().head(2) + distanceToIntermediateGoal * directionToGoal.vector().head(2);
  firstIntermediateGoal.z() = yawAngleToGoalInGoalFrame;

  // Second intermediate goal.
  PlanarPose secondIntermediateGoal;
  secondIntermediateGoal.matrix().head(2) = goalPlanarPoseInGoalFrame.matrix().head(2) - distanceToIntermediateGoal * directionToGoal.vector().head(2);
  secondIntermediateGoal.z() = yawAngleToGoalInGoalFrame;

  // Add intermediate goals to list of goal poses.
  goalPoses_.insert(goalPoses_.begin(), getPoseFromPlanarPose(secondIntermediateGoal));
  goalPoses_.insert(goalPoses_.begin(), getPoseFromPlanarPose(firstIntermediateGoal));
}

void OneStepFootstepPlanner::determineStartFootprintAndPose()
{
  free_gait::StepQueue previousPlan;
  previousPlan.add(previousPlan_);
  startPose_ = getNextFootprintPose(adapter_, previousPlan);
}

void OneStepFootstepPlanner::computePoseDifference()
{
  Pose goalPoseInWorld(goalToWorldFrameTransform_ * goalPoses_.front());
  Position positionDifference = startPose_.inverseTransform(goalPoseInWorld.getPosition());
  positionDifference.z() = 0.0;
  RotationVector rotationDifferenceVector(startPose_.getRotation().inverted() * goalPoseInWorld.getRotation());
  rotationDifferenceVector.setUnique();
  goalPoseInStartFrame_.x() = positionDifference.x();
  goalPoseInStartFrame_.y() = positionDifference.y();
  goalPoseInStartFrame_.z() = rotationDifferenceVector.z();
  MELO_DEBUG_STREAM("Goal footprint pose in start pose frame: " << goalPoseInStartFrame_);
}

void OneStepFootstepPlanner::updateStepPose(const bool tentative)
{
  size_t nCycles = 0;
  const auto& direction = gaitPattern_.getDirection();
  robot_utils::Ellipsoid ellipsoid(Vector(parameters_.getMaxPoseDifferenceForGaitCycle().matrix()));
  Vector maxPoseDifferenceForGaitCycleVector(parameters_.getMaxPoseDifferenceForGaitCycle(speedFactor_));
  ellipsoid.constrainVectorToVolume(maxPoseDifferenceForGaitCycleVector);
  PlanarPose maxPoseDifferenceForGaitCycle(maxPoseDifferenceForGaitCycleVector.vector());
  if (direction == GaitPatternBase::Direction::FORWARD || direction == GaitPatternBase::Direction::BACKWARD) {
    nCycles = (size_t) std::fabs(goalPoseInStartFrame_.x() / maxPoseDifferenceForGaitCycle.x()) + 1;
  } else if (direction == GaitPatternBase::Direction::LEFT || direction == GaitPatternBase::Direction::RIGHT) {
    nCycles = (size_t) std::fabs(goalPoseInStartFrame_.y() / maxPoseDifferenceForGaitCycle.y()) + 1;
  } else if (direction == GaitPatternBase::Direction::ROTATION_LEFT || direction == GaitPatternBase::Direction::ROTATION_RIGHT) {
    nCycles = (size_t) std::fabs(goalPoseInStartFrame_.z() / maxPoseDifferenceForGaitCycle.z()) + 1;
  }
  currentStepPose_ = goalPoseInStartFrame_ / (double) nCycles;
  MELO_DEBUG_STREAM("Updated step pose to: " << currentStepPose_);

  if (nCycles == 1) {
    if (!tentative) {
      ++lastCycleToGoalCounter_;
    }
  } else {
    lastCycleToGoalCounter_ = 0;
  }
  MELO_DEBUG_STREAM("Last cycle to goal counter: " << lastCycleToGoalCounter_);
}

bool OneStepFootstepPlanner::determineNextSwingLeg(const bool directionChanged)
{
  // Find leg which would take the biggest step in desired direction.
  free_gait::StepQueue previousPlan;
  previousPlan.add(previousPlan_);
  LimbEnum limbWithMaxStepLength(LimbEnum::LF_LEG);
  std::unordered_map<LimbEnum, double, EnumClassHash> stepLengths;
  for (const auto& limb : adapter_.getLimbs()) {
    const Position currentFootPosition = getFootPositionAfterActiveStep(limb, adapter_, previousPlan);
    const Position footstepInStartFrame = getPoseFromPlanarPose(currentStepPose_).transform(
        getPositionFromPlanarPosition(parameters_.getNominalPlanarStanceForLimb(limb)));
    const Position footstepInWorldFrame = startPose_.transform(footstepInStartFrame);
    const Vector stepVector(footstepInWorldFrame - currentFootPosition);
    stepLengths[limb] = stepVector.vector().head(2).norm();
    MELO_DEBUG_STREAM("Candidate swing leg is " << limb << " with step length " << stepLengths[limb] << " m.");
    if (stepLengths[limb] > stepLengths[limbWithMaxStepLength]) {
      limbWithMaxStepLength = limb;
    }
  }
  MELO_DEBUG_STREAM("Next swing leg is " << limbWithMaxStepLength << " with step length " << stepLengths[limbWithMaxStepLength] << " m.");

  if (directionChanged) {
    MELO_DEBUG_STREAM("Direction changed: Redefining next gait pattern step as step with biggest step length.");
    gaitPattern_.setFirstStep(limbWithMaxStepLength);
  } else {
    gaitPattern_.advance();
    if (stepLengths[limbWithMaxStepLength] > 0.02 &&
        stepLengths[limbWithMaxStepLength] > 2.0 * stepLengths[gaitPattern_.getCurrentLimb()]) {
      gaitPattern_.setFirstStep(limbWithMaxStepLength);
      MELO_DEBUG_STREAM("Direction unchanged, but next step is too small: Redefining next gait pattern step as step with biggest step length.");
    }
  }

  return true;
}

bool OneStepFootstepPlanner::determineDirection()
{
  const PlanarPose poseDifference = parameters_.getMaxPoseDifferenceForGaitCycle(speedFactor_);
  const Eigen::Array3d nStepsPerDirection = (goalPoseInStartFrame_ / poseDifference).abs();
  size_t maxCoeff;
  nStepsPerDirection.maxCoeff(&maxCoeff);

  // Determine the walking direction. Keep current direction if the goal is near.
  auto direction = gaitPattern_.getDirection();
  if (maxCoeff == 0 && goalPoseInStartFrame_[maxCoeff] > 0.02)
    direction = GaitPatternBase::Direction::FORWARD;
  else if (maxCoeff == 0 && goalPoseInStartFrame_[maxCoeff] < -0.02)
    direction = GaitPatternBase::Direction::BACKWARD;
  else if (maxCoeff == 1 && goalPoseInStartFrame_[maxCoeff] > 0.02)
    direction = GaitPatternBase::Direction::LEFT;
  else if (maxCoeff == 1 && goalPoseInStartFrame_[maxCoeff] < -0.02)
    direction = GaitPatternBase::Direction::RIGHT;
  else if (maxCoeff == 2 && goalPoseInStartFrame_[maxCoeff] > 0.005)
    direction = GaitPatternBase::Direction::ROTATION_LEFT;
  else if (maxCoeff == 2 && goalPoseInStartFrame_[maxCoeff] < -0.005)
    direction = GaitPatternBase::Direction::ROTATION_RIGHT;

  bool directionChanged = direction != gaitPattern_.getDirection();
  gaitPattern_.setDirection(direction);
  MELO_DEBUG_STREAM("Locomotion direction: " << gaitPattern_.getDirection());
  return directionChanged;
}

const free_gait::Footstep OneStepFootstepPlanner::computeNextFootstep()
{
  free_gait::Footstep footstep(gaitPattern_.getCurrentLimb());
  const Position footstepInStartFrame = getPoseFromPlanarPose(currentStepPose_).transform(
      getPositionFromPlanarPosition(parameters_.getNominalPlanarStanceForLimb(gaitPattern_.getCurrentLimb())));
  const Position footstepInWorldFrame = startPose_.transform(footstepInStartFrame);
  footstep.setTargetPosition(adapter_.getWorldFrameId(), footstepInWorldFrame);
  parameters_.populateFoostepParameters(speedFactor_, footstep);
  return footstep;
}

} /* namespace locomotion_planner */
