/**
 * @authors     Peter Fankhauser, Francisco Giraldez Gamez, Valentin Yuryev
 * @affiliation ANYbotics
 * @brief       Implementation of LocomotionPlanner class
 */

#include "locomotion_planner/LocomotionPlanner.hpp"
#include "locomotion_planner/common/geometry.hpp"
#include "locomotion_planner/footstep_planner/OneStepFootstepPlanner.hpp"
#include "locomotion_planner/gait_pattern/GaitPatternLongitudinalCrawling.hpp"
#include "locomotion_planner/swing_trajectory_planner/spline_swing_trajectory_planner/SplineSwingTrajectoryPlanner.hpp"
#include "locomotion_planner/swing_trajectory_planner/SwingHeightPlanner.hpp"

#include <message_logger/message_logger.hpp>

#include <chrono>
#include <thread>

namespace locomotion_planner {

LocomotionPlanner::LocomotionPlanner(free_gait::AdapterBase& adapter, Parameters& parameters) :
    speedFactor_(0.0),
    adapter_(adapter),
    parameters_(parameters),
    currentPlannerModule_(nullptr)
{
  elevationMapUser_.reset(new ElevationMapUser(parameters_));
  gaitPattern_.reset(new GaitPatternCrawling);
  footstepPlanner_.reset(new OneStepFootstepPlanner(adapter_, parameters_, *gaitPattern_));
  footholdOptimizerBackup_.reset(new FootholdOptimizerSameHeight(adapter_, parameters_, planningData_));
  footholdOptimizerElevationMapAndKinematics_.reset(new FootholdOptimizerElevationMapAndKinematics(adapter_, parameters_, planningData_, elevationMapUser_));
  swingTrajectoryPlanner_.reset(new SplineSwingTrajectoryPlanner(adapter_, parameters_, elevationMapUser_));
  swingTrajectoryPlannerBackup_.reset(new SwingHeightPlanner(adapter_, parameters_, elevationMapUser_));
}

void LocomotionPlanner::registerResultCallback(std::function<void(const LocomotionPlanner::GoalPoseResult&)> callback) {
  resultCallback_ = callback;
}

void LocomotionPlanner::registerSendGoalCallback(
    std::function<void(const free_gait::StepQueue&)> callback)
{
  sendGoalCallback_ = callback;
}

void LocomotionPlanner::registerGetActionStateCallback(
    std::function<bool(free_gait::FreeGaitActionClient::ActionState&, free_gait::ExecutorState&)> callback)
{
  getActionStateCallback_ = callback;
}

void LocomotionPlanner::registerGetTransformCallback(std::function<bool(const std::string&, const std::string&, Transform&)> callback)
{
  getTransformCallback_ = callback;
}

void LocomotionPlanner::registerVisualizePlanningDataCallback(std::function<void(const PlanningData&)> callback)
{
  visualizePlanningDataCallback_ = callback;
}

void LocomotionPlanner::initialize()
{
  // Fill in max base velocity information
  free_gait::BaseAuto baseFastMotion;
  parameters_.populateBaseAutoParameters(1.0, baseFastMotion);
  maxTwistCommand_.x() = baseFastMotion.getAverageLinearVelocity();
  maxTwistCommand_.y() = baseFastMotion.getAverageLinearVelocity();
  maxTwistCommand_.z() = baseFastMotion.getAverageAngularVelocity();

  // Initialize empty goal pose
  resetGoal();

  // Clear elevation map
  elevationMapUser_->clearMap();
}

LocomotionPlanner::GoalPoseStatus
LocomotionPlanner::setNewGoalPose(const std::string &frameId,
                                  const Pose &pose) {
  isTwistCommandActive_ = false;
  newGoalFrameId_ = frameId;
  return setNewGoalPose(pose, parameters_.getDefaultSpeedFactor());
}

LocomotionPlanner::GoalPoseStatus
LocomotionPlanner::setNewRelativeGoalPose(const std::string &frameId,
                                          const Pose &pose) {
  isTwistCommandActive_ = false;
  newGoalFrameId_ = frameId;
  const Pose targetPose = getTargetPoseFromRelativeGoalPose(pose, frameId);
  return setNewGoalPose(targetPose, parameters_.getDefaultSpeedFactor());
}

void LocomotionPlanner::setNewTwistCommand(const Twist& twist)
{
  if ((twistCommand_.getVector() - twist.getVector()).norm() < 1e-6) return;
  twistCommand_ = twist;
  isTwistCommandActive_ = true;
  defineNewGoalPoseFromTwistCommand();
}

void LocomotionPlanner::getElevationMapRegion(grid_map::Position& position, grid_map::Length& length)
{
  double footprintRadius, maxDistance;
  parameters_.getElevationMapRegionParameters(footprintRadius, maxDistance);
  Position positionBaseToElevationMapFrameInElevationMapFrame = adapter_.getPositionWorldToBaseInWorldFrame();
  if (!elevationMapUser_->getMap().getFrameId().empty()) {
  positionBaseToElevationMapFrameInElevationMapFrame = adapter_.transformPosition(adapter_.getWorldFrameId(), elevationMapUser_->getMap().getFrameId(),adapter_.getPositionWorldToBaseInWorldFrame());
  }
  const grid_map::Position startPosition(getPlanarPositionFromPosition(positionBaseToElevationMapFrameInElevationMapFrame).vector());
  Position positionGoalToElevationMapFrameInElevationMapFrame = goalPose_.getPosition();
  if (!elevationMapUser_->getMap().getFrameId().empty()) {
    if (goalFrameId_.empty()) {
      positionGoalToElevationMapFrameInElevationMapFrame =
          adapter_.transformPosition(adapter_.getBaseFrameId(),
                                     elevationMapUser_->getMap().getFrameId(),
                                     goalPose_.getPosition());
    } else {
      positionGoalToElevationMapFrameInElevationMapFrame =
          adapter_.transformPosition(goalFrameId_,
                                     elevationMapUser_->getMap().getFrameId(),
                                     goalPose_.getPosition());
    }
  }
  grid_map::Position goalPosition(getPlanarPositionFromPosition(positionGoalToElevationMapFrameInElevationMapFrame).vector());
  if ((goalPosition - startPosition).norm() > maxDistance) {
    goalPosition = startPosition + (goalPosition - startPosition).normalized() * maxDistance;
  }
  grid_map::Polygon area(grid_map::Polygon::convexHullOfTwoCircles(startPosition, goalPosition, footprintRadius, 8));
  area.getBoundingBox(position, length);
}

void LocomotionPlanner::setElevationMap(const grid_map::GridMap& elevationMap)
{
  MELO_DEBUG("Received elevation map.");
  std::lock_guard<std::mutex> guard(elevationMapUser_->getMutex());
  elevationMapUser_->setMap(elevationMap);
  elevationMapUser_->computeSignedDistanceField(parameters_.getCollisionLayer()); // We do this here to safe time during planning.
}

void LocomotionPlanner::clearElevationMap()
{
  std::lock_guard<std::mutex> guard(elevationMapUser_->getMutex());
  elevationMapUser_->clearMap();
}

void LocomotionPlanner::start() {
  MELO_DEBUG("Requested to start planner.")
  if(spinnerThread_.joinable()) {
    spinnerThread_.join();
  }
  spinnerThread_ = std::thread(std::bind(&LocomotionPlanner::spinPlanner, this));
  MELO_DEBUG("Planner started.")
}

void LocomotionPlanner::stop()
{
  // Guard against function re-entry, to prevent deadlock when two separate threads call join().
  if (isRequestedToStop_) {
    return;
  }
  MELO_INFO("Requesting to stop Locomotion Planner.");
  isRequestedToStop_ = true;
  if (spinnerThread_.joinable()) {
    spinnerThread_.join();
  }
  MELO_INFO("Locomotion plannner stopped.");
  isRequestedToStop_ = false;
  resetGoal();
}

LocomotionPlanner::GoalPoseStatus LocomotionPlanner::setNewGoalPose(const Pose& pose, const double speedFactor)
{
  // Check for unit quaternion in pose orientation 
  constexpr double quaternionNormTolerance = 1e-3; 
  if (abs(1.0 - pose.getRotation().norm()) >= quaternionNormTolerance) {
    MELO_ERROR_STREAM("New goal pose rotation is defined by a non-unit quaternion!\n" << pose.getRotation() << "\nQuaternion norm is " << pose.getRotation().norm());
    return GoalPoseStatus::REJECTED_INVALID_TRANSFORM;
  }

  // Check if new goal is same as robots current pose
  constexpr double poseTolerance = 1e-2; // In meters/radians
  Pose currentRobotPose = Pose(adapter_.getPositionWorldToBaseInWorldFrame(), adapter_.getOrientationBaseToWorld());
  bool isNewGoalSameAsCurrentPose = false;
  if (newGoalFrameId_ == adapter_.getWorldFrameId()) {
    isNewGoalSameAsCurrentPose = compareGoalPoses(pose, currentRobotPose, poseTolerance);
  } else {
    // If in different frames, compare them in world
    try {
      const Pose poseInWorld = adapter_.transformPose(newGoalFrameId_, adapter_.getWorldFrameId(), pose);
      isNewGoalSameAsCurrentPose = compareGoalPoses(poseInWorld, currentRobotPose, poseTolerance);
    } catch (const std::invalid_argument& exception) {
      MELO_ERROR("Caught an exception while comparing goal poses: %s", exception.what());
      return GoalPoseStatus::REJECTED_FRAME_ERROR; //TODO( fgiraldez): propagate exceptions rather than use status?
    }
  }
  if (isNewGoalSameAsCurrentPose) {
    MELO_WARN("The robot is already at this goal.");
    return GoalPoseStatus::ALREADY_THERE;
  }

  // Check if new goal is same as previous.
  bool isNewGoalSameAsPrevious = false;
  if (isThereGoal_) {
    if ((newGoalFrameId_ == goalFrameId_)) {
      isNewGoalSameAsPrevious = compareGoalPoses(pose, goalPose_, poseTolerance);
    } else {
      // If in different frames, compare them in world
      try {
        const Pose poseInWorld = adapter_.transformPose(newGoalFrameId_, adapter_.getWorldFrameId(), pose);
        const Pose goalPoseInWorld = adapter_.transformPose(goalFrameId_, adapter_.getWorldFrameId(), goalPose_);
        isNewGoalSameAsPrevious = compareGoalPoses(poseInWorld, goalPoseInWorld, poseTolerance);
      } catch (const std::invalid_argument& exception) {
        MELO_ERROR("Caught an exception while comparing goal poses: %s", exception.what());
        return GoalPoseStatus::REJECTED_FRAME_ERROR; //TODO( fgiraldez): propagate exceptions rather than use status?
      }
    }
  }

  if (isNewGoalSameAsPrevious) {
    MELO_WARN("Ignoring new goal pose because it is the same as the existing goal");
    return GoalPoseStatus::IGNORED_SAME_AS_CURRENT;
  }

  MELO_DEBUG_STREAM("Locomotion planner will track goal pose:" << std::endl << pose << std::endl << "In frame: " << newGoalFrameId_);

  isNewGoal_ = true;
  goalPose_ = pose;
  goalFrameId_ = newGoalFrameId_;
  speedFactor_ = speedFactor;
  isLastPlanToGoal_ = false;
  // Check if we are currently tracking a goal pose
  GoalPoseStatus status;
  if (isThereGoal_) {
    status = GoalPoseStatus::OVERWRITE_CURRENT_GOAL;
  } else {
    status = GoalPoseStatus::ACCEPTED_NEW_GOAL;
    isThereGoal_ = true;
  }
  hasStartedPlanning_ = false;
  hasFinishedPlanning_ = false;
  isPlanSent_ = false;
  return status;
}

void LocomotionPlanner::defineNewGoalPoseFromTwistCommand()
{
  // If set to "auto", the goal will be sent in the elevation map frame
  if (parameters_.getDefaultGoalFrameId() == "auto") {
    if(!elevationMapUser_->isMapValid()) {
      MELO_WARN_THROTTLE_STREAM(2.0 ,"Elevation map has no valid reference frame. Desired goal will be defined in world frame: " <<  adapter_.getWorldFrameId());
      newGoalFrameId_ = adapter_.getWorldFrameId();
    } else {
      newGoalFrameId_ = elevationMapUser_->getMap().getFrameId();
    }
  } else {
    newGoalFrameId_ = parameters_.getDefaultGoalFrameId();
  }
  // C-wise normalization
  const PlanarTwist twist = getPlanarTwistFromTwist(twistCommand_);
  PlanarTwist normalizedTwist = twist / maxTwistCommand_;
  MELO_DEBUG_STREAM("Defining new goal from normalized planar twist (" << normalizedTwist.transpose() << ").");
  // Obtain next footprint pose in the default goal frame
  const Pose nextFootprintPose = adapter_.transformPose(adapter_.getWorldFrameId(), newGoalFrameId_, getNextFootprintPose(adapter_, previousPlan_));
  // Calculate speed factor, if at least one coefficient is above 1.0 set max possible speed
  double speedFactor = std::min(normalizedTwist.matrix().norm(), 1.0);
  PlanarPose goalPlanarPoseInBase = parameters_.getTwistScaling() * normalizedTwist * parameters_.getMaxPoseDifferenceForGaitCycle();
  goalPlanarPoseInBase = goalPlanarPoseInBase.min(parameters_.getMaxPoseDifferenceForGaitCycle()).max(-parameters_.getMaxPoseDifferenceForGaitCycle());
  const Pose goalPoseInGoalFrame = nextFootprintPose * getPoseFromPlanarPose(goalPlanarPoseInBase);
  GoalPoseStatus status = setNewGoalPose(goalPoseInGoalFrame, speedFactor);
  MELO_DEBUG_STREAM("Got goal pose status " << static_cast<int>(status));
}

void LocomotionPlanner::resetGoal() {
  twistCommand_ = Twist();
  isTwistCommandActive_ = false;
  isThereGoal_ = false;
  isLastPlanToGoal_ = false;
  goalFrameId_ = adapter_.getWorldFrameId();
  goalPose_ = Pose();
}

void LocomotionPlanner::spinPlanner()
{
  while (!isRequestedToStop_) {
    if (isThereGoal_) {
      free_gait::FreeGaitActionClient::ActionState actionState = free_gait::FreeGaitActionClient::ActionState::ERROR;
      free_gait::ExecutorState executorState;
      getActionStateCallback_(actionState, executorState);

      // Check if step action had error
      if (isPlanSent_ && (actionState == free_gait::FreeGaitActionClient::ActionState::ERROR || actionState == free_gait::FreeGaitActionClient::ActionState::PREEMPTED)) {
        MELO_DEBUG_STREAM("Free Gait step failed or was preempted. ");
        resetGoal();
        if(resultCallback_) {
          resultCallback_(GoalPoseResult::EXECUTION_FAILURE);
          return;
        }
      }

      // Check if planning failed
      if (hasFinishedPlanning_ && !planSuccessful_) {
        MELO_DEBUG_STREAM("Planning failed.");
        resetGoal();
        if(resultCallback_) {
          resultCallback_(GoalPoseResult::PLANNER_FAILURE);
          return;
        }
      }

      // Check if goal was reached successfully
      if (isLastPlanToGoal_ && actionState == free_gait::FreeGaitActionClient::ActionState::SUCCEEDED) {
        MELO_DEBUG_STREAM("Reached goal successfully.");
        resetGoal();
        if(resultCallback_) {
          resultCallback_(GoalPoseResult::SUCCESS);
          return;
        }
      }

      // On-line replanning.
      // Check time for starting planner.
      if (actionState == free_gait::FreeGaitActionClient::ActionState::ACTIVE) {

        if (!hasStartedPlanning_ && executorState > free_gait::ExecutorState(2, 0.5) && !isLastPlanToGoal_) {
          hasStartedPlanning_ = true;
          if (plannerThread_.joinable()) {
            MELO_DEBUG("Starting to re-plan. Wait for current planner thread to finish.");
            plannerThread_.join();
          }
          MELO_DEBUG("Starting to re-plan. Spawning new thread.");
          plannerThread_ = std::thread(std::bind(&LocomotionPlanner::plan, this));
          continue;
        }

        if (executorState > free_gait::ExecutorState(2, 0.7) && hasFinishedPlanning_
            && !isPlanSent_ && !plan_.empty()) {
          MELO_DEBUG("Sending plan.");
          sendPlan();
          continue;
        }

        // New plan arrived at server.
        // TODO: Could this be nicer solved with callback from Free Gait action server?
        if (hasFinishedPlanning_ && executorState <= free_gait::ExecutorState(1, 0.5)) {
          MELO_DEBUG("Trigger re-planning.");
          hasStartedPlanning_ = false;
          hasFinishedPlanning_ = false;
          isPlanSent_ = false;
          previousPlan_ = plan_;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // TODO Switch to ROS time?

      } else if (actionState == free_gait::FreeGaitActionClient::ActionState::SUCCEEDED ||
          actionState == free_gait::FreeGaitActionClient::ActionState::INITIALIZED ||
          actionState == free_gait::FreeGaitActionClient::ActionState::PREEMPTED ||
          actionState == free_gait::FreeGaitActionClient::ActionState::ERROR) {

        // Starting and new goals.
        if (isNewGoal_ && !hasStartedPlanning_ && !isLastPlanToGoal_) {
          hasStartedPlanning_ = true;
          if (plannerThread_.joinable()) {
            MELO_DEBUG("Starting to plan. Wait for current thread to finish.");
            plannerThread_.join();
          }
          MELO_DEBUG("Starting to plan. Spawning new thread.");
          plannerThread_ = std::thread(std::bind(&LocomotionPlanner::plan, this));
          continue;
        }

        // Was not able to plan in time.
        if (hasFinishedPlanning_ && !isPlanSent_ && !plan_.empty()) {
          MELO_WARN("Planning took longer than expected.");
          sendPlan();
          continue;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // TODO Switch to ROS time?
      }
      else {
        MELO_DEBUG_THROTTLE_STREAM(5.0, "Action client is in state " << static_cast<int>(actionState) << ". Nothing to be done by Locomotion Planner.");
      }
    }
    else {
      MELO_DEBUG_THROTTLE(5.0, "There is no goal pose. Nothing to be done by Locomotion Planner.");
    }
  }
  if (isRequestedToStop_) {
    if (plannerThread_.joinable()) {
      MELO_DEBUG("Requested to stop. Joining planner thread");
      plannerThread_.join();
    }
    if(resultCallback_) {
      resultCallback_(GoalPoseResult::PREEMPTED);
    }
  }
  MELO_DEBUG("Spinner thread finished execution.");
}

void LocomotionPlanner::plan() {
  if (currentPlannerModule_) currentPlannerModule_->stop();
  const auto startTime = std::chrono::high_resolution_clock::now();

  plan_.clear();
  planningData_.clearAll();
  currentPlannerModule_ = footstepPlanner_.get();
  bool lastStepToGoal;
  std::vector<Step> stepPlan;

  {
    std::lock_guard<std::mutex> guard(elevationMapUser_->getMutex());  // Don't update map during planning.
    Transform goalToWorldFrameTransform;
    getTransformCallback_(goalFrameId_, adapter_.getWorldFrameId(), goalToWorldFrameTransform);

    // Plan nominal footsteps.

    if (!currentPlannerModule_->plan(goalPose_, goalToWorldFrameTransform, speedFactor_, isNewGoal_, !isTwistCommandActive_, stepPlan,
                                     lastStepToGoal)) {
      MELO_ERROR("Planner failed to find a plan!");
      hasFinishedPlanning_ = true;
      planSuccessful_ = false;
      return;
    }

    planningData_.addNominalFootsteps(stepPlan);

    // Optimize footholds.
    if (!footholdOptimizerElevationMapAndKinematics_->optimizeFootholds(stepPlan)) {
      // We do not want warnings if there's no elevation map to optimize footholds
      if (elevationMapUser_->isMapValid()) {
        MELO_WARN("Foothold optimizer failed. Using back-up optimizer.");
      }
      if (!footholdOptimizerBackup_->optimizeFootholds(stepPlan)) {
        MELO_ERROR("Planner failed to find viable foothoolds.");
        hasFinishedPlanning_ = true;
        planSuccessful_ = false;
        return;
      }
    }

    planningData_.addOptimizedFootsteps(stepPlan);

    // Optimize feet trajectories.
    bool swingTrajectoryPlanned = false;
    if (parameters_.getUsePrimarySwingTrajectory()) {
      if (!swingTrajectoryPlanner_->planSwingTrajectories(stepPlan)) {
        MELO_WARN("Swing trajectory planner failed. Using back-up planner.");
      } else {
        swingTrajectoryPlanned = true;
      }
    }

    if (!swingTrajectoryPlanned) {
      if (!swingTrajectoryPlannerBackup_->planSwingTrajectories(stepPlan)) {
        if (parameters_.getStopOnFootTrajectoryFailure()) {
          MELO_ERROR("Planner failed to find a collision-free swing trajectory.");
          hasFinishedPlanning_ = true;
          planSuccessful_ = false;
          return;
        } else {
          // We do not want warnings if there's no elevation map to plan trajectories upon
            if (elevationMapUser_->isMapValid()) {
              MELO_WARN("Swing trajectory planner failed. Using default footsteps.");

            }
        }
      }
    }
  }

  isLastPlanToGoal_ = lastStepToGoal;
  plan_.add(stepPlan);

  const auto endTime = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> elapsed = endTime - startTime;

  MELO_DEBUG_STREAM("Planner took " << elapsed.count() << " ms.");

  currentPlannerModule_ = nullptr;
  isNewGoal_ = false;
  hasFinishedPlanning_ = true;
  planSuccessful_ = true;
}

void LocomotionPlanner::sendPlan()
{
  if (isLastPlanToGoal_) { MELO_DEBUG("Last step to goal."); }
  MELO_DEBUG("Sending plan.");
  isPlanSent_ = true;
  sendGoalCallback_(plan_);
  visualizePlanningDataCallback_(planningData_);
}

size_t LocomotionPlanner::getNumberOfStepsForReplanningHorizon() const
{
  return 4;
}

double LocomotionPlanner::getExpectedUpperPlanningTime() const
{
  return footstepPlanner_->getExpectedUpperPlanningTime();
}

bool LocomotionPlanner::compareGoalPoses(const Pose &newGoalPose, const Pose &oldGoalPose, const double tolerance) const {
  return std::abs(newGoalPose.getPosition().x() - oldGoalPose.getPosition().x()) < tolerance
         && std::abs(newGoalPose.getPosition().y() - oldGoalPose.getPosition().y()) < tolerance
         && newGoalPose.getRotation().isNear(oldGoalPose.getRotation(), tolerance);
}

Pose
LocomotionPlanner::getTargetPoseFromRelativeGoalPose(const Pose &relativePoseInBaseFrame, const std::string& goalFrameId) const {
  return adapter_.transformPose(adapter_.getBaseFrameId(), goalFrameId, relativePoseInBaseFrame);
}

}
