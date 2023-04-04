/*
 * MissionControlZmp.cpp
 *
 *  Created on: Aug 22, 2014
 *      Author: C. Dario Bellicoso, Christian Gehring
 */

// loco
#include "loco/mission_control/MissionControlZmp.hpp"
#include <loco/common/loco_common.hpp>

// motion generation utils
#include <motion_generation_utils/VelocityClipping.hpp>

// motion generation utils
#include <motion_generation_utils/VelocityClipping.hpp>

// message logger
#include "message_logger/message_logger.hpp"

// signal logger
#include <signal_logger/signal_logger.hpp>

// tinyxml tools
#include <tinyxml_tools/tinyxml_tools.hpp>

using namespace message_logger::color;

namespace loco {

MissionControlZmp::MissionControlZmp(kindr::TwistLocalD* desRobotTwist, kindr::HomTransformQuatD* desRobotPose, WholeBody& wholeBody,
                                     ContactScheduleZmp& contactSchedule)
    : desRobotTwist_(desRobotTwist),
      desRobotPose_(desRobotPose),
      wholeBody_(wholeBody),
      contactSchedule_(contactSchedule),
      speedFilter_(),
      locomotionState_(LocomotionState::Standing),
      locomotionMode_(LocomotionMode::ModeStanding),
      targetLocomotionMode_(LocomotionMode::ModeStanding),
      modeSwitchStatus_(loco::contact_schedule::ContactScheduleSwitchStatus::Undefined),
      modeSwitchCondVar_(),
      modeSwitchMutex_(),
      didPin_(false),
      isSwitchingToStand_(false),
      switchToStandVelRefThresholdStart_(0.05),
      switchToStandVelRefThresholdEnd_(0.05),
      timeThresholdSwitchToStand_(0.10),
      isSwitchingDirection_(false),
      switchDirectionVelRefThresholdStart_(0.005),
      switchDirectionVelRefThresholdEnd_(0.0),
      timeThresholdSwitchDirection_(0.10),
      timeThresholdForceStance_(3.0),
      timeThresholdVelocityError_(0.8),
      velocityErrorThresholdStart_(0.09),
      velocityErrorThresholdEnd_(0.25),
      shrinkPolygonFactor_(0.8),
      params_(),
      previousActiveGaitId_(100000u),
      previousDesiredGaitId_(100000u),
      filteredBaseLinearVelocitiesInControlFrame_(),
      velocityClippingType_(VelocityClippingType::Box),
      enableVelocityOffsetsOnSlopes_(false),
      velocityOffsetMultiplierForSlopesPerDeg_(Eigen::Vector2d::Zero()),
      linearVelocityOffsetOnSlope_() {}

bool MissionControlZmp::initialize(double dt) {
  if (!speedFilter_.initialize(dt)) {
    MELO_WARN_STREAM("[MissionControlZmp::initialize] Failed to initialize speed filter.");
    return false;
  }

  didPin_ = false;
  isSwitchingToStand_ = false;
  isSwitchingDirection_ = false;

  locomotionState_ = LocomotionState::Standing;
  locomotionMode_ = LocomotionMode::ModeStanding;
  targetLocomotionMode_ = LocomotionMode::ModeStanding;
  modeSwitchStatus_ = loco::contact_schedule::ContactScheduleSwitchStatus::Running;
  wholeBody_.getTorsoPtr()->getPropertiesPtr()->setMaximumBaseTwistInControlFrame(speedFilter_.getMaximumBaseTwistInControlFrame());

  Eigen::Vector3d zero = Eigen::Vector3d::Zero();
  wholeBody_.getTorsoPtr()->getDesiredStatePtr()->setLinearVelocityTargetInControlFrame(static_cast<LinearVelocity>(zero));
  wholeBody_.getTorsoPtr()->getDesiredStatePtr()->setAngularVelocityBaseInControlFrame(static_cast<LocalAngularVelocity>(zero));
  filteredBaseLinearVelocitiesInControlFrame_.reset();

  linearVelocityOffsetOnSlope_.reset();

  previousActiveGaitId_ = 100000u;
  previousDesiredGaitId_ = 100000u;
  return true;
}

bool MissionControlZmp::loadParameters(const TiXmlHandle& handle) {
  MELO_DEBUG_STREAM(magenta << "[MissionControlZmp] " << blue << "Load parameters." << def)

  TiXmlHandle missionHandle = handle;
  if (!tinyxml_tools::getChildHandle(missionHandle, handle, "Mission")) {
    return false;
  }

  TiXmlHandle configHandle = handle;
  if (!tinyxml_tools::getChildHandle(configHandle, missionHandle, "Configuration")) {
    return false;
  }

  TiXmlHandle locomotionSwitchesHandle = handle;
  if (!tinyxml_tools::getChildHandle(locomotionSwitchesHandle, missionHandle, "LocomotionStateSwitches")) {
    return false;
  }

  TiXmlHandle speedHandle = handle;
  if (!tinyxml_tools::getChildHandle(speedHandle, missionHandle, "Speed")) {
    return false;
  }

  // Velocity thresholds to switch to walk/stand.
  TiXmlHandle switchToStandHandle = handle;
  if (!tinyxml_tools::getChildHandle(switchToStandHandle, locomotionSwitchesHandle, "SwitchToStand")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(switchToStandVelRefThresholdStart_, switchToStandHandle, "start_vel_threshold")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(switchToStandVelRefThresholdEnd_, switchToStandHandle, "end_vel_threshold")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(timeThresholdSwitchToStand_, switchToStandHandle, "time_threshold")) {
    return false;
  }

  // Velocity thresholds to switch directions.
  TiXmlHandle switchDirectionHandle = handle;
  if (!tinyxml_tools::getChildHandle(switchDirectionHandle, locomotionSwitchesHandle, "SwitchDirection")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(switchDirectionVelRefThresholdStart_, switchDirectionHandle, "start_vel_threshold")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(switchDirectionVelRefThresholdEnd_, switchDirectionHandle, "end_vel_threshold")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(timeThresholdSwitchDirection_, switchDirectionHandle, "time_threshold")) {
    return false;
  }

  // Velocity thresholds for balancing gait.
  TiXmlHandle balancingHandle = handle;
  if (!tinyxml_tools::getChildHandle(balancingHandle, locomotionSwitchesHandle, "Balancing")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(velocityErrorThresholdStart_, balancingHandle, "start_vel_threshold")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(velocityErrorThresholdEnd_, balancingHandle, "end_vel_threshold")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(timeThresholdVelocityError_, balancingHandle, "time_threshold")) {
    return false;
  }
  auto shrinkPolygonFactor = shrinkPolygonFactor_;
  tinyxml_tools::loadParameter(shrinkPolygonFactor_, balancingHandle, "shrink_polygon_factor", shrinkPolygonFactor);

  // Speed filter.
  if (!speedFilter_.loadParameters(handle)) {
    return false;
  }

  // Velocity clipping.
  TiXmlHandle velocityClippingHandle = handle;
  if (tinyxml_tools::getChildHandle(velocityClippingHandle, speedHandle, "VelocityClipping")) {
    std::string velocityClippingType;
    if (!tinyxml_tools::loadParameter(velocityClippingType, velocityClippingHandle, "type")) {
      return false;
    }
    if (velocityClippingType == "box") {
      velocityClippingType_ = VelocityClippingType::Box;
    } else if (velocityClippingType == "ellipsoid") {
      velocityClippingType_ = VelocityClippingType::Ellipsoid;
    } else {
      MELO_WARN_STREAM("Incorrect velocity clipping type. Choosing velocity clipping in a box as default.");
      velocityClippingType_ = VelocityClippingType::Box;
    }
  } else {
    MELO_WARN_STREAM("Missing handle 'VelocityClipping' in Mission.xml. Choosing velocity clipping in a box as default.");
    velocityClippingType_ = VelocityClippingType::Box;  // default option
  }

  // Load zmp parameters.
  std::vector<TiXmlElement*> gaitElements;
  if (!tinyxml_tools::getChildElements(gaitElements, speedHandle, "Gait")) {
    return false;
  }
  if (!contact_schedule::loadGaitParameters(gaitElements, params_, contactSchedule_.getMapGaitNameToId())) {
    return false;
  }

  // Load parameters for max speed offsets.
  TiXmlHandle maxVelOffsetHandle = handle, maxVelOffsetValuesHandle = handle;
  if (!tinyxml_tools::getChildHandle(maxVelOffsetHandle, speedHandle, "VelocityOffsetOnSlopes")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(enableVelocityOffsetsOnSlopes_, maxVelOffsetHandle, "enable")) {
    return false;
  }
  if (!tinyxml_tools::getChildHandle(maxVelOffsetValuesHandle, maxVelOffsetHandle, "OffsetPerDeg")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(velocityOffsetMultiplierForSlopesPerDeg_.x(), maxVelOffsetValuesHandle, "x")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(velocityOffsetMultiplierForSlopesPerDeg_.y(), maxVelOffsetValuesHandle, "y")) {
    return false;
  }
  constexpr double dt = 0.0025;
  double timeConstant = 0.0, velocityScaling = 0.0;
  TiXmlHandle velocityOffsetFilterHandle = handle;
  if (!tinyxml_tools::getChildHandle(velocityOffsetFilterHandle, maxVelOffsetHandle, "VelocityOffsetFilter")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(timeConstant, velocityOffsetFilterHandle, "time_constant")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(velocityScaling, velocityOffsetFilterHandle, "velocity_scaling")) {
    return false;
  }
  linearVelocityOffsetOnSlope_.setFilterParameters(dt, timeConstant, velocityScaling);

  TiXmlHandle baseVelocityFilterHandle = handle;
  if (!tinyxml_tools::getChildHandle(baseVelocityFilterHandle, locomotionSwitchesHandle, "BaseVelocityFilter")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(timeConstant, baseVelocityFilterHandle, "time_constant")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(velocityScaling, baseVelocityFilterHandle, "velocity_scaling")) {
    return false;
  }
  filteredBaseLinearVelocitiesInControlFrame_.setFilterParameters(dt, timeConstant, velocityScaling);

  return true;
}

const std::string& MissionControlZmp::getLocomotionModeName() const {
  return modeNameMap_.at(locomotionMode_);
}

void MissionControlZmp::enforceWithinBoundaries(Position& positionOffset) const {
  robot_utils::boundToRange(&positionOffset.x(), getMinimalPoseOffset().getPosition().x(), getMaximalPoseOffset().getPosition().x());
  robot_utils::boundToRange(&positionOffset.y(), getMinimalPoseOffset().getPosition().y(), getMaximalPoseOffset().getPosition().y());
  robot_utils::boundToRange(&positionOffset.z(), getMinimalPoseOffset().getPosition().z(), getMaximalPoseOffset().getPosition().z());
}

void MissionControlZmp::enforceWithinBoundaries(EulerAnglesZyx& orientationOffset) const {
  orientationOffset.setRoll(robot_utils::boundToRange(orientationOffset.roll(), speedFilter_.getMinimalDesiredOrientationOffset().x(),
                                                      speedFilter_.getMaximalDesiredOrientationOffset().x()));
  orientationOffset.setPitch(robot_utils::boundToRange(orientationOffset.pitch(), speedFilter_.getMinimalDesiredOrientationOffset().y(),
                                                       speedFilter_.getMaximalDesiredOrientationOffset().y()));
  orientationOffset.setYaw(robot_utils::boundToRange(orientationOffset.yaw(), speedFilter_.getMinimalDesiredOrientationOffset().z(),
                                                     speedFilter_.getMaximalDesiredOrientationOffset().z()));
  orientationOffset.setUnique();
}

void MissionControlZmp::enforceWithinBoundaries(LinearVelocity& desiredLinearVelocity, VelocityClippingType velocityClippingType) {
  const LinearVelocity& maxTranslationalVelocity = getMaximumBaseTwistInControlFrame().getTranslationalVelocity();
  if (enableVelocityOffsetsOnSlopes_) {
    // Get the velocity offset if the robot is walking on a slope (increase max velocity when going up and decrease when going down).
    linearVelocityOffsetOnSlope_.advance(getMaximumVelocityOffsetOnSlope(maxTranslationalVelocity.toImplementation()));

    if (velocityClippingType_ == VelocityClippingType::Box) {
      motion_generation_utils::clipToBox(desiredLinearVelocity.toImplementation(), maxTranslationalVelocity.toImplementation(),
                                         linearVelocityOffsetOnSlope_.getFilteredValue().toImplementation().head<2>());
    } else {
      motion_generation_utils::clipToEllipsoid(desiredLinearVelocity.toImplementation(), maxTranslationalVelocity.toImplementation(),
                                               linearVelocityOffsetOnSlope_.getFilteredValue().toImplementation().head<2>());
    }
  } else {
    if (velocityClippingType_ == VelocityClippingType::Box) {
      motion_generation_utils::clipToBox(desiredLinearVelocity.toImplementation(), maxTranslationalVelocity.toImplementation());
    } else {
      motion_generation_utils::clipToEllipsoid(desiredLinearVelocity.toImplementation(), maxTranslationalVelocity.toImplementation());
    }
  }
}

void MissionControlZmp::enforceWithinBoundaries(LocalAngularVelocity& desiredAngularVelocity,
                                                VelocityClippingType velocityClippingType) const {
  const LocalAngularVelocity& maxRotationalVelocity = getMaximumBaseTwistInControlFrame().getRotationalVelocity();

  if (velocityClippingType == VelocityClippingType::Box) {
    motion_generation_utils::clipToBox(desiredAngularVelocity.toImplementation(), maxRotationalVelocity.toImplementation());
  } else {
    motion_generation_utils::clipToEllipsoid(desiredAngularVelocity.toImplementation(), maxRotationalVelocity.toImplementation());
  }
}

bool MissionControlZmp::advance(double dt) {
  // Update locomotion mode if needed, when possible.
  if (targetLocomotionMode_ != locomotionMode_) {
    switch (targetLocomotionMode_) {
      case LocomotionMode::ModeStanding:
        if (locomotionState_ == LocomotionState::Standing) {
          setLocomotionMode(LocomotionMode::ModeStanding);
        } else {
          if (!switchToStand(modeSwitchStatus_)) return false;
        }
        break;
      case LocomotionMode::ModeWalking:
        modeSwitchStatus_ = contact_schedule::ContactScheduleSwitchStatus::Switched;
        setLocomotionMode(LocomotionMode::ModeWalking);
        break;
      case LocomotionMode::ModeStopping:
        if (!switchToStand(modeSwitchStatus_)) return false;
        setLocomotionMode(LocomotionMode::ModeStopping);
        break;
      default:
        MELO_ERROR_STREAM("[MissionControlZmp::advance] Target locomotion mode " << (int)targetLocomotionMode_ << " unknown!");
        return false;
    }
  }

  // Set the unfiltered base commands depending on locomotion mode.
  if (!setUnfilteredDesiredBaseCommandsInControlFrame(dt)) {
    MELO_WARN_STREAM("[MissionControlZmp::advance] Failed set unfiltered desired base commands.");
    return false;
  }

  // Filter the desired twist (and position).
  speedFilter_.advance(dt);

  // Filter measured base linear velocity.
  filteredBaseLinearVelocitiesInControlFrame_.advance(
      wholeBody_.getTorso().getMeasuredState().inControlFrame().getLinearVelocityBaseInControlFrame());

  // Set the desired (filtered) twist to the torso module.
  wholeBody_.getTorsoPtr()->getDesiredStatePtr()->setLinearVelocityTargetInControlFrame(
      speedFilter_.getDesiredBaseTwistInControlFrame().getTranslationalVelocity());
  wholeBody_.getTorsoPtr()->getDesiredStatePtr()->setAngularVelocityBaseInControlFrame(
      speedFilter_.getDesiredBaseTwistInControlFrame().getRotationalVelocity());

  // We store the commanded twists here as well, as the linearVelocityTargetInControlFrame_ and angularVelocityBaseInControlFrame will be
  // overwritten by the optimized values in Torso Control.
  wholeBody_.getTorsoPtr()->getDesiredStatePtr()->setLinearVelocityCommandedTargetInControlFrame(
      speedFilter_.getDesiredBaseTwistInControlFrame().getTranslationalVelocity());
  wholeBody_.getTorsoPtr()->getDesiredStatePtr()->setAngularVelocityCommandedBaseInControlFrame(
      speedFilter_.getDesiredBaseTwistInControlFrame().getRotationalVelocity());

  // Set desired (filtered) offset.
  wholeBody_.getTorsoPtr()->getDesiredStatePtr()->setDesiredOrientationOffset(speedFilter_.getFilteredDesiredOrientationOffset());
  wholeBody_.getTorsoPtr()->getDesiredStatePtr()->setDesiredPositionOffsetInWorldFrame(
      speedFilter_.getFilteredPositionOffsetInWorldFrame());

  // Send velocity errors to contact schedule.
  // ToDo: this should go to contact schedule (but there is the problem of the order of the modules).
  if (!contactSchedule_.updateVelocityFeedback(wholeBody_.getTorso().getDesiredState().getLinearVelocityTargetInControlFrame())) {
    MELO_WARN_STREAM("[MissionControlZmp::advance] Failed to set velocity feedback.");
    return false;
  }

  // Update parameters.
  if (!triggerParameterSwitch()) {
    MELO_WARN_STREAM("[MissionControlZmp::advance] Failed trigger parameter switch.");
    return false;
  }

  return true;
}

bool MissionControlZmp::updateLocomotionState() {
  switch (contactSchedule_.getStatus()) {
    case contact_schedule::Status::Stand: {
      // If we were walking before.
      if (locomotionState_ == LocomotionState::SwitchToStanceFromForwards ||
          locomotionState_ == LocomotionState::SwitchToStanceFromBackwards) {
        // Switch to stance.
        locomotionState_ = LocomotionState::Standing;

        // Start balancing gait (if required and if we do not go back to walk again).
        const double sign = (locomotionState_ == LocomotionState::SwitchToStanceFromForwards ? 1.0 : -1.0);
        const bool changeDirectionOfMotionWhileWalking = (sign * desRobotTwist_->getTranslationalVelocity().x() < -1e-5);
        if (!changeDirectionOfMotionWhileWalking && contactSchedule_.stopActiveGaitWithBalancingGait()) {
          if (!switchWalk(locomotionState_ == LocomotionState::SwitchToStanceFromForwards, true)) {
            return false;
          }
        }
      }

      // If balancing gait was active before -> go directly to stance.
      else if (locomotionState_ == LocomotionState::SwitchToStanceFromForceWalk) {
        locomotionState_ = LocomotionState::Standing;
      }

    } break;

    case contact_schedule::Status::Walk: {
      // If we were standing before and switch is completed -> switch to walk.
      if (locomotionState_ == LocomotionState::SwitchToWalkingForwards) {
        locomotionState_ = LocomotionState::WalkingForwards;
      } else if (locomotionState_ == LocomotionState::SwitchToWalkingBackwards) {
        locomotionState_ = LocomotionState::WalkingBackwards;
      }
    } break;

    case contact_schedule::Status::ForceStance: {
      // Active force stance mode and start to count the time spend in this mode.
      if (locomotionState_ != LocomotionState::ForceStance) {
        locomotionState_ = LocomotionState::ForceStance;
        chronoTimer_.pinTime();
        MELO_INFO_STREAM("[MissionControlZmp::updateLocomotionState] Enable force-stance according to gait status.");
      }
    } break;
    default:
      break;
  }

  return true;
}

bool MissionControlZmp::triggerSwitches() {
  // Update trigger switches.
  const std::string activeGaitName = contactSchedule_.getActiveGaitName();
  Eigen::Matrix<double, 9, 1> triggerSignals = Eigen::Matrix<double, 9, 1>::Zero();
  triggerSignals.segment<3>(0) = desRobotTwist_->getTranslationalVelocity().toImplementation();
  triggerSignals.segment<3>(3) = desRobotTwist_->getRotationalVelocity().toImplementation();
  // Add measured base velocity into the trigger signals only for maintained gaits.
  // Certain gaits such as flying trot have large base velocities which will cause ANYmal to continue walking in place.
  if (activeGaitName == "walk" || activeGaitName == "run" || activeGaitName == "crawl") {
    triggerSignals.segment<3>(6) = filteredBaseLinearVelocitiesInControlFrame_.getFilteredValue().toImplementation();
  }
  // Trigger switch to walk/stand.
  switch (locomotionState_) {
    case (LocomotionState::Standing): {
      if (!triggerSwitchToWalk(triggerSignals)) {
        return false;
      }
      if (!triggerSwitchToForceWalk(shrinkPolygonFactor_)) {
        return false;
      }
    } break;

    case (LocomotionState::WalkingForwards): {
      if (!triggerSwitchWalkingDirection(true)) {
        return false;
      }
      if (!triggerSwitchToStand(triggerSignals, true)) {
        return false;
      }
    } break;

    case (LocomotionState::WalkingBackwards): {
      if (!triggerSwitchWalkingDirection(false)) {
        return false;
      }
      if (!triggerSwitchToStand(triggerSignals, false)) {
        return false;
      }
    } break;

    case (LocomotionState::ForceWalk): {
      if (!triggerSwitchForceWalkToStand()) {
        return false;
      }
    } break;

    case (LocomotionState::ForceStance): {
      // Make sure the gait has not changed the status itself.
      if (contactSchedule_.getStatus() != contact_schedule::Status::ForceStance) {
        MELO_WARN_STREAM("[MissionControlZmp::triggerSwitches] Gait should be in force-stance mode!");
        contactSchedule_.stopGait();
      }

      // Release force stance mode after we spend certain amount of time in that mode.
      if (chronoTimer_.getElapsedTimeSec() > timeThresholdForceStance_) {
        MELO_INFO_STREAM("[MissionControlZmp::triggerSwitches] Free force stance because stance time was larger than threshold "
                         << timeThresholdForceStance_ << ".");
        contactSchedule_.freeForceStance();
        locomotionState_ = LocomotionState::Standing;
      }
    } break;

    default:
      break;
  }

  return true;
}

template <typename Derived>
bool MissionControlZmp::triggerSwitchToWalk(const Eigen::MatrixBase<Derived>& triggerSignals) {
  if (contactSchedule_.getStatus() == contact_schedule::Status::ExecuteOneCycle) {
    return true;
  }

  /**************************************************************
   * Case 1: Switch to walk due to increased reference velocity *
   **************************************************************/
  // Check if we should start switch-to-walk.
  if (!(triggerSignals.head(6).array().abs() > 1e-4).isZero()) {
    return switchWalk(desRobotTwist_->getTranslationalVelocity().x() >= 0.0, false);
  }
  /**************************************************************/

  return true;
}

bool MissionControlZmp::triggerSwitchToForceWalk(double shrinkPolygonFactor) {
  if (contactSchedule_.getStatus() == contact_schedule::Status::ExecuteOneCycle) {
    return true;
  }
  /**********************************************************
   * Case 2: Switch to walk due to increased velocity error *
   **********************************************************/

  VertexList positionBaseToEndEffectorsInBaseFrame;
  const RotationQuaternion& orientationWorldToBase = wholeBody_.getTorso().getMeasuredState().getOrientationWorldToBase();
  Position positionBaseToTargetCoMInWorldFrame;
  getPositionBaseToTargetPointInWorldFrame(wholeBody_, positionBaseToTargetCoMInWorldFrame);
  const Position positionBaseToTargetCoMInBaseFrame = orientationWorldToBase.rotate(positionBaseToTargetCoMInWorldFrame);
  // Create vector of current EE positions
  for (auto leg : wholeBody_.getLegs()) {
    const Position& positionBaseToEndEffectorInBaseFrame = leg->getFoot().getStateMeasured().getPositionBaseToEndEffectorInBaseFrame();
    positionBaseToEndEffectorsInBaseFrame.emplace_back(positionBaseToEndEffectorInBaseFrame.toImplementation().head<2>());
  }
  // Compute the largest rectangle that can be inscribed inside the quadrilateral.
  // The rectangle must have one side parallel to the heading direction of the robot.
  robot_utils::geometry::Tetragon inscribingPolygonInBaseFrame(getLargestInscribingRectangle(positionBaseToEndEffectorsInBaseFrame));
  // Shrink the polygon by given factor and update the line coefficients.
  inscribingPolygonInBaseFrame.resizeVerticesTowardsCentroid(shrinkPolygonFactor);
  inscribingPolygonInBaseFrame.updateLineCoefficients();

  // Check if the WB CoM is within support margin distance from any side of the polygon
  auto isCoMInsidePolygon = inscribingPolygonInBaseFrame.isPointInPolygon(positionBaseToTargetCoMInBaseFrame.toImplementation().head<2>());

  if (std::fabs(contactSchedule_.getLumpedUnFilteredVelocityError()) > velocityErrorThresholdStart_ || !isCoMInsidePolygon) {
    auto startBalancingGait = true;
    return switchWalk(true, startBalancingGait);
  }
  /**********************************************************/

  return true;
}

bool MissionControlZmp::triggerSwitchWalkingDirection(bool walkingForwards) {
  // If gait switcher is running, we cannot switch to stance.
  if (contactSchedule_.isGaitNonPeriodic()) {
    return true;
  }

  if (isSwitchingToStand_) {
    isSwitchingDirection_ = false;
    return true;
  }
  const double sign = (walkingForwards ? 1.0 : -1.0);

  // Check if we should start switching direction.
  const bool shouldChangeWalkingDirection = sign * desRobotTwist_->getTranslationalVelocity().x() <= -switchDirectionVelRefThresholdStart_;
  if (shouldChangeWalkingDirection && !didPin_) {
    chronoTimer_.pinTime();
    didPin_ = true;
    isSwitchingDirection_ = true;
  }

  // Check if we should abort switching direction.
  const bool shouldNotChangeWalkingDirection = sign * desRobotTwist_->getTranslationalVelocity().x() > switchDirectionVelRefThresholdEnd_;
  if (shouldNotChangeWalkingDirection && didPin_) {
    didPin_ = false;
  }

  // Check if direction change is definitive.
  if (didPin_ && chronoTimer_.getElapsedTimeSec() > timeThresholdSwitchDirection_) {
    contact_schedule::ContactScheduleSwitchStatus status;
    if (contactSchedule_.isActiveGaitReversible()) {
      // Reversible gaits can directly change direction.
      if (!switchWalkingDirection(status, !walkingForwards)) {
        return false;
      }
    } else {
      // Non-reversible gaits need to stand to flip the contact schedule.
      if (!switchToStand(status)) {
        return false;
      }
    }
    didPin_ = false;
    isSwitchingDirection_ = false;
  }

  return true;
}

template <typename Derived>
bool MissionControlZmp::triggerSwitchToStand(const Eigen::MatrixBase<Derived>& triggerSignals, bool useGaitPatternFw) {
  // If gait switcher is running, we cannot switch to stance.
  if (contactSchedule_.isGaitNonPeriodic()) {
    return true;
  }

  if (isSwitchingDirection_) {
    isSwitchingToStand_ = false;
    return true;
  }

  // Check if we should start switch-to-stand.
  const bool isCommandZero = (triggerSignals.head(6).array() == 0.0).isOnes();
  const bool isAllTriggerSignalsBelowStartThreshold = (triggerSignals.array().abs() < switchToStandVelRefThresholdStart_).isOnes();
  const bool isAnyTriggerSignalAboveEndThreshold = !(triggerSignals.array().abs() > switchToStandVelRefThresholdEnd_).isZero();

  const bool shouldGoToStand = isCommandZero && isAllTriggerSignalsBelowStartThreshold;
  if (shouldGoToStand && !didPin_) {
    chronoTimer_.pinTime();
    didPin_ = true;
    isSwitchingToStand_ = true;
  }

  // Check if we should abort switch-to-stand.
  const bool shouldNotGoToStand = !isCommandZero || isAnyTriggerSignalAboveEndThreshold;
  if (shouldNotGoToStand && didPin_) {
    didPin_ = false;
    isSwitchingToStand_ = false;
  }

  // Check if switch-to-stand is definitive.
  if (didPin_ && chronoTimer_.getElapsedTimeSec() > timeThresholdSwitchToStand_) {
    contact_schedule::ContactScheduleSwitchStatus status;
    if (!switchToStand(status)) {
      return false;
    }
    didPin_ = false;
    isSwitchingToStand_ = false;
  }

  return true;
}

bool MissionControlZmp::triggerSwitchForceWalkToStand() {
  // Check if we should start switch-to-stand.
  if (!didPin_ && std::fabs(contactSchedule_.getLumpedFilteredVelocityError()) < velocityErrorThresholdEnd_) {
    chronoTimer_.pinTime();
    didPin_ = true;
  }

  // Check if we should abort switch-to-stand.
  if (didPin_ && std::fabs(contactSchedule_.getLumpedFilteredVelocityError()) > velocityErrorThresholdStart_) {
    didPin_ = false;
  }

  // Check if switch-to-stand is definitive.
  if (didPin_ && chronoTimer_.getElapsedTimeSec() > timeThresholdVelocityError_) {
    contact_schedule::ContactScheduleSwitchStatus status;
    if (!switchToStand(status)) {
      return false;
    }
    didPin_ = false;
  }

  return true;
}

bool MissionControlZmp::setUnfilteredDesiredBaseCommandsInControlFrame(double dt) {
  if (!updateLocomotionState()) {
    MELO_WARN_STREAM("[MissionControlZmp::advance] Failed to update locomotion state.");
    return false;
  }

  switch (locomotionMode_) {
    case LocomotionMode::ModeStanding: {
      // Clip desired velocity to box constraints.
      enforceWithinBoundaries(desRobotPose_->getPosition());
      EulerAnglesZyx desRobotOrientation(desRobotPose_->getRotation());
      desRobotOrientation = EulerAnglesZyx(-desRobotOrientation.yaw(), desRobotOrientation.pitch(), desRobotOrientation.roll());

      // Set unfiltered reference signals.
      speedFilter_.setUnfilteredDesiredPositionMiddleOfFeetToBaseInWorldFrame(desRobotPose_->getPosition());
      speedFilter_.setUnfilteredDesiredOrientationOffset(RotationQuaternion(desRobotOrientation.getUnique()));
      speedFilter_.setUnfilteredDesiredBaseTwistInControlFrame(Twist());
    } break;

    case LocomotionMode::ModeWalking: {
      if (!triggerSwitches()) {
        MELO_WARN_STREAM("[MissionControlZmp::advance] Failed to trigger switches.");
        return false;
      }

      // Set unfiltered reference signals.
      speedFilter_.setUnfilteredDesiredOrientationOffset(RotationQuaternion());
      speedFilter_.setUnfilteredDesiredPositionMiddleOfFeetToBaseInWorldFrame(Position());
      if (!setUnfilteredDesiredBaseTwistInControlFrame()) {
        MELO_WARN_STREAM("[MissionControlZmp::advance] Failed set unfiltered desired base twist.");
        return false;
      }
    } break;

    case LocomotionMode::ModeStopping: {
      // Set references to 0.
      speedFilter_.setUnfilteredDesiredPositionMiddleOfFeetToBaseInWorldFrame(Position());
      speedFilter_.setUnfilteredDesiredOrientationOffset(RotationQuaternion());
      speedFilter_.setUnfilteredDesiredBaseTwistInControlFrame(Twist());
    } break;

    default:
      break;
  }
  return true;
}

bool MissionControlZmp::setUnfilteredDesiredBaseTwistInControlFrame() {
  if (locomotionMode_ != LocomotionMode::ModeWalking) {
    speedFilter_.setUnfilteredDesiredBaseTwistInControlFrame(Twist());
    return false;
  }

  switch (locomotionState_) {
    // Case standing -> zero twist.
    case LocomotionState::Standing:
    case LocomotionState::ForceStance:
    case LocomotionState::ForceWalk:
    case LocomotionState::SwitchToStanceFromForceWalk: {
      speedFilter_.setUnfilteredDesiredBaseTwistInControlFrame(Twist());
    } break;

    // Case switch to stance -> desired twist.
    case LocomotionState::SwitchToStanceFromForwards:
    case LocomotionState::SwitchToStanceFromBackwards: {
      speedFilter_.setUnfilteredDesiredBaseTwistInControlFrame(Twist());
    } break;

    // Case switch to walk -> desired twist.
    case LocomotionState::SwitchToWalkingForwards:
    case LocomotionState::SwitchToWalkingBackwards: {
      const bool remainsForwards =
          (locomotionState_ == LocomotionState::SwitchToWalkingForwards && desRobotTwist_->getTranslationalVelocity().x() > 1e-4);
      const bool remainsBackwards =
          (locomotionState_ == LocomotionState::SwitchToWalkingBackwards && desRobotTwist_->getTranslationalVelocity().x() < -1e-4);
      if (remainsForwards || remainsBackwards) {
        Twist saturatedRobotTwist = *desRobotTwist_;
        enforceWithinBoundaries(saturatedRobotTwist.getTranslationalVelocity(), velocityClippingType_);
        enforceWithinBoundaries(saturatedRobotTwist.getRotationalVelocity(), velocityClippingType_);
        speedFilter_.setUnfilteredDesiredBaseTwistInControlFrame(saturatedRobotTwist);
      } else {
        Twist saturatedRobotTwist = *desRobotTwist_;
        saturatedRobotTwist.getTranslationalVelocity().x() = 0.0;
        enforceWithinBoundaries(saturatedRobotTwist.getTranslationalVelocity(), velocityClippingType_);
        enforceWithinBoundaries(saturatedRobotTwist.getRotationalVelocity(), velocityClippingType_);
        speedFilter_.setUnfilteredDesiredBaseTwistInControlFrame(saturatedRobotTwist);
      }
    } break;

    // Case walking -> desired twist.
    case LocomotionState::WalkingForwards:
    case LocomotionState::WalkingBackwards: {
      const bool remainsForwards =
          (locomotionState_ == LocomotionState::WalkingForwards && desRobotTwist_->getTranslationalVelocity().x() > 1e-4);
      const bool remainsBackwards =
          (locomotionState_ == LocomotionState::WalkingBackwards && desRobotTwist_->getTranslationalVelocity().x() < -1e-4);
      Twist saturatedRobotTwist = *desRobotTwist_;
      enforceWithinBoundaries(saturatedRobotTwist.getTranslationalVelocity(), velocityClippingType_);
      enforceWithinBoundaries(saturatedRobotTwist.getRotationalVelocity(), velocityClippingType_);
      speedFilter_.setUnfilteredDesiredBaseTwistInControlFrame(saturatedRobotTwist);
    } break;

    default: {
      MELO_WARN_STREAM("[MissionControlZmp::setUnfilteredDesiredBaseTwistInControlFrame] Unknown locomotion state.");
      return false;
    } break;
  }

  return true;
}

const Twist& MissionControlZmp::getDesiredBaseTwistInControlFrame() const {
  return speedFilter_.getDesiredBaseTwistInControlFrame();
}

const Twist& MissionControlZmp::getMaximumBaseTwistInControlFrame() const {
  return speedFilter_.getMaximumBaseTwistInControlFrame();
}

const Pose& MissionControlZmp::getMinimalPoseOffset() const {
  return speedFilter_.getMinimalPoseOffset();
}

const Pose& MissionControlZmp::getMaximalPoseOffset() const {
  return speedFilter_.getMaximalPoseOffset();
}

double MissionControlZmp::interpolateJoystickAxis(double value, double minValue, double maxValue) {
  return 0.5 * (maxValue - minValue) * (value + 1.0) + minValue;
}

bool MissionControlZmp::switchToStand(contact_schedule::ContactScheduleSwitchStatus& status) {
  if (locomotionState_ != LocomotionState::WalkingBackwards && locomotionState_ != LocomotionState::WalkingForwards &&
      locomotionState_ != LocomotionState::ForceWalk) {
    MELO_DEBUG_STREAM("[MissionControlZmp::switchToStand] Locomotion state is not walking. Switching to stance has no effect.");
    status = contact_schedule::ContactScheduleSwitchStatus::Running;
    return true;
  }

  if (!contactSchedule_.switchToStand(status)) {
    MELO_WARN_STREAM("[MissionControlZmp::switchToStand] Failed to switch to stand!");
    return false;
  }

  switch (locomotionState_) {
    case LocomotionState::WalkingForwards: {
      locomotionState_ = LocomotionState::SwitchToStanceFromForwards;
    } break;
    case LocomotionState::WalkingBackwards: {
      locomotionState_ = LocomotionState::SwitchToStanceFromBackwards;
    } break;
    case LocomotionState::ForceWalk: {
      locomotionState_ = LocomotionState::SwitchToStanceFromForceWalk;
    } break;
    default: {
      MELO_DEBUG_STREAM("[MissionControlZmp::switchToStand] Undefined locomotion state.");
      status = contact_schedule::ContactScheduleSwitchStatus::NotFound;
      return false;
    }
  }
  return true;
}

bool MissionControlZmp::switchWalkingDirection(contact_schedule::ContactScheduleSwitchStatus& status, bool useGaitPatternFw) {
  if (locomotionState_ != LocomotionState::WalkingBackwards && locomotionState_ != LocomotionState::WalkingForwards) {
    MELO_WARN_STREAM("[MissionControlZmp::switchWalkingDirection] Locomotion state is not walking. Cannot switch walking direction");
    return false;
  }

  if (!contactSchedule_.switchToWalk(status, useGaitPatternFw, false)) {
    MELO_WARN_STREAM("[MissionControlZmp::switchWalkingDirection] Failed to switch walking direction");
    return false;
  }

  switch (locomotionState_) {
    case LocomotionState::WalkingForwards: {
      locomotionState_ = LocomotionState::WalkingBackwards;
    } break;
    case LocomotionState::WalkingBackwards: {
      locomotionState_ = LocomotionState::WalkingForwards;
    } break;
    default: {
      MELO_WARN_STREAM("[MissionControlZmp::switchWalkingDirection] Undefined locomotion state.");
      status = contact_schedule::ContactScheduleSwitchStatus::NotFound;
      return false;
    } break;
  }
}

bool MissionControlZmp::switchWalk(bool useGaitPatternFw, bool startGaitForBalancing) {
  contact_schedule::ContactScheduleSwitchStatus status;
  if (locomotionState_ != LocomotionState::Standing) {
    MELO_DEBUG_STREAM("[MissionControlZmp::switchWalk] Locomotion state is not standing. Switching to walk has no effect.");
    return true;
  }

  if (!contactSchedule_.switchToWalk(status, useGaitPatternFw, startGaitForBalancing)) {
    MELO_WARN_STREAM("[MissionControlZmp::switchWalk] Failed to switch to walk");
    return false;
  }

  if (!startGaitForBalancing) {
    locomotionState_ = (useGaitPatternFw ? LocomotionState::SwitchToWalkingForwards : LocomotionState::SwitchToWalkingBackwards);
  } else {
    locomotionState_ = LocomotionState::ForceWalk;
  }
  return true;
}

void MissionControlZmp::setTargetLocomotionMode(const LocomotionMode& locomotionMode) {
  targetLocomotionMode_ = locomotionMode;
}

bool MissionControlZmp::updateParameters(const std::string& gaitName) {
  // Update gait depending parameters.
  Twist maximumBaseTwistInControlFrame = getMaximumBaseTwistInControlFrameOfGait(gaitName);
  speedFilter_.setMaximumBaseTwistInControlFrame(maximumBaseTwistInControlFrame);
  wholeBody_.getTorsoPtr()->getPropertiesPtr()->setMaximumBaseTwistInControlFrame(speedFilter_.getMaximumBaseTwistInControlFrame());
  return true;
}

Twist MissionControlZmp::getMaximumBaseTwistInControlFrameOfGait(const std::string& gaitName) const {
  // Update gait depending parameters.
  const auto& params = params_.getParams(gaitName);
  Twist maximumBaseTwistInControlFrame = Twist();
  maximumBaseTwistInControlFrame.getTranslationalVelocity().x() = params.maxHeadinVel_;
  maximumBaseTwistInControlFrame.getTranslationalVelocity().y() = params.maxLateralVel_;
  maximumBaseTwistInControlFrame.getRotationalVelocity().z() = params.maxTurningVel_;
  return maximumBaseTwistInControlFrame;
}

bool MissionControlZmp::triggerParameterSwitch() {
  /*
   * Update gait depending parameters. Update to the
   * > active gait if it has a larger max heading velocity,
   * > desired gait if it has a larger max heading velocity.
   */
  const bool isActiveMaxVelLarger = (params_.getParams(contactSchedule_.getActiveGaitName()).maxHeadinVel_ >
                                     params_.getParams(contactSchedule_.getDesiredGaitName()).maxHeadinVel_);

  if (previousActiveGaitId_ != contactSchedule_.getActiveGaitId() && previousDesiredGaitId_ != contactSchedule_.getDesiredGaitId()) {
    previousActiveGaitId_ = contactSchedule_.getActiveGaitId();
    previousDesiredGaitId_ = contactSchedule_.getDesiredGaitId();
    return updateParameters(contactSchedule_.getActiveGaitName());
  } else if (previousActiveGaitId_ != contactSchedule_.getActiveGaitId()) {
    previousActiveGaitId_ = contactSchedule_.getActiveGaitId();
    if (!isActiveMaxVelLarger) {
      return updateParameters(contactSchedule_.getActiveGaitName());
    }
  } else if (previousDesiredGaitId_ != contactSchedule_.getDesiredGaitId()) {
    previousDesiredGaitId_ = contactSchedule_.getDesiredGaitId();
    if (isActiveMaxVelLarger) {
      return updateParameters(contactSchedule_.getDesiredGaitName());
    }
  }

  return true;
}

bool MissionControlZmp::isStopped() const noexcept {
  // TODO: (paco) evaluate accelerations!
  bool isBaseAtRest = (wholeBody_.getTorso().getMeasuredState().getAngularVelocityBaseInBaseFrame().toImplementation().norm() < 0.01) &&
                      (wholeBody_.getTorso().getMeasuredState().getLinearVelocityBaseInBaseFrame().toImplementation().norm() < 0.01);
  return (locomotionMode_ == LocomotionMode::ModeStopping && locomotionState_ == LocomotionState::Standing && isBaseAtRest);
}

const MissionControlZmp::LocomotionMode& MissionControlZmp::getLocomotionMode() const {
  return locomotionMode_;
}

const contact_schedule::ContactScheduleSwitchStatus& MissionControlZmp::getLocomotionModeSwitchStatus() const {
  return modeSwitchStatus_;
}

void MissionControlZmp::setLocomotionMode(const LocomotionMode& locomotionMode) {
  locomotionMode_ = locomotionMode;
  modeSwitchCondVar_.notify_all();
}

bool MissionControlZmp::waitForLocomotionModeSwitch(const LocomotionMode& desiredLocomotionMode,
                                                    const std::chrono::duration<int, std::milli>& timeout) {
  if (locomotionMode_ == desiredLocomotionMode) {
    return true;
  }
  std::unique_lock<std::mutex> lck(modeSwitchMutex_);
  if (!modeSwitchCondVar_.wait_for(
          lck, timeout, [desiredLocomotionMode, &locomotionMode = locomotionMode_] { return locomotionMode == desiredLocomotionMode; })) {
    MELO_WARN_STREAM("[MissionControlZmp::waitForLocomotionModeSwitch] Could not switch to "
                     << modeNameMap_.at(desiredLocomotionMode) << " within " << timeout.count() << " milliseconds");
    return false;
  }
  return (locomotionMode_ == desiredLocomotionMode);
}

Tetragon MissionControlZmp::getLargestInscribingRectangle(VertexList& vertices) {
  // Initialization.
  double minAlongPlusX = std::numeric_limits<double>::infinity();
  double maxAlongMinusX = -std::numeric_limits<double>::infinity();
  double minAlongPlusY = std::numeric_limits<double>::infinity();
  double maxAlongMinusY = -std::numeric_limits<double>::infinity();

  for (const Eigen::Vector2d& vertex : vertices) {
    // Compute projections along Unit X and Y axes
    const double pX = vertex.x();
    const double pY = vertex.y();

    // Compute the min absolute x and y value in each halfspace.
    // This provides us the vertices of the largest inscribing rectangle.
    minAlongPlusX = pX >= 0.0 ? std::min(pX, minAlongPlusX) : minAlongPlusX;
    minAlongPlusY = pY >= 0.0 ? std::min(pY, minAlongPlusY) : minAlongPlusY;
    maxAlongMinusX = pX < 0.0 ? std::max(pX, maxAlongMinusX) : maxAlongMinusX;
    maxAlongMinusY = pY < 0.0 ? std::max(pY, maxAlongMinusY) : maxAlongMinusY;
  }

  // Vertices in the counter clockwise order (-,-) --> (+,-) --> (+,+) --> (-,+)
  VertexList inscriberVertices = {Eigen::Vector2d(maxAlongMinusX, maxAlongMinusY), Eigen::Vector2d(minAlongPlusX, maxAlongMinusY),
                                  Eigen::Vector2d(minAlongPlusX, minAlongPlusY), Eigen::Vector2d(maxAlongMinusX, minAlongPlusY)};
  return Tetragon(inscriberVertices, VertexOrder::CounterClockWise);
}

LinearVelocity MissionControlZmp::getMaximumVelocityOffsetOnSlope(const Eigen::Vector3d& maximumVelocity) const {
  const RotationQuaternion& orientationControlToWorld =
      wholeBody_.getTorso().getMeasuredState().inControlFrame().getOrientationWorldToControl().inverted();

  // Get the roll and pitch of the terrain by converting the rotation in terms of ZYX Euler angles (arbitrary choice of rotation sequence).
  EulerAnglesZyx orientationControlToWorldEulerAngleZyx(orientationControlToWorld);
  const double terrainRoll = orientationControlToWorldEulerAngleZyx.getUnique().roll();
  // Sign of the pitch should be flipped so that angle is positive when the robot is standing/walking/facing uphill considering the
  // direction of our Y axis.
  const double terrainPitch = orientationControlToWorldEulerAngleZyx.getUnique().pitch() * (-1.0);

  auto roundToZeroIfSmall = [](double x) { return (abs(x) < 1e-2) ? 0.0 : x; };
  auto radToDeg = [](double x) { return x * 180.0 / M_PI; };
  return LinearVelocity(roundToZeroIfSmall(velocityOffsetMultiplierForSlopesPerDeg_.x() * maximumVelocity.x() * radToDeg(terrainPitch)),
                        roundToZeroIfSmall(velocityOffsetMultiplierForSlopesPerDeg_.y() * maximumVelocity.y() * radToDeg(terrainRoll)),
                        0.0);
}

bool MissionControlZmp::addVariablesToLog(const std::string& ns) const {
  signal_logger::add(filteredBaseLinearVelocitiesInControlFrame_.getFilteredValue(), "filteredBaseLinearVelocityInControlFrame", ns);
  signal_logger::add(locomotionMode_, "locomotionMode", ns);
  signal_logger::add(locomotionState_, "locomotionState", ns);
  signal_logger::add(modeSwitchStatus_, "modeSwitchStatus", ns);
  signal_logger::add(targetLocomotionMode_, "targetLocomotionMode", ns);
  signal_logger::add(linearVelocityOffsetOnSlope_.getFilteredValue(), "linearVelocityOffsetOnSlope", ns);
  contactSchedule_.addVariablesToLog(ns + "/contactSchedule");
  speedFilter_.addVariablesToLog(ns + "/speedFilter");
}

} /* namespace loco */
