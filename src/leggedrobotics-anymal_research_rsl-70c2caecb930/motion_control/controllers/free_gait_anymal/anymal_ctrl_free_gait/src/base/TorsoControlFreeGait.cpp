/*
 * TorsoControlFreeGait.cpp
 *
 *  Created on: Jan 14, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "anymal_ctrl_free_gait/base/TorsoControlFreeGait.hpp"
#include <loco/torso_control/TorsoControlDynamicGaitFreePlane.hpp>
#include <robot_utils/math/math.hpp>

// Free Gait
#include <anymal_ctrl_free_gait/base/AdapterLoco.hpp>
#include <free_gait_core/free_gait_core.hpp>

// STD
#include <cmath>
#include <exception>
#include <limits>

// Message Logger
#include <message_logger/message_logger.hpp>

// Anymal model
#include <anymal_model/AnymalModel.hpp>

namespace loco {

TorsoControlFreeGait::TorsoControlFreeGait(WholeBody& wholeBody, anymal_model::AnymalModel& anymalModelDesired, TerrainModelBase& terrain,
                                           free_gait::Executor& executor, ComSupportControlBase& comControl)
    : TorsoControlGaitContainer(wholeBody, terrain, comControl),
      anymalModelDesired_(anymalModelDesired),
      executor_(executor),
      loadFactorLoadDuration_(0.0),
      loadFactorLowerBound_(0.0),
      allowEarlyTouchDown_(true) {
  for (const auto& leg : wholeBody.getLegs()) {
    durationSinceLegInSupport_[AD::mapKeyIdToKeyEnum<AD::LimbEnum>(leg->getLimbUInt())] = 0.0;
  }
}

bool TorsoControlFreeGait::initialize(double dt) {
  for (const auto& leg : legs_) {
    durationSinceLegInSupport_[AD::mapKeyIdToKeyEnum<AD::LimbEnum>(leg->getLimbUInt())] = std::numeric_limits<double>::infinity();
  }
  return reset();
}

bool TorsoControlFreeGait::reset() {
  desiredPoseInWorld_.getPosition() = wholeBody_.getTorso().getMeasuredState().getPositionWorldToBaseInWorldFrame();
  desiredPoseInWorld_.getRotation() = wholeBody_.getTorso().getMeasuredState().getOrientationWorldToBase().inverted();
  setDesiredPose();
  setDesiredVelocity();
  return true;
}

bool TorsoControlFreeGait::advance(double dt) {
  if (!executor_.isInitialized()) {
    throw std::runtime_error("[TorsoControlFreeGait]: Free gait executor not initialized!");
  }

  if (executor_.getState().getControlSetup(free_gait::BranchEnum::BASE).at(free_gait::ControlLevel::Position)) {
    desiredPoseInWorld_.getPosition() = executor_.getState().getPositionWorldToBaseInWorldFrame();
    desiredPoseInWorld_.getRotation() = executor_.getState().getOrientationBaseToWorld();
  } else {
    desiredPoseInWorld_.getPosition() = wholeBody_.getTorso().getMeasuredState().getPositionWorldToBaseInWorldFrame();
    desiredPoseInWorld_.getRotation() = wholeBody_.getTorso().getMeasuredState().getOrientationWorldToBase().inverted();
  }

  if (executor_.getState().getControlSetup(free_gait::BranchEnum::BASE).at(free_gait::ControlLevel::Velocity)) {
    desiredTwistInBase_.getTranslationalVelocity() = wholeBody_.getTorso().getMeasuredState().getOrientationWorldToBase().rotate(
        executor_.getState().getLinearVelocityBaseInWorldFrame());
    desiredTwistInBase_.getRotationalVelocity() = executor_.getState().getAngularVelocityBaseInBaseFrame();
  } else {
    desiredTwistInBase_.setZero();
  }

  // Set desired base pose in control frame.
  setDesiredPose();
  setDesiredVelocity();
  setDesiredLoadFactors(dt);

  // For logging.
  loco::EulerAnglesZyx desiredOrientationEulerAnglesZyxBaseToWorld(
      (wholeBody_.getTorso().getDesiredState().getOrientationControlToBase() *
       wholeBody_.getTorso().getMeasuredState().inControlFrame().getOrientationWorldToControl())
          .inverted());
  desiredOrientationEulerAnglesZyxBaseToWorld.setUnique();
  wholeBody_.getTorsoPtr()->getDesiredStatePtr()->setOrientationEulerAnglesZyxBaseToWorld(desiredOrientationEulerAnglesZyxBaseToWorld);

  return true;
}

void TorsoControlFreeGait::setDesiredLoadFactors(double dt) {
  for (auto leg : legs_) {
    const auto& limbEnum = AD::mapKeyIdToKeyEnum<AD::LimbEnum>(leg->getLimbUInt());

    switch (leg->getStateSwitcher().getState()) {
      case (StateSwitcher::States::StanceNormal):
      case (StateSwitcher::States::StanceSlipping):
      case (StateSwitcher::States::StanceLostContact):
      case (StateSwitcher::States::SwingEarlyTouchDown): {
        double loadFactorFromPattern;
        if (executor_.isInitialized()) {
          if (executor_.getState().isSupportLeg(limbEnum)) {
            try {
              const auto& stateLoco = dynamic_cast<const free_gait::StateLoco&>(executor_.getState());
              loadFactorFromPattern = stateLoco.getLegLoadFactor(limbEnum);
            } catch (...) {
              MELO_ERROR_STREAM("[TorsoControlFreeGait::setDesiredLoadFactors] Could not cast state!");
            }
          } else {
            loadFactorFromPattern = 1.0;
          }
        } else {
          loadFactorFromPattern = 1.0;
        }

        double loadFactorFromController =
            loadFactorLowerBound_ +
            (1.0 - loadFactorLowerBound_) * robot_utils::mapTo01Range(durationSinceLegInSupport_[limbEnum], 0.0, loadFactorLoadDuration_);
        double loadFactor = loadFactorFromPattern < loadFactorFromController ? loadFactorFromPattern : loadFactorFromController;
        leg->setLoadFactor(loadFactor);

        if (allowEarlyTouchDown_ || leg->getStateSwitcher().getState() != StateSwitcher::States::SwingEarlyTouchDown) {
          durationSinceLegInSupport_[limbEnum] += dt;
        }
        break;
      }

      case (StateSwitcher::States::SwingNormal):
      case (StateSwitcher::States::SwingLateLiftOff):
      case (StateSwitcher::States::SwingExpectingContact):
      case (StateSwitcher::States::SwingBumpedIntoObstacle):
        leg->setLoadFactor(0.0);
        durationSinceLegInSupport_[limbEnum] = 0.0;
        break;

      default:
        MELO_WARN_STREAM("[TorsoControlFreeGait::setDesiredLoadFactors] Unhandled state: "
                         << leg->getStateSwitcher().getStateNameFromEnum(leg->getStateSwitcher().getState()));
        break;
    }
  }
}

void TorsoControlFreeGait::setDesiredPose() {
  // This is required for the VMC.
  const RotationQuaternion orientationControlToWorld = torso_.getMeasuredState().inControlFrame().getOrientationWorldToControl().inverted();
  const Position targetPositionInControl = orientationControlToWorld.inverseRotate(desiredPoseInWorld_.getPosition());
  const RotationQuaternion targetOrientationInControl = orientationControlToWorld.inverted() * desiredPoseInWorld_.getRotation();
  torso_.getDesiredStatePtr()->setPositionControlToTargetInControlFrame(targetPositionInControl);
  torso_.getDesiredStatePtr()->setOrientationControlToBase(targetOrientationInControl.inverted());
  Position positionErrorInControlFrame;
  getBasePositionError(wholeBody_, positionErrorInControlFrame, torso_.getDesiredState().getPositionControlToTargetInControlFrame());
  torso_.getDesiredStatePtr()->setPositionErrorInControlFrame(positionErrorInControlFrame);
}

void TorsoControlFreeGait::setDesiredVelocity() {
  const RotationQuaternion orientationBaseToControl = torso_.getMeasuredState().inControlFrame().getOrientationControlToBase().inverted();
  const LinearVelocity linearVelocityBaseInControl = orientationBaseToControl.rotate(desiredTwistInBase_.getTranslationalVelocity());
  torso_.getDesiredStatePtr()->setLinearVelocityTargetInControlFrame(linearVelocityBaseInControl);
  const LocalAngularVelocity angularVelocityBaseInControl = orientationBaseToControl.rotate(desiredTwistInBase_.getRotationalVelocity());
  torso_.getDesiredStatePtr()->setAngularVelocityBaseInControlFrame(angularVelocityBaseInControl);
  LinearVelocity linearVelocityErrorInControlFrame;
  getBaseLinearVelocityError(wholeBody_, linearVelocityErrorInControlFrame,
                             torso_.getDesiredState().getLinearVelocityTargetInControlFrame());
  torso_.getDesiredStatePtr()->setLinearVelocityErrorInControlFrame(linearVelocityErrorInControlFrame);
}

bool TorsoControlFreeGait::loadParameters(const TiXmlHandle& handle) {
  TiXmlHandle handleToTerrainParameters(handle.FirstChild("TorsoControl").FirstChild("LoadFactor"));

  TiXmlElement* pElem = handleToTerrainParameters.Element();
  if (pElem == nullptr) {
    printf("[TorsoControlFreeGait::loadParameters] Could not find TorsoControl::LoadFactor section in parameter file.\n");
    return false;
  }

  if (pElem->QueryDoubleAttribute("loadDuration", &loadFactorLoadDuration_) != TIXML_SUCCESS) {
    printf("[TorsoControlFreeGait::loadParameters] Could not find TorsoControl::loadDuration in parameter file.\n");
    return false;
  }

  if (pElem->QueryDoubleAttribute("lowerBound", &loadFactorLowerBound_) != TIXML_SUCCESS) {
    printf("[TorsoControlFreeGait::loadParameters] Could not find TorsoControl::lowerBound in parameter file.\n");
    return false;
  }

  if (pElem->QueryBoolAttribute("allowEarlyTouchdown", &allowEarlyTouchDown_) != TIXML_SUCCESS) {
    printf("[TorsoControlFreeGait::loadParameters] Could not find TorsoControl::allowEarlyTouchDown in parameter file.\n");
    return false;
  }

  return true;
}

bool TorsoControlFreeGait::setToInterpolated(const TorsoControlBase& torsoController1, const TorsoControlBase& torsoController2, double t) {
  return false;
}

} /* namespace loco */
