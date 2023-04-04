/*
 * AdapterAnymalPreview.cpp
 *
 *  Created on: Dec 2, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include "free_gait_anymal_common/AdapterAnymalPreview.hpp"

// ROS
#include <pluginlib/class_list_macros.h>

namespace free_gait {

AdapterAnymalPreview::AdapterAnymalPreview()
    : AdapterAnymal()
{
}

AdapterAnymalPreview::AdapterAnymalPreview(anymal_model::AnymalModel& anymalModel, std::unique_ptr<geometry_utils::TransformListener>& tfListener)
    : AdapterAnymal(anymalModel, tfListener)
{
}

bool AdapterAnymalPreview::updateExtrasAfter(const StepQueue& stepQueue, State& state)
{
  if (!writeSupportLegMotion(stepQueue, state)) return false;
  if (!setInternalDataFromState(state)) return false;
  return true;
}

bool AdapterAnymalPreview::isExecutionOk() const
{
  return true;
}

bool AdapterAnymalPreview::isLegGrounded(const LimbEnum& limb) const
{
  // todo: add mapping
  const auto contactEnum = AD::mapKeyIdToKeyEnum<AD::ContactEnum>(AD::mapKeyEnumToKeyId(limb));
  AD::ContactStateEnum contactState = anymalModel_->getContactContainer()[contactEnum]->getState();
  if (contactState == AD::ContactStateEnum::CLOSED) return true;
  return false;
}

ControlSetup AdapterAnymalPreview::getControlSetup(const BranchEnum& branch) const
{
  // TODO Ok like this?
  ControlSetup controlSetup;
  controlSetup[ControlLevel::Position] = false;
  controlSetup[ControlLevel::Velocity] = false;
  controlSetup[ControlLevel::Acceleration] = false;
  controlSetup[ControlLevel::Effort] = false;
  return controlSetup;
}

bool AdapterAnymalPreview::writeSupportLegMotion(const StepQueue& stepQueue, State& state)
{
  if (!stepQueue.active()) return true;

  const auto& step = stepQueue.getCurrentStep();
  if (!step.hasBaseMotion()) return true;

  // Save actual foot positions.
  std::unordered_map<LimbEnum, Position, EnumClassHash> footPositionsInWorld;
  for (const auto& limb : getLimbs()) {
    footPositionsInWorld[limb] = getPositionWorldToFootInWorldFrame(limb);
  }

  // Copy state since we are changing it.
  const anymal_model::AnymalState originalState(anymalModel_->getState());

  // Set desired base pose.
  anymal_model::AnymalState tempState(anymalModel_->getState());
  tempState.setPositionWorldToBaseInWorldFrame(state.getPositionWorldToBaseInWorldFrame());
  tempState.setOrientationBaseToWorld(state.getOrientationBaseToWorld());
  anymalModel_->setState(tempState, true, true, false);

  // Compute leg joints for desired base pose.
  double time = step.getTime();
  for (const auto& limb : getLimbs()) {
    if (!state.isSupportLeg(limb)) continue;
    Position footPositionInBaseFrame = transformPosition(getWorldFrameId(), "base", footPositionsInWorld[limb]);
    JointPositionsLeg jointPositions;
    if (!getLimbJointPositionsFromPositionBaseToFootInBaseFrame(footPositionInBaseFrame, limb, jointPositions)) {
      std::cerr << "Failed to compute joint positions from end effector position for " << limb << "." << std::endl;
      return false;
    }
    state.setJointPositionsForLimb(limb, jointPositions);
  }

  // TODO Can we do this more efficiently?
  // Revert to original state.
  anymalModel_->setState(originalState, true, true, false);

  return true;
}

} /* namespace free_gait */

// Declare the AdapterAnymal as a Free Gait Adapter class.
PLUGINLIB_EXPORT_CLASS(free_gait::AdapterAnymalPreview, free_gait::AdapterBase)
