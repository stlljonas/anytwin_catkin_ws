/*
 * AdapterAnymal.cpp
 *
 *  Created on: Sep 1, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "free_gait_anymal_common/AdapterAnymal.hpp"

// ROS
#include <pluginlib/class_list_macros.h>

namespace free_gait {

AdapterAnymal::AdapterAnymal()
    : AdapterBase(),
      worldFrameId_("odom"),
      baseFrameId_("base")
{
  for (const auto& limbKey : AD::getLimbKeys()) {
    limbs_.push_back(limbKey.getEnum());
  }

  for (const auto& branchKey : AD::getBranchKeys()) {
    branches_.push_back(branchKey.getEnum());
  }
}

AdapterAnymal::AdapterAnymal(anymal_model::AnymalModel& anymalModel)
    : AdapterAnymal()
{
  anymalModel_ = &anymalModel;
  copyOfState_.reset(new anymal_model::AnymalState(anymalModel_->getState()));
}

AdapterAnymal::AdapterAnymal(anymal_model::AnymalModel& anymalModel, std::unique_ptr<geometry_utils::TransformListener>& tfListener)
    : AdapterAnymal()
{
  initialize(anymalModel, tfListener);
}

void AdapterAnymal::initialize(anymal_model::AnymalModel& anymalModel, std::unique_ptr<geometry_utils::TransformListener>& tfListener)
{
  anymalModel_ = &anymalModel;
  copyOfState_.reset(new anymal_model::AnymalState(anymalModel_->getState()));
  setTfListener(tfListener);
}

void AdapterAnymal::initialize(anymal_model::AnymalModel* anymalModel, std::unique_ptr<geometry_utils::TransformListener>& tfListener)
{
  anymalModel_ = anymalModel;
  copyOfState_.reset(new anymal_model::AnymalState(anymalModel_->getState()));
  setTfListener(tfListener);
}

anymal_model::AnymalModel& AdapterAnymal::getAnymalModel()
{
  return *anymalModel_;
}

geometry_utils::TransformListener& AdapterAnymal::getTfListener()
{
  return *tfListener_;
}

bool AdapterAnymal::setTfListener(std::unique_ptr<geometry_utils::TransformListener>& tfListener)
{
  if(!tfListener) {
    return false;
  }
  tfListener_.swap(tfListener);
  return true;
}

bool AdapterAnymal::resetExtrasWithRobot(const StepQueue& stepQueue, State& state)
{
  return updateFrameTransforms();
}

bool AdapterAnymal::updateExtrasBefore(const StepQueue& stepQueue, State& state)
{
  if (stepQueue.hasSwitchedStep()) updateFrameTransforms();
  return true;
}

bool AdapterAnymal::updateExtrasAfter(const StepQueue& stepQueue, State& state)
{
  throw std::runtime_error("AdapterAnymal::updateExtrasAfter() is not implemented.");
}

const std::string& AdapterAnymal::getWorldFrameId() const
{
  return worldFrameId_;
}

const std::string& AdapterAnymal::getBaseFrameId() const
{
  return baseFrameId_;
}

const std::vector<LimbEnum>& AdapterAnymal::getLimbs() const
{
  return limbs_;
}

const std::vector<BranchEnum>& AdapterAnymal::getBranches() const
{
  return branches_;
}

LimbEnum AdapterAnymal::getLimbEnumFromLimbString(const std::string& limb) const
{
  return AD::mapKeyNameToKeyEnum<AD::LimbEnum>(limb);
}

std::string AdapterAnymal::getLimbStringFromLimbEnum(const LimbEnum& limb) const
{
  return AD::mapKeyEnumToKeyName(limb);
}

std::string AdapterAnymal::getBaseString() const
{
  return AD::getBranchKeys()[AD::BranchEnum::BASE].getName();
}

JointNodeEnum AdapterAnymal::getJointNodeEnumFromJointNodeString(const std::string& jointNode) const
{
  return AD::mapKeyNameToKeyEnum<AD::JointNodeEnum>(jointNode);
}

std::string AdapterAnymal::getJointNodeStringFromJointNodeEnum(const JointNodeEnum& jointNode) const
{
  return AD::mapKeyEnumToKeyName(jointNode);
}

bool AdapterAnymal::getLimbJointPositionsFromPositionBaseToFootInBaseFrame(
    const Position& positionBaseToFootInBaseFrame, const LimbEnum& limb,
    JointPositionsLeg& jointPositions) const
{
  Eigen::VectorXd computedJointPositions(anymal_model::AD::getNumDofLimb());
  bool success = false;
  if (useAnalyticInverseKinematics_) {
    Eigen::Vector3d computedJointPositionsTemp;
    success = anymalModel_->getLimbJointPositionsFromPositionBaseToFootInBaseFrame(computedJointPositionsTemp, positionBaseToFootInBaseFrame.toImplementation(), limb);
    computedJointPositions = computedJointPositionsTemp;
  } else {
    success = anymalModel_->getLimbJointPositionsFromLimbEnumIteratively(computedJointPositions, positionBaseToFootInBaseFrame.toImplementation(), limb);
  }

  if (success) {
    jointPositions = JointPositionsLeg(computedJointPositions);
  }
  return success;
}

Position AdapterAnymal::getPositionBaseToFootInBaseFrame(
    const LimbEnum& limb, const JointPositionsLeg& jointPositions) const
{
  // Copy state since we are changing it.
  const anymal_model::AnymalState originalState(anymalModel_->getState());

  // Set joint positions.
  anymal_model::AnymalState state;
  anymal_model::JointPositions allJointPositions;
  allJointPositions.vector().segment<AD::getNumDofLimb()>(AD::getLimbStartIndexInJ(limb)) = jointPositions.vector();
  state.setJointPositions(allJointPositions);
  anymalModel_->setState(state, true, false, false);

  // Compute position.
  Position positionBaseToFootInBaseFrame(
      anymalModel_->getPositionBodyToBody(
          AD::BodyEnum::BASE, AD::mapEnums<AD::BranchEnum>(limb),
          AD::BodyNodeEnum::FOOT, AD::CoordinateFrameEnum::BASE));

  // TODO Can we do this more efficiently?
  // Revert to original state.
  anymalModel_->setState(originalState, true, true, false);

  return positionBaseToFootInBaseFrame;
}

Position AdapterAnymal::getPositionBaseToHipInBaseFrame(const LimbEnum& limb) const
{
  return Position(
      anymalModel_->getPositionBodyToBody(AD::BodyEnum::BASE,
                                             AD::mapEnums<AD::BranchEnum>(limb),
                                             AD::BodyNodeEnum::HIP,
                                             AD::CoordinateFrameEnum::BASE));
}

bool AdapterAnymal::isExecutionOk() const
{
  throw std::runtime_error("AdapterAnymal::isExecutionOk() is not implemented.");
}

bool AdapterAnymal::isLegGrounded(const LimbEnum& limb) const
{
  throw std::runtime_error("AdapterAnymal::isLegGrounded() is not implemented.");
}

JointPositionsLeg AdapterAnymal::getJointPositionsForLimb(const LimbEnum& limb) const
{
  return JointPositionsLeg(anymalModel_->getState().getJointPositions().vector().segment<AD::getNumDofLimb()>(
      AD::getLimbStartIndexInJ(limb)));
}

JointPositions AdapterAnymal::getAllJointPositions() const
{
  return JointPositions(anymalModel_->getState().getJointPositions());
}

JointVelocitiesLeg AdapterAnymal::getJointVelocitiesForLimb(const LimbEnum& limb) const
{
  return JointVelocitiesLeg(anymalModel_->getState().getJointVelocities().vector().segment<AD::getNumDofLimb()>(
      AD::getLimbStartIndexInJ(limb)));
}

JointVelocities AdapterAnymal::getAllJointVelocities() const
{
  return JointVelocities(anymalModel_->getState().getJointVelocities());
}

JointAccelerationsLeg AdapterAnymal::getJointAccelerationsForLimb(const LimbEnum& limb) const
{
  // TODO
}

JointAccelerations AdapterAnymal::getAllJointAccelerations() const
{
  // TODO
}

JointEffortsLeg AdapterAnymal::getJointEffortsForLimb(const LimbEnum& limb) const
{
  return JointEffortsLeg(anymalModel_->getJointTorques().vector().segment<AD::getNumDofLimb()>(
      AD::getLimbStartIndexInJ(limb)));
}

JointEfforts AdapterAnymal::getAllJointEfforts() const
{
  return JointEfforts(anymalModel_->getJointTorques());
}

Position AdapterAnymal::getPositionWorldToBaseInWorldFrame() const
{
  return anymalModel_->getState().getPositionWorldToBaseInWorldFrame();
}

RotationQuaternion AdapterAnymal::getOrientationBaseToWorld() const
{
  return anymalModel_->getState().getOrientationBaseToWorld();
}

LinearVelocity AdapterAnymal::getLinearVelocityBaseInWorldFrame() const
{
  return anymalModel_->getState().getLinearVelocityBaseInWorldFrame();
}

LocalAngularVelocity AdapterAnymal::getAngularVelocityBaseInBaseFrame() const
{
  return anymalModel_->getState().getAngularVelocityBaseInBaseFrame();
}

LinearAcceleration AdapterAnymal::getLinearAccelerationBaseInWorldFrame() const
{
  return LinearAcceleration(); // TODO Maybe ExtendedRobotState?
}

AngularAcceleration AdapterAnymal::getAngularAccelerationBaseInBaseFrame() const
{
  return AngularAcceleration(); // TODO Maybe ExtendedRobotState?
}

Position AdapterAnymal::getPositionBaseToFootInBaseFrame(const LimbEnum& limb) const
{
  auto branch = AD::mapEnums<AD::BranchEnum>(limb);
  return Position(anymalModel_->getPositionBodyToBody(AD::BodyEnum::BASE, branch,
                  AD::BodyNodeEnum::FOOT,
                  AD::CoordinateFrameEnum::BASE));
}

Position AdapterAnymal::getPositionWorldToFootInWorldFrame(const LimbEnum& limb) const
{
  Position position;
  anymalModel_->getPositionWorldToBody(position.toImplementation(),
                                          AD::mapEnums<BranchEnum>(limb),
                                          AD::BodyNodeEnum::FOOT,
                                          AD::CoordinateFrameEnum::WORLD);
  return position;
}

Position AdapterAnymal::getCenterOfMassInWorldFrame() const
{
  return Position(anymalModel_->getPositionWorldToCom(AD::CoordinateFrameEnum::WORLD));
}

void AdapterAnymal::getAvailableFrameTransforms(std::vector<std::string>& frameTransforms) const
{
  return;
}

bool AdapterAnymal::frameIdExists(const std::string& frameId) const
{
  if (AdapterBase::frameIdExists(frameId)) {
    return true;
  }
  if(!tfListener_) {
    MELO_ERROR_STREAM("[AdapterAnymal::frameIdExists] Transform Listener not initialized!");
    return false;
  }
  // Check latest transform with no timeout
  return tfListener_->canTransform(worldFrameId_, frameId, any_measurements::Time(0.0), 0.0);
}

bool AdapterAnymal::getFrameTransform(const std::string& frameId, Pose& pose) const
{
  if (!frameIdExists(frameId)) {
    MELO_ERROR_STREAM("Frame '" << frameId << "' is not available.")
    return false;
  }

  if (frameId == worldFrameId_) {
    pose = Pose().setIdentity();
    return true;
  }

  geometry_utils::TransformStamped transform;
  // Get latest transform with no timeout
  if(!tfListener_->getTransformation(&transform, worldFrameId_, frameId, any_measurements::Time(0.0), 0.0)) {
    MELO_ERROR_STREAM("Transform to frame '" << frameId << "' could not be obtained.")
    return false;
  }

  // Check transform age TODO(paco): Not hardcode time!
  if ((tfListener_->getCurrentTime() - transform.stamp_) > any_measurements::Time(1.0)) {
    MELO_ERROR_STREAM("Transform to frame '" << frameId << "' is older than 1.0 seconds.")
    return false;
  }

  pose = transform.transform_;
  return true;
}

ControlSetup AdapterAnymal::getControlSetup(const BranchEnum& branch) const
{
  throw std::runtime_error("AdapterAnymal::getControlSetup() is not implemented.");
}

ControlSetup AdapterAnymal::getControlSetup(const LimbEnum& limb) const
{
  return getControlSetup(AD::mapEnums<AD::BranchEnum>(limb));
}

JointVelocitiesLeg AdapterAnymal::getJointVelocitiesFromEndEffectorLinearVelocityInWorldFrame(
    const LimbEnum& limb, const LinearVelocity& endEffectorLinearVelocityInWorldFrame) const
{
  return JointVelocitiesLeg::Zero();
}

JointAccelerationsLeg AdapterAnymal::getJointAccelerationsFromEndEffectorLinearAccelerationInWorldFrame(
    const LimbEnum& limb, const LinearAcceleration& endEffectorLinearAccelerationInWorldFrame) const
{
  return JointAccelerationsLeg::Zero();
}

LinearVelocity AdapterAnymal::getEndEffectorLinearVelocityFromJointVelocities(const LimbEnum& limb,
                                                                              const JointVelocitiesLeg& jointVelocities,
                                                                              const std::string& frameId) const
{
  return LinearVelocity::Zero();
}

bool AdapterAnymal::setInternalDataFromState(const State& state, bool updateContacts, bool updatePosition,
                                             bool updateVelocity, bool updateAcceleration) const
{
  anymalModel_->setState(state, updatePosition, updateVelocity, updateAcceleration);

  if (updateContacts) {
    for (const auto& contactKey : AD::getContactKeys()) {
      const auto contactEnum = contactKey.getEnum();
      const auto limb = AD::mapEnums<AD::LimbEnum>(contactEnum);
      AD::ContactStateEnum contactState = state.isSupportLeg(limb) ? AD::ContactStateEnum::CLOSED
                                                                   : AD::ContactStateEnum::OPEN;
      const auto& branch = AD::mapEnums<AD::BranchEnum>(limb);
      anymalModel_->getContactContainer()[contactEnum]->setState(contactState);
    }
  }

  // TODO set frame transforms, torques etc.
  return true;
}

void AdapterAnymal::createCopyOfState() const
{
  *copyOfState_ = anymalModel_->getState();
}

void AdapterAnymal::resetToCopyOfState() const
{
  if (copyOfState_) {
    anymalModel_->setState(*copyOfState_, true, false, false);
  }
}

bool AdapterAnymal::updateFrameTransforms()
{
  return true;
}

void AdapterAnymal::setIterativeInverseKinematics() {
  useAnalyticInverseKinematics_ = false;
}
void AdapterAnymal::setAnalyticInverseKinematics() {
  useAnalyticInverseKinematics_ = true;
}

} /* namespace free_gait */

// Declare the AdapterAnymal as a Free Gait Adapter class.
PLUGINLIB_EXPORT_CLASS(free_gait::AdapterAnymal, free_gait::AdapterBase)
