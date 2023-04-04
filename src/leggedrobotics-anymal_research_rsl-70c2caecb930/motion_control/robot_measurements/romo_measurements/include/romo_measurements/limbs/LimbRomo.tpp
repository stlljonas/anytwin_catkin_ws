/*!
 * @file    LimbRomo.tpp
 * @author  Gabriel Hottiger
 * @date	  Nov, 2017
 */

#pragma once

// romo_measurements
#include "romo_measurements/limbs/LimbRomo.hpp"

namespace romo_measurements {

template <typename ConcreteDescription_, typename RobotState_, typename LimbBase_>
template <typename _LimbBase>
LimbRomo<ConcreteDescription_, RobotState_, LimbBase_>::LimbRomo(
  const LimbEnum limb,
  const std::string& name,
  const RobotModel& model,
  loco::LimbPropertiesPtr&& properties,
  loco::EndEffectorBasePtr&& endEffector,
  const bool advanceDynamics,
  typename std::enable_if<!std::is_same<loco::LegBase, _LimbBase>::value>::type*)
    : _LimbBase(name, RD::getNumDofLimb(limb), std::move(properties), std::move(endEffector)),
      model_(model),
      limbEnum_(limb),
      branchEnum_(RD::template mapEnums<BranchEnum>(limb)),
      branchStartBody_(RD::getBranchStartBody(branchEnum_)),
      limbStartIndexInJ_(RD::getLimbStartIndexInJ(limb)),
      branchStartIndexInU_(RD::getBranchStartIndexInU(branchEnum_)),
      advanceDynamics_(advanceDynamics)
{
  create();
}

template <typename ConcreteDescription_, typename RobotState_, typename LimbBase_>
template <typename _LimbBase>
LimbRomo<ConcreteDescription_, RobotState_, LimbBase_>::LimbRomo(
  const LimbEnum limb,
  const std::string& name,
  const RobotModel& model,
  loco::LimbPropertiesPtr&& properties,
  loco::EndEffectorBasePtr&& endEffector,
  loco::LimbStateMeasuredPtr&& stateMeasured,
  loco::LimbStateDesiredPtr&& stateDesired,
  const bool advanceDynamics,
  typename std::enable_if<!std::is_same<loco::LegBase, _LimbBase>::value>::type*)
  : _LimbBase(name, RD::getNumDofLimb(limb), std::move(properties), std::move(endEffector),
              std::move(stateMeasured), std::move(stateDesired)),
    model_(model),
    limbEnum_(limb),
    branchEnum_(RD::template mapEnums<BranchEnum>(limb)),
    branchStartBody_(RD::getBranchStartBody(branchEnum_)),
    limbStartIndexInJ_(RD::getLimbStartIndexInJ(limb)),
    branchStartIndexInU_(RD::getBranchStartIndexInU(branchEnum_)),
    advanceDynamics_(advanceDynamics)
{
  create();
}

template <typename ConcreteDescription_, typename RobotState_, typename LimbBase_>
template <typename _LimbBase>
LimbRomo<ConcreteDescription_, RobotState_, LimbBase_>::LimbRomo(
  const LimbEnum limb,
  const std::string& name,
  const RobotModel& model,
  loco::LegPropertiesPtr&& properties,
  loco::FootBasePtr&& endEffector,
  const bool advanceDynamics,
  typename std::enable_if<std::is_same<loco::LegBase, _LimbBase>::value>::type*)
  : _LimbBase(name, RD::getNumDofLimb(limb), std::move(properties), std::move(endEffector)),
    model_(model),
    limbEnum_(limb),
    branchEnum_(RD::template mapEnums<BranchEnum>(limb)),
    branchStartBody_(RD::getBranchStartBody(branchEnum_)),
    limbStartIndexInJ_(RD::getLimbStartIndexInJ(limb)),
    branchStartIndexInU_(RD::getBranchStartIndexInU(branchEnum_)),
    advanceDynamics_(advanceDynamics)
{
  create();
}

template <typename ConcreteDescription_, typename RobotState_, typename LimbBase_>
template <typename _LimbBase>
LimbRomo<ConcreteDescription_, RobotState_, LimbBase_>::LimbRomo(
  const LimbEnum limb,
  const std::string& name,
  const RobotModel& model,
  loco::LegPropertiesPtr&& properties,
  loco::FootBasePtr&& endEffector,
  loco::LimbStateMeasuredPtr&& stateMeasured,
  loco::LimbStateDesiredPtr&& stateDesired,
  const bool advanceDynamics,
  typename std::enable_if<std::is_same<loco::LegBase, _LimbBase>::value>::type*)
  : _LimbBase(name, RD::getNumDofLimb(limb), std::move(properties), std::move(endEffector),
              std::move(stateMeasured), std::move(stateDesired)),
    model_(model),
    limbEnum_(limb),
    branchEnum_(RD::template mapEnums<BranchEnum>(limb)),
    branchStartBody_(RD::getBranchStartBody(branchEnum_)),
    limbStartIndexInJ_(RD::getLimbStartIndexInJ(limb)),
    branchStartIndexInU_(RD::getBranchStartIndexInU(branchEnum_)),
    advanceDynamics_(advanceDynamics)
{
  create();
}

template <typename ConcreteDescription_, typename RobotState_, typename LimbBase_>
void LimbRomo<ConcreteDescription_, RobotState_, LimbBase_>::create() {
  // Set control modes
  loco::JointControlModes controlModes = this->getInitializedJointControlModes();
  controlModes.setConstant(loco::ControlMode::MODE_FREEZE);
  this->getLimbStateDesiredPtr()->setJointControlModes(controlModes);

  // Set joint names
  this->jointNames_ = this->getInitializedJointNames();
  for( unsigned int i = 0; i < this->getNumDofLimb(); ++i ) {
    this->jointNames_.at(i) = RD::template mapKeyIdToKeyName<JointEnum>(limbStartIndexInJ_ + i);
  }

  // Add all limb links
  for( const auto &body : model_.getBodyBranchNodeContainer().at(branchEnum_).values() ) {
    if (body->getIsFixedBody()) { continue; }

    this->getLinksPtr()->addItem(typename loco::GroupContainer<loco::LimbLink>::ItemSmartPtr(new loco::LimbLink(this->getNumDofLimb())));
    this->getLinksPtr()->back()->setLinkId(RD::template mapKeyEnumToKeyId(body->getBodyEnum()));
    this->getLinksPtr()->back()->setMass(model_.getBodyMass(body->getBodyEnum()));
  }
}

template <typename ConcreteDescription_, typename RobotState_, typename LimbBase_>
bool LimbRomo<ConcreteDescription_, RobotState_, LimbBase_>::initialize(double dt) {

  // Initialize properties
  if(!this->getLimbPropertiesPtr()->initialize(dt)) {
    MELO_WARN_STREAM("[LimbRomo]: Limb " << std::string( RD::mapKeyEnumToKeyName(limbEnum_) )
                                         << " could not initialize properties!");
    return false;
  }

  // Initialize endeffector
  if(!this->getEndEffectorPtr()->initialize(dt)) {
    MELO_WARN_STREAM("[LimbRomo]: Limb " << std::string( RD::mapKeyEnumToKeyName(limbEnum_) )
                                         << " could not initialize the endeffector!");
    return false;
  }

  // Initialize measured joint space quantities
  this->getLimbStateMeasuredPtr()->setJointTorques(this->getInitializedJointTorques());
  this->getLimbStateMeasuredPtr()->setJointAccelerations(this->getInitializedJointAccelerations());
  this->getLimbStateMeasuredPtr()->setGravityJointTorques(this->getInitializedJointTorques());
  this->getLimbStateMeasuredPtr()->setJointControlModes(this->getInitializedJointControlModes());

  // Initialize joint motion references
  this->getLimbStateDesiredPtr()->setJointPositions(this->getLimbStateMeasured().getJointPositions());
  this->getLimbStateDesiredPtr()->setJointVelocities(this->getInitializedJointVelocities());
  this->getLimbStateDesiredPtr()->setJointTorques(this->getInitializedJointTorques());
  this->getLimbStateDesiredPtr()->setJointAccelerations(this->getInitializedJointAccelerations());
  this->getLimbStateDesiredPtr()->setJointControlModes(this->getInitializedJointControlModes());

  // Set joint limits from the model
  for (unsigned int jointId = 0; jointId < this->getNumDofLimb(); ++jointId) {
    const JointEnum jointEnum = RD::template mapKeyIdToKeyEnum<JointEnum>(limbStartIndexInJ_ + jointId);
    // joint position limits
    const loco::JointPosition& jointMinPosition = model_.getLimits()->getJointMinPosition(jointEnum);
    this->getLimbStateMeasuredPtr()->setJointMinPosition(jointId, jointMinPosition);
    const loco::JointPosition& jointMaxPosition = model_.getLimits()->getJointMaxPosition(jointEnum);
    this->getLimbStateMeasuredPtr()->setJointMaxPosition(jointId, jointMaxPosition);
    // joint velocity limits
    const loco::JointVelocity& jointMinVelocity = model_.getLimits()->getJointMinVelocity(jointEnum);
    this->getLimbStateMeasuredPtr()->setJointMinVelocity(jointId, jointMinVelocity);
    const loco::JointVelocity& jointMaxVelocity = model_.getLimits()->getJointMaxVelocity(jointEnum);
    this->getLimbStateMeasuredPtr()->setJointMaxVelocity(jointId, jointMaxVelocity);
  }

  // Limb strategy
  this->getLimbStrategyPtr()->setLimbStrategyEnum(loco::LimbStrategyEnum::Undefined);

  // Set factors
  this->setFrictionModulation(1.0);
  this->setLoadFactor(1.0);

  return this->advance(dt);
}

template <typename ConcreteDescription_, typename RobotState_, typename LimbBase_>
bool LimbRomo<ConcreteDescription_, RobotState_, LimbBase_>::advance(double dt) {

  // Reset joint control modes to undefined
  loco::JointControlModes controlModes = this->getInitializedJointControlModes();
  controlModes.setConstant(loco::ControlMode::MODE_UNDEFINED);
  this->getLimbStateDesiredPtr()->setJointControlModes(controlModes);

  // --- Advance Leg Properties
  if(!this->getLimbPropertiesPtr()->advance(dt)) {
    MELO_WARN_STREAM("[LimbRomo]: Limb " << std::string( RD::mapKeyEnumToKeyName(limbEnum_) )
                                         << " could not advance properties!");
    return false;
  }

  // Advance joint state
  this->getLimbStateMeasuredPtr()->setJointPositions(model_.getState().getJointPositions().getSegment(
    limbStartIndexInJ_, this->getNumDofLimb()).toImplementation());
  this->getLimbStateMeasuredPtr()->setJointVelocities(model_.getState().getJointVelocities().getSegment(
    limbStartIndexInJ_, this->getNumDofLimb()).toImplementation());

  if(advanceDynamics_) {
    this->getLimbStateMeasuredPtr()->setGravityJointTorques(
      model_.getGravityTerms().segment(branchStartIndexInU_, this->getNumDofLimb()));
  }

  // Set joint accelerations and torques if robot model has extended state
  setExtendedRobotState();

  // Start body pose and twist
  Eigen::Vector3d positionWorldToLimbBaseInWorldFrame =
    model_.getPositionWorldToBody(branchStartBody_, CoordinateFrameEnum::WORLD);
  Eigen::Vector3d linearVelocityWorldToLimbBaseInWorldFrame =
    model_.getLinearVelocityWorldToBody(branchStartBody_, CoordinateFrameEnum::WORLD);
  Eigen::Vector3d positionWorldToBaseInWorldFrame =
    model_.getPositionWorldToBody(BodyEnum::BASE, CoordinateFrameEnum::WORLD);
  Eigen::Matrix3d orientationWorldToBase = model_.getOrientationWorldToBody(BodyEnum::BASE);

  this->getLimbStateMeasuredPtr()->setPositionWorldToLimbBaseInWorldFrame(
    loco::Position( positionWorldToLimbBaseInWorldFrame ) );
  this->getLimbStateMeasuredPtr()->setPositionWorldToLimbBaseInBaseFrame(
    loco::Position( orientationWorldToBase * positionWorldToLimbBaseInWorldFrame ) );
  this->getLimbStateMeasuredPtr()->setPositionBaseToLimbBaseInBaseFrame( loco::Position(
    orientationWorldToBase * (positionWorldToLimbBaseInWorldFrame - positionWorldToBaseInWorldFrame) ) );
  this->getLimbStateMeasuredPtr()->setLinearVelocityLimbBaseInWorldFrame(
    loco::LinearVelocity( linearVelocityWorldToLimbBaseInWorldFrame ) );

  // Advance links
  Eigen::MatrixXd jacobianBaseToLinkComInBaseFrame( this->getInitializedTranslationJacobian() );
  for(const auto& body : model_.getBodyBranchNodeContainer().at(branchEnum_).values()) {
    if (body->getIsFixedBody()) { continue; }

    jacobianBaseToLinkComInBaseFrame.setZero();
    model_.getJacobianTranslationFloatingBaseToBodyCom(jacobianBaseToLinkComInBaseFrame,
                                                       body->getBranchEnum(),
                                                       body->getBodyNodeEnum(),
                                                       CoordinateFrameEnum::BASE);
    const auto bodyId = RD::template mapKeyEnumToKeyId(body->getBodyEnum());
    this->getLinksPtr()->getLimbLinkPtrById(bodyId)->setTranslationJacobianBaseToCoMInBaseFrame(
      jacobianBaseToLinkComInBaseFrame.middleCols(branchStartIndexInU_,
                                                  this->getNumDofLimb()) );
    this->getLinksPtr()->getLimbLinkPtrById(bodyId)->setBaseToCoMPositionInBaseFrame( loco::Position(
      model_.getPositionBodyToBodyCom(BodyEnum::BASE, body->getBodyEnum(), CoordinateFrameEnum::BASE) ) );

    this->getLinksPtr()->getLimbLinkPtrById(bodyId)->setBaseToLinkPositionInBaseFrame( loco::Position(
      model_.getPositionBodyToBody(BodyEnum::BASE, body->getBodyEnum(), CoordinateFrameEnum::BASE) ) );
  }

  // Advance Endeffector
  if(!this->getEndEffectorPtr()->advance(dt)) {
    MELO_WARN_STREAM("[LimbRomo]: Limb " << std::string( RD::mapKeyEnumToKeyName(limbEnum_) )
                                         << " could not advance endeffector!");
    return false;
  }

  return true;
}

template <typename ConcreteDescription_, typename RobotState_, typename LimbBase_>
bool LimbRomo<ConcreteDescription_, RobotState_, LimbBase_>::addVariablesToLog(const std::string& ns) const {
  return LimbBase_::addVariablesToLog(ns);
}

}  // namespace romo_measurements
