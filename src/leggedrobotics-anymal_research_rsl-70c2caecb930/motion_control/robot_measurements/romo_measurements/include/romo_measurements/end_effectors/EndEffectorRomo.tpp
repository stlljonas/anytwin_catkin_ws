/*!
 * @file	 EndEffectorRomo.tpp
 * @author Gabriel Hottiger
 * @date	 Nov, 2017
 */

// romo_measurements
#include "romo_measurements/end_effectors/EndEffectorRomo.hpp"

// message logger
#include "message_logger/message_logger.hpp"

namespace romo_measurements {

template <typename ConcreteDescription_, typename RobotState_, typename EndEffectorBase_>
template <typename _EndEffectorBase>
EndEffectorRomo<ConcreteDescription_, RobotState_, EndEffectorBase_>::EndEffectorRomo(
  BodyEnum bodyEnum,
  const std::string& name,
  const RobotModel& model,
  loco::EndEffectorPropertiesPtr&& endeffectorProperties,
  const ContactsMap & contactPointsMap,
  const std::vector<TimeInstant>& timeInstants,
  bool autoAdvanceOfContactPoints,
  typename std::enable_if<std::is_same<loco::EndEffectorBase, _EndEffectorBase>::value>::type* /**/)
    : loco::EndEffectorBase(std::move(endeffectorProperties), isContactAtOrigin(contactPointsMap)),
      branchEnum_(RD::template mapEnums<BranchEnum>(bodyEnum)),
      limbEnum_(RD::template mapEnums<LimbEnum>(branchEnum_)),
      bodyEnum_(bodyEnum),
      bodyNodeEnum_(RD::template mapEnums<BodyNodeEnum>(bodyEnum)),
      model_(model),
      contactPointsMap_(contactPointsMap),
      autoAdvanceOfContactPoints_(autoAdvanceOfContactPoints)
{
  create(loco::EndEffectorStateDesired(), loco::EndEffectorStateMeasured(), timeInstants);
}

template <typename ConcreteDescription_, typename RobotState_, typename EndeffectorBase_>
template <typename _EndeffectorBase>
EndEffectorRomo<ConcreteDescription_, RobotState_, EndeffectorBase_>::EndEffectorRomo(
  BodyEnum bodyEnum,
  const std::string& name,
  const RobotModel& model,
  const ContactsMap & contactPointsMap,
  const std::vector<TimeInstant>& timeInstants,
  bool autoAdvanceOfContactPoints,
  typename std::enable_if<std::is_same<loco::EndEffectorBase, _EndeffectorBase>::value>::type* /**/)
  : EndEffectorRomo(bodyEnum, name, model, loco::EndEffectorPropertiesPtr(new EndEffectorPropertiesRomo(bodyEnum, model)),
                    contactPointsMap, timeInstants, autoAdvanceOfContactPoints)
{
}

template <typename ConcreteDescription_, typename RobotState_, typename EndeffectorBase_>
template <typename _EndeffectorBase>
EndEffectorRomo<ConcreteDescription_, RobotState_, EndeffectorBase_>::EndEffectorRomo(
  BodyEnum bodyEnum,
  const std::string& name,
  const RobotModel& model,
  loco::EndEffectorPropertiesPtr&& endeffectorProperties,
  const ContactsMap & contactPointsMap,
  const std::vector<TimeInstant>& timeInstants,
  bool autoAdvanceOfContactPoints,
  typename std::enable_if<std::is_same<loco::FootBase, _EndeffectorBase>::value>::type* /**/)
  : loco::FootBase(std::move(endeffectorProperties), isContactAtOrigin(contactPointsMap)),
    branchEnum_(RD::template mapEnums<BranchEnum>(bodyEnum)),
    limbEnum_(RD::template mapEnums<LimbEnum>(branchEnum_)),
    bodyEnum_(bodyEnum),
    bodyNodeEnum_(RD::template mapEnums<BodyNodeEnum>(bodyEnum)),
    model_(model),
    contactPointsMap_(contactPointsMap),
    autoAdvanceOfContactPoints_(autoAdvanceOfContactPoints)
{
  create(loco::FootBaseStateDesired(), loco::FootBaseStateMeasured(), timeInstants);
}

template <typename ConcreteDescription_, typename RobotState_, typename EndeffectorBase_>
template <typename _EndeffectorBase>
EndEffectorRomo<ConcreteDescription_, RobotState_, EndeffectorBase_>::EndEffectorRomo(
  BodyEnum bodyEnum,
  const std::string& name,
  const RobotModel& model,
  const ContactsMap & contactPointsMap,
  const std::vector<TimeInstant>& timeInstants,
  bool autoAdvanceOfContactPoints,
  typename std::enable_if<std::is_same<loco::FootBase, _EndeffectorBase>::value>::type* /**/)
  : EndEffectorRomo(bodyEnum, name, model, loco::EndEffectorPropertiesPtr(new EndEffectorPropertiesRomo(bodyEnum, model)),
                    contactPointsMap, timeInstants, autoAdvanceOfContactPoints)
{

}

template <typename ConcreteDescription_, typename RobotState_, typename EndeffectorBase_>
template <typename _EndeffectorBase>
EndEffectorRomo<ConcreteDescription_, RobotState_, EndeffectorBase_>::EndEffectorRomo(
  BodyEnum bodyEnum,
  const std::string& name,
  const RobotModel& model,
  loco::WheelPropertiesPtr&& wheelProperties,
  const ContactsMap & contactPointsMap,
  const std::vector<TimeInstant>& timeInstants,
  bool autoAdvanceOfContactPoints,
  typename std::enable_if<std::is_same<loco::Wheel, _EndeffectorBase>::value>::type* /**/)
  : loco::Wheel(std::move(wheelProperties), RD::getWheelIndexInJ(RD::template mapEnums<LimbEnum>(bodyEnum)) -
                  RD::getLimbStartIndexInJ(RD::template mapEnums<LimbEnum>(bodyEnum)),
                isContactAtOrigin(contactPointsMap) ),
    branchEnum_(RD::template mapEnums<BranchEnum>(bodyEnum)),
    limbEnum_(RD::template mapEnums<LimbEnum>(branchEnum_)),
    bodyEnum_(bodyEnum),
    bodyNodeEnum_(RD::template mapEnums<BodyNodeEnum>(bodyEnum)),
    model_(model),
    contactPointsMap_(contactPointsMap),
    autoAdvanceOfContactPoints_(autoAdvanceOfContactPoints)
{
  create(loco::WheelStateDesired(this->indexInLimbJoints_), loco::WheelStateMeasured(this->indexInLimbJoints_), timeInstants);
}

template <typename ConcreteDescription_, typename RobotState_, typename EndeffectorBase_>
template <typename _EndeffectorBase>
EndEffectorRomo<ConcreteDescription_, RobotState_, EndeffectorBase_>::EndEffectorRomo(
  BodyEnum bodyEnum,
  const std::string& name,
  const RobotModel& model,
  const ContactsMap & contactPointsMap,
  const std::vector<TimeInstant>& timeInstants,
  bool autoAdvanceOfContactPoints,
  typename std::enable_if<std::is_same<loco::Wheel, _EndeffectorBase>::value>::type* /**/)
  : EndEffectorRomo(bodyEnum, name, model, loco::WheelPropertiesPtr(new WheelPropertiesRomo(bodyEnum, model)),
                    contactPointsMap, timeInstants, autoAdvanceOfContactPoints)
{

}

template <typename ConcreteDescription_, typename RobotState_, typename EndeffectorBase_>
template <typename _EndeffectorBase>
EndEffectorRomo<ConcreteDescription_, RobotState_, EndeffectorBase_>::EndEffectorRomo(
                BodyEnum bodyEnum,
                unsigned int numFingers,
                const std::string& name,
                const RobotModel& model,
                loco::EndEffectorPropertiesPtr && endEffectorProperties,
                const ContactsMap & contactPointsMap,
                const std::vector<TimeInstant>& timeInstants,
                bool autoAdvanceOfContactPoints,
                typename std::enable_if<std::is_same<loco::Hand, _EndeffectorBase>::value>::type* /**/)
: loco::Hand(std::move(endEffectorProperties), numFingers, isContactAtOrigin(contactPointsMap)),
    branchEnum_(RD::template mapEnums<BranchEnum>(bodyEnum)),
    limbEnum_(RD::template mapEnums<LimbEnum>(branchEnum_)),
    bodyEnum_(bodyEnum),
    bodyNodeEnum_(RD::template mapEnums<BodyNodeEnum>(bodyEnum)),
    model_(model),
    contactPointsMap_(contactPointsMap),
    autoAdvanceOfContactPoints_(autoAdvanceOfContactPoints)
{
  create(loco::HandStateDesired(numFingers), loco::HandStateMeasured(numFingers), timeInstants);
}

template <typename ConcreteDescription_, typename RobotState_, typename EndeffectorBase_>
template <typename _EndeffectorBase>
EndEffectorRomo<ConcreteDescription_, RobotState_, EndeffectorBase_>::EndEffectorRomo(
                BodyEnum bodyEnum,
                unsigned int numFingers,
                const std::string& name,
                const RobotModel& model,
                const ContactsMap & contactPointsMap,
                const std::vector<TimeInstant>& timeInstants,
                bool autoAdvanceOfContactPoints,
                typename std::enable_if<std::is_same<loco::Hand, _EndeffectorBase>::value>::type* /**/)
:
EndEffectorRomo(bodyEnum, numFingers, name, model, loco::EndEffectorPropertiesPtr(new EndEffectorPropertiesRomo(bodyEnum, model, loco::handNumberOfContactConstraints)),
                    contactPointsMap, timeInstants, autoAdvanceOfContactPoints)
{

}



template <typename ConcreteDescription_, typename RobotState_, typename EndeffectorBase_>
template<typename EndeffectorStateDesired_, typename EndeffectorStateMeasured_>
bool EndEffectorRomo<ConcreteDescription_, RobotState_, EndeffectorBase_>::
  create(const EndeffectorStateDesired_& defaultEndeffectorStateDesired,
         const EndeffectorStateMeasured_& defaultEndeffectorStateMeasured,
         const std::vector<TimeInstant>& timeInstants)
{
  for(const auto & timeInstant : timeInstants) {
    // Check if contacts are valid for this body
    for(const auto & contactPair : contactPointsMap_) {
      // Don't override origin when multiple contacts are defined, this would be a violation
      assert(this->contactIsAtOrigin_ || contactPair.first != loco::EndEffectorEnum::Origin);
      // Contact must lie on the endeffector body
      assert(RD::template mapEnums<BodyEnum>(contactPair.second) == bodyEnum_);
      // Add them to the endEffectorStates
      this->endEffectorStateDesired_[timeInstant][contactPair.first].reset(
        new EndeffectorStateDesired_(defaultEndeffectorStateDesired));
      this->endEffectorStateMeasured_[timeInstant][contactPair.first].reset(
        new EndeffectorStateMeasured_(defaultEndeffectorStateMeasured));
    }
    // Always add origin
    this->endEffectorStateDesired_[timeInstant][loco::EndEffectorEnum::Origin].reset(
      new EndeffectorStateDesired_(defaultEndeffectorStateDesired));
    this->endEffectorStateMeasured_[timeInstant][loco::EndEffectorEnum::Origin].reset(
      new EndeffectorStateMeasured_(defaultEndeffectorStateMeasured));
  }

  return true;
}

template <typename ConcreteDescription_, typename RobotState_, typename EndeffectorBase_>
bool EndEffectorRomo<ConcreteDescription_,RobotState_,EndeffectorBase_>::initialize(double dt) {

  // --- Initialize properties
  if(!this->getPropertiesPtr()->initialize(dt)) {
    MELO_WARN_STREAM("[EndEffectorRomo]: Endeffector at body " << RD::mapKeyEnumToKeyName(bodyEnum_)
                                                               << " could not initialize properties!");
    return false;
  }

  // --- Advance measured state
  if(!this->advance(dt)) {
    MELO_WARN_STREAM("[EndEffectorRomo]: Endeffector at body " << RD::mapKeyEnumToKeyName(bodyEnum_)
                                                               << " could not advance during initialization!");
    return false;
  }

  // --- Set accelerations to zero
  // Todo move acceleration to advance once we can fill them with appropriate values
  this->getStateMeasuredPtr(loco::TimePoint::Now,loco::EndEffectorEnum::Origin)->
    setLinearAccelerationEndEffectorInWorldFrame(loco::LinearAcceleration::Zero());

  this->getStateMeasuredPtr(loco::TimePoint::Now,loco::EndEffectorEnum::Origin)->
    setAngularAccelerationEndEffectorInWorldFrame(loco::AngularAcceleration::Zero());

  if(!this->contactIsAtOrigin_) {
    for(const auto & contactPoints : contactPointsMap_) {
      this->getStateMeasuredPtr(loco::TimePoint::Now, contactPoints.first)->
        setLinearAccelerationEndEffectorInWorldFrame(loco::LinearAcceleration::Zero());

      this->getStateMeasuredPtr(loco::TimePoint::Now, contactPoints.first)->
        setAngularAccelerationEndEffectorInWorldFrame(loco::AngularAcceleration::Zero());
    }
  }

  // Set desired states
  for(auto & endEffectorStateDesired : this->endEffectorStateDesired_[loco::TimePoint::Now]) {
    endEffectorStateDesired.second->setPositionWorldToEndEffectorInWorldFrame(this->getStateMeasured(
      loco::TimePoint::Now, endEffectorStateDesired.first).getPositionWorldToEndEffectorInWorldFrame() );
    endEffectorStateDesired.second->setOrientationWorldToEndEffector(this->getStateMeasured(
      loco::TimePoint::Now,endEffectorStateDesired.first).getOrientationWorldToEndEffector() );

    endEffectorStateDesired.second->setLinearVelocityEndEffectorInWorldFrame(loco::LinearVelocity() );
    endEffectorStateDesired.second->setLinearAccelerationEndEffectorInWorldFrame(loco::LinearAcceleration() );
    endEffectorStateDesired.second->setAngularVelocityEndEffectorInWorldFrame(loco::LocalAngularVelocity() );
    endEffectorStateDesired.second->setAngularAccelerationEndEffectorInWorldFrame(loco::AngularAcceleration() );
    endEffectorStateDesired.second->setForceAtEndEffectorInWorldFrame(loco::Force() );
    endEffectorStateDesired.second->setTorqueAtEndEffectorInWorldFrame(loco::Torque() );
  }

  return true;
}

template <typename ConcreteDescription_, typename RobotState_, typename EndeffectorBase_>
bool EndEffectorRomo<ConcreteDescription_,RobotState_,EndeffectorBase_>::advance(double dt) {
  // --- Initialize properties
  if(!this->getPropertiesPtr()->advance(dt)) {
    MELO_WARN_STREAM("[EndEffectorRomo]: Endeffector at body " << RD::mapKeyEnumToKeyName(bodyEnum_)
                                                               << " could not advance properties!");
    return false;
  }

  const loco::RotationMatrix rotationWorldToBase(model_.getOrientationWorldToBody(BodyEnum::BASE));
  const loco::Position positionWorldToBaseInWorldFrame(
    model_.getPositionWorldToBody(BodyEnum::BASE, CoordinateFrameEnum::WORLD));

  // Advance the origin
  const loco::Position positionWorldToContactBodyOriginInWorldFrame(
    model_.getPositionWorldToBody(bodyEnum_, CoordinateFrameEnum::WORLD));

  this->endEffectorStateMeasured_[loco::TimePoint::Now][loco::EndEffectorEnum::Origin]->
    setPositionWorldToEndEffectorInWorldFrame(positionWorldToContactBodyOriginInWorldFrame);

  this->endEffectorStateMeasured_[loco::TimePoint::Now][loco::EndEffectorEnum::Origin]->
    setPositionBaseToEndEffectorInBaseFrame(rotationWorldToBase.rotate(
    positionWorldToContactBodyOriginInWorldFrame - positionWorldToBaseInWorldFrame) );

  this->endEffectorStateMeasured_[loco::TimePoint::Now][loco::EndEffectorEnum::Origin]->
    setOrientationWorldToEndEffector(loco::RotationQuaternion( loco::RotationMatrix(
    model_.getOrientationWorldToBody(bodyEnum_))) );

  this->endEffectorStateMeasured_[loco::TimePoint::Now][loco::EndEffectorEnum::Origin]->
    setLinearVelocityEndEffectorInWorldFrame(loco::LinearVelocity(
    model_.getLinearVelocityWorldToBody(bodyEnum_, CoordinateFrameEnum::WORLD)) );

  this->endEffectorStateMeasured_[loco::TimePoint::Now][loco::EndEffectorEnum::Origin]->
    setAngularVelocityEndEffectorInWorldFrame(loco::LocalAngularVelocity(
    model_.getAngularVelocityWorldToBody(bodyEnum_, CoordinateFrameEnum::WORLD)) );

  Eigen::MatrixXd jacobianSpatialWorldContactBodyOriginInWorldFrame = RobotModel::JacobianSpatial::Zero();
  model_.getJacobianSpatialWorldToBody(jacobianSpatialWorldContactBodyOriginInWorldFrame,
                                      branchEnum_, bodyNodeEnum_, CoordinateFrameEnum::WORLD);

  Eigen::MatrixXd jacobianSpatialTimeDerivativeWorldContactBodyOriginInWorldFrame = RobotModel::JacobianSpatial::Zero();
  model_.getJacobianSpatialTimeDerivativeWorldToBody(jacobianSpatialTimeDerivativeWorldContactBodyOriginInWorldFrame,
                                                    jacobianSpatialWorldContactBodyOriginInWorldFrame,
                                                    branchEnum_, bodyNodeEnum_, CoordinateFrameEnum::WORLD);

  Eigen::MatrixXd jacobianSpatialTimeDerivativeWorldContactBodyOriginInBaseFrame = RobotModel::JacobianSpatial::Zero();
  model_.getJacobianSpatialTimeDerivativeWorldToBody(jacobianSpatialTimeDerivativeWorldContactBodyOriginInBaseFrame,
                                                    jacobianSpatialWorldContactBodyOriginInWorldFrame,
                                                    branchEnum_, bodyNodeEnum_, CoordinateFrameEnum::BASE);

  this->endEffectorStateMeasured_[loco::TimePoint::Now][loco::EndEffectorEnum::Origin]->
    setTranslationJacobianWorldToEndEffectorInWorldFrame(
    jacobianSpatialWorldContactBodyOriginInWorldFrame.bottomRows(RD::getNumTranslationalDof()));

  this->endEffectorStateMeasured_[loco::TimePoint::Now][loco::EndEffectorEnum::Origin]->
    setTranslationJacobianBaseToEndEffectorInBaseFrame( rotationWorldToBase.toImplementation() *
    jacobianSpatialWorldContactBodyOriginInWorldFrame.block(
      RD::getNumRotationalDof(), RD::getBranchStartIndexInU(branchEnum_),
      RD::getNumTranslationalDof(), RD::getNumDofLimb(limbEnum_)) );

  this->endEffectorStateMeasured_[loco::TimePoint::Now][loco::EndEffectorEnum::Origin]->
    setRotationJacobianWorldToEndEffectorInWorldFrame(
    jacobianSpatialWorldContactBodyOriginInWorldFrame.topRows(RD::getNumRotationalDof()));

  this->endEffectorStateMeasured_[loco::TimePoint::Now][loco::EndEffectorEnum::Origin]->
    setRotationJacobianBaseToEndEffectorInBaseFrame(
    rotationWorldToBase.toImplementation() * jacobianSpatialWorldContactBodyOriginInWorldFrame.block(
      0, RD::getBranchStartIndexInU(branchEnum_), RD::getNumRotationalDof(), RD::getNumDofLimb(limbEnum_)) );

  this->endEffectorStateMeasured_[loco::TimePoint::Now][loco::EndEffectorEnum::Origin]->
    setTranslationJacobianTimeDerivativeWorldToEndEffectorInWorldFrame(
    jacobianSpatialTimeDerivativeWorldContactBodyOriginInWorldFrame.bottomRows(RD::getNumTranslationalDof()));

  this->endEffectorStateMeasured_[loco::TimePoint::Now][loco::EndEffectorEnum::Origin]->
    setTranslationJacobianTimeDerivativeBaseToEndEffectorInBaseFrame(
    jacobianSpatialTimeDerivativeWorldContactBodyOriginInBaseFrame.block(
      RD::getNumRotationalDof(), RD::getBranchStartIndexInU(branchEnum_),
      RD::getNumTranslationalDof(), RD::getNumDofLimb(limbEnum_)) );

  this->endEffectorStateMeasured_[loco::TimePoint::Now][loco::EndEffectorEnum::Origin]->
    setRotationJacobianTimeDerivativeWorldToEndEffectorInWorldFrame(
    jacobianSpatialTimeDerivativeWorldContactBodyOriginInWorldFrame.topRows(RD::getNumRotationalDof()));

  this->endEffectorStateMeasured_[loco::TimePoint::Now][loco::EndEffectorEnum::Origin]->
    setRotationJacobianTimeDerivativeBaseToEndEffectorInBaseFrame(
    jacobianSpatialTimeDerivativeWorldContactBodyOriginInBaseFrame.block(
      0, RD::getBranchStartIndexInU(branchEnum_), RD::getNumRotationalDof(), RD::getNumDofLimb(limbEnum_)) );

  // Force only contains not-nan values in the case where the origin is treated as a contact
  if(this->contactIsAtOrigin_) {
    this->endEffectorStateMeasured_[loco::TimePoint::Now][loco::EndEffectorEnum::Origin]->
      setForceAtEndEffectorInWorldFrame(model_.getContactContainer().at(contactPointsMap_.
      at(loco::EndEffectorEnum::Origin))->getForce(CoordinateFrameEnum::WORLD) );

    this->endEffectorStateMeasured_[loco::TimePoint::Now][loco::EndEffectorEnum::Origin]->
        setTorqueAtEndEffectorInWorldFrame(model_.getContactContainer().at(contactPointsMap_.
        at(loco::EndEffectorEnum::Origin))->getTorque(CoordinateFrameEnum::WORLD) );
  } else if (autoAdvanceOfContactPoints_) {
    if(!advanceContactPoints(dt)) {
      MELO_WARN_STREAM("[EndEffectorRomo]: Endeffector at body " << RD::mapKeyEnumToKeyName(bodyEnum_)
                                                                 << " could not advance contact points!");
      return false;
    }
  }

  return true;
}

template <typename ConcreteDescription_, typename RobotState_, typename EndeffectorBase_>
bool EndEffectorRomo<ConcreteDescription_,RobotState_,EndeffectorBase_>::advanceContactPoints(double /* dt */) {
  const loco::RotationMatrix rotationWorldToBase(model_.getOrientationWorldToBody(BodyEnum::BASE));
  const loco::Position positionWorldToBaseInWorldFrame(
    model_.getPositionWorldToBody(BodyEnum::BASE, CoordinateFrameEnum::WORLD));

  this->endEffectorStateMeasured_[loco::TimePoint::Now][loco::EndEffectorEnum::Origin]->
    setForceAtEndEffectorInWorldFrame(loco::Force( Eigen::Vector3d::Constant( std::nan("") ) ) );

  this->endEffectorStateMeasured_[loco::TimePoint::Now][loco::EndEffectorEnum::Origin]->
      setTorqueAtEndEffectorInWorldFrame(loco::Torque( Eigen::Vector3d::Constant( std::nan("") ) ) );

  for( const auto &contactPoints : contactPointsMap_ ) {
    const loco::Position positionWorldToContactInWorldFrame =
      model_.getContactContainer().at(contactPoints.second)->getPositionWorldToContact(CoordinateFrameEnum::WORLD);

    this->endEffectorStateMeasured_[loco::TimePoint::Now][contactPoints.first]->
      setPositionWorldToEndEffectorInWorldFrame(positionWorldToContactInWorldFrame);

    this->endEffectorStateMeasured_[loco::TimePoint::Now][contactPoints.first]->
      setPositionBaseToEndEffectorInBaseFrame(rotationWorldToBase.rotate(
      positionWorldToContactInWorldFrame - positionWorldToBaseInWorldFrame));

    this->endEffectorStateMeasured_[loco::TimePoint::Now][contactPoints.first]->setOrientationWorldToEndEffector(
      model_.getContactContainer().at(contactPoints.second)->getOrientationWorldToContact());

    this->endEffectorStateMeasured_[loco::TimePoint::Now][contactPoints.first]->
      setLinearVelocityEndEffectorInWorldFrame(model_.getContactContainer().at(contactPoints.second)->
      getLinearVelocityWorldToContact(CoordinateFrameEnum::WORLD));

    this->endEffectorStateMeasured_[loco::TimePoint::Now][contactPoints.first]->
      setAngularVelocityEndEffectorInWorldFrame(model_.getContactContainer().at(contactPoints.second)->
      getAngularVelocityWorldToContact(CoordinateFrameEnum::WORLD));

    Eigen::MatrixXd jacobianSpatialInWorldFrame = RobotModel::JacobianSpatial::Zero();
    model_.getContactContainer().at(contactPoints.second)->getJacobianSpatialWorldToContact(
      jacobianSpatialInWorldFrame,
      CoordinateFrameEnum::WORLD);

    Eigen::MatrixXd jacobianSpatialTimeDerivativeInWorldFrame = RobotModel::JacobianSpatial::Zero();
    model_.getContactContainer().at(contactPoints.second)->getJacobianSpatialTimeDerivativeWorldToContact(
      jacobianSpatialTimeDerivativeInWorldFrame, jacobianSpatialInWorldFrame, CoordinateFrameEnum::WORLD);

    Eigen::MatrixXd jacobianSpatialTimeDerivativeInBaseFrame = RobotModel::JacobianSpatial::Zero();
    model_.getContactContainer().at(contactPoints.second)->getJacobianSpatialTimeDerivativeWorldToContact(
      jacobianSpatialTimeDerivativeInBaseFrame, jacobianSpatialInWorldFrame, CoordinateFrameEnum::BASE);

    this->endEffectorStateMeasured_[loco::TimePoint::Now][contactPoints.first]->
      setTranslationJacobianWorldToEndEffectorInWorldFrame(
      jacobianSpatialInWorldFrame.bottomRows(RD::getNumTranslationalDof()));

    this->endEffectorStateMeasured_[loco::TimePoint::Now][contactPoints.first]->
      setTranslationJacobianBaseToEndEffectorInBaseFrame(rotationWorldToBase.toImplementation() *
      jacobianSpatialInWorldFrame.block(RD::getNumRotationalDof(), RD::getBranchStartIndexInU(branchEnum_),
                                        RD::getNumTranslationalDof(), RD::getNumDofLimb(limbEnum_)));

    this->endEffectorStateMeasured_[loco::TimePoint::Now][contactPoints.first]->
      setRotationJacobianWorldToEndEffectorInWorldFrame(
      jacobianSpatialInWorldFrame.topRows(RD::getNumRotationalDof()));

    this->endEffectorStateMeasured_[loco::TimePoint::Now][contactPoints.first]->
      setRotationJacobianBaseToEndEffectorInBaseFrame(rotationWorldToBase.toImplementation() *
      jacobianSpatialInWorldFrame.block(0, RD::getBranchStartIndexInU(branchEnum_),
                                        RD::getNumRotationalDof(), RD::getNumDofLimb(limbEnum_)));

    this->endEffectorStateMeasured_[loco::TimePoint::Now][contactPoints.first]->
      setTranslationJacobianTimeDerivativeWorldToEndEffectorInWorldFrame(
      jacobianSpatialTimeDerivativeInWorldFrame.bottomRows(RD::getNumTranslationalDof()));

    this->endEffectorStateMeasured_[loco::TimePoint::Now][contactPoints.first]->
      setTranslationJacobianTimeDerivativeBaseToEndEffectorInBaseFrame(
      jacobianSpatialTimeDerivativeInBaseFrame.block(RD::getNumRotationalDof(),
                                                     RD::getBranchStartIndexInU(branchEnum_),
                                                     RD::getNumTranslationalDof(),
                                                     RD::getNumDofLimb(limbEnum_)));

    this->endEffectorStateMeasured_[loco::TimePoint::Now][contactPoints.first]->
      setRotationJacobianTimeDerivativeWorldToEndEffectorInWorldFrame(
      jacobianSpatialTimeDerivativeInWorldFrame.topRows(RD::getNumRotationalDof()));

    this->endEffectorStateMeasured_[loco::TimePoint::Now][contactPoints.first]->
      setRotationJacobianTimeDerivativeBaseToEndEffectorInBaseFrame(
      jacobianSpatialTimeDerivativeInBaseFrame.block(0, RD::getBranchStartIndexInU(branchEnum_),
                                                     RD::getNumRotationalDof(), RD::getNumDofLimb(limbEnum_)));

    this->endEffectorStateMeasured_[loco::TimePoint::Now][contactPoints.first]->setForceAtEndEffectorInWorldFrame(
      model_.getContactContainer().at(contactPoints.second)->getForce(CoordinateFrameEnum::WORLD));

    this->endEffectorStateMeasured_[loco::TimePoint::Now][contactPoints.first]->setTorqueAtEndEffectorInWorldFrame(
        model_.getContactContainer().at(contactPoints.second)->getTorque(CoordinateFrameEnum::WORLD));
  }

  return true;
}


template <typename ConcreteDescription_, typename RobotState_, typename EndeffectorBase_>
loco::JointPositions EndEffectorRomo<ConcreteDescription_, RobotState_, EndeffectorBase_>::
getJointPositionsFromPositionBaseToEndEffectorInBaseFrame(const loco::Position& /* positionBaseToEndEffectorInBaseFrame */) {
  MELO_WARN_STREAM(
    "[EndEffectorRomo]:getJointPositionsFromPositionBaseToEndEffectorInBaseFrame for endeffector at limb "
      << RD::mapKeyEnumToKeyName(limbEnum_) << " is not implemented!");
  return loco::JointPositions::Zero(RD::getNumDofLimb(limbEnum_));
}

template <typename ConcreteDescription_, typename RobotState_, typename EndeffectorBase_>
loco::JointPositions EndEffectorRomo<ConcreteDescription_, RobotState_, EndeffectorBase_>::
getJointPositionsFromPositionBaseToEndEffectorInBaseFrameIteratively(
  const loco::Position& /* positionBaseToEndEffectorInBaseFrame */) {
  MELO_WARN_STREAM(
    "[EndEffectorRomo]:getJointPositionsFromPositionBaseToEndEffectorInBaseFrameIteratively for endeffector at limb "
      << RD::mapKeyEnumToKeyName(limbEnum_) << " is not implemented!");
  return loco::JointPositions::Zero(RD::getNumDofLimb(limbEnum_));
}

template <typename ConcreteDescription_, typename RobotState_, typename EndeffectorBase_>
loco::JointVelocities EndEffectorRomo<ConcreteDescription_,RobotState_,EndeffectorBase_>::
getJointVelocitiesFromLinearVelocityBaseToEndEffectorInBaseFrame(const loco::LinearVelocity& /* velocity */) {
  MELO_WARN_STREAM(
    "[EndEffectorRomo]:getJointVelocitiesFromLinearVelocityBaseToEndEffectorInBaseFrame for endeffector at limb "
      << RD::mapKeyEnumToKeyName(limbEnum_) << " is not implemented!");
  return loco::JointVelocities::Zero(RD::getNumDofLimb(limbEnum_));
}

template <typename ConcreteDescription_, typename RobotState_, typename EndeffectorBase_>
loco::Position EndEffectorRomo<ConcreteDescription_,RobotState_,EndeffectorBase_>::
getPositionWorldToEndEffectorInWorldFrame(const loco::JointPositions& /* jointPositions */) {
  MELO_WARN_STREAM("[EndEffectorRomo]:getPositionWorldToEndEffectorInWorldFrame for endeffector at limb "
                     << RD::mapKeyEnumToKeyName(limbEnum_) << " is not implemented!");
  return loco::Position();
}

template <typename ConcreteDescription_, typename RobotState_, typename EndeffectorBase_>
loco::Position EndEffectorRomo<ConcreteDescription_,RobotState_,EndeffectorBase_>::
getPositionWorldToEndEffectorInBaseFrame(const loco::JointPositions& /* jointPositions */) {
  MELO_WARN_STREAM("[EndEffectorRomo]:getPositionWorldToEndEffectorInBaseFrame for endeffector at limb "
                     << RD::mapKeyEnumToKeyName(limbEnum_) << " is not implemented!");
  return loco::Position();
}

template <typename ConcreteDescription_, typename RobotState_, typename EndeffectorBase_>
loco::Position EndEffectorRomo<ConcreteDescription_,RobotState_,EndeffectorBase_>::
getPositionBaseToEndEffectorInBaseFrame( const loco::JointPositions& /* jointPositions */) {
  MELO_WARN_STREAM("[EndEffectorRomo]:getPositionBaseToEndEffectorInBaseFrame for endeffector at limb "
                     << RD::mapKeyEnumToKeyName(limbEnum_) << " is not implemented!");
  return loco::Position();
}

template <typename ConcreteDescription_, typename RobotState_, typename EndeffectorBase_>
bool EndEffectorRomo<ConcreteDescription_,RobotState_,EndeffectorBase_>::
isInContact() const {
  return std::find_if(this->contactPointsMap_.begin(), this->contactPointsMap_.end(),
      [this] (const typename ContactsMap::value_type& pair) {
        return (
            this->model_.getContactContainer()[pair.second]->getState() == (RD::ContactStateEnum::CLOSED) ||
            this->model_.getContactContainer()[pair.second]->getState() == (RD::ContactStateEnum::SLIPPING)
            );
      }) != this->contactPointsMap_.end();
}

template <typename ConcreteDescription_, typename RobotState_, typename EndeffectorBase_>
bool EndEffectorRomo<ConcreteDescription_,RobotState_,EndeffectorBase_>::
isSlipping() const {
  return std::find_if(this->contactPointsMap_.begin(), this->contactPointsMap_.end(),
      [this] (const typename ContactsMap::value_type& pair) {
        return this->model_.getContactContainer()[pair.second]->getState() == RD::ContactStateEnum::SLIPPING;
      }) != this->contactPointsMap_.end();
}

} /* namespace loco */
