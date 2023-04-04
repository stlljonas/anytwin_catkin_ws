/*!
 * @file	  WholeBodyRomo.tpp
 * @author	Gabriel Hottiger
 * @date	  Nov, 2017
 */

// romo_measurements
#include "romo_measurements/whole_body/WholeBodyRomo.hpp"

// message_logger
#include "message_logger/message_logger.hpp"

namespace romo_measurements {

template <typename ConcreteDescription_, typename RobotState_>
bool WholeBodyRomo<ConcreteDescription_,RobotState_>::initialize(double dt) {
  // Init properties
  if(!this->getWholeBodyPropertiesPtr()->initialize(dt)) {
    MELO_WARN_STREAM("[WholeBodyRomo]: WholeBody could not initialize properties!");
    return false;
  }

  this->setWholeBodyMassMatrix(RD::MassMatrix::Zero());
  this->setWholeBodyNonlinearEffects(RD::NonlinearEffects::Zero());
  this->setWholeBodyGravityTerms(RD::GravityTerms::Zero());
  
  this->getWholeBodyStateDesiredPtr()->setPositionWorldToWholeBodyCenterOfMassInWorldFrame(
    this->getWholeBodyStateMeasured().getPositionWorldToWholeBodyCenterOfMassInWorldFrame() );
  this->getWholeBodyStateDesiredPtr()->setLinearVelocityWholeBodyCenterOfMassInWorldFrame(
    this->getWholeBodyStateMeasured().getLinearVelocityWholeBodyCenterOfMassInWorldFrame() );
  this->getWholeBodyStateDesiredPtr()->setLinearAccelerationWholeBodyCenterOfMassInWorldFrame(
    loco::LinearAcceleration());

  return advance(dt);
}

template <typename ConcreteDescription_, typename RobotState_>
bool WholeBodyRomo<ConcreteDescription_,RobotState_>::advance(double dt) {
  // Properties
  if(!this->getWholeBodyPropertiesPtr()->advance(dt)) {
    MELO_WARN_STREAM("[WholeBodyRomo]: WholeBody could not advance properties!");
    return false;
  }

  this->getWholeBodyStateMeasuredPtr()->setPositionWorldToWholeBodyCenterOfMassInWorldFrame(
    loco::Position(model_.getPositionWorldToCom(CoordinateFrameEnum::WORLD)));
  this->getWholeBodyStateMeasuredPtr()->setLinearVelocityWholeBodyCenterOfMassInWorldFrame(
    loco::LinearVelocity(model_.getLinearVelocityWorldToCom(CoordinateFrameEnum::WORLD)));

  // Update center of pressure position (assumes flat floor)
  loco::Position positionWorldToCenterOfPressureInWorldFrame;
  double totalNormalForce = 0.0;
  for (auto limb : getLimbs()) {
    const auto strategy = limb->getLimbStrategy().getLimbStrategyEnum();
    if ((strategy == loco::LimbStrategyEnum::Support) || (strategy == loco::LimbStrategyEnum::ContactInvariant)) {
      positionWorldToCenterOfPressureInWorldFrame +=
        limb->getEndEffector().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame()
          * limb->getEndEffector().getStateMeasured().getForceAtEndEffectorInWorldFrame().z();
      totalNormalForce += limb->getEndEffector().getStateMeasured().getForceAtEndEffectorInWorldFrame().z();
    }
  }
  if (totalNormalForce > 0.0) {
    positionWorldToCenterOfPressureInWorldFrame /= totalNormalForce;
  }
  wholeBodyStateMeasured_.setPositionWorldToCenterOfPressureInWorldFrame(positionWorldToCenterOfPressureInWorldFrame);

  // Update divergent component of motion (assumes flat floor)
  const auto& measured = this->getWholeBodyStateMeasuredPtr();
  const loco::Position& com = measured->getPositionWorldToWholeBodyCenterOfMassInWorldFrame();
  const loco::Position& zmp = positionWorldToCenterOfPressureInWorldFrame;
  double height = (com - zmp).z();
  double omega = std::sqrt(model_.getGravityAcceleration() / height);
  this->getWholeBodyStateMeasuredPtr()->updatePositionWorldToDCMInWorldFrame(omega);

  // Advance dynamics
  if(this->isUpdatingDynamics()) {
    this->setWholeBodyMassMatrix(model_.getMassInertiaMatrix());
    this->setWholeBodyNonlinearEffects(model_.getNonlinearEffects());
    this->setWholeBodyGravityTerms(model_.getGravityTerms());

    for (auto limb : *this->getLimbsPtr()) {
      limb->getLimbStateMeasuredPtr()->setGravityJointTorques( loco::JointTorques(
        wholeBodyGravityTerms_.segment(
          RD::getBranchStartIndexInU(RD::template mapKeyIdToKeyEnum<BranchEnum>(limb->getBranchUInt())),
          limb->getNumDofLimb())));
    }
  }

  return true;
}

} /* namespace romo_measurements */
