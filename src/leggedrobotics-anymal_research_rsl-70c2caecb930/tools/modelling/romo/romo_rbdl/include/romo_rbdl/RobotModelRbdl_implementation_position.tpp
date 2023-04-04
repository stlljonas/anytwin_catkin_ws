/*
 * RobotModelRbdl_implementation_position.tpp
 *
 *  Created on: Mar 9, 2016
 *      Author: Dario Bellicoso
 */
#include "romo_rbdl/RobotModelRbdl.hpp"

#include "romo_rbdl/rbdl_kinematics.hpp"

namespace romo_rbdl {

template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getPositionWorldToBody(
    Eigen::Vector3d& position,
    BodyEnum bodyEnum,
    CoordinateFrameEnum frame) const
{
  position = this->bodyContainer_[bodyEnum]->getPositionWorldToBody(frame);
  return true;
}


template<typename ConcreteDescription_, typename RobotState_>
Eigen::Vector3d RobotModelRbdl<ConcreteDescription_, RobotState_>::getPositionWorldToBody(
    BodyEnum bodyEnum,
    CoordinateFrameEnum frame) const
{
  return this->bodyContainer_[bodyEnum]->getPositionWorldToBody(frame);
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getPositionWorldToBody(
    Eigen::Vector3d& position,
    BranchEnum branch,
    BodyNodeEnum node,
    CoordinateFrameEnum frame) const
{
  position = this->bodyBranchNodeContainer_.at(branch).at(node)->getPositionWorldToBody(frame);
  return true;
}


template<typename ConcreteDescription_, typename RobotState_>
Eigen::Vector3d RobotModelRbdl<ConcreteDescription_, RobotState_>::getPositionWorldToBody(
     BranchEnum branch,
     BodyNodeEnum node,
     CoordinateFrameEnum frame) const
{
  return this->bodyBranchNodeContainer_.at(branch).at(node)->getPositionWorldToBody(frame);
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getPositionWorldToPointOnBody(
    Eigen::Vector3d& position,
    const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
    BodyEnum bodyEnum,
    CoordinateFrameEnum frame) const
{
  position = this->bodyContainer_[bodyEnum]->getPositionWorldToPointOnBody(positionBodyToPointOnBodyInBodyFrame, frame);
  return true;
}


template<typename ConcreteDescription_, typename RobotState_>
Eigen::Vector3d RobotModelRbdl<ConcreteDescription_, RobotState_>::getPositionWorldToPointOnBody(
    const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
    BodyEnum bodyEnum,
    CoordinateFrameEnum frame) const
{
  return this->bodyContainer_[bodyEnum]->getPositionWorldToPointOnBody(positionBodyToPointOnBodyInBodyFrame, frame);
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getPositionWorldToPointOnBody(
    Eigen::Vector3d& position,
    const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
    BranchEnum branch,
    BodyNodeEnum node,
    CoordinateFrameEnum frame) const
{
  position = this->bodyBranchNodeContainer_.at(branch).at(node)->getPositionWorldToPointOnBody(positionBodyToPointOnBodyInBodyFrame, frame);
  return true;
}


template<typename ConcreteDescription_, typename RobotState_>
Eigen::Vector3d RobotModelRbdl<ConcreteDescription_, RobotState_>::getPositionWorldToPointOnBody(
     const Eigen::Vector3d& positionBodyToPointOnBodyInBodyFrame,
     BranchEnum branch,
     BodyNodeEnum node,
     CoordinateFrameEnum frame) const
{
  return this->bodyBranchNodeContainer_.at(branch).at(node)->getPositionWorldToPointOnBody(positionBodyToPointOnBodyInBodyFrame, frame);
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getPositionWorldToBodyCom(
  Eigen::Vector3d& position,
  BodyEnum bodyEnum,
  CoordinateFrameEnum frame) const
{
  position = this->bodyContainer_[bodyEnum]->getPositionWorldToBodyCom(frame);
  return true;
}


template<typename ConcreteDescription_, typename RobotState_>
Eigen::Vector3d RobotModelRbdl<ConcreteDescription_, RobotState_>::getPositionWorldToBodyCom(
  BodyEnum bodyEnum,
  CoordinateFrameEnum frame) const
{
  return this->bodyContainer_[bodyEnum]->getPositionWorldToBodyCom(frame);
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getPositionWorldToBodyCom(
  Eigen::Vector3d& position,
  BranchEnum branch,
  BodyNodeEnum node,
  CoordinateFrameEnum frame) const
{
  position = this->bodyBranchNodeContainer_.at(branch).at(node)->getPositionWorldToBodyCom(frame);
  return true;
}


template<typename ConcreteDescription_, typename RobotState_>
Eigen::Vector3d RobotModelRbdl<ConcreteDescription_, RobotState_>::getPositionWorldToBodyCom(
  BranchEnum branch,
  BodyNodeEnum node,
  CoordinateFrameEnum frame) const
{
  return this->bodyBranchNodeContainer_.at(branch).at(node)->getPositionWorldToBodyCom(frame);
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getPositionBodyToBody(
  Eigen::Vector3d& position,
  BodyEnum fromBody,
  BodyEnum toBody,
  CoordinateFrameEnum frame) const {
  position = (this->bodyContainer_[toBody]->getPositionWorldToBody(frame)
    - this->bodyContainer_[fromBody]->getPositionWorldToBody(frame));
  return true;
}


template<typename ConcreteDescription_, typename RobotState_>
Eigen::Vector3d RobotModelRbdl<ConcreteDescription_, RobotState_>::getPositionBodyToBody(
    BodyEnum fromBody,
    BodyEnum toBody,
    CoordinateFrameEnum frame) const
{
  return (this->bodyContainer_[toBody]->getPositionWorldToBody(frame)
        - this->bodyContainer_[fromBody]->getPositionWorldToBody(frame));
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getPositionBodyToBody(
  Eigen::Vector3d& position,
  BodyEnum fromBody,
  BranchEnum toBranch,
  BodyNodeEnum toNode,
  CoordinateFrameEnum frame) const {
  position = (this->bodyBranchNodeContainer_.at(toBranch).at(toNode)->getPositionWorldToBody(frame)
    - this->bodyContainer_[fromBody]->getPositionWorldToBody(frame));
  return true;
}


template<typename ConcreteDescription_, typename RobotState_>
Eigen::Vector3d RobotModelRbdl<ConcreteDescription_, RobotState_>::getPositionBodyToBody(
    BodyEnum fromBody,
    BranchEnum toBranch,
    BodyNodeEnum toNode,
    CoordinateFrameEnum frame) const
{
  return (this->bodyBranchNodeContainer_.at(toBranch).at(toNode)->getPositionWorldToBody(frame)
        - this->bodyContainer_[fromBody]->getPositionWorldToBody(frame));
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getPositionBodyToBody(
  Eigen::Vector3d& position,
  BranchEnum fromBranch,
  BodyNodeEnum fromNode,
  BranchEnum toBranch,
  BodyNodeEnum toNode,
  CoordinateFrameEnum frame) const
{
  position = (this->bodyBranchNodeContainer_.at(toBranch).at(toNode)->getPositionWorldToBody(frame)
    - this->bodyBranchNodeContainer_.at(fromBranch).at(fromNode)->getPositionWorldToBody(frame));
  return true;
}


template<typename ConcreteDescription_, typename RobotState_>
Eigen::Vector3d RobotModelRbdl<ConcreteDescription_, RobotState_>::getPositionBodyToBody(
      BranchEnum fromBranch,
      BodyNodeEnum fromNode,
      BranchEnum toBranch,
      BodyNodeEnum toNode,
      CoordinateFrameEnum frame) const {
  return (this->bodyBranchNodeContainer_.at(toBranch).at(toNode)->getPositionWorldToBody(frame)
        - this->bodyBranchNodeContainer_.at(fromBranch).at(fromNode)->getPositionWorldToBody(frame));
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getPositionBodyToBodyCom(
  Eigen::Vector3d& position,
  BodyEnum fromBody,
  BodyEnum toBody,
  CoordinateFrameEnum frame) const {
  position = (this->bodyContainer_[toBody]->getPositionWorldToBodyCom(frame)
    - this->bodyContainer_[fromBody]->getPositionWorldToBody(frame));
  return true;
}


template<typename ConcreteDescription_, typename RobotState_>
Eigen::Vector3d RobotModelRbdl<ConcreteDescription_, RobotState_>::getPositionBodyToBodyCom(
  BodyEnum fromBody,
  BodyEnum toBody,
  CoordinateFrameEnum frame) const
{
  return (this->bodyContainer_[toBody]->getPositionWorldToBodyCom(frame)
    - this->bodyContainer_[fromBody]->getPositionWorldToBody(frame));
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getPositionBodyToBodyCom(
  Eigen::Vector3d& position,
  BodyEnum fromBody,
  BranchEnum toBranch,
  BodyNodeEnum toNode,
  CoordinateFrameEnum frame) const {
  position = (this->bodyBranchNodeContainer_.at(toBranch).at(toNode)->getPositionWorldToBodyCom(frame)
    - this->bodyContainer_[fromBody]->getPositionWorldToBody(frame));
  return true;
}


template<typename ConcreteDescription_, typename RobotState_>
Eigen::Vector3d RobotModelRbdl<ConcreteDescription_, RobotState_>::getPositionBodyToBodyCom(
  BodyEnum fromBody,
  BranchEnum toBranch,
  BodyNodeEnum toNode,
  CoordinateFrameEnum frame) const
{
  return (this->bodyBranchNodeContainer_.at(toBranch).at(toNode)->getPositionWorldToBodyCom(frame)
    - this->bodyContainer_[fromBody]->getPositionWorldToBody(frame));
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::getPositionBodyToBodyCom(
  Eigen::Vector3d& position,
  BranchEnum fromBranch,
  BodyNodeEnum fromNode,
  BranchEnum toBranch,
  BodyNodeEnum toNode,
  CoordinateFrameEnum frame) const
{
  position = (this->bodyBranchNodeContainer_.at(toBranch).at(toNode)->getPositionWorldToBodyCom(frame)
    - this->bodyBranchNodeContainer_.at(fromBranch).at(fromNode)->getPositionWorldToBody(frame));
  return true;
}


template<typename ConcreteDescription_, typename RobotState_>
Eigen::Vector3d RobotModelRbdl<ConcreteDescription_, RobotState_>::getPositionBodyToBodyCom(
  BranchEnum fromBranch,
  BodyNodeEnum fromNode,
  BranchEnum toBranch,
  BodyNodeEnum toNode,
  CoordinateFrameEnum frame) const {
  return (this->bodyBranchNodeContainer_.at(toBranch).at(toNode)->getPositionWorldToBodyCom(frame)
    - this->bodyBranchNodeContainer_.at(fromBranch).at(fromNode)->getPositionWorldToBody(frame));
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::setPositionBodyToBodyCom(
    const Eigen::Vector3d& position,
    BodyEnum body,
    CoordinateFrameEnum frame,
    bool updateBodyInertia) const
{
  const std::shared_ptr<romo::RigidBody<ConcreteDescription_>>& bodyPtr = this->bodyContainer_[body];
  if (bodyPtr->getIsFixedBody()){
    MELO_ERROR_STREAM("[RobotModelRbdl::setPositionBodyToBodyCom] You cannot use setPositionBodyToBodyCom on a fixed body (called on body "<< bodyPtr->getName()<<"); use setPositionBodyToIndividualBodyCom instead" );
    return false;
  } else {
    unsigned int bodyId = bodyPtr->getBodyId();
    rbdlModel_->mBodies[bodyId]->AbsorbFixedChildren(false);

    switch (frame) {
      case (CoordinateFrameEnum::WORLD): {
        rbdlModel_->mBodies[bodyId]->SetIndividualCenterOfMass(bodyPtr->getOrientationWorldToBody()*position, updateBodyInertia);
        return true;
      }
      case (CoordinateFrameEnum::BASE): {
        rbdlModel_->mBodies[bodyId]->SetIndividualCenterOfMass(rbdlModel_->X_base[rbdlModel_->rootId].E*(bodyPtr->getOrientationWorldToBody().transpose()*position), updateBodyInertia);
        return true;
      }
      case (CoordinateFrameEnum::BODY): {
        rbdlModel_->mBodies[bodyId]->SetIndividualCenterOfMass(position, updateBodyInertia);
        return true;
      }

      default:
        throw std::runtime_error("[RobotModelRbdl::setPositionBodyToBodyCom] Invalid frame.");
    }
  }
}


template<typename ConcreteDescription_, typename RobotState_>
bool RobotModelRbdl<ConcreteDescription_, RobotState_>::setPositionBodyToIndividualBodyCom(
    const Eigen::Vector3d& position,
    BodyEnum body,
    CoordinateFrameEnum frame,
    bool updateBodyInertia) const
{
  const std::shared_ptr<romo::RigidBody<ConcreteDescription_>>& bodyPtr = this->bodyContainer_[body];
  if (bodyPtr->getIsFixedBody()){
    unsigned int bodyId = bodyPtr->getBodyId() - rbdlModel_->fixed_body_discriminator;
    if (rbdlModel_->mFixedBodies[bodyId]->GetIsNulled()){
      MELO_WARN_THROTTLE_STREAM(1.0, "[RobotModel::setPositionBodyToIndividualBodyCom] The body "<<bodyPtr->getName()<<" is a fixed body that was nulled (you called a setBodyXXXX function in the past), but you are setting it's individual CoM.");
    }

    switch (frame) {
      case (CoordinateFrameEnum::WORLD): {
        rbdlModel_->mFixedBodies[bodyId]->SetIndividualCenterOfMass(bodyPtr->getOrientationWorldToBody()*position, updateBodyInertia);
        return true;
      }
      case (CoordinateFrameEnum::BASE): {
        rbdlModel_->mFixedBodies[bodyId]->SetIndividualCenterOfMass(rbdlModel_->X_base[rbdlModel_->rootId].E*(bodyPtr->getOrientationWorldToBody().transpose()*position), updateBodyInertia);
        return true;
      }
      case (CoordinateFrameEnum::BODY): {
        rbdlModel_->mFixedBodies[bodyId]->SetIndividualCenterOfMass(position, updateBodyInertia);
        return true;
      }

      default:
        throw std::runtime_error("[RobotModelRbdl::setPositionBodyToIndividualBodyCom] Invalid frame.");
    }
  } else {
    unsigned int bodyId = bodyPtr->getBodyId();
    if (rbdlModel_->mBodies[bodyId]->GetChildrenWereAbsorbed()){
      MELO_WARN_THROTTLE_STREAM(1.0, "[RobotModel::setPositionBodyToIndividualBodyCom] The body "<<bodyPtr->getName()<<" absorbed its children (you called a setBodyXXXX function in the past), but you are setting it's individual CoM, instead of setPositionBodyToBodyCom.");
    }

    switch (frame) {
      case (CoordinateFrameEnum::WORLD): {
        rbdlModel_->mBodies[bodyId]->SetIndividualCenterOfMass(bodyPtr->getOrientationWorldToBody()*position, updateBodyInertia);
        return true;
      }
      case (CoordinateFrameEnum::BASE): {
        rbdlModel_->mBodies[bodyId]->SetIndividualCenterOfMass(rbdlModel_->X_base[rbdlModel_->rootId].E*(bodyPtr->getOrientationWorldToBody().transpose()*position), updateBodyInertia);
        return true;
      }
      case (CoordinateFrameEnum::BODY): {
        rbdlModel_->mBodies[bodyId]->SetIndividualCenterOfMass(position, updateBodyInertia);
        return true;
      }

      default:
        throw std::runtime_error("[RobotModelRbdl::setPositionBodyToIndividualBodyCom] Invalid frame.");
    }
  }
}


template<typename ConcreteDescription_, typename RobotState_>
Eigen::Vector3d RobotModelRbdl<ConcreteDescription_, RobotState_>::getPositionBaseToLimbComInBaseFrame(BranchEnum branch) const {
  double limbMass = 0.0;
  Eigen::Vector3d totalCenterOfMassInBaseFrame = Eigen::Vector3d::Zero();

  const auto& container = this->bodyBranchNodeContainer_.at(branch);
  for (const auto& body : container | boost::adaptors::map_values ) {
    if (body->getIsFixedBody()) {
      continue;
    }
    const double mass = body->getMass();
    totalCenterOfMassInBaseFrame += mass*getPositionBodyToBodyCom(BodyEnum::BASE, body->getBodyEnum(), CoordinateFrameEnum::BASE);
    limbMass += mass;
  }

  if (limbMass > 0.0) {
    totalCenterOfMassInBaseFrame /= limbMass;
  }

  return totalCenterOfMassInBaseFrame;
}


template<typename ConcreteDescription_, typename RobotState_>
void RobotModelRbdl<ConcreteDescription_, RobotState_>::getPositionWorldToCom(Eigen::VectorXd& positionWorldToCom, CoordinateFrameEnum frame) const {
  switch (frame) {
    case(CoordinateFrameEnum::WORLD): {
      positionWorldToCom = RigidBodyDynamics::CalcPositionWorldToCoMInWorldFrame(*rbdlModel_, stateGeneralizedPositionsQuaternionRBDL_, false);
    } break;

    case(CoordinateFrameEnum::BASE): {
      const Eigen::Matrix3d& orientationWorldToBase = getOrientationWorldToBody(BodyEnum::BASE);
      positionWorldToCom = orientationWorldToBase*RigidBodyDynamics::CalcPositionWorldToCoMInWorldFrame(*rbdlModel_, stateGeneralizedPositionsQuaternionRBDL_, false);
    } break;

    default: throw std::runtime_error("[RobotModel::getPositionWorldToCom] frame is not supported!"); break;
  }
}


template<typename ConcreteDescription_, typename RobotState_>
Eigen::Vector3d RobotModelRbdl<ConcreteDescription_, RobotState_>::getPositionWorldToCom(CoordinateFrameEnum frame) const {
  switch (frame) {
    case (CoordinateFrameEnum::WORLD):
      return RigidBodyDynamics::CalcPositionWorldToCoMInWorldFrame(*rbdlModel_, stateGeneralizedPositionsQuaternionRBDL_, false);

    case(CoordinateFrameEnum::BASE): {
      const Eigen::Matrix3d& orientationWorldToBase = getOrientationWorldToBody(BodyEnum::BASE);
      return orientationWorldToBase*RigidBodyDynamics::CalcPositionWorldToCoMInWorldFrame(*rbdlModel_, stateGeneralizedPositionsQuaternionRBDL_, false);
    }

    default: throw std::runtime_error("[RobotModel::getPositionWorldToCom] frame is not supported!"); break;
  }
}


template<typename ConcreteDescription_, typename RobotState_>
const Eigen::Matrix3d& RobotModelRbdl<ConcreteDescription_, RobotState_>::getOrientationWorldToBody(BodyEnum bodyEnum) const {
  return this->bodyContainer_[bodyEnum]->getOrientationWorldToBody();
}


template<typename ConcreteDescription_, typename RobotState_>
const Eigen::Matrix3d& RobotModelRbdl<ConcreteDescription_, RobotState_>::getOrientationWorldToBody(BranchEnum branch, BodyNodeEnum bodyNode) const {
  return this->bodyBranchNodeContainer_.at(branch).at(bodyNode)->getOrientationWorldToBody();
}


template<typename ConcreteDescription_, typename RobotState_>
const Eigen::Matrix3d RobotModelRbdl<ConcreteDescription_, RobotState_>::getOrientationBodyToBody(
  BodyEnum fromBody, BodyEnum toBody) const {
  return  (this->bodyContainer_[toBody]->getOrientationWorldToBody()
    * this->bodyContainer_[fromBody]->getOrientationWorldToBody().transpose());
}


template<typename ConcreteDescription_, typename RobotState_>
const Eigen::Matrix3d RobotModelRbdl<ConcreteDescription_, RobotState_>::getOrientationBodyToBody(
    BranchEnum fromBranch,
    BodyNodeEnum fromBodyNode,
    BranchEnum toBranch,
    BodyNodeEnum toBodyNode) const {
  return  (this->bodyBranchNodeContainer_.at(toBranch).at(toBodyNode)->getOrientationWorldToBody()
         * this->bodyBranchNodeContainer_.at(fromBranch).at(fromBodyNode)->getOrientationWorldToBody().transpose());
}

} /* namespace romo */
