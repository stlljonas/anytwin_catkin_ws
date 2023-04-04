/*
 * RobotModelRbdl_implementation_identification.tpp
 *
 *  Created on: Oct 14, 2019
 *      Author: Ioannis Mandralis
 */

#include <romo/common/RigidBody.hpp>
#include "romo_rbdl/rbdl_kinematics.hpp"
#include "romo_rbdl/RobotModelRbdl.hpp"
#include "robot_utils/math/LinearAlgebra.hpp"

namespace romo_rbdl {

namespace internal {
const Eigen::IOFormat eigenFormat(4, 0, ",", "\n", "[", "]");
}

template <typename ConcreteDescription_, typename RobotState_>
Eigen::MatrixXd RobotModelRbdl<ConcreteDescription_, RobotState_>::getBranchRegressor(const BranchEnum& branchEnum, bool verbose) {

  // Get the DoF of the branch as well as the moveable links count
  const unsigned int branchDofCount = RD::getNumDofLimb(RD::mapBranchEnumToLimbEnum::at(branchEnum));
  const unsigned int branchMoveableLinksCount = this->getBranchMoveableLinksCount(branchEnum);

  // Declare the branchRegressor
  Eigen::MatrixXd branchRegressor = Eigen::MatrixXd::Zero(branchDofCount, branchMoveableLinksCount * 10);

  // Declare the individualBodyRegressorVec as well as the branchJointAxesVec and branchParentToChildTransformVec
  std::vector<Eigen::MatrixXd> individualBodyRegressorVec(branchMoveableLinksCount, Eigen::MatrixXd::Zero(6, 10));
  std::vector<Eigen::VectorXd> branchJointAxesVec(branchDofCount, Eigen::VectorXd::Zero(6));
  std::vector<RigidBodyDynamics::Math::SpatialTransform> branchParentToChildTransformVec(branchMoveableLinksCount,
                                                                                         RigidBodyDynamics::Math::SpatialTransform());

  // Get the romo IDs of the start and end bodies of the branch
  const int startBodyId = RD::template mapKeyEnumToKeyId<BodyEnum, BodyEnum>(RD::getBranchStartBody(branchEnum));
  const int endBodyId = RD::template mapKeyEnumToKeyId<BodyEnum, BodyEnum>(RD::getBranchEndBody(branchEnum));

  if (verbose) {
    MELO_DEBUG_STREAM("Looping over branch and getting body regressors, joint axes, and spatial transforms..." << std::endl << std::endl);
  }
  int body_i = 0;
  for (int body_id = startBodyId; body_id <= endBodyId; body_id++) {
    // Get the current body
    auto bodyEnum = RD::template mapKeyIdToKeyEnum<BodyEnum, BodyEnum>(body_id);
    auto body = this->bodyContainer_[bodyEnum];

    if (verbose) {
      MELO_DEBUG_STREAM("Current body is: " << RD::template mapKeyEnumToKeyName<BodyEnum>(bodyEnum) << std::endl);
    }

    if (body->getIsFixedBody()) {
      continue;  // skip
    }

    // Get the individual regressor of the current body
    Eigen::MatrixXd individualBodyRegressor = this->getIndividualBodyRegressor(bodyEnum, verbose);

    // Append it to the individualBodyRegressorVec
    individualBodyRegressorVec[body_i] = individualBodyRegressor;

    // Get the joint axis of the current body
    branchJointAxesVec[body_i] = rbdlModel_->S[body->getBodyId()];

    // Get the parent to child spatial transform matrix of the current body
    branchParentToChildTransformVec[body_i] = rbdlModel_->X_lambda[body->getBodyId()];

    if (verbose) {
      MELO_DEBUG_STREAM("individualBodyRegressorVec[" << body_i << "] is: " << std::endl
                << individualBodyRegressorVec[body_i] << std::endl
                << std::endl);
      MELO_DEBUG_STREAM("branchJointAxesVec[" << body_i << "] is: " << std::endl << branchJointAxesVec[body_i] << std::endl << std::endl);
      MELO_DEBUG_STREAM("X_lambda[" << body_i << "] is: " << std::endl << branchParentToChildTransformVec[body_i] << std::endl << std::endl);
    } // print the regressor, joint axes, and spatial transforms for debug.

    body_i++;
  }

  if (verbose) {
    MELO_DEBUG_STREAM("Filling out branchRegressor block for current sample..." << std::endl << std::endl);
  }
  for (unsigned int link = 0; link < branchDofCount; link++) {
    RigidBodyDynamics::Math::SpatialTransform childToLinkTransform = RigidBodyDynamics::Math::SpatialTransform();

    // Get the transform from the child link to the current link that is needed for the full branchRegressor
    for (unsigned int child = link; child < branchMoveableLinksCount; child++) {
      if (child > link) {
        childToLinkTransform = childToLinkTransform * (branchParentToChildTransformVec[child].inverse());
      }

      // Fill out the block of branchRegressor corresponding to the current link and child
      branchRegressor.block<1, 10>(link, 10 * child) =
          branchJointAxesVec[link].transpose() * childToLinkTransform.inverse().toMatrix().transpose() * individualBodyRegressorVec[child];
    }
  }
  return branchRegressor;
}

template<typename ConcreteDescription_, typename RobotState_>
Eigen::MatrixXd RobotModelRbdl<ConcreteDescription_, RobotState_>::getIndividualBodyRegressor(const BodyEnum& bodyEnum, bool verbose) {
  return getIndividualBodyRegressorImpl(bodyEnum, verbose);
}

template<typename ConcreteDescription_, typename RobotState_>
template <typename CD_, typename RS_>
Eigen::MatrixXd RobotModelRbdl<ConcreteDescription_, RobotState_>::getIndividualBodyRegressorImpl(const BodyEnum& bodyEnum, bool verbose, typename std::enable_if<std::is_base_of<romo::ExtendedRobotState<CD_>, RS_>::value>::type*){
// Get body and body name for debug
auto body = this->bodyContainer_[bodyEnum];
auto bodyName = RD::template mapKeyEnumToKeyName<BodyEnum, BodyEnum>(bodyEnum);

// Define the generalized acceleration vector assuming the robot base is fixed
int baseGeneralizedVelocitiesDimension = RD::getBaseGeneralizedVelocitiesDimension();
Eigen::VectorXd ddq = Eigen::VectorXd::Zero(this->getDofCount());
ddq.segment(0, baseGeneralizedVelocitiesDimension) = Eigen::VectorXd::Zero(baseGeneralizedVelocitiesDimension);
ddq.segment(baseGeneralizedVelocitiesDimension, state_.getNumberOfJointVelocities()) = this->getState().getJointAccelerations().toImplementation();

// Extract spatial velocity, acceleration and the transforms for each body
if (verbose) {MELO_DEBUG_STREAM("Extracting body angular velocities and accelerations:" << std::endl);}

Eigen::Vector3d omega = rbdlModel_->v[body->getBodyId()].head(3);
Eigen::Vector3d domega = rbdlModel_->a[body->getBodyId()].head(3);

if (verbose) {
MELO_DEBUG_STREAM("Omega " << bodyName << ": " << std::endl << omega.format(internal::eigenFormat) << std::endl);
MELO_DEBUG_STREAM("dOmega " << bodyName << ": " << std::endl << domega.format(internal::eigenFormat) << std::endl << std::endl);
}

// In RBDL body frames are defined at the joint. Extract the acceleration of the body joint frame relative to world
auto accJointFrameToWorldInWorld =
    RigidBodyDynamics::CalcPointAcceleration(*rbdlModel_, this->getStateGeneralizedPositionsQuaternionRBDL(), this->getStateGeneralizedVelocitiesAngularRBDL(), ddq, body->getBodyId(),
                                             Eigen::Vector3d::Zero(), false, true);

if (verbose) {
MELO_DEBUG_STREAM("accJointFrameToWorldInWorld: " << std::endl << accJointFrameToWorldInWorld.format(internal::eigenFormat) << ":" << std::endl);
}

const auto R_body_base = RigidBodyDynamics::CalcBodyWorldOrientation(*rbdlModel_, this->getStateGeneralizedPositionsQuaternionRBDL(), body->getBodyId(), false);
const auto accJointFrameToWorldInBody = R_body_base * accJointFrameToWorldInWorld;

if (verbose) {
MELO_DEBUG_STREAM("dd_d_world_i_in_i: " << std::endl << accJointFrameToWorldInBody.format(internal::eigenFormat) << ":" << std::endl << std::endl);
}

// Initialize the individual body regressor:
Eigen::MatrixXd individualBodyRegressor = Eigen::MatrixXd::Zero(6, 10);

// Construct individualBodyRegressor matrix as in Handbook of Robotics: equation (6.40) page 123.
individualBodyRegressor.block<3, 3>(0, 1) = -robot_utils::skew(accJointFrameToWorldInBody);
individualBodyRegressor.block<3, 6>(0, 4) = robot_utils::L(domega) + robot_utils::skew(omega) * robot_utils::L(omega);
individualBodyRegressor.block<3, 1>(3, 0) = accJointFrameToWorldInBody;
individualBodyRegressor.block<3, 3>(3, 1) = robot_utils::skew(domega) + robot_utils::skew(omega) * robot_utils::skew(omega);

return individualBodyRegressor;
}

template<typename ConcreteDescription_, typename RobotState_>
template <typename CD_, typename RS_>
Eigen::MatrixXd RobotModelRbdl<ConcreteDescription_, RobotState_>::getIndividualBodyRegressorImpl(const BodyEnum& bodyEnum, bool verbose, typename std::enable_if<!std::is_base_of<romo::ExtendedRobotState<CD_>, RS_>::value>::type*){
  throw std::runtime_error("The function getIndividualBodyRegressor is not compiled when using RobotState instead of ExtendedRobotState");
}

template<typename ConcreteDescription_, typename RobotState_>
Eigen::VectorXd RobotModelRbdl<ConcreteDescription_, RobotState_>::getInertialParameters() {

  // Get the moveable links count of the whole robot
  const unsigned int branchMoveableLinksCount = (1 + state_.getNumberOfJointVelocities());
  const unsigned int numParametersPerLink = 10;

  // Declare the inertialParameters vector
  Eigen::VectorXd inertialParameters = Eigen::VectorXd::Zero(branchMoveableLinksCount * numParametersPerLink);

  // Loop over all robot moveable bodies
  unsigned int linkId = 0;
  for (auto body : this->bodyContainer_) {

    if (body->getIsFixedBody()) {
      continue;// Skip if body is fixed.
    }

    // Get the inertial parameters of an individual body
    Eigen::VectorXd bodyInertialParameters = this->getBodyInertialParameters(body->getBodyEnum());

    // Append these to the inertialParameters vector
    inertialParameters.segment<10>(linkId * numParametersPerLink) = bodyInertialParameters;
    linkId++;
  }
  return inertialParameters;
}

template<typename ConcreteDescription_, typename RobotState_>
unsigned int RobotModelRbdl<ConcreteDescription_, RobotState_>::getBranchMoveableLinksCount(const BranchEnum& branchEnum) {

  // Get the romo IDs of the start and end bodies of the branch
  const int startBodyId = RD::template mapKeyEnumToKeyId<BodyEnum, BodyEnum>(RD::getBranchStartBody(branchEnum));
  const int endBodyId = RD::template mapKeyEnumToKeyId<BodyEnum, BodyEnum>(RD::getBranchEndBody(branchEnum));

  // Loop over the bodies on the branch
  unsigned int branchMoveableLinksCount = 0;
  for (int body_id = startBodyId; body_id <= endBodyId; body_id++){
    // Get the current body
    auto bodyEnum = RD::template mapKeyIdToKeyEnum<BodyEnum, BodyEnum>(body_id);
    auto body = this->bodyContainer_[bodyEnum];

    // If the body is moveable increment the counter
    if (!(body->getIsFixedBody())){
      branchMoveableLinksCount++;
    }
  }
  return branchMoveableLinksCount;
}

template<typename ConcreteDescription_, typename RobotState_>
Eigen::VectorXd RobotModelRbdl<ConcreteDescription_, RobotState_>::getBranchInertialParameters(BranchEnum branchEnum) {

  // Get the DoF of the branch as well as the moveable links count
  const unsigned int branchMoveableLinksCount = this->getBranchMoveableLinksCount(branchEnum);
  const unsigned int numParametersPerLink = 10;

  // Declare the branchInertialParameters vector
  Eigen::VectorXd branchInertialParameters = Eigen::VectorXd::Zero(branchMoveableLinksCount * numParametersPerLink);

  // Get the romo IDs of the start and end bodies of the branch
  const int startBodyId = RD::template mapKeyEnumToKeyId<BodyEnum, BodyEnum>(RD::getBranchStartBody(branchEnum));
  const int endBodyId = RD::template mapKeyEnumToKeyId<BodyEnum, BodyEnum>(RD::getBranchEndBody(branchEnum));

  // Loop over the bodies on the branch
  int linkId = 0;
  for (int body_id = startBodyId; body_id <= endBodyId; body_id++) {

    // Get the current body
    auto bodyEnum = RD::template mapKeyIdToKeyEnum<BodyEnum, BodyEnum>(body_id);
    auto body = this->bodyContainer_[bodyEnum];

    if (body->getIsFixedBody()) {
      continue;// skip
    }

    // Get the inertial parameters of an individual body
    Eigen::VectorXd bodyInertialParameters = this->getBodyInertialParameters(bodyEnum);

    // Append these to the branchInertialParameters vector
    branchInertialParameters.segment<10>(linkId * numParametersPerLink)= bodyInertialParameters;
    linkId++;
  }
  return branchInertialParameters;
}

template<typename ConcreteDescription_, typename RobotState_>
Eigen::VectorXd RobotModelRbdl<ConcreteDescription_, RobotState_>::getBodyInertialParameters(const BodyEnum& bodyEnum) {

  // Get the body and inertial parameters (10 total parameters)
  auto body = this->bodyContainer_[bodyEnum];
  Eigen::VectorXd bodyInertialParameters = Eigen::VectorXd::Zero(10);

  // Get the body mass, inertia around CoM, center of mass, and first mass moment
  const double bodyMass = body->getMass();
  const Eigen::Matrix3d bodyInertiaCom = body->getInertiaMatrix();
  const Eigen::Vector3d bodyCenterOfMass = body->getPositionBodyToBodyCom(CoordinateFrameEnum::BODY);
  const Eigen::Vector3d bodyFirstMassMoment = bodyMass * (body->getPositionBodyToBodyCom(CoordinateFrameEnum::BODY));

  // Huygens Steiner Theorem to get bodyInertiaOrigin
  Eigen::Matrix3d bodyInertiaOrigin = Eigen::Matrix3d::Zero();
  bodyInertiaOrigin = bodyInertiaCom + bodyMass * ((bodyCenterOfMass.transpose() * bodyCenterOfMass) * Eigen::Matrix3d::Identity() - bodyCenterOfMass * (bodyCenterOfMass.transpose()));

  // Write bodyInertiaOrigin as a vector bodyInertiaOriginVec
  Eigen::VectorXd bodyInertiaOriginVec = Eigen::VectorXd::Zero(6);
  bodyInertiaOriginVec(0) = bodyInertiaOrigin(0,0);
  bodyInertiaOriginVec(1) = bodyInertiaOrigin(0,1);
  bodyInertiaOriginVec(2) = bodyInertiaOrigin(0,2);
  bodyInertiaOriginVec(3) = bodyInertiaOrigin(1,1);
  bodyInertiaOriginVec(4) = bodyInertiaOrigin(1,2);
  bodyInertiaOriginVec(5) = bodyInertiaOrigin(2,2);

  // Write bodyInertialParameters in format that allows inertial parameter estimation
  bodyInertialParameters(0) = bodyMass;
  bodyInertialParameters.segment<3>(1) = bodyFirstMassMoment;
  bodyInertialParameters.segment<6>(4) = bodyInertiaOriginVec;

  return bodyInertialParameters;
}

} // namespace romo_rbdl
