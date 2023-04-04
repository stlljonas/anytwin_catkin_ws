/*
 * KinematicsModelLwf.cpp
 *
 *  Created on: Oct 21, 2015
 *      Author: Dario Bellicoso
 */

#include <anymal_state_estimator_lwf/anymal_filter_lwf/KinematicsModelLwf.hpp>

namespace anymal_state_estimator_lwf {

KinematicsModelLwf::KinematicsModelLwf(double dt) : anymal_model::AnymalModel(dt) {}

Eigen::Vector3d KinematicsModelLwf::forwardKinematicsBaseToFootInBaseFrame(const Eigen::Vector3d& legJoints,
                                                                           int limbId) {
  const auto branchEnum = RD::mapKeyIdToKeyEnum<RD::LimbEnum, RD::BranchEnum>(limbId);

  anymal_model::AnymalState anymalState;
  anymal_model::JointPositions jointPositions;
  jointPositions.toImplementation().segment<RD::getNumDofLimb()>(limbId * 3) = legJoints;
  anymalState.setJointPositions(jointPositions);
  setState(anymalState, true, false, false);
  return getPositionBodyToBody(RD::BodyEnum::BASE, branchEnum, RD::BodyNodeEnum::FOOT, RD::CoordinateFrameEnum::BASE);
}

Eigen::Matrix3d KinematicsModelLwf::getJacobianTranslationBaseToFootInBaseFrame(const Eigen::Vector3d& legJoints,
                                                                                int limbId) {
  const auto branchEnum = RD::mapKeyIdToKeyEnum<RD::LimbEnum, RD::BranchEnum>(limbId);

  anymal_model::AnymalState anymalState;
  anymal_model::JointPositions jointPositions;
  jointPositions.toImplementation().segment<RD::getNumDofLimb()>(limbId * 3) = legJoints;
  anymalState.setJointPositions(jointPositions);
  setState(anymalState, true, false, false);
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(3, getDofCount());
  getJacobianTranslationWorldToBody(jacobian, branchEnum, RD::BodyNodeEnum::FOOT, RD::CoordinateFrameEnum::BASE);

  return jacobian.middleCols<RD::getNumDofLimb()>(RD::getBranchStartIndexInU(branchEnum));
}

}  // namespace anymal_state_estimator_lwf
