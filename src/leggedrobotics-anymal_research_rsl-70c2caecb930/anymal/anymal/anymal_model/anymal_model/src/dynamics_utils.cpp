/*
 * dynamics_utils.cpp
 *
 *  Created on: Nov 22, 2015
 *      Author: Dario Bellicoso
 */

// anymal model
#include "anymal_model/dynamics_utils.hpp"

// robot utils
#include <robot_utils/math/LinearAlgebra.hpp>

// kindr
#include <kindr/math/LinearAlgebra.hpp>

namespace anymal_model {

bool computeBranchEndPointReferenceAccelerationFromDesiredMotion(
    const AD::LimbEnum limb, const Eigen::Vector3d& desiredPositionWorldToFootInWorldFrame,
    const Eigen::Vector3d& desiredLinearVelocityFootInWorldFrame, const Eigen::Vector3d& desiredLinearAccelerationFootInWorldFrame,
    const Eigen::Vector3d& proportionalGains, const Eigen::Vector3d& derivativeGains, const Eigen::Vector3d& feedforwardGains,
    const AnymalModel& model, Eigen::VectorXd& taskSpaceAccelerations) {
  constexpr auto dofCount = AD::getNumDof();
  const auto branch = AD::mapEnums<AD::BranchEnum>(limb);
  // get leg jacobian
  Eigen::MatrixXd jacobianTranslationWorldToFootInWorldFrame = Eigen::MatrixXd::Zero(3, dofCount);
  model.getJacobianTranslationWorldToBody(jacobianTranslationWorldToFootInWorldFrame, branch, AD::BodyNodeEnum::FOOT,
                                          AD::CoordinateFrameEnum::WORLD);

  Eigen::Vector3d positionWorldToFootInWorldFrame;
  model.getPositionWorldToBody(positionWorldToFootInWorldFrame, branch, AD::BodyNodeEnum::FOOT, AD::CoordinateFrameEnum::WORLD);

  const Eigen::Vector3d positionErrorInWorldFrame = desiredPositionWorldToFootInWorldFrame - positionWorldToFootInWorldFrame;
  const Eigen::Vector3d velocityErrorInWorldFrame =
      desiredLinearVelocityFootInWorldFrame - jacobianTranslationWorldToFootInWorldFrame * model.getState().getGeneralizedVelocities();

  taskSpaceAccelerations = feedforwardGains.cwiseProduct(desiredLinearAccelerationFootInWorldFrame) +
                           derivativeGains.cwiseProduct(velocityErrorInWorldFrame) +
                           proportionalGains.cwiseProduct(positionErrorInWorldFrame);

  return true;
}

bool computeInverseDynamicsBranchTorquesFromDesiredMotion(const AD::BranchEnum branch,
                                                          const Eigen::Vector3d& desiredPositionBaseToFootInBaseFrame,
                                                          const Eigen::Vector3d& desiredLinearVelocityFootInBaseFrame,
                                                          const Eigen::Vector3d& desiredLinearAccelerationFootInBaseFrame,
                                                          const Eigen::Vector3d& proportionalGains, const Eigen::Vector3d& derivativeGains,
                                                          const Eigen::Vector3d& feedforwardGains, const AnymalModel& model,
                                                          Eigen::VectorXd& torques) {
  constexpr auto dofCount = AD::getNumDof();
  const auto limb = AD::mapEnums<AD::LimbEnum>(branch);

  Eigen::VectorXd nonlinearEffects = Eigen::VectorXd::Zero(model.getDofCount());
  model.getNonlinearEffects(nonlinearEffects);

  Eigen::MatrixXd massMatrix = Eigen::MatrixXd::Zero(dofCount, dofCount);
  model.getMassInertiaMatrix(massMatrix);

  // get leg jacobian
  Eigen::MatrixXd jacobianTranslationWorldToFootInBaseFrame = Eigen::MatrixXd::Zero(3, dofCount);
  model.getJacobianTranslationWorldToBody(jacobianTranslationWorldToFootInBaseFrame, branch, AD::BodyNodeEnum::FOOT,
                                          AD::CoordinateFrameEnum::BASE);

  const Eigen::Matrix3d jacobianTranslationBaseToFootInBaseFrame =
      jacobianTranslationWorldToFootInBaseFrame.middleCols<AD::getNumDofLimb()>(AD::getBranchStartIndexInU(branch));

  // get leg jacobian time derivative
  Eigen::MatrixXd jacobianTranslationTimeDerivativeWorldToFootInBaseFrame = Eigen::MatrixXd::Zero(3, dofCount);
  model.getJacobianTranslationTimeDerivativeWorldToBody(jacobianTranslationTimeDerivativeWorldToFootInBaseFrame, branch,
                                                        AD::BodyNodeEnum::FOOT, AD::CoordinateFrameEnum::BASE);
  const Eigen::Matrix3d jacobianTimeDerivativeBaseToFootInBaseFrame =
      jacobianTranslationTimeDerivativeWorldToFootInBaseFrame.middleCols<AD::getNumDofLimb()>(AD::getBranchStartIndexInU(branch));
  const Eigen::Vector3d& branchJointVelocities =
      model.getState().getJointVelocities().toImplementation().segment<AD::getNumDofLimb()>(AD::getLimbStartIndexInJ(limb));

  Eigen::Vector3d positionBaseToFootInBaseFrame;
  model.getPositionBodyToBody(positionBaseToFootInBaseFrame, AD::BodyEnum::BASE, branch, AD::BodyNodeEnum::FOOT,
                              AD::CoordinateFrameEnum::BASE);

  const Eigen::Vector3d positionErrorInBaseFrame = desiredPositionBaseToFootInBaseFrame - positionBaseToFootInBaseFrame;
  const Eigen::Vector3d velocityErrorInBaseFrame =
      desiredLinearVelocityFootInBaseFrame - jacobianTranslationBaseToFootInBaseFrame * branchJointVelocities;

  const Eigen::Vector3d referenceAcceleration =
      robot_utils::pseudoInverseAdaptiveDls(jacobianTranslationBaseToFootInBaseFrame) *
      (feedforwardGains.cwiseProduct(desiredLinearAccelerationFootInBaseFrame) + derivativeGains.cwiseProduct(velocityErrorInBaseFrame) +
       proportionalGains.cwiseProduct(positionErrorInBaseFrame) - jacobianTimeDerivativeBaseToFootInBaseFrame * branchJointVelocities);

  torques =
      massMatrix.block<AD::getNumDofLimb(), AD::getNumDofLimb()>(AD::getBranchStartIndexInU(branch), AD::getBranchStartIndexInU(branch)) *
          referenceAcceleration +
      nonlinearEffects.segment<AD::getNumDofLimb()>(AD::getBranchStartIndexInU(branch));

  return true;
}

}  // namespace anymal_model
