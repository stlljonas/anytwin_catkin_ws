/*
 * ContactProbabilityDynamics.cpp
 *
 *  Created on: Dez 21, 2018
 *      Author: Fabian Jenelten
 */


// contact state estimation
#include "probabilistic_contact_estimation/measurement_model/ContactProbabilityDynamics.hpp"

// signal logger.
#include <signal_logger/signal_logger.hpp>

// tinyxml tools
#include "tinyxml_tools/tinyxml_tools.hpp"

// robot utils.
#include "robot_utils/math/Stochastics.hpp"

namespace contact_estimation {

ContactProbabilityDynamics::ContactProbabilityDynamics(
    AD::ContactEnum contactEnum,
    unsigned int numDofLimb) :
  ContactProbability(contactEnum, ProbabilityEnum::Dynamics, numDofLimb),
  VarJointAccelerations_(Eigen::Matrix3d::Identity()*30.0*30.0),
  VarDisturbance_(Eigen::Matrix3d::Identity()*1.0*1.0),
  VarContact_(Eigen::Matrix3d::Identity()*35.0*35.0) {

}

bool ContactProbabilityDynamics::loadParameters(const TiXmlHandle& handle) {
  TiXmlHandle contactEstimationHandle = handle;
  if(!tinyxml_tools::getChildHandle(contactEstimationHandle, handle, "ProbabilisticContactEstimation")) { return false; }

  TiXmlHandle measurementHandle = handle;
  if(!tinyxml_tools::getChildHandle(measurementHandle, contactEstimationHandle, "MeasurementModel")) { return false; }

  // Standard Deviations.
  double x; double y; double z;
  TiXmlHandle stdDeviationHandle = handle;
  if(!tinyxml_tools::getChildHandle(stdDeviationHandle, measurementHandle, "StandardDeviations")) { return false; }

  TiXmlHandle jointAccelerationHandle = handle;
  if(!tinyxml_tools::getChildHandle(jointAccelerationHandle, stdDeviationHandle, "JointAcceleration")) { return false; }
  if(!tinyxml_tools::loadParameter(x, jointAccelerationHandle, "x")) { return false; }
  if(!tinyxml_tools::loadParameter(y, jointAccelerationHandle, "y")) { return false; }
  if(!tinyxml_tools::loadParameter(z, jointAccelerationHandle, "z")) { return false; }
  Eigen::Vector3d sigmaJointAccelerations(x, y, z);

  TiXmlHandle disturbanceHandle = handle;
  if(!tinyxml_tools::getChildHandle(disturbanceHandle, stdDeviationHandle, "Disturbance")) { return false; }
  if(!tinyxml_tools::loadParameter(x, disturbanceHandle, "x")) { return false; }
  if(!tinyxml_tools::loadParameter(y, disturbanceHandle, "y")) { return false; }
  if(!tinyxml_tools::loadParameter(z, disturbanceHandle, "z")) { return false; }
  Eigen::Vector3d sigmaDisturbance(x, y, z);

  TiXmlHandle contactForceHandle = handle;
  if(!tinyxml_tools::getChildHandle(contactForceHandle, stdDeviationHandle, "ContactForce")) { return false; }
  if(!tinyxml_tools::loadParameter(x, contactForceHandle, "x")) { return false; }
  if(!tinyxml_tools::loadParameter(y, contactForceHandle, "y")) { return false; }
  if(!tinyxml_tools::loadParameter(z, contactForceHandle, "z")) { return false; }
  Eigen::Vector3d sigmaContact(x, y, z);

  // Convert std-deviation to variance.
  VarJointAccelerations_.setZero();
  VarJointAccelerations_.diagonal() = sigmaJointAccelerations.cwiseAbs2();

  VarDisturbance_.setZero();
  VarDisturbance_.diagonal() = sigmaDisturbance.cwiseAbs2();

  VarContact_.setZero();
  VarContact_.diagonal() = sigmaContact.cwiseAbs2();

  return true;
}

bool ContactProbabilityDynamics::addVariablesToLog(const std::string& ns) const {
  const auto keyName = std::string(AD::mapKeyEnumToKeyName<AD::ContactEnum, AD::BranchEnum>(contactEnum_));
  signal_logger::add(prob_(contactId), "/pContactFromDynamics_" + keyName, "/contact_estimation", "-");
  signal_logger::add(prob_(airId), "/pAirFromDynamics_" + keyName, "/contact_estimation", "-");
  return true;
}

bool ContactProbabilityDynamics::advance(double dt, const State* const state) {
  constexpr auto numDofBase = 6u;
  const auto baseIdInU = AD::getBranchStartIndexInU(AD::BranchEnum::BASE);
  const auto branchEnum = AD::mapKeyEnumToKeyEnum<AD::ContactEnum, AD::BranchEnum>(contactEnum_);
  const auto branchIdInU = AD::getBranchStartIndexInU(branchEnum);

  // Base and limb acceleration.
  const auto& filteredGeneralizedBaseAcceleration = state->getGeneralizedAccelerations().segment<numDofBase>(baseIdInU);
  const auto& filteredGeneralizedLimbAcceleration = state->getGeneralizedAccelerations().segment(branchIdInU, numDofLimb_);

  // Extract jacobian of leg.
  // J = [I_33 - R_IB*S(I_r_BF) 0_33 0_33 0_33 I_J_BF]
  const Eigen::MatrixXd& jacobianTranslationBaseToFootInWorldFrame = state->getJacobianTranslationWorldToEndEffectorInWorldFrame(contactEnum_).middleCols(branchIdInU, numDofLimb_);
  if (std::abs(jacobianTranslationBaseToFootInWorldFrame.determinant()) < eps_ ) {
    MELO_WARN_STREAM("[ProbabilisticContactEstimation::advance] Jacobian is singular.");
    return true; // dont update contact probability.
  }

  // Linear acceleration leg, assuming leg is swinging (Equation 5 with F = 0)
  const auto estimatedLinearAccelerationLegInWorldFrameInAir = state->getLegMassMatrixInverted(contactEnum_) * (
      state->getNonlinearityAndTorque().segment(branchIdInU, numDofLimb_)
      - state->getMassMatrix().block(branchIdInU, baseIdInU, numDofLimb_, numDofBase)*filteredGeneralizedBaseAcceleration
  );

  // End-effector force (Equation 6).
  const auto forceFootInWorldFrame = -1.0*state->getContactForceInWorldFrame(contactEnum_);

  // Linear acceleration leg, assuming leg is grounded (Equation 5).
  const auto estimatedLinearAccelerationLegInWorldFrameInContact =
      estimatedLinearAccelerationLegInWorldFrameInAir
      - state->getLegMassMatrixInverted(contactEnum_) * jacobianTranslationBaseToFootInWorldFrame.transpose()*forceFootInWorldFrame;

  // Measured probability for kinematics, assuming leg is swinging (covariance follows from Equation 7).
  const auto CovAir = VarJointAccelerations_ + state->getLegMassMatrixInverted(contactEnum_) * VarDisturbance_ * state->getLegMassMatrixInverted(contactEnum_).transpose();
  prob_(airId) = robot_utils::normalPdf<3>(
      filteredGeneralizedLimbAcceleration,
      estimatedLinearAccelerationLegInWorldFrameInAir,
      CovAir
  );

  // Measured probability for kinematics, assuming leg is grounded (covariance follows from Equation 7).
  const auto massMatrixInvertedTimesJacobian = state->getLegMassMatrixInverted(contactEnum_) * jacobianTranslationBaseToFootInWorldFrame.transpose();
  const auto CovContact = CovAir + massMatrixInvertedTimesJacobian*VarContact_*massMatrixInvertedTimesJacobian.transpose();
  prob_(contactId) = robot_utils::normalPdf<3>(
      filteredGeneralizedLimbAcceleration,
      estimatedLinearAccelerationLegInWorldFrameInContact,
      CovContact
  );

  return normalize();
}


} /* namespace contact_estimation */
