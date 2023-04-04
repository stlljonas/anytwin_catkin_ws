/*
 * SlippingProbabilityDiffKinematics.cpp
 *
 *  Created on: Dez 21, 2018
 *      Author: Fabian Jenelten
 */


// contact state estimation
#include "probabilistic_contact_estimation/measurement_model/SlippingProbabilityDiffKinematics.hpp"

// signal logger.
#include <signal_logger/signal_logger.hpp>

// tinyxml tools
#include "tinyxml_tools/tinyxml_tools.hpp"

// robot utils.
#include "robot_utils/math/Stochastics.hpp"

namespace contact_estimation {

SlippingProbabilityDiffKinematics::SlippingProbabilityDiffKinematics(
    AD::ContactEnum contactEnum,
    unsigned int numDofLimb) :
  ContactProbability(contactEnum, ProbabilityEnum::DiffKinematics, numDofLimb),
  lambdaSlippingFootVelocity_(0.5) {

}

bool SlippingProbabilityDiffKinematics::loadParameters(const TiXmlHandle& handle) {
  TiXmlHandle contactEstimationHandle = handle;
  if(!tinyxml_tools::getChildHandle(contactEstimationHandle, handle, "ProbabilisticContactEstimation")) { return false; }

  TiXmlHandle measurementHandle = handle;
  if(!tinyxml_tools::getChildHandle(measurementHandle, contactEstimationHandle, "MeasurementModel")) { return false; }

  // Standard Deviations.
  TiXmlHandle stdDeviationHandle = handle;
  if(!tinyxml_tools::getChildHandle(stdDeviationHandle, measurementHandle, "StandardDeviations")) { return false; }

  TiXmlHandle slippingFootVelocityHandle = handle;
  double sigma;
  if(!tinyxml_tools::getChildHandle(slippingFootVelocityHandle, stdDeviationHandle, "FootVelocity")) { return false; }
  if(!tinyxml_tools::loadParameter(sigma, slippingFootVelocityHandle, "xy")) { return false; }
  lambdaSlippingFootVelocity_ = 1.0/(2.0*sigma*sigma);

  return true;
}

bool SlippingProbabilityDiffKinematics::addVariablesToLog(const std::string& ns) const {
  const auto keyName = std::string(AD::mapKeyEnumToKeyName<AD::ContactEnum, AD::BranchEnum>(contactEnum_));
  signal_logger::add(prob_(contactId), "/pContactFromDiffKinSlipping" + keyName, "/contact_estimation", "-");
  signal_logger::add(prob_(airId), "/pAirFromDiffKinSlipping" + keyName, "/contact_estimation", "-");
  return true;
}

bool SlippingProbabilityDiffKinematics::advance(double dt, const State* const state) {
  // Compute probability that leg is moving base on end-effector velocity.
  const Eigen::Vector3d filteredLinearVelocityFootInPlaneFrame = state->getOrientationWorldToPlane().rotate(state->getLinearVelocityEndEffectorInWorldFrame(contactEnum_));
  const double filteredLinearVelocityFootSquaredNorm = filteredLinearVelocityFootInPlaneFrame.head<2>().squaredNorm();

  // Probability of slipping.
  prob_(airId) = expCdf(
      filteredLinearVelocityFootSquaredNorm,
      lambdaSlippingFootVelocity_
  );

  // Probability of not slipping.
  prob_(contactId) = 1.0-prob_(airId);

  return clip(prob_);
}


} /* namespace contact_estimation */
