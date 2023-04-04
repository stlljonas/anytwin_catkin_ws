/*
 * ContactProbabilityDiffKinematics.cpp
 *
 *  Created on: Dez 21, 2018
 *      Author: Fabian Jenelten
 */


// contact state estimation
#include "probabilistic_contact_estimation/measurement_model/ContactProbabilityDiffKinematics.hpp"

// signal logger.
#include <signal_logger/signal_logger.hpp>

// tinyxml tools
#include "tinyxml_tools/tinyxml_tools.hpp"

// robot utils.
#include "robot_utils/math/Stochastics.hpp"

namespace contact_estimation {

ContactProbabilityDiffKinematics::ContactProbabilityDiffKinematics(
    AD::ContactEnum contactEnum,
    unsigned int numDofLimb) :
  ContactProbability(contactEnum, ProbabilityEnum::DiffKinematics, numDofLimb),
  lambdaFootVelocity_(0.5) {

}

bool ContactProbabilityDiffKinematics::loadParameters(const TiXmlHandle& handle) {
  TiXmlHandle contactEstimationHandle = handle;
  if(!tinyxml_tools::getChildHandle(contactEstimationHandle, handle, "ProbabilisticContactEstimation")) { return false; }

  TiXmlHandle measurementHandle = handle;
  if(!tinyxml_tools::getChildHandle(measurementHandle, contactEstimationHandle, "MeasurementModel")) { return false; }

  // Standard Deviations.
  TiXmlHandle stdDeviationHandle = handle;
  if(!tinyxml_tools::getChildHandle(stdDeviationHandle, measurementHandle, "StandardDeviations")) { return false; }

  TiXmlHandle footVelocityHandle = handle;
  double sigma;
  if(!tinyxml_tools::getChildHandle(footVelocityHandle, stdDeviationHandle, "FootVelocity")) { return false; }
  if(!tinyxml_tools::loadParameter(sigma, footVelocityHandle, "z")) { return false; }
  lambdaFootVelocity_ = 1.0/(2.0*sigma*sigma);

  return true;
}

bool ContactProbabilityDiffKinematics::addVariablesToLog(const std::string& ns) const {
  const auto keyName = std::string(AD::mapKeyEnumToKeyName<AD::ContactEnum, AD::BranchEnum>(contactEnum_));
  signal_logger::add(prob_(contactId), "/pContactFromDiffKin_" + keyName, "/contact_estimation", "-");
  signal_logger::add(prob_(airId), "/pAirFromDiffKin_" + keyName, "/contact_estimation", "-");
  return true;
}

bool ContactProbabilityDiffKinematics::advance(double dt, const State* const state) {
	  // Compute probability that leg is moving base on end-effector velocity.
	  const Eigen::Vector3d filteredLinearVelocityFootInPlaneFrame = state->getOrientationWorldToPlane().rotate(state->getLinearVelocityEndEffectorInWorldFrame(contactEnum_));
	  const double filteredLinearVelocityFootSquaredNorm = filteredLinearVelocityFootInPlaneFrame.z()*filteredLinearVelocityFootInPlaneFrame.z();

	  // Probability of being in the air.
	  prob_(airId) = expCdf(
	      filteredLinearVelocityFootSquaredNorm,
	      lambdaFootVelocity_
	  );

	  // Probability of being grounded.
	  prob_(contactId) = 1.0-prob_(airId);

	  return clip(prob_);
}


} /* namespace contact_estimation */
