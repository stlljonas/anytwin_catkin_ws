/*
 * ContactProbabilityKinematics.cpp
 *
 *  Created on: Dez 21, 2018
 *      Author: Fabian Jenelten
 */


// contact state estimation
#include "probabilistic_contact_estimation/measurement_model/ContactProbabilityKinematics.hpp"

// signal logger.
#include <signal_logger/signal_logger.hpp>

// tinyxml tools
#include "tinyxml_tools/tinyxml_tools.hpp"

// robot utils.
#include "robot_utils/math/Stochastics.hpp"

namespace contact_estimation {

ContactProbabilityKinematics::ContactProbabilityKinematics(
    AD::ContactEnum contactEnum,
    unsigned int numDofLimb) :
  ContactProbability(contactEnum, ProbabilityEnum::Kinematics, numDofLimb),
  sigmaGroundHeight_(0.1) {

}

bool ContactProbabilityKinematics::loadParameters(const TiXmlHandle& handle) {
  TiXmlHandle contactEstimationHandle = handle;
  if(!tinyxml_tools::getChildHandle(contactEstimationHandle, handle, "ProbabilisticContactEstimation")) { return false; }

  TiXmlHandle measurementHandle = handle;
  if(!tinyxml_tools::getChildHandle(measurementHandle, contactEstimationHandle, "MeasurementModel")) { return false; }

  // Standard Deviations.
  TiXmlHandle stdDeviationHandle = handle;
  if(!tinyxml_tools::getChildHandle(stdDeviationHandle, measurementHandle, "StandardDeviations")) { return false; }

  double sigma;
  TiXmlHandle groundHeightHandle = handle;
  if(!tinyxml_tools::getChildHandle(groundHeightHandle, stdDeviationHandle, "GroundHeight")) { return false; }
  if(!tinyxml_tools::loadParameter(sigmaGroundHeight_, groundHeightHandle, "value")) { return false; }

  return true;
}

bool ContactProbabilityKinematics::addVariablesToLog(const std::string& ns) const {
  const auto keyName = std::string(AD::mapKeyEnumToKeyName<AD::ContactEnum, AD::BranchEnum>(contactEnum_));
  signal_logger::add(prob_(contactId), "/pContactFromKin_" + keyName, "/contact_estimation", "-");
  signal_logger::add(prob_(airId), "/pAirFromKin_" + keyName, "/contact_estimation", "-");
  return true;
}

bool ContactProbabilityKinematics::advance(double dt, const State* const state) {
  /*
   * Note: Kinematics is used only to avoid false contact detections in the middle of the swing leg trajectory.
   * Since it is not very accurate we do not want it to detect a contact. Hence, it is saturated at 0.5.
   */
  const double endEffectorHeight =
      state->getPositionWorldToEndEffectorInWorldFrame(contactEnum_).z() -
      state->getExpectedGroundHeightInWorldFrame(contactEnum_);

  // Compute air probability.
  prob_(airId) = std::fmax(robot_utils::normalCdf(endEffectorHeight, 0.0, sigmaGroundHeight_), 0.5);

  // Compute contact probability.
  prob_(contactId) = 1.0 - prob_(airId);

  return clip(prob_);
}


} /* namespace contact_estimation */
