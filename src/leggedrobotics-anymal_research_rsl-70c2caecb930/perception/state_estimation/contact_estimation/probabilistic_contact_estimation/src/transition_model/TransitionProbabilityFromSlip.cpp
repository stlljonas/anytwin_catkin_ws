/*
 * TransitionProbabilityFromSlip.cpp
 *
 *  Created on: Jan 15, 2019
 *      Author: Fabian Jenelten
 */


// contact state estimation
#include "probabilistic_contact_estimation/transition_model/TransitionProbabilityFromSlip.hpp"

// signal logger.
#include <signal_logger/signal_logger.hpp>

// tinyxml tools
#include "tinyxml_tools/tinyxml_tools.hpp"

// robot utils.
#include "robot_utils/math/Stochastics.hpp"

namespace contact_estimation {

TransitionProbabilityFromSlip::TransitionProbabilityFromSlip(AD::ContactEnum contactEnum, unsigned int numDofLimb) :
  TransitionProbability(contactEnum, numDofLimb),
  lambdaFootVelocityZ_(0.5),
  previousLinearVelocityEndEffectorInPlaneFrameXY_(0.0),
  firstOrderFilterVelocityEndEffector_(),
  filterTimeConstant_(0.01) {
}

bool TransitionProbabilityFromSlip::initialize(double dt) {
  if (!TransitionProbability::initialize(dt)) { return false; }
  firstOrderFilterVelocityEndEffector_.setFilterParameters(dt, filterTimeConstant_, 1.0, Eigen::Vector3d::Zero());
  return true;
}

bool TransitionProbabilityFromSlip::loadParameters(const TiXmlHandle& handle) {
  TiXmlHandle contactEstimationHandle = handle;
  if(!tinyxml_tools::getChildHandle(contactEstimationHandle, handle, "ProbabilisticContactEstimation")) { return false; }

  TiXmlHandle transitionHandle = handle;
  if(!tinyxml_tools::getChildHandle(transitionHandle, contactEstimationHandle, "TransitionModel")) { return false; }

  TiXmlHandle stdDeviationHandle = handle;
  if(!tinyxml_tools::getChildHandle(stdDeviationHandle, transitionHandle, "StandardDeviations")) { return false; }

  TiXmlHandle footVelocityHandle = handle;
  if(!tinyxml_tools::getChildHandle(footVelocityHandle, stdDeviationHandle, "FootVelocity")) { return false; }
  double sigma;
  if(!tinyxml_tools::loadParameter(sigma, footVelocityHandle, "xy")) { return false; }

  TiXmlHandle filterHandle = handle;
  if(!tinyxml_tools::getChildHandle(filterHandle, transitionHandle, "Filters")) { return false; }
  if(!tinyxml_tools::loadParameter(filterTimeConstant_, filterHandle, "vel_filter_constant")) { return false; }

  // Convert std-deviation to variance.
  lambdaFootVelocityZ_ = 1.0/(2.0*sigma*sigma);

  return true;
}

bool TransitionProbabilityFromSlip::addVariablesToLog(const std::string& ns) const {
  const auto keyName = std::string(AD::mapKeyEnumToKeyName<AD::ContactEnum, AD::BranchEnum>(contactEnum_));
  signal_logger::add(prob_(noTransitionId), "/pTransitionSlipToSlip_" + keyName, "/contact_estimation", "-");
  signal_logger::add(prob_(transitionId), "/pTransitionSlipToNoSlip_" + keyName, "/contact_estimation", "-");
  return true;
}

bool TransitionProbabilityFromSlip::advance(double dt, const State* const state) {
  const Eigen::Vector3d filteredLinearVelocityEndEffectorInWorldFrame = firstOrderFilterVelocityEndEffector_.advance(state->getLinearVelocityEndEffectorInWorldFrame(contactEnum_));
  const Eigen::Vector3d filteredLinearVelocityEndEffectorInPlaneFrame = state->getOrientationWorldToPlane().rotate(filteredLinearVelocityEndEffectorInWorldFrame);
  const double filteredLinearVelocityEndEffectorInPlaneFrameXY = filteredLinearVelocityEndEffectorInPlaneFrame.head<2>().squaredNorm();

  // Compute slip probability.
  prob_(transitionId) = expPdf(
      previousLinearVelocityEndEffectorInPlaneFrameXY_,
      lambdaFootVelocityZ_
  ) - expPdf(
      filteredLinearVelocityEndEffectorInPlaneFrameXY,
      lambdaFootVelocityZ_
  );

  // Clip to box contraints and normalize.
  prob_(transitionId) = std::fmax(-prob_(transitionId), 0.0)/lambdaFootVelocityZ_;

  // Compute no- probability.
  prob_(noTransitionId) = 1.0 - prob_(transitionId);

  // Remember.
  previousLinearVelocityEndEffectorInPlaneFrameXY_ = filteredLinearVelocityEndEffectorInPlaneFrameXY;

  return true;
}


} /* namespace contact_estimation */
