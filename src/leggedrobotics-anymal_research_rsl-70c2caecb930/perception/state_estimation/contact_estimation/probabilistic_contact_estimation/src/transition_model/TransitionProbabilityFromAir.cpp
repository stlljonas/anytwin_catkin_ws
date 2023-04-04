/*
 * TransitionProbabilityFromAir.cpp
 *
 *  Created on: Jan 15, 2019
 *      Author: Fabian Jenelten
 */


// contact state estimation
#include "probabilistic_contact_estimation/transition_model/TransitionProbabilityFromAir.hpp"

// signal logger.
#include <signal_logger/signal_logger.hpp>

namespace contact_estimation {

TransitionProbabilityFromAir::TransitionProbabilityFromAir(AD::ContactEnum contactEnum, unsigned int numDofLimb) :
  TransitionProbability(contactEnum, numDofLimb) {
}

bool TransitionProbabilityFromAir::addVariablesToLog(const std::string& ns) const {
  const auto keyName = std::string(AD::mapKeyEnumToKeyName<AD::ContactEnum, AD::BranchEnum>(contactEnum_));
  signal_logger::add(prob_(noTransitionId), "/pTransitionAirToAir_" + keyName, "/contact_estimation", "-");
  signal_logger::add(prob_(transitionId), "/pTransitionAirToContact_" + keyName, "/contact_estimation", "-");
  return true;
}

bool TransitionProbabilityFromAir::advance(double dt, const State* const state) {
  // Average approximated stance duration for any gait.
  constexpr double swingDuration = 0.5;

  prob_(transitionId) = dt/swingDuration;
  prob_(noTransitionId) = 1.0 - prob_(transitionId);

  return true;
}


} /* namespace contact_estimation */
