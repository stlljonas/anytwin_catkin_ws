/*
 * TransitionProbabilityFromContact.cpp
 *
 *  Created on: Jan 15, 2019
 *      Author: Fabian Jenelten
 */


// contact state estimation
#include "probabilistic_contact_estimation/transition_model/TransitionProbabilityFromContact.hpp"

// signal logger.
#include <signal_logger/signal_logger.hpp>

namespace contact_estimation {

TransitionProbabilityFromContact::TransitionProbabilityFromContact(AD::ContactEnum contactEnum, unsigned int numDofLimb) :
  TransitionProbability(contactEnum, numDofLimb) {
}

bool TransitionProbabilityFromContact::addVariablesToLog(const std::string& ns) const {
  const auto keyName = std::string(AD::mapKeyEnumToKeyName<AD::ContactEnum, AD::BranchEnum>(contactEnum_));
  signal_logger::add(prob_(noTransitionId), "/pTransitionContactToContact_" + keyName, "/contact_estimation", "-");
  signal_logger::add(prob_(transitionId), "/pTransitionContactToAir_" + keyName, "/contact_estimation", "-");
  return true;
}

bool TransitionProbabilityFromContact::advance(double dt, const State* const state) {
  // Average approximated swing duration for any gait.
  constexpr double stanceDuration = 0.5;

  prob_(transitionId) = dt/stanceDuration;
  prob_(noTransitionId) = 1.0 - prob_(transitionId);

  return true;
}


} /* namespace contact_estimation */
