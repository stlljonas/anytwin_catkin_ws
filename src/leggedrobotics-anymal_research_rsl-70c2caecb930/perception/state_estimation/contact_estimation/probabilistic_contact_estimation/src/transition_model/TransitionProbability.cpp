/*
 * TransitionProbability.cpp
 *
 *  Created on: Jan 15, 2019
 *      Author: Fabian Jenelten
 */


// contact state estimation
#include "probabilistic_contact_estimation/transition_model/TransitionProbability.hpp"

namespace contact_estimation {

TransitionProbability::TransitionProbability(AD::ContactEnum contactEnum, unsigned int numDofLimb) :
  prob_(Probability(1.0, 0.0)),
  contactEnum_(contactEnum),
  numDofLimb_(numDofLimb) {

}

bool TransitionProbability::initialize(const double dt) {
  prob_ = initProbability;
  return true;
}

AD::ContactEnum TransitionProbability::getContactEnum() const noexcept {
  return contactEnum_;
}

double TransitionProbability::getProbabilityOfStateConstant() const noexcept {
  return prob_(noTransitionId);
}

double TransitionProbability::getProbabilityOfStateTransition() const noexcept {
  return prob_(transitionId);
}


} /* namespace contact_estimation */
