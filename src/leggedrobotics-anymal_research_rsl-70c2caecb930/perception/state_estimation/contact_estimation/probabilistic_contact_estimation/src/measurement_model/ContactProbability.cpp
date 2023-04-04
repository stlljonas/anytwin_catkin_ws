/*
 * ContactProbability.cpp
 *
 *  Created on: Dez 21, 2018
 *      Author: Fabian Jenelten
 */


// contact state estimation
#include "probabilistic_contact_estimation/measurement_model/ContactProbability.hpp"

namespace contact_estimation {

ContactProbability::ContactProbability(
    AD::ContactEnum contactEnum,
    ProbabilityEnum contactProbability,
    unsigned int numDofLimb) :
  prob_(Probability(1.0, 0.0)),
  contactEnum_(contactEnum),
  probabilityEnum_(contactProbability),
  numDofLimb_(numDofLimb) {

}

bool ContactProbability::initialize(const double dt) {
  prob_ = initProbability;
  return true;
}

AD::ContactEnum ContactProbability::getContactEnum() const noexcept {
  return contactEnum_;
}

ProbabilityEnum ContactProbability::getProbabilityEnum() const noexcept {
  return probabilityEnum_;
}

std::string ContactProbability::getProbabilityName() const {
  return probabilityEnumMap[probabilityEnum_];
}

double ContactProbability::getContactProbability() const noexcept {
  return prob_(contactId);
}

double ContactProbability::getAirProbability() const noexcept {
  return prob_(airId);
}

bool ContactProbability::normalize() {
  return normalize2dProb(prob_);
}


} /* namespace contact_estimation */
