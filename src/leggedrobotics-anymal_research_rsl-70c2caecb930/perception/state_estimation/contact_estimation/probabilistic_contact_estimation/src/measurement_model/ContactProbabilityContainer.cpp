/*
 * ContactProbabilityContainer.cpp
 *
 *  Created on: Dez 21, 2018
 *      Author: Fabian Jenelten
 */


// contact state estimation
#include "probabilistic_contact_estimation/measurement_model/ContactProbabilityContainer.hpp"

// signal logger.
#include <signal_logger/signal_logger.hpp>

namespace contact_estimation {
ContactProbabilityContainer::ContactProbabilityContainer():
    contactProbabilities_() {

}

bool ContactProbabilityContainer::loadParameters(const TiXmlHandle& handle) {
  for (auto const& cp : contactProbabilities_) {
    if(!cp->loadParameters(handle)) {
      MELO_WARN_STREAM("[ContactProbabilityContainer::loadParameters] Failed to load parameters for contact probability with name "
          << cp->getProbabilityName() << ".");
      return false;
    }
  }

  return true;
}

bool ContactProbabilityContainer::advance(double dt, const State* const state) {
  for (auto const& cp : contactProbabilities_) {
    if(!cp->advance(dt, state)) {
      MELO_WARN_STREAM("[ContactProbabilityContainer::advance] Failed to advance contact probability with name "
          << cp->getProbabilityName() << ".");
      return false;
    }
  }

  return true;
}

bool ContactProbabilityContainer::initialize(const double dt) {
  for (auto const& cp : contactProbabilities_) {
    if(!cp->initialize(dt)) {
      MELO_WARN_STREAM("[ContactProbabilityContainer::initialize] Failed to initialize contact probability with name "
          << cp->getProbabilityName() <<  ".");
      return false;
    }
  }

  return true;
}

bool ContactProbabilityContainer::addVariablesToLog(const std::string& ns) const {
  for (auto const& cp : contactProbabilities_) {
    if(!cp->addVariablesToLog(ns)) {
      MELO_WARN_STREAM("[ContactProbabilityContainer::addVariablesToLog] Failed to log contact probability with name "
          << cp->getProbabilityName() << ".");
      return false;
    }
  }
  return true;
}

void ContactProbabilityContainer::clear() {
  contactProbabilities_.clear();
}

void ContactProbabilityContainer::addProbability(
    ContactProbability* cp,
    const AD::ContactEnum& contactEnum) {
  contactProbabilities_.emplace_back(std::unique_ptr<ContactProbability>(cp));
}

const std::vector<std::unique_ptr<ContactProbability>>& ContactProbabilityContainer::getProbabilities() const {
  return contactProbabilities_;
}


} /* namespace contact_estimation */
