/*
 * TransitionProbabilityContainer.cpp
 *
 *  Created on: Jan 15, 2018
 *      Author: Fabian Jenelten
 */


// probabilistic contact estimation.
#include "probabilistic_contact_estimation/transition_model/TransitionProbabilityContainer.hpp"
#include "probabilistic_contact_estimation/transition_model/TransitionProbabilityFromContact.hpp"
#include "probabilistic_contact_estimation/transition_model/TransitionProbabilityFromAir.hpp"

// signal logger.
#include <signal_logger/signal_logger.hpp>

namespace contact_estimation {
TransitionProbabilityContainer::TransitionProbabilityContainer():
    transitionProbabilityFromContact_(),
    transitionProbabilityFromAir_(),
    mapContactEnumToId_() {

}

bool TransitionProbabilityContainer::loadParameters(const TiXmlHandle& handle) {
  for (auto const& cp : transitionProbabilityFromContact_) {
    if(!cp->loadParameters(handle)) {
      MELO_WARN_STREAM("[TransitionProbabilityContainer::loadParameters] Failed to load parameters for transition probability from contact.");
      return false;
    }
  }

  for (auto const& cp : transitionProbabilityFromAir_) {
    if(!cp->loadParameters(handle)) {
      MELO_WARN_STREAM("[TransitionProbabilityContainer::loadParameters] Failed to load parameters for transition probability from air.");
      return false;
    }
  }

  return true;
}

bool TransitionProbabilityContainer::advance(double dt, const State* const state) {
  for (auto const& cp : transitionProbabilityFromContact_) {
    if(!cp->advance(dt, state)) {
      MELO_WARN_STREAM("[TransitionProbabilityContainer::advance] Failed to advance transition probability from contact.");
      return false;
    }
  }

  for (auto const& cp : transitionProbabilityFromAir_) {
    if(!cp->advance(dt, state)) {
      MELO_WARN_STREAM("[TransitionProbabilityContainer::advance] Failed to advance transition probability from air.");
      return false;
    }
  }

  return true;
}

bool TransitionProbabilityContainer::initialize(const double dt) {
  for (auto const& cp : transitionProbabilityFromContact_) {
    if(!cp->initialize(dt)) {
      MELO_WARN_STREAM("[TransitionProbabilityContainer::initialize] Failed to initialize transition probability from contact.");
      return false;
    }
  }

  for (auto const& cp : transitionProbabilityFromAir_) {
    if(!cp->initialize(dt)) {
      MELO_WARN_STREAM("[TransitionProbabilityContainer::initialize] Failed to initialize transition probability from air.");
      return false;
    }
  }

  return true;
}

bool TransitionProbabilityContainer::addVariablesToLog(const std::string& ns) const {
  for (auto const& cp : transitionProbabilityFromContact_) {
    if(!cp->addVariablesToLog(ns)) {
      MELO_WARN_STREAM("[TransitionProbabilityContainer::addVariablesToLog] Failed to log transition probability from contact.");
      return false;
    }
  }

  for (auto const& cp : transitionProbabilityFromAir_) {
    if(!cp->addVariablesToLog(ns)) {
      MELO_WARN_STREAM("[TransitionProbabilityContainer::addVariablesToLog] Failed to log transition probability from air.");
      return false;
    }
  }

  return true;
}

void TransitionProbabilityContainer::clear() {
  transitionProbabilityFromContact_.clear();
  transitionProbabilityFromAir_.clear();
  mapContactEnumToId_.clear();
}

void TransitionProbabilityContainer::addProbability(
    TransitionProbability* transitionProbabilityFromContact,
    TransitionProbability* transitionProbabilityFromAir,
    const AD::ContactEnum& contactEnum) {
  mapContactEnumToId_.insert(std::make_pair(contactEnum, transitionProbabilityFromContact_.size()));
  transitionProbabilityFromContact_.emplace_back(std::unique_ptr<TransitionProbability>(transitionProbabilityFromContact));
  transitionProbabilityFromAir_.emplace_back(std::unique_ptr<TransitionProbability>(transitionProbabilityFromAir));
}

const TransitionProbability& TransitionProbabilityContainer::getTransitionProbabilityFromContactEnum(const AD::ContactEnum& contactEnum) const {
  return *transitionProbabilityFromContact_[mapContactEnumToId_.at(contactEnum)];
}

const TransitionProbability& TransitionProbabilityContainer::getTransitionProbabilityFromAir(const AD::ContactEnum& contactEnum) const {
  return *transitionProbabilityFromAir_[mapContactEnumToId_.at(contactEnum)];
}

bool TransitionProbabilityContainer::containsTransitionProbability(const AD::ContactEnum& contactEnum) const {
  return (mapContactEnumToId_.find(contactEnum)!=mapContactEnumToId_.end());
}


} /* namespace contact_estimation */
