/*
 * TransitionProbabilityContainer.hpp
 *
 *  Created on: Jan 15, 2018
 *      Author: Fabian Jenelten
 */

// stl (for unique pointer)
#include <memory>

// probabilistic contact estimation.
#include "probabilistic_contact_estimation/transition_model/TransitionProbability.hpp"
#include "probabilistic_contact_estimation/State.hpp"

// tinyxml.
#include <tinyxml.h>

#pragma once

namespace contact_estimation {

class TransitionProbabilityContainer {
  public:
  TransitionProbabilityContainer();
  ~TransitionProbabilityContainer() = default;

  bool loadParameters(const TiXmlHandle& handle);
  bool advance(double dt, const State* const state);
  bool initialize(const double dt);
  bool addVariablesToLog(const std::string& ns = "") const;

  //! Clears vector of contact probabilities.
  void clear();

  //! Add a new transition probability to the limb associated with contactEnum.
  void addProbability(
      TransitionProbability* transitionProbabilityFromContact,
      TransitionProbability* transitionProbabilityFromAir,
      const AD::ContactEnum& contactEnum);

  const TransitionProbability& getTransitionProbabilityFromContactEnum(const AD::ContactEnum& contactEnum) const;
  const TransitionProbability& getTransitionProbabilityFromAir(const AD::ContactEnum& contactEnum) const;

  //! True if an transition probability pair exist for contactEnum.
  bool containsTransitionProbability(const AD::ContactEnum& contactEnum) const;

  private:
  //! Vector of transition probabilities (from contact).
  std::vector<std::unique_ptr< TransitionProbability>> transitionProbabilityFromContact_;

  //! Vector of transition probabilities (from air).
  std::vector<std::unique_ptr< TransitionProbability>> transitionProbabilityFromAir_;

  //! Map contact enum to id in transition probability vector.
  std::map<AD::ContactEnum, unsigned int> mapContactEnumToId_;
};

} /* namespace contact_estimation */
