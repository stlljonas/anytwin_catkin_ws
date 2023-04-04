/*
 * ContactProbabilityContainer.hpp
 *
 *  Created on: Dez 21, 2018
 *      Author: Fabian Jenelten
 */

// stl (for unique pointer)
#include <memory>

#include "probabilistic_contact_estimation/measurement_model/ContactProbability.hpp"

// contact state estimation.
#include "probabilistic_contact_estimation/State.hpp"

// tinyxml.
#include <tinyxml.h>

#pragma once

namespace contact_estimation {

class ContactProbabilityContainer {
  public:
  ContactProbabilityContainer();
  ~ContactProbabilityContainer() = default;

  bool loadParameters(const TiXmlHandle& handle);
  bool advance(double dt, const State* const state);
  bool initialize(const double dt);
  bool addVariablesToLog(const std::string& ns = "") const;

  //! Clears vector of contact probabilities.
  void clear();

  //! Add a new contact probability to the limb associated with contactEnum.
  void addProbability(
      ContactProbability* contactProbability,
      const AD::ContactEnum& contactEnum);

  const std::vector<std::unique_ptr<ContactProbability>>& getProbabilities() const;

  private:
  //! Vector of measured contact probabilities.
  std::vector<std::unique_ptr<ContactProbability>> contactProbabilities_;
};

} /* namespace contact_estimation */
