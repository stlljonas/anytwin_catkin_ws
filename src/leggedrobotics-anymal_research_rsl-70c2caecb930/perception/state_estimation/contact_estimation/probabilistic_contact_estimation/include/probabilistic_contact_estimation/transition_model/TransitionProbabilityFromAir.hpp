/*
 * TransitionProbabilityFromAir.hpp
 *
 *  Created on: Jan 15, 2019
 *      Author: Fabian Jenelten
 */

#pragma once

// contact state estimation.
#include "probabilistic_contact_estimation/transition_model/TransitionProbability.hpp"

namespace contact_estimation {

class TransitionProbabilityFromAir : public TransitionProbability {
  public:
    TransitionProbabilityFromAir(AD::ContactEnum contactEnum, unsigned int numDofLimb);
    ~TransitionProbabilityFromAir() override = default;

    bool addVariablesToLog(const std::string& ns) const override;
    bool advance(double dt, const State* const state) override;
};

} /* namespace contact_estimation */
