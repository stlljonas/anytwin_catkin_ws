/*
 * TransitionProbability.hpp
 *
 *  Created on: Jan 15, 2019
 *      Author: Fabian Jenelten
 */

#pragma once

// contact state estimation.
#include "probabilistic_contact_estimation/probabilistic_contact_estimation.hpp"
#include "probabilistic_contact_estimation/State.hpp"

// tinyxml.
#include <tinyxml.h>

namespace contact_estimation {

class TransitionProbability {
  public:
    TransitionProbability(AD::ContactEnum contactEnum, unsigned int numDofLimb);
    virtual ~TransitionProbability() = default;

    virtual bool loadParameters(const TiXmlHandle& handle) { return true; }
    virtual bool advance(double dt, const State* const state) = 0;
    virtual bool addVariablesToLog(const std::string& ns) const { return true; }
    virtual bool initialize(const double dt);

    //! Returns the contact enum.
    AD::ContactEnum getContactEnum() const noexcept;

    //! Return probability of constant state.
    double getProbabilityOfStateConstant() const noexcept;

    //! Return probability of state transition.
    double getProbabilityOfStateTransition() const noexcept;

  protected:
    // number of dofs per limb.
    const unsigned int numDofLimb_;

    //! Probability of that contact does not change (0) or that contact does change (1).
    Probability prob_;

    //! Contact enum (leg, limb, ...)
    AD::ContactEnum contactEnum_;
};

} /* namespace contact_estimation */
