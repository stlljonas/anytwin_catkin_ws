/*
 * ContactProbability.hpp
 *
 *  Created on: Dez 21, 2018
 *      Author: Fabian Jenelten
 */

#pragma once

// contact state estimation.
#include "probabilistic_contact_estimation/probabilistic_contact_estimation.hpp"
#include "probabilistic_contact_estimation/State.hpp"

// tinyxml.
#include <tinyxml.h>

namespace contact_estimation {

class ContactProbability {
  public:
    ContactProbability(
        AD::ContactEnum contactEnum,
        ProbabilityEnum contactProbability,
        unsigned int numDofLimb);

    virtual ~ContactProbability() = default;

    virtual bool loadParameters(const TiXmlHandle& handle) { return true; }
    virtual bool advance(double dt, const State* const state) = 0;
    virtual bool addVariablesToLog(const std::string& ns) const { return true; }
    bool initialize(const double dt);

    //! Returns the contact enum.
    AD::ContactEnum getContactEnum() const noexcept;

    //! Returns the probability type.
    ProbabilityEnum getProbabilityEnum() const noexcept;

    //! Return the name of the probability type.
    std::string getProbabilityName() const;

    //! Return probability of being in contact.
    double getContactProbability() const noexcept;

    //! Return probability of being not in contact.
    double getAirProbability() const noexcept;


  protected:
    bool normalize();

    // number of dofs per limb.
    const unsigned int numDofLimb_;

    //! Probability of being in contact (0) or being in the air (1)
    Probability prob_;

    //! Contact enum (leg, limb, ...)
    AD::ContactEnum contactEnum_;

    //! Type of the contact probability.
    ProbabilityEnum probabilityEnum_;
};

} /* namespace contact_estimation */
