/*
 * ContactProbabilityDiffKinematics.hpp
 *
 *  Created on: Dez 21, 2018
 *      Author: Fabian Jenelten
 */

#pragma once

// contact state estimation.
#include "probabilistic_contact_estimation/measurement_model/ContactProbability.hpp"

namespace contact_estimation {

class ContactProbabilityDiffKinematics : public ContactProbability {
  public:
    ContactProbabilityDiffKinematics(
        AD::ContactEnum contactEnum,
        unsigned int numDofLimb);

    ~ContactProbabilityDiffKinematics() override = default;

    bool loadParameters(const TiXmlHandle& handle) override;
    bool addVariablesToLog(const std::string& ns) const override;
    bool advance(double dt, const State* const state) override;

  protected:
    double lambdaFootVelocity_;
};

} /* namespace contact_estimation */
