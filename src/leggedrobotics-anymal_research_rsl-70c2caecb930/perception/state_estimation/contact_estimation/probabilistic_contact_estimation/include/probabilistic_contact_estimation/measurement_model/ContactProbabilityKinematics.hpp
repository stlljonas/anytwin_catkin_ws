/*
 * ContactProbabilityKinematics.hpp
 *
 *  Created on: Dez 21, 2018
 *      Author: Fabian Jenelten
 */

#pragma once

// contact state estimation.
#include "probabilistic_contact_estimation/measurement_model/ContactProbability.hpp"


namespace contact_estimation {

class ContactProbabilityKinematics : public ContactProbability {
  public:
    ContactProbabilityKinematics(
        AD::ContactEnum contactEnum,
        unsigned int numDofLimb);

    ~ContactProbabilityKinematics() override = default;

    bool loadParameters(const TiXmlHandle& handle) override;
    bool addVariablesToLog(const std::string& ns) const override;
    bool advance(double dt, const State* const state) override;

  protected:
    double sigmaGroundHeight_;
};

} /* namespace contact_estimation */
