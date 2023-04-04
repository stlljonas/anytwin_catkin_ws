/*
 * ContactProbabilityDynamics.hpp
 *
 *  Created on: Dez 21, 2018
 *      Author: Fabian Jenelten
 */

#pragma once

// contact state estimation.
#include "probabilistic_contact_estimation/measurement_model/ContactProbability.hpp"

namespace contact_estimation {

class ContactProbabilityDynamics : public ContactProbability {
  public:
    ContactProbabilityDynamics(
        AD::ContactEnum contactEnum,
        unsigned int numDofLimb);

    ~ContactProbabilityDynamics() override = default;

    bool loadParameters(const TiXmlHandle& handle) override;
    bool addVariablesToLog(const std::string& ns) const override;
    bool advance(double dt, const State* const state) override;

  protected:
    Eigen::Matrix3d VarJointAccelerations_;
    Eigen::Matrix3d VarDisturbance_;
    Eigen::Matrix3d VarContact_;
};

} /* namespace contact_estimation */
