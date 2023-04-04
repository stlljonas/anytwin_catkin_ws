/*
 * TransitionProbabilityFromNoSlip.hpp
 *
 *  Created on: Jan 15, 2019
 *      Author: Fabian Jenelten
 */

#pragma once

// contact state estimation.
#include "probabilistic_contact_estimation/transition_model/TransitionProbability.hpp"

// basic filters.
#include "basic_filters/filters.hpp"


namespace contact_estimation {

class TransitionProbabilityFromNoSlip : public TransitionProbability {
  public:
    TransitionProbabilityFromNoSlip(AD::ContactEnum contactEnum, unsigned int numDofLimb);
    ~TransitionProbabilityFromNoSlip() override = default;

    bool loadParameters(const TiXmlHandle& handle) override;
    bool addVariablesToLog(const std::string& ns) const override;
    bool advance(double dt, const State* const state) override;
    bool initialize(const double dt) override;

  protected:

    //! Variance of foot velocity.
    double lambdaFootVelocityZ_;

    //! previous end-effector velocity.
    double previousLinearVelocityEndEffectorInPlaneFrameXY_;

    //! First order filter to smooth end-effector velocity in world frame.
    basic_filters::FirstOrderFilter<Eigen::Vector3d> firstOrderFilterVelocityEndEffector_;

    //! Filter time constant for velocity filter.
    double filterTimeConstant_;
};

} /* namespace contact_estimation */
