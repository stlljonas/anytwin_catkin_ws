/*
 * FootholdPlanInvPend.hpp
 *
 *  Created on: Mar 03, 2018
 *      Author: Fabian Jenelten
 */

#pragma once

// loco
#include "loco/foothold_generation/FootholdPlan.hpp"

namespace loco {
namespace foothold_generator {

class FootholdPlanInvPend : public FootholdPlan {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Weight = Eigen::DiagonalMatrix<double, 2, 2>;

  using Base = FootholdPlan;

  FootholdPlanInvPend()
        : FootholdPlan(),
          positionWorldToVelocityProjectionsInWorldFrame_(),
          weightsVelocityProjection_(Weight(1.0, 1.0)),
          weightsPreviousSolution_(Weight(0.01, 0.01))
  {

  }

  ~FootholdPlanInvPend() override = default;

  bool initialize(const loco::WholeBody& wholeBody) override {
    if (!Base::initialize(wholeBody)) { return false; }
    // Position world to footholds are set in the Base::initialize function
    positionWorldToVelocityProjectionsInWorldFrame_ = positionsWorldToFootholdsInWorldFrame_;
    return true;
  }

  virtual void setPositionWorldToVelocityProjectionInWorldFrame(
      const Position& positionWorldToVelocityProjectionInWorldFrame,
      unsigned int endEffectorId) {
    positionWorldToVelocityProjectionsInWorldFrame_[endEffectorId] = positionWorldToVelocityProjectionInWorldFrame;
  }

  virtual const Position& getPositionWorldToVelocityProjectionInWorldFrame(unsigned int endEffectorId) const {
    return positionWorldToVelocityProjectionsInWorldFrame_[endEffectorId];
  }

  virtual void setWeights(
      const Weight& weightsVelocityProjection,
      const Weight& weightsPreviousSolution) {
    weightsVelocityProjection_ = weightsVelocityProjection;
    weightsPreviousSolution_   = weightsPreviousSolution;
  }

  virtual const Weight& getWeightVelocityProjection() const {
    return weightsVelocityProjection_;
  }

  virtual const Weight& getWeightPreviousSolution() const {
    return weightsPreviousSolution_;
  }

 protected:

  //! Desired footholds as given by any foothold generator.
  std::vector<Position, Eigen::aligned_allocator<Position>> positionWorldToVelocityProjectionsInWorldFrame_;

  // Weights.
  Weight weightsVelocityProjection_;
  Weight weightsPreviousSolution_;

};

} /* namespace foothold_generator */
} /* namespace loco */

