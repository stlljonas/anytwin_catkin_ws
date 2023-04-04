/*
 * IndicatorTerrain.hpp
 *
 *  Created on: Aug 08, 2019
 *      Author: Fabian Jenelten
 */

#pragma once

// state checker.
#include "state_checker/IndicatorBase.hpp"

// robot utils.
#include "robot_utils/math/math.hpp"

class TiXmlHandle;

namespace loco {
namespace state_checker {

class IndicatorTerrain : public IndicatorBase {
 public:
  IndicatorTerrain() : IndicatorBase(), maxAllowedTerrainDrift_(0.12) { name_ = "terrain"; }

  ~IndicatorTerrain() override = default;

  bool initialize(double /*dt*/, WholeBody& /*wholeBody*/, TerrainModelBase& /*terrain*/) override { return true; }

  bool loadParameters(const TiXmlHandle& /*handle*/) override { return true; };

  double computeIndicatorValue(double /*dt*/, WholeBody& wholeBody, TerrainModelBase& terrain) override {
    double indicatorValue = 0.0;
    const auto& legs = wholeBody.getLegs();
    auto numOfGroundedLegs = 0u;

    for (auto leg : legs) {
      if (leg->getContactSchedule().isAndShouldBeGrounded()) {
        const auto positionWorldToEndEffectorInWorldFrame = leg->getFoot().getStateMeasured().getPositionWorldToEndEffectorInWorldFrame();
        double footHeight;
        if (!terrain.getHeight(positionWorldToEndEffectorInWorldFrame, footHeight)) {
          return 0.0;
        }
        indicatorValue += std::fabs(footHeight - positionWorldToEndEffectorInWorldFrame.z());
        ++numOfGroundedLegs;
      }
    }

    if (numOfGroundedLegs != 0u) {
      indicatorValue /= (maxAllowedTerrainDrift_ * static_cast<double>(numOfGroundedLegs));
    }

    return std::fmin(indicatorValue, 1.0);
  }

 protected:
  //! Max allowed drift between end-effectors and terrain perception.
  double maxAllowedTerrainDrift_;
};

} /* namespace state_checker */
} /* namespace loco */
