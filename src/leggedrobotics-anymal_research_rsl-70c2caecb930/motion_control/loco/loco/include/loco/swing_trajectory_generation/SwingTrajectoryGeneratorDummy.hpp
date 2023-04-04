/*
 * SwingTrajectoryGeneratorBase.hpp
 *
 *  Created on: Feb 1, 2015
 *      Author: C. Dario Bellicoso, Christian Gehring
 */

#pragma once

// tinyxml
#include <tinyxml.h>

// loco
#include "loco/common/legs/LegBase.hpp"
#include "loco/common/typedefs.hpp"
#include "loco/swing_trajectory_generation/SwingTrajectoryGeneratorBase.hpp"

namespace loco {

class SwingTrajectoryGeneratorDummy : public SwingTrajectoryGeneratorBase {
 public:
  SwingTrajectoryGeneratorDummy() = default;
  ~SwingTrajectoryGeneratorDummy() override = default;

  bool loadParameters(const TiXmlHandle& handle) override { return true; }

  bool getDesiredFootState(Position& positionWorldToDesiredFootInWorldFrame, LinearVelocity& linearVelocityDesiredFootInWorldFrame,
                           LinearAcceleration& linearAccelerationDesiredFootInWorldFrame,
                           const Position& positionWorldToDesiredFootholdInControlFrame, LegBase* leg, double dt) override {
    return true;
  }
};

} /* namespace loco */
