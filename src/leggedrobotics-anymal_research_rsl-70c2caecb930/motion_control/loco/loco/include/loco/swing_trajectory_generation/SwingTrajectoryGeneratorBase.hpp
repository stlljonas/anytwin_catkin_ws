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

namespace loco {

/*!
 * Generate swing foot trajectories.
 *
 * Inputs:
 * - in ``contactSchedule``:
 *   - swingPhase
 *   - swingDuration
 * - leg->positionWorldToLastOrCurrentContactInWorldFrame
 * - torso->orientationWorldToControl
 *
 * Outputs: N/A
 *
 * Advanced by: foot placement strategy.
 *
 */
class SwingTrajectoryGeneratorBase {
 public:
  SwingTrajectoryGeneratorBase() = default;
  virtual ~SwingTrajectoryGeneratorBase() = default;
  virtual bool loadParameters(const TiXmlHandle& handle) = 0;
  virtual bool getDesiredFootState(Position& positionWorldToDesiredFootInWorldFrame, LinearVelocity& linearVelocityDesiredFootInWorldFrame,
                                   LinearAcceleration& linearAccelerationDesiredFootInWorldFrame,
                                   const Position& positionWorldToDesiredFootholdInControlFrame, LegBase* leg, double dt) = 0;
};

} /* namespace loco */
