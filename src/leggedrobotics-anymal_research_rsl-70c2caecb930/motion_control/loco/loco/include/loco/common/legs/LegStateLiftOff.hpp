/*
 * LegStateLiftOff.hpp
 *
 *  Created on: Feb 26, 2014
 *      Author: Christian Gehring, Dario Bellicoso
 */

#pragma once

// loco
#include "loco/common/legs/LegStateBase.hpp"
#include "loco/common/typedefs.hpp"

namespace loco {

//!  State of the leg at the event of lift-off
/*!
 */
class LegStateLiftOff : public loco::LegStateBase {
 public:
  LegStateLiftOff();
  ~LegStateLiftOff() override = default;

  const Position& getPositionWorldToHipInWorldFrame() const;
  const Position& getPositionWorldToFootInWorldFrame() const;
  const Position& getPositionWorldToHipOnTerrainAlongWorldZAtLiftOffInWorldFrame() const;

  void setPositionWorldToHipInWorldFrame(const Position& positionWorldToHipInWorldFrame);
  void setPositionWorldToFootInWorldFrame(const Position& positionWorldToFootInWorldFrame);
  void setPositionWorldToHipOnTerrainAlongWorldZInWorldFrame(const Position& positionWorldToHipOnTerrainAlongWorldZAtLiftOffInWorldFrame);

 protected:
  Position positionWorldToFootInWorldFrame_;
  Position positionWorldToHipInWorldFrame_;
  Position positionWorldToHipOnTerrainAlongWorldZAtLiftOffInWorldFrame_;
};

} /* namespace loco */