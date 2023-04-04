/*
 * FootholdGeneratorFreePlane.hpp
 *
 *  Created on: Aug 27, 2015
 *      Author: Christian Gehring
 */
#pragma once

#include "loco/foothold_generation/FootholdGeneratorBase.hpp"

namespace loco {

//! Interface class for a foothold generator for a terrain that is modeled as a free plane
class FootholdGeneratorFreePlane : public FootholdGeneratorBase {
 public:
  FootholdGeneratorFreePlane() = default;
  ~FootholdGeneratorFreePlane() override = default;

  virtual Position computeWorldToFootholdInWorldFrame(int legId) = 0;
  virtual const Position& getPositionHipOnTerrainToDesiredFootHoldOnTerrainFeedBackInControlFrame(int legId) const = 0;
  virtual const Position& getPositionHipOnTerrainToDesiredFootHoldOnTerrainFeedForwardInControlFrame(int legId) const = 0;
};

} /* namespace loco */
