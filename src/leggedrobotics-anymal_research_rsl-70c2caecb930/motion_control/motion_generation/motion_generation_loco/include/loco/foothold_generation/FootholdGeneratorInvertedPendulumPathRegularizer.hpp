/*
 * FootholdGeneratorInvertedPendulumPathRegularizer.hpp
 *
 *  Created on: Jan 09, 2020
 *      Author: Fabian Jenelten
 */

#pragma once

//loco.
#include "loco/foothold_generation/FootholdGeneratorInvertedPendulumMotionGen.hpp"
#include "loco/torso_control/ComSupportControlZmp.hpp"

namespace loco {

class FootholdGeneratorInvertedPendulumPathRegularizer : public FootholdGeneratorInvertedPendulumMotionGen {
 public:
  FootholdGeneratorInvertedPendulumPathRegularizer(
      WholeBody& wholeBody,
      TerrainModelBase& terrain,
      ContactScheduleZmp& contactSchedule,
      ComSupportControlZmp& comSupportControl);

  ~FootholdGeneratorInvertedPendulumPathRegularizer() override = default;

  bool loadParameters(const TiXmlHandle& handle) override;

 protected:
  Position evaluateDesiredFootHoldOnTerrainFeedForwardInControlFrame(const LegBase& leg) override;

  //! A reference to com support control.
  const ComSupportControlZmp& comSupportControl_;

  //! If true, computes footholds along path regularizer.
  //! If false, computes foothold w.r.t. torso base.
  bool adaptFoothooldToPathRegularizer_;
};


} /* namespace loco */
