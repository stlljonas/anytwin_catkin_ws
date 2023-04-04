/*
 * WholeBodyAnymal.hpp
 *
 *  Created on: Jan 30, 2018
 *      Author: Dario Bellicoso
 */

#pragma once

// stl
#include <string>

// loco anymal
#include <loco_anymal/typedefs.hpp>

// loco
#include <loco/common/arms/Arms.hpp>
#include <loco/common/WholeBodyProperties.hpp>
#include <loco/common/torso/TorsoBase.hpp>

namespace loco_anymal {

class WholeBodyAnymal : public WholeBodyRomo {
 public:
  template<typename LegGroup_ = loco::Legs>
  WholeBodyAnymal(const RobotModel & model,
                     loco::TorsoBase& torso,
                     const LegGroup_& legs,
                     bool updateDynamics = false) :
      WholeBodyRomo(model, torso, legs, loco::Arms(), updateDynamics)
  { }

  template<typename LegGroup_ = loco::Legs>
  WholeBodyAnymal(const RobotModel & model,
                     loco::TorsoBase& torso,
                     loco::WholeBodyPropertiesPtr&& wholeBodyProperties,
                     const LegGroup_& legs,
                     bool updateDynamics = false) :
      WholeBodyRomo(model, torso, std::move(wholeBodyProperties), legs, loco::Arms(), updateDynamics)
  { }

  ~WholeBodyAnymal() override = default;
};

} /* namespace loco_anymal */
