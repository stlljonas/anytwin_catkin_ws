/*
 * LegPropertiesAnymal.hpp
 *
 *  Created on: Jan 16, 2018
 *      Author: Dario Bellicoso
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once

// loco anymal
#include <loco_anymal/typedefs.hpp>

// anymal model
#include <anymal_model/AnymalModel.hpp>


namespace loco_anymal {

class LegPropertiesAnymal: public LegPropertiesRomo {
 public:
  LegPropertiesAnymal(LimbEnum limb,
                         const anymal_model::AnymalModel& model);
  ~LegPropertiesAnymal() override = default;

  double getMaximumLimbExtension() const override;
  double getMinimumLimbExtension() const override;

 protected:
  const anymal_model::AnymalModel& model_;
};

} /* namespace loco_anymal */
