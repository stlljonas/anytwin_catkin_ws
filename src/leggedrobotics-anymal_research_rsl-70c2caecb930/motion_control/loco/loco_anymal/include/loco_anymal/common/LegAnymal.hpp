/*
 * LegAnymal.hpp
 *
 *  Created on: Dec 5, 2016
 *      Author: Dario Bellicoso, Peter Fankhauser
 */

#pragma once

// stl
#include <string>

// loco
#include "loco/common/legs/LegBase.hpp"
#include "loco/common/end_effectors/FootBase.hpp"

// loco anymal
#include <loco_anymal/typedefs.hpp>

namespace loco_anymal {

class LegAnymal : public LegRomo {
 public:
  LegAnymal(const std::string& name,
               anymal_description::AnymalDescription::ContactEnum contact,
               const anymal_model::AnymalModel& model,
               loco::FootBasePtr&& foot,
               bool updateDynamics = false);

  LegAnymal(const std::string& name,
               anymal_description::AnymalDescription::ContactEnum contact,
               const anymal_model::AnymalModel& model,
               loco::FootBasePtr&& foot,
               loco::LegPropertiesPtr&& properties,
               bool updateDynamics = false);

  ~LegAnymal() override = default;

  bool initialize(double dt) override;

  BranchEnum getBranch() const { return branchEnum_; }

  //! Gets the configuration (X or O) of the leg.
  anymal_description::AnymalTopology::LegConfigEnum getLegConfiguration() const {
    using AT = anymal_description::AnymalTopology;
    if (anymal_description::AnymalDefinitions::mapLimbToLongitudinal::at(this->limbEnum_) == AT::LongitudinalEnum::FORE) {
      return model_.getLegConfigurations()[this->getId()] ? AT::LegConfigEnum::XConfiguration : AT::LegConfigEnum::OConfiguration;
    } else {
      return model_.getLegConfigurations()[this->getId()] ? AT::LegConfigEnum::OConfiguration : AT::LegConfigEnum::XConfiguration;
    }
  }

 protected:
  //! Reference to the model updated with measurements.
  const anymal_model::AnymalModel& model_;
};

} /* namespace loco_anymal */
