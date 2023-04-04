/*
 * LegAnymal.cpp
 *
 *  Created on: Dec 5, 2016
 *      Author: Dario Bellicoso, Peter Fankhauser
 */

// loco anymal
#include <loco_anymal/common/LegAnymal.hpp>
#include <loco_anymal/common/LegPropertiesAnymal.hpp>
#include <loco_anymal/common/PointFootAnymal.hpp>

// loco
#include <loco/common/end_effectors/FootBase.hpp>

// anymal description
#include <anymal_description/AnymalDescription.hpp>

namespace loco_anymal {

LegAnymal::LegAnymal(const std::string& name,
                           ContactEnum contact,
                           const anymal_model::AnymalModel& model,
                           loco::FootBasePtr&& foot,
                           loco::LegPropertiesPtr&& properties,
                           bool updateDynamics) :
  LegRomo(anymal_description::AnymalDescription::mapEnums<LimbEnum>(contact),
          name, model, std::move(properties), std::move(foot), updateDynamics),
  model_(model)
{ }

LegAnymal::LegAnymal(const std::string& name,
                           ContactEnum contact,
                           const anymal_model::AnymalModel& model,
                           loco::FootBasePtr&& foot,
                           bool updateDynamics) :
    LegRomo(anymal_description::AnymalDescription::mapEnums<LimbEnum>(contact), name, model,
            loco::LegPropertiesPtr(new loco_anymal::LegPropertiesAnymal(
                anymal_description::AnymalDescription::mapEnums<LimbEnum>(contact), model)), std::move(foot), updateDynamics),
    model_(model)
{ }

bool LegAnymal::initialize(double dt) {
  bool success = LegRomo::initialize(dt);

  // Init contact schedule.
  this->getContactSchedulePtr()->setIsGrounded(this->getFoot().isInContact());
  this->getContactSchedulePtr()->setIsSlipping(this->getFoot().isSlipping());
  this->getContactSchedulePtr()->setIsInStandConfiguration(true);
  this->getContactSchedulePtr()->setIsLosingContact(false);
  this->getContactSchedulePtr()->setIsSlipping(false);
  this->getContactSchedulePtr()->setShouldBeGrounded(true);
  this->getContactSchedulePtr()->setWasGrounded(true);
  this->getLimbStrategyPtr()->setLimbStrategyEnum(loco::LimbStrategyEnum::Support);

  return success;
}

} /* namespace loco_anymal */
