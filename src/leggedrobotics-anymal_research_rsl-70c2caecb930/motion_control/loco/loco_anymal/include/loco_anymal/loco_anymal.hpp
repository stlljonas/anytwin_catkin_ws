//
// Created by Dario Bellicoso on 15/12/17.
//

#pragma once


// anymal model
#include <anymal_model/AnymalModel.hpp>

// loco
#include <loco/common/end_effectors/FootBase.hpp>

// loco anymal
#include <loco_anymal/common/LegAnymal.hpp>
#include <loco_anymal/common/LegPropertiesAnymal.hpp>
#include <loco_anymal/common/LegsAnymal.hpp>
#include <loco_anymal/common/PointFootAnymal.hpp>
#include <loco_anymal/typedefs.hpp>

namespace loco_anymal {

namespace internal {

//! Create a leg object. Returns a raw pointer owned by the caller.
//! Do not use this method directly, use make_unique_leg(...) instead.
static inline LegAnymal* make_leg(
    std::string name,
    anymal_model::AnymalModel& model,
    anymal_model::AnymalModel& anymalModelDesired,
    anymal_description::AnymalDescription::ContactEnum contact,
    bool updateDynamics = false) {

  auto foot = loco_anymal::PointFootAnymalPtr(
      new loco_anymal::PointFootAnymal(
          contact, name + "_foot", model, anymalModelDesired)
  );

  return new loco_anymal::LegAnymal(
      name, contact, model, std::move(foot), updateDynamics);
}

} /* namespace internal */

//! Create a unique_ptr to a LegAnymal.

static inline std::unique_ptr<LegAnymal> make_unique_leg(
    std::string name,
    anymal_model::AnymalModel& model,
    anymal_model::AnymalModel& modelDesired,
    anymal_description::AnymalDescription::ContactEnum contact,
    bool updateDynamics = false) {
  return std::unique_ptr<LegAnymal>(internal::make_leg(std::move(name), model, modelDesired, contact, updateDynamics));
}

//! Create a unique_ptr to a group of LegAnymal.
static inline std::unique_ptr<LegsAnymal> make_unique_leg_group(
    anymal_model::AnymalModel& model,
    anymal_model::AnymalModel& modelDesired,
    bool updateDynamics = false) {
  using AD = anymal_description::AnymalDescription;

  auto leftForeLegPtr = make_unique_leg("leftFore", model, modelDesired, AD::ContactEnum::LF_FOOT, updateDynamics);
  auto rightForeLegPtr = make_unique_leg("rightFore", model, modelDesired, AD::ContactEnum::RF_FOOT, updateDynamics);
  auto leftHindLegPtr = make_unique_leg("leftHind", model, modelDesired, AD::ContactEnum::LH_FOOT, updateDynamics);
  auto rightHindLegPtr = make_unique_leg("rightHind", model, modelDesired, AD::ContactEnum::RH_FOOT, updateDynamics);

  return std::unique_ptr<LegsAnymal>(
      new loco_anymal::LegsAnymal(
          std::move(leftForeLegPtr), std::move(rightForeLegPtr),
          std::move(leftHindLegPtr), std::move(rightHindLegPtr))
  );
}

} /* namespace loco_anymal */
