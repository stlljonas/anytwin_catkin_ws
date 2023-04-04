/*
 * contact_schedule_anymal.hpp
 *
 *  Created on: April 18, 2018
 *      Author: Fabian Jenelten
 */

#pragma once

#include <map>
#include <vector>

// anymal_description
#include <anymal_description/LegEnum.hpp>

// std utils
#include <std_utils/std_utils.hpp>

namespace loco {
namespace contact_schedule {

using LegEnumAnymal = anymal_description::LegEnum;

constexpr auto numOfLegs = static_cast<unsigned int>(LegEnumAnymal::SIZE);

//! Transform leg index forward to backward motion.
inline LegEnumAnymal transformLegIdForDirectionChange(LegEnumAnymal legId) noexcept {
  switch (legId) {
    case LegEnumAnymal::LF:
      return LegEnumAnymal::RH;
      break;
    case LegEnumAnymal::RF:
      return LegEnumAnymal::LH;
      break;
    case LegEnumAnymal::LH:
      return LegEnumAnymal::RF;
      break;
    case LegEnumAnymal::RH:
      return LegEnumAnymal::LF;
      break;
    default:
      break;
  }

  return LegEnumAnymal::SIZE;
}

}  // namespace contact_schedule
}  // namespace loco
