/*
 * contact_schedule_periodic.hpp
 *
 *  Created on: April 18, 2018
 *      Author: Fabian Jenelten
 */

#pragma once


// loco
#include "loco/gait_pattern/contact_schedule.hpp"


namespace loco {
namespace contact_schedule {

  inline std::vector<std::string> extractGaitNamesFromString(const std::string& gaitPatternTypeStr) {
    std::vector<std::string> gaitNames;
    std::size_t start_of_name = 0;
    std::size_t end_of_name = gaitPatternTypeStr.find(",");

    while (true) {
      if (end_of_name != std::string::npos) {
        gaitNames.push_back(gaitPatternTypeStr.substr(start_of_name, end_of_name-start_of_name));
      }
      else {
        gaitNames.push_back(gaitPatternTypeStr.substr(start_of_name, gaitPatternTypeStr.length()-start_of_name));
        break;
      }

      start_of_name = gaitPatternTypeStr.find_first_not_of(",:;. ", end_of_name);
      end_of_name = gaitPatternTypeStr.find(",", start_of_name);
    }

    return gaitNames;
  }

  inline LegEnumAnymal nextLeg(LegEnumAnymal legId) noexcept {
    switch(legId) {
      case LegEnumAnymal::LF :
        return LegEnumAnymal::RF; break;
      case LegEnumAnymal::RF :
        return LegEnumAnymal::LH; break;
      case LegEnumAnymal::LH :
        return LegEnumAnymal::RH; break;
      case LegEnumAnymal::RH :
        return LegEnumAnymal::LF; break;
      default : break;
    }

    return LegEnumAnymal::SIZE;
  }
}
} // namespace loco
