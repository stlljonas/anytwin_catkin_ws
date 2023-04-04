/*
 * contact_schedule_zmp.hpp
 *
 *  Created on: April 18, 2018
 *      Author: Fabian Jenelten
 */

#pragma once


// loco
#include "motion_generation/contact_schedule_periodic.hpp"

// tinyxml tools
#include <tinyxml_tools/tinyxml_tools.hpp>


namespace loco {
namespace contact_schedule {

  //! Load parameters into module for different gaits.
  template<typename ModuleType_>
  static bool loadGaitParameters(
      const std::vector<TiXmlElement*>& gaitElements,
      ModuleType_& module,
      const std::map<std::string, unsigned int>& mapGaitNameToId) {
    module.clear();

    // Helper variables.
    std::string gaitPatternTypeStr;
    unsigned int numOfAddedGait = 0u;
    unsigned int gaitIndex = 0u;

    for (const auto& gait : gaitElements) {
      if(!tinyxml_tools::loadParameter(gaitPatternTypeStr, gait, "type")) { return false; }
      const auto gaitNames = extractGaitNamesFromString(gaitPatternTypeStr);

      bool didLoadFirstGait = false;
      for (const auto& gaitName : gaitNames) {

        // Check if gait exists
        if (mapGaitNameToId.find(gaitName) == mapGaitNameToId.end()) {
          continue;
        }

        // Load gait parameters.
        if (!didLoadFirstGait) {
          ++numOfAddedGait; gaitIndex = numOfAddedGait - 1u;
          didLoadFirstGait = true;
          MELO_DEBUG_STREAM(message_logger::color::blue << "  Load parameters for " << message_logger::color::red << gaitName)
          if(!module.loadParameters(gait, gaitIndex, gaitName)) {
            MELO_WARN_STREAM("Failed to load gait parameters for "<< gaitName)
            return false;
          }
        }

        // Parameters already loaded, add gait index.
        else {
          MELO_DEBUG_STREAM(message_logger::color::blue << "   Add parameters for " << message_logger::color::red << gaitName)
          if(!module.addGait(gaitIndex, gaitName)) {
            MELO_WARN_STREAM("Failed to add gait parameters for "<< gaitName)
            return false;
          }
        }
        MELO_DEBUG_STREAM(message_logger::color::blue << " gait (" << gaitIndex << ")" << message_logger::color::def)

      }
    }

    if (numOfAddedGait == 0u) {
      MELO_WARN_STREAM("No gaits have been loaded.")
      return true;
    }

    // Check if we have loaded all required gaits.
    if(!module.check(mapGaitNameToId)) {
      MELO_WARN_STREAM("Something went wrong while loading.")
      return false;
    }

    return true;
  }

}
} // namespace loco
