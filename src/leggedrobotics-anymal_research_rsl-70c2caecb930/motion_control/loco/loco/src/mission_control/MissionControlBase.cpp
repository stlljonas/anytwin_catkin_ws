/*
 * MissionControlBase.cpp
 *
 *  Created on: Mar 7, 2014
 *      Author: Christian Gehring
 */

// loco
#include "loco/mission_control/MissionControlBase.hpp"

namespace loco {

MissionControlBase::MissionControlBase() : ModuleBase() {}

bool MissionControlBase::setToInterpolated(const MissionControlBase& missionController1, const MissionControlBase& missionController2,
                                           double t) {
  return false;
}

} /* namespace loco */
