/*
 * WholeBody.cpp
 *
 *  Created on: Feb 8, 2016
 *      Author: Dario Bellicoso, Christian Gehring, Gabriel Hottiger
 */

// loco
#include <loco/common/WholeBody.hpp>

// signal logger
#include <signal_logger/signal_logger.hpp>

namespace loco {

bool WholeBody::allLegsGrounded() {
  return std::find_if(legs_.begin(), legs_.end(), [](LegBase* leg) { return !leg->getContactSchedule().isGrounded(); }) == legs_.end();
}

bool WholeBody::addVariablesToLog(const std::string& ns) const {
  std::string nameSpace = ns + std::string("/loco/whole_body/");

  signal_logger::add(getWholeBodyStateMeasured().getPositionWorldToWholeBodyCenterOfMassInWorldFrame(), "positionWorldToComInWorldFrame",
                     nameSpace, "m");

  signal_logger::add(getWholeBodyStateMeasured().getPositionWorldToDCMInWorldFrame(), "positionWorldToDcmInWorldFrame", nameSpace, "m");

  signal_logger::add(getWholeBodyStateMeasured().getLinearVelocityWholeBodyCenterOfMassInWorldFrame(), "linearVelocityComInWorldFrame",
                     nameSpace, "m/s");

  signal_logger::add(getWholeBodyStateMeasured().getLinearVelocityWholeBodyCenterOfMassInWorldFrame(), "linearVelocityWbComInWorldFrame",
                     nameSpace, "m/s");

  signal_logger::add(getWholeBodyStateDesired().getPositionWorldToWholeBodyCenterOfMassInWorldFrame(),
                     "positionWorldToDesiredComInWorldFrame", nameSpace, "m");

  signal_logger::add(getWholeBodyStateDesired().getLinearVelocityWholeBodyCenterOfMassInWorldFrame(),
                     "linearVelocityDesiredComInWorldFrame", nameSpace, "m/s");

  signal_logger::add(getWholeBodyStateMeasured().getPositionControlToWholeBodyCenterOfMassInControlFrame(),
                     "positionControlToComInControlFrame", nameSpace, "m");

  signal_logger::add(getWholeBodyStateMeasured().getLinearVelocityWholeBodyCenterOfMassInControlFrame(), "linearVelocityComInControlFrame",
                     nameSpace, "m/s");

  signal_logger::add(getWholeBodyStateMeasured().getPositionWorldToCenterOfPressureInWorldFrame(), "positionWorldToZmpInWorldFrame",
                     nameSpace, "m");

  return true;
}

} /* namespace loco */
