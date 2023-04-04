#include "series_elastic_actuator_sim/SeActuatorBase.hpp"

// series elastic actuator sim
namespace series_elastic_actuator_sim {


SeActuatorBase::SeActuatorBase():
    prevMode_(SeActuatorCommand::SeActuatorMode::MODE_DISABLE),
    controllers_()
{

}

SeActuatorBase::~SeActuatorBase()
{

}

SeActuatorBase::Controllers& SeActuatorBase::getControllers() {
  return controllers_;
}

bool SeActuatorBase::initializeControllers(const SeActuatorCommand& command, const SeActuatorState& state, double dt) {
  bool successful = true;
  for (auto & controller : controllers_) {
    if (!controller.second->initialize(command, state, dt)) {
      successful = false;
    }
  }
  return successful;
}

bool SeActuatorBase::advanceController(SeActuatorReading& reading, const SeActuatorCommand& command, double dt) {
  const SeActuatorCommand::SeActuatorMode mode = static_cast<SeActuatorCommand::SeActuatorMode>(command.getMode());

  // Check if the mode has changed.
  if (mode != prevMode_) {

    // Mode has changed. Initialize controller
    if (!controllers_.at(mode)->initialize(command, reading.getState(), dt)) {
      return false;
    }
    prevMode_ = mode;
  }
  // Update controller.
  if (!controllers_.at(static_cast<SeActuatorCommand::SeActuatorMode>(command.getMode()))->advance(reading.getCommanded(), command, reading.getState(), dt)) {
    return false;
  }
  return true;
}


} // series_elastic_actuator_sim
