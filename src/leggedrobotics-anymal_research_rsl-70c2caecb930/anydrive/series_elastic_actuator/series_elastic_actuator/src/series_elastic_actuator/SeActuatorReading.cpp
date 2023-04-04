/*!
 * @file    SeActuatorReading.cpp
 * @author  Christian Gehring
 * @date    Mar 18, 2015
 * @version 0.0
 *
 */
// series elastic actuator
#include "series_elastic_actuator/SeActuatorReading.hpp"


namespace series_elastic_actuator {


SeActuatorReading::SeActuatorReading() {}

SeActuatorReading::~SeActuatorReading() {}

const SeActuatorCommand& SeActuatorReading::getCommanded() const {
  return commanded_;
}

SeActuatorCommand& SeActuatorReading::getCommanded() {
  return commanded_;
}

void SeActuatorReading::setCommanded(const SeActuatorCommand& commanded) {
  commanded_ = commanded;
}

const SeActuatorState& SeActuatorReading::getState() const {
  return state_;
}

SeActuatorState& SeActuatorReading::getState() {
  return state_;
}

void SeActuatorReading::setState(const SeActuatorState& state) {
  state_ = state;
}


} // series_elastic_actuator
