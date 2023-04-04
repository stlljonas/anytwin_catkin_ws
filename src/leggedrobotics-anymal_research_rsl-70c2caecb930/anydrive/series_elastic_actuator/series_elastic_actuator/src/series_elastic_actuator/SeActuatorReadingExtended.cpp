/*!
 * @file    SeActuatorReadingExtended.cpp
 * @author  Remo Diethelm
 * @date    Jul 29, 2016
 * @version 0.0
 *
 */
// series elastic actuator
#include "series_elastic_actuator/SeActuatorReadingExtended.hpp"


namespace series_elastic_actuator {


SeActuatorReadingExtended::SeActuatorReadingExtended() {}

SeActuatorReadingExtended::~SeActuatorReadingExtended() {}

const SeActuatorCommand& SeActuatorReadingExtended::getCommanded() const {
  return commanded_;
}

SeActuatorCommand& SeActuatorReadingExtended::getCommanded() {
  return commanded_;
}

void SeActuatorReadingExtended::setCommanded(const SeActuatorCommand& commanded) {
  commanded_ = commanded;
}

const SeActuatorStateExtended& SeActuatorReadingExtended::getState() const {
  return state_;
}

SeActuatorStateExtended& SeActuatorReadingExtended::getState() {
  return state_;
}

void SeActuatorReadingExtended::setState(const SeActuatorStateExtended& state) {
  state_ = state;
}


} // series_elastic_actuator
