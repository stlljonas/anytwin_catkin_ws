/*!
 * @file    SeActuatorReading.hpp
 * @author  Christian Gehring
 * @date    Mar 18, 2015
 * @version 0.0
 *
 */
#pragma once


// series elastic actuator
#include "series_elastic_actuator/SeActuatorCommand.hpp"
#include "series_elastic_actuator/SeActuatorState.hpp"


namespace series_elastic_actuator {


class SeActuatorReading
{
 public:
  SeActuatorReading();
  virtual ~SeActuatorReading();

  const SeActuatorCommand& getCommanded() const;
  SeActuatorCommand& getCommanded();
  void setCommanded(const SeActuatorCommand& commanded);

  const SeActuatorState& getState() const;
  SeActuatorState& getState();
  void setState(const SeActuatorState& state);

 protected:
  SeActuatorCommand commanded_;
  SeActuatorState state_;
};


} // series_elastic_actuator
