/*!
 * @file    SeActuatorReadingExtended.hpp
 * @author  Remo Diethelm
 * @date    Jul 29, 2016
 * @version 0.0
 *
 */
#pragma once


// series elastic actuator
#include "series_elastic_actuator/SeActuatorCommand.hpp"
#include "series_elastic_actuator/SeActuatorStateExtended.hpp"


namespace series_elastic_actuator {


class SeActuatorReadingExtended
{
 public:
  SeActuatorReadingExtended();
  virtual ~SeActuatorReadingExtended();

  const SeActuatorCommand& getCommanded() const;
  SeActuatorCommand& getCommanded();
  void setCommanded(const SeActuatorCommand& commanded);

  const SeActuatorStateExtended& getState() const;
  SeActuatorStateExtended& getState();
  void setState(const SeActuatorStateExtended& state);

 protected:
  SeActuatorCommand commanded_;
  SeActuatorStateExtended state_;
};


} // series_elastic_actuator
