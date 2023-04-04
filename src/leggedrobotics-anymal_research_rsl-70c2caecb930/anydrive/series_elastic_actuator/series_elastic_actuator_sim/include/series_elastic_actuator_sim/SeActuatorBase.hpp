/*!
 * @file    SeActuatorCommand.hpp
 * @author  Christian Gehring
 * @date    Mar 18, 2015
 * @version 0.0
 *
 */
#pragma once


// c++
#include <map>

// series elastic actuator sim
#include "series_elastic_actuator_sim/controller/ControllerBase.hpp"
#include "series_elastic_actuator_sim/usings.hpp"


namespace series_elastic_actuator_sim {


class SeActuatorBase
{
 public:
  typedef std::map<SeActuatorCommand::SeActuatorMode, controller::ControllerBase*> Controllers;

 public:
  SeActuatorBase();
  virtual ~SeActuatorBase();
  virtual bool initialize(const SeActuatorCommand& command, const SeActuatorState& state, double dt) = 0;
  virtual bool advance(SeActuatorReading& reading, const SeActuatorCommand& command, double dt) = 0;

 protected:
  virtual Controllers& getControllers();
  virtual bool initializeControllers(const SeActuatorCommand& command, const SeActuatorState& state, double dt);
  virtual bool advanceController(SeActuatorReading& reading, const SeActuatorCommand& command, double dt);

 protected:
  SeActuatorCommand::SeActuatorMode prevMode_;
  Controllers controllers_;
};


} // series_elastic_actuator_sim
