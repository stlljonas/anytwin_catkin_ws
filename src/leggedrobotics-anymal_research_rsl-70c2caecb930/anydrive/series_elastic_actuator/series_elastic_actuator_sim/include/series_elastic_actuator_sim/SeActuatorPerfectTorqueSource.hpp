/*!
 * @file    SeActuatorPerfectTorqueSource.hpp
 * @author  Christian Gehring
 * @date    Mar 18, 2015
 * @version 0.0
 *
 */
#pragma once


// series elastic actuator sim
#include "series_elastic_actuator_sim/controller/TorqueControllerDisable.hpp"
#include "series_elastic_actuator_sim/controller/TorqueControllerFreeze.hpp"
#include "series_elastic_actuator_sim/controller/TorqueControllerTorque.hpp"
#include "series_elastic_actuator_sim/controller/TorqueControllerJointPositionVelocityPid.hpp"
#include "series_elastic_actuator_sim/controller/TorqueControllerJointPositionVelocityTorque.hpp"
#include "series_elastic_actuator_sim/controller/TorqueControllerJointPositionVelocityTorquePidGains.hpp"
#include "series_elastic_actuator_sim/SeActuatorBase.hpp"


namespace series_elastic_actuator_sim {


class SeActuatorPerfectTorqueSource : public SeActuatorBase
{
 public:
  SeActuatorPerfectTorqueSource();
  virtual ~SeActuatorPerfectTorqueSource();

  virtual bool initialize(const SeActuatorCommand& command, const SeActuatorState& state, double dt);
  virtual bool advance(SeActuatorReading& reading, const SeActuatorCommand& command, double dt);

  controller::TorqueControllerDisable& getControllerDisable();
  controller::TorqueControllerFreeze& getControllerFreeze();
  controller::TorqueControllerTorque& getControllerTorque();
  controller::TorqueControllerJointPositionVelocityPid& getControllerJointPosition();
  controller::TorqueControllerJointPositionVelocityPid& getControllerJointPositionVelocity();
  controller::TorqueControllerJointPositionVelocityTorque& getControllerJointPositionVelocityTorque();
  controller::TorqueControllerJointPositionVelocityTorquePidGains& getControllerJointPositionVelocityTorquePidGains();

 protected:
  controller::TorqueControllerDisable controllerDisable_;
  controller::TorqueControllerFreeze controllerFreeze_;
  controller::TorqueControllerTorque controllerTorque_;
  controller::TorqueControllerJointPositionVelocityPid controllerJointPosition_;
  controller::TorqueControllerJointPositionVelocityPid controllerJointPositionVelocity_;
  controller::TorqueControllerJointPositionVelocityTorque controllerJointPositionVelocityTorque_;
  controller::TorqueControllerJointPositionVelocityTorquePidGains controllerJointPositionVelocityTorquePidGains_;
};


} // series_elastic_actuator_sim
