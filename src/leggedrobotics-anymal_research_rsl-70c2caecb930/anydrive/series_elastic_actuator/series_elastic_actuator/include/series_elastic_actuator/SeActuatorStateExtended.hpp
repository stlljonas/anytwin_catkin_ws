/*!
 * @file    SeActuatorStateExtended.hpp
 * @author  Remo Diethelm
 * @date    Jul 29, 2016
 * @version 0.0
 *
 */
#pragma once


// series elastic actuator
#include "series_elastic_actuator/SeActuatorState.hpp"


namespace series_elastic_actuator {


class SeActuatorStateExtended : public SeActuatorState
{
 public:
  SeActuatorStateExtended();
  virtual ~SeActuatorStateExtended();

  const double& getMotorPosition() const;
  void setMotorPosition(double motorPosition);

  const double& getMotorVelocity() const;
  void setMotorVelocity(double motorVelocity);

  const int32_t& getGearPositionTicks() const;
  void setGearPositionTicks(int32_t gearPositionTicks);

  const int32_t& getJointPositionTicks() const;
  void setJointPositionTicks(int32_t jointPositionTicks);

  const double& getTemperature() const;
  void setTemperature(double temperature);

  const double& getVoltage() const;
  void setVoltage(double voltage);

  const unsigned long int& getTimestamp() const;
  void setTimestamp(unsigned long int timestamp);

  const double& getDesiredCurrentD() const;
  void setDesiredCurrentD(double current);

  const double& getMeasuredCurrentD() const;
  void setMeasuredCurrentD(double current);

  const double& getDesiredCurrentQ() const;
  void setDesiredCurrentQ(double current);

  const double& getMeasuredCurrentQ() const;
  void setMeasuredCurrentQ(double current);

  const double& getMeasuredCurrentPhaseU() const;
  void setMeasuredCurrentPhaseU(double current);

  const double& getMeasuredCurrentPhaseV() const;
  void setMeasuredCurrentPhaseV(double current);

  const double& getMeasuredCurrentPhaseW() const;
  void setMeasuredCurrentPhaseW(double current);

  const double& getMeasuredVoltagePhaseU() const;
  void setMeasuredVoltagePhaseU(double voltage);

  const double& getMeasuredVoltagePhaseV() const;
  void setMeasuredVoltagePhaseV(double voltage);

  const double& getMeasuredVoltagePhaseW() const;
  void setMeasuredVoltagePhaseW(double voltage);

  friend std::ostream& operator<<(std::ostream& out, const SeActuatorStateExtended& state);

 protected:
  //! Motor position [rad]
  double motorPosition_ = 0.0;
  //! Motor velocity [rad/s]
  double motorVelocity_ = 0.0;
  //! Position of the spring on the gear side [ticks]
  int32_t gearPositionTicks_ = 0;
  //! Position of the spring on the joint side [ticks]
  int32_t jointPositionTicks_ = 0;
  //! Temperature of the actuator [°C]
  double temperature_ = 0.0;
  //! Voltage [V]
  double voltage_ = 0.0;

  unsigned int long timestamp_ = 0;
  double desiredCurrentD_ = 0.0;
  double measuredCurrentD_ = 0.0;
  double desiredCurrentQ_ = 0.0;
  double measuredCurrentQ_ = 0.0;
  double measuredCurrentPhaseU_ = 0.0;
  double measuredVoltagePhaseU_ = 0.0;
  double measuredCurrentPhaseV_ = 0.0;
  double measuredVoltagePhaseV_ = 0.0;
  double measuredCurrentPhaseW_ = 0.0;
  double measuredVoltagePhaseW_ = 0.0;

};


} // series_elastic_actuator
