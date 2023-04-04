#pragma once

#include "anydrive/State.hpp"

namespace anydrive {

//! Extended state of the ANYdrive, containing additional information to the state.
class StateExtended : public State {
 protected:
  //! Motor position [rad].
  double motorPosition_ = 0.0;
  //! Motor velocity [rad/s].
  double motorVelocity_ = 0.0;
  //! Position of the spring on the gear side [ticks].
  int32_t gearPositionTicks_ = 0;
  //! Position of the spring on the joint side [ticks].
  int32_t jointPositionTicks_ = 0;
  //! Temperature of the actuator [°C].
  double temperature_ = 0.0;
  //! Measured input voltage [V].
  double voltage_ = 0.0;
  //! Time on the ANYdrive [1/(25e6s)].
  uint64_t timestamp_ = 0;
  //! D part of the desired current [A].
  double desiredCurrentD_ = 0.0;
  //! D part of the measured current [A].
  double measuredCurrentD_ = 0.0;
  //! Q part of the desired current [A].
  double desiredCurrentQ_ = 0.0;
  //! Q part of the measured current [A].
  double measuredCurrentQ_ = 0.0;
  //! Alpha from Inverse Park Transform.
  double alpha_ = 0.0;
  //! Beta from Inverse Park Transform.
  double beta_ = 0.0;
  //! U phase of the measured current [A].
  double measuredCurrentPhaseU_ = 0.0;
  //! U phase of the measured voltage [V].
  double measuredVoltagePhaseU_ = 0.0;
  //! V phase of the measured current [A].
  double measuredCurrentPhaseV_ = 0.0;
  //! V phase of the measured voltage [V].
  double measuredVoltagePhaseV_ = 0.0;
  //! W phase of the measured current [A].
  double measuredCurrentPhaseW_ = 0.0;
  //! W phase of the measured voltage [V].
  double measuredVoltagePhaseW_ = 0.0;
  //! U duty cycle.
  double dutyCycleU_ = 0.0;
  //! V duty cycle.
  double dutyCycleV_ = 0.0;
  //! W duty cycle.
  double dutyCycleW_ = 0.0;

 public:
  StateExtended();
  ~StateExtended() override = default;

  double getMotorPosition() const;
  void setMotorPosition(const double motorPosition);

  double getMotorVelocity() const;
  void setMotorVelocity(const double motorVelocity);

  int32_t getGearPositionTicks() const;
  void setGearPositionTicks(const int32_t gearPositionTicks);

  int32_t getJointPositionTicks() const;
  void setJointPositionTicks(const int32_t jointPositionTicks);

  double getTemperature() const;
  void setTemperature(const double temperature);

  double getVoltage() const;
  void setVoltage(const double voltage);

  uint64_t getTimestamp() const;
  void setTimestamp(const uint64_t timestamp);

  double getDesiredCurrentD() const;
  void setDesiredCurrentD(const double current);

  double getMeasuredCurrentD() const;
  void setMeasuredCurrentD(const double current);

  double getDesiredCurrentQ() const;
  void setDesiredCurrentQ(const double current);

  double getMeasuredCurrentQ() const;
  void setMeasuredCurrentQ(const double current);

  double getAlpha() const;
  void setAlpha(const double alpha);

  double getBeta() const;
  void setBeta(const double beta);

  double getMeasuredCurrentPhaseU() const;
  void setMeasuredCurrentPhaseU(const double current);

  double getMeasuredCurrentPhaseV() const;
  void setMeasuredCurrentPhaseV(const double current);

  double getMeasuredCurrentPhaseW() const;
  void setMeasuredCurrentPhaseW(const double current);

  double getMeasuredVoltagePhaseU() const;
  void setMeasuredVoltagePhaseU(const double voltage);

  double getMeasuredVoltagePhaseV() const;
  void setMeasuredVoltagePhaseV(const double voltage);

  double getMeasuredVoltagePhaseW() const;
  void setMeasuredVoltagePhaseW(const double voltage);

  double getDutyCycleU() const;
  void setDutyCycleU(const double dutyCycle);

  double getDutyCycleV() const;
  void setDutyCycleV(const double dutyCycle);

  double getDutyCycleW() const;
  void setDutyCycleW(const double dutyCycle);

  std::string asString(const std::string& prefix) const override;
};

std::ostream& operator<<(std::ostream& out, const StateExtended& stateExtended);

}  // namespace anydrive
