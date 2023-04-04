/*!
 * @file    SeActuatorCommand.hpp
 * @author  Christian Gehring
 * @date    Mar 18, 2015
 * @version 0.0
 *
 */
#pragma once


// c++
#include <limits>
#include <ostream>
#include <string>

// any measurements
#include <any_measurements/Time.hpp>


namespace series_elastic_actuator {


class SeActuatorCommand
{
public:
  typedef int16_t SeActuatorModeType;
  enum SeActuatorMode : SeActuatorModeType {
    MODE_NA                                       = 0,  // Not available
    MODE_FREEZE                                   = 1,  // Freeze motor
    MODE_DISABLE                                  = 2,  // Disable motor
    MODE_CURRENT                                  = 3,  // Track current
    MODE_MOTOR_POSITION                           = 4,  // Track motor position
    MODE_MOTOR_VELOCITY                           = 5,  // Track motor velocity
    MODE_GEAR_POSITION                            = 6,  // Track gear position
    MODE_GEAR_VELOCITY                            = 7,  // Track gear velocity
    MODE_JOINT_POSITION                           = 8,  // Track joint position
    MODE_JOINT_VELOCITY                           = 9,  // Track joint velocity
    MODE_JOINT_TORQUE                             = 10, // Track joint torque
    MODE_JOINT_POSITION_VELOCITY                  = 11, // Track joint position with feedforward velocity
    MODE_JOINT_POSITION_VELOCITY_TORQUE           = 12, // Track joint position with feedforward velocity and torque
    MODE_JOINT_POSITION_VELOCITY_TORQUE_PID_GAINS = 13, // Track joint position with feedforward velocity and torque using custom joint position gains
  };
  typedef SeActuatorMode Mode;

  struct PidGains {
    double pGain_ = 0.0;
    double iGain_ = 0.0;
    double dGain_ = 0.0;
  };

public:
  SeActuatorCommand();
  SeActuatorCommand(SeActuatorModeType mode);
  virtual ~SeActuatorCommand();

  const any_measurements::Time& getStamp() const;
  void setStamp(const any_measurements::Time& stamp);

  static std::string getModeName(SeActuatorModeType mode);
  std::string getModeName() const;

  const SeActuatorModeType& getMode() const;
  void setMode(const SeActuatorModeType& mode);

  SeActuatorMode getModeEnum() const;
  void setModeEnum(const SeActuatorMode& modeEnum);

  const double& getCurrent() const;
  void setCurrent(double current);

  const double& getMotorPosition() const;
  void setMotorPosition(double motorPosition);

  const double& getMotorVelocity() const;
  void setMotorVelocity(double motorVelocity);

  const double& getGearPosition() const;
  void setGearPosition(double gearPosition);

  const double& getGearVelocity() const;
  void setGearVelocity(double gearVelocity);

  const double& getJointPosition() const;
  void setJointPosition(double jointPosition);

  const double& getJointVelocity() const;
  void setJointVelocity(double jointVelocity);

  const double& getJointTorque() const;
  void setJointTorque(double jointTorque);

  const double& getPidGainsP() const;
  void setPidGainsP(double pidGainsP);

  const double& getPidGainsI() const;
  void setPidGainsI(double pidGainsI);

  const double& getPidGainsD() const;
  void setPidGainsD(double pidGainsD);

  void setPidGains(PidGains& gains);

  const double& getCurrentMin() const;
  void setCurrentMin(double currentMin);
  const double& getCurrentMax() const;
  void setCurrentMax(double currentMax);

  const double& getMotorPositionMin() const;
  void setMotorPositionMin(double motorPositionMin);
  const double& getMotorPositionMax() const;
  void setMotorPositionMax(double motorPositionMax);

  const double& getMotorVelocityMin() const;
  void setMotorVelocityMin(double motorVelocityMin);
  const double& getMotorVelocityMax() const;
  void setMotorVelocityMax(double motorVelocityMax);

  const double& getGearPositionMin() const;
  void setGearPositionMin(double gearPositionMin);
  const double& getGearPositionMax() const;
  void setGearPositionMax(double gearPositionMax);

  const double& getGearVelocityMin() const;
  void setGearVelocityMin(double gearVelocityMin);
  const double& getGearVelocityMax() const;
  void setGearVelocityMax(double gearVelocityMax);

  const double& getJointPositionMin() const;
  void setJointPositionMin(double jointPositionMin);
  const double& getJointPositionMax() const;
  void setJointPositionMax(double jointPositionMax);

  const double& getJointVelocityMin() const;
  void setJointVelocityMin(double jointVelocityMin);
  const double& getJointVelocityMax() const;
  void setJointVelocityMax(double jointVelocityMax);

  const double& getJointTorqueMin() const;
  void setJointTorqueMin(double jointTorqueMin);
  const double& getJointTorqueMax() const;
  void setJointTorqueMax(double jointTorqueMax);


  //! Limits all values.
  void limit();

  //! @returns true if all values are finite.
  bool isFinite() const;
  //! @returns true if all values within the limits.
  bool isWithinLimits() const;
  //! @returns true if all values are finite and within the limits.
  bool isValid() const;

  friend std::ostream& operator<<(std::ostream& out, const SeActuatorCommand& command);

protected:
  any_measurements::Time stamp_;

  SeActuatorModeType mode_ = SeActuatorMode::MODE_FREEZE;

  double current_ = 0.0;
  double motorPosition_ = 0.0;
  double motorVelocity_ = 0.0;
  double gearPosition_ = 0.0;
  double gearVelocity_ = 0.0;
  double jointPosition_ = 0.0;
  double jointVelocity_ = 0.0;
  double jointTorque_ = 0.0;
  double pidGainsP_ = 0.0;
  double pidGainsI_ = 0.0;
  double pidGainsD_ = 0.0;

  double currentMin_ = -std::numeric_limits<double>::max();
  double currentMax_ = std::numeric_limits<double>::max();
  double motorPositionMin_ = -std::numeric_limits<double>::max();
  double motorPositionMax_ = std::numeric_limits<double>::max();
  double motorVelocityMin_ = -std::numeric_limits<double>::max();
  double motorVelocityMax_ = std::numeric_limits<double>::max();
  double gearPositionMin_ = -std::numeric_limits<double>::max();
  double gearPositionMax_ = std::numeric_limits<double>::max();
  double gearVelocityMin_ = -std::numeric_limits<double>::max();
  double gearVelocityMax_ = std::numeric_limits<double>::max();
  double jointPositionMin_ = -std::numeric_limits<double>::max();
  double jointPositionMax_ = std::numeric_limits<double>::max();
  double jointVelocityMin_ = -std::numeric_limits<double>::max();
  double jointVelocityMax_ = std::numeric_limits<double>::max();
  double jointTorqueMin_ = -std::numeric_limits<double>::max();
  double jointTorqueMax_ = std::numeric_limits<double>::max();
};


} // series_elastic_actuator
