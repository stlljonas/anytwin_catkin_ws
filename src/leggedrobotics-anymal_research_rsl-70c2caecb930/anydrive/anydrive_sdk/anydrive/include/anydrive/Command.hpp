#pragma once

#include <ostream>

#include <any_measurements/Time.hpp>

#include "anydrive/mode/ModeEnum.hpp"
#include "anydrive/mode/PidGains.hpp"

namespace anydrive {

//! ANYdrive command.
class Command {
 protected:
  //! Stamp.
  any_measurements::Time stamp_;
  //! Mode.
  mode::ModeEnum modeEnum_ = mode::ModeEnum::NA;
  //! Current [A].
  double current_ = 0.0;
  //! Motor position [rad].
  double motorPosition_ = 0.0;
  //! Motor velocity [rad/s].
  double motorVelocity_ = 0.0;
  //! Gear position [rad].
  double gearPosition_ = 0.0;
  //! Gear velocity [rad/s].
  double gearVelocity_ = 0.0;
  //! Joint position [rad].
  double jointPosition_ = 0.0;
  //! Joint velocity [rad/s].
  double jointVelocity_ = 0.0;
  //! Joint torque [Nm].
  double jointTorque_ = 0.0;
  //! PID gains.
  mode::PidGainsF pidGains_;

 public:
  Command() = default;
  virtual ~Command() = default;

  const any_measurements::Time& getStamp() const;
  void setStamp(const any_measurements::Time& stamp);

  mode::ModeEnum getModeEnum() const;
  void setModeEnum(const mode::ModeEnum modeEnum);

  double getCurrent() const;
  void setCurrent(const double current);

  double getMotorPosition() const;
  void setMotorPosition(const double motorPosition);

  double getMotorVelocity() const;
  void setMotorVelocity(const double motorVelocity);

  double getGearPosition() const;
  void setGearPosition(const double gearPosition);

  double getGearVelocity() const;
  void setGearVelocity(const double gearVelocity);

  double getJointPosition() const;
  void setJointPosition(const double jointPosition);

  double getJointVelocity() const;
  void setJointVelocity(const double jointVelocity);

  double getJointTorque() const;
  void setJointTorque(const double jointTorque);

  mode::PidGainsF& getPidGains();
  const mode::PidGainsF& getPidGains() const;
  void setPidGains(const mode::PidGainsF& pidGains);

  /*!
   * Check if the command is valid:
   * - Stamp is non-zero
   * - Mode is set
   * - None of the values is Inf or NaN
   * @return True if valid.
   */
  bool isValid() const;

  virtual std::string asString(const std::string& prefix) const;
};

std::ostream& operator<<(std::ostream& out, const Command& command);

}  // namespace anydrive
