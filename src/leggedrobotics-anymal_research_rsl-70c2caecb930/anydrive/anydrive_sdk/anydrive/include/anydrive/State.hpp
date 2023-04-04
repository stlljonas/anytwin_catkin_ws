#pragma once

#include <ostream>

#include <any_measurements/Imu.hpp>
#include <any_measurements/Time.hpp>

#include "anydrive/Statusword.hpp"

namespace anydrive {

//! State of the ANYdrive.
class State {
 protected:
  //! Time when the state was read out.
  any_measurements::Time stamp_;
  //! Statusword.
  Statusword statusword_;
  //! Current [A].
  double current_ = 0.0;
  //! Gear position [rad].
  double gearPosition_ = 0.0;
  //! Gear velocity [rad/s].
  double gearVelocity_ = 0.0;
  //! Joint position [rad].
  double jointPosition_ = 0.0;
  //! Joint velocity [rad/s].
  double jointVelocity_ = 0.0;
  //! Joint velocity [rad/s²].
  double jointAcceleration_ = 0.0;
  //! Joint torque [Nm].
  double jointTorque_ = 0.0;
  //! IMU.
  any_measurements::Imu imu_;

 public:
  State() = default;
  virtual ~State() = default;

  const any_measurements::Time& getStamp() const;
  void setStamp(const any_measurements::Time& stamp);

  const Statusword& getStatusword() const;
  void setStatusword(const Statusword& statusword);

  double getCurrent() const;
  void setCurrent(const double current);

  double getGearPosition() const;
  void setGearPosition(const double gearPosition);

  double getGearVelocity() const;
  void setGearVelocity(const double gearVelocity);

  double getJointPosition() const;
  void setJointPosition(const double jointPosition);

  double getJointVelocity() const;
  void setJointVelocity(const double jointVelocity);

  double getJointAcceleration() const;
  void setJointAcceleration(const double jointAcceleration);

  double getJointTorque() const;
  void setJointTorque(const double jointTorque);

  const any_measurements::Imu& getImu() const;
  void setImu(const any_measurements::Imu& imu);

  /*!
   * Check if the state is valid:
   * - Stamp is non-zero
   * - Statusword is valid
   * - None of the values is Inf or NaN
   * @return True if valid.
   */
  bool isValid() const;

  virtual std::string asString(const std::string& prefix) const;
};

std::ostream& operator<<(std::ostream& out, const State& state);

}  // namespace anydrive
