/*!
 * @file     State.hpp
 * @author   Christian Gehring, Dario Bellicoso
 * @date     Dec, 2014
 */

#pragma once

// any measurements
#include <any_measurements/Imu.hpp>
#include <any_measurements/PoseWithCovariance.hpp>

// eigen
#include <Eigen/Core>

// kindr
#include <kindr/Core>

// stl
#include <ostream>

// anymal model
#include <anymal_model/AnymalModel.hpp>
#include <anymal_model/AnymalContainers.hpp>
#include <anymal_model/actuator_containers.hpp>

// robot utils
#include <robot_utils/force_calibrators/ForceCalibratorStats.hpp>
#include <robot_utils/sensors/Joystick.hpp>

// roco
#include <roco/model/StateInterface.hpp>

namespace anymal_roco {

class RocoState: public roco::StateInterface
{
 private:
  using AD = anymal_description::AnymalDescription;

 public:
  using StateStatus = anymal_model::StateStatus;

  using JointPositions = anymal_model::JointPositions;
  using JointVelocities = anymal_model::JointVelocities;
  using JointTorques = anymal_model::JointTorques;

  using GeneralizedCoordinates = anymal_model::GeneralizedCoordinates;
  using GeneralizedVelocities = anymal_model::GeneralizedVelocities;
  using GeneralizedAccelerations = anymal_model::GeneralizedAccelerations;

  using EulerAnglesZyx = anymal_model::EulerAnglesZyx;
  using RotationQuaternion = anymal_model::RotationQuaternion;
  using RotationMatrix = anymal_model::RotationMatrix;
  using AngleAxis = anymal_model::AngleAxis;

  using Position = anymal_model::Position;
  using LinearVelocity = anymal_model::LinearVelocity;
  using LocalAngularVelocity = anymal_model::LocalAngularVelocity;

  //! The localization contains the map's pose expressed in the world frame.
  using Localization = any_measurements::PoseWithCovariance;

 public:
  RocoState(anymal_model::AnymalModel* anymalModel = nullptr);
  virtual ~RocoState() = default;

  anymal_model::AnymalModel* const getAnymalModelPtr() const;
  anymal_model::AnymalModel* getDesiredAnymalModelPtr() const;

  void setAnymalModelPtr(anymal_model::AnymalModel* anymalModel);
  void setDesiredAnymalModelPtr(anymal_model::AnymalModel* anymalModel);

  const anymal_model::AnymalModel& getAnymalModel() const;
  const anymal_model::AnymalModel& getDesiredAnymalModel() const;

  void setImu(const any_measurements::Imu& imu);
  const any_measurements::Imu& getImu() const;
  StateStatus getStatus() const;
  void setStatus(StateStatus status);

  bool checkState() const;
  friend std::ostream& operator<<(std::ostream& out, const RocoState& state);

  robot_utils::Joystick* getJoystickPtr() const;
  kindr::TwistLocalD* getDesiredRobotVelocityPtr() const;
  kindr::HomTransformQuatD* getDesiredRobotPosePtr() const;

  const anymal_model::ActuatorReadingRobotContainer& getActuatorReadings() const;
  anymal_model::ActuatorReadingRobotContainer& getActuatorReadings();

  anymal_model::ContactForceCalibratorStatsContainer& getForceCalibratorStats();

  void setLocalization(const Localization& localization);
  const Localization& getLocalization() const;

  void addVariablesToLog(bool updateLogger);
 protected:
  anymal_model::AnymalModel* anymalModel_;
  anymal_model::AnymalModel* desiredAnymalModel_;

  any_measurements::Imu imu_;
  robot_utils::Joystick joystick_;
  kindr::TwistLocalD desiredRobotTwist_;
  kindr::HomTransformQuatD desiredRobotPose_;
  StateStatus status_;
  anymal_model::ActuatorReadingRobotContainer actuatorReadings_;
  anymal_model::ContactForceCalibratorStatsContainer forceCalibratorStats_;
  Localization localization_;
};

}
