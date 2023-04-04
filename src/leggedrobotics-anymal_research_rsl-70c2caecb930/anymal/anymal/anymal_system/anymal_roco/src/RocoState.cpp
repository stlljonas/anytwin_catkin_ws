/*!
 * @file     RocoState.cpp
 * @author   Christian Gehring, Dario Bellicoso
 * @date     Dec, 2014
 */

// roco
#include <anymal_roco/RocoState.hpp>

// message logger
#include <message_logger/message_logger.hpp>

// signal logger
#include <signal_logger/signal_logger.hpp>

// romo std
#include <romo_std/common/container_utils.hpp>

// stl
#include <iomanip>
#include <math.h>       /* isfinite  */

namespace anymal_roco {

using AD = anymal_description::AnymalDescription;

RocoState::RocoState(anymal_model::AnymalModel* anymalModel) :
    roco::StateInterface(),
    anymalModel_(anymalModel),
    desiredAnymalModel_(nullptr),
    joystick_(),
    desiredRobotTwist_(),
    desiredRobotPose_(),
    status_(StateStatus::STATUS_ERROR_UNKNOWN)
{
  romo_std::fillContainer<AD>(actuatorReadings_);
}

anymal_model::AnymalModel* const RocoState::getAnymalModelPtr() const {
  return anymalModel_;
}
const anymal_model::AnymalModel& RocoState::getAnymalModel() const {
  return *anymalModel_;
}

anymal_model::AnymalModel* RocoState::getDesiredAnymalModelPtr() const {
  return desiredAnymalModel_;
}
const anymal_model::AnymalModel& RocoState::getDesiredAnymalModel() const {
  return *desiredAnymalModel_;
}

void RocoState::setAnymalModelPtr(anymal_model::AnymalModel* anymalModel) {
  anymalModel_ = anymalModel;
}

void RocoState::setDesiredAnymalModelPtr(anymal_model::AnymalModel* anymalModel) {
  desiredAnymalModel_ = anymalModel;
}

bool RocoState::checkState() const {
  bool isOk = true;

  switch (status_) {
  	case StateStatus::STATUS_OK:
      break;
    case StateStatus::STATUS_ERROR_SENSOR:
      MELO_WARN("State includes a sensor error!");
      isOk = false;
      break;
    case StateStatus::STATUS_ERROR_ESTIMATOR:
      MELO_WARN("State includes an estimator error!");
      isOk = false;
      break;
    case StateStatus::STATUS_ERROR_UNKNOWN:
      MELO_WARN("State is unknown!");
      isOk = false;
      break;
    default:
      MELO_WARN("Status of state is undefined (%d)!", static_cast<int>(status_));
      isOk = false;
      break;
  }

  if (!std::isfinite(anymalModel_->getState().getPositionWorldToBaseInWorldFrame().x())) {
    MELO_WARN("positionWorldToBaseInWorldFrame_.x() is NaN or Inf!");
    isOk = false;
  }
  if (!std::isfinite(anymalModel_->getState().getPositionWorldToBaseInWorldFrame().y())) {
    MELO_WARN("positionWorldToBaseInWorldFrame_.y() is NaN or Inf!");
    isOk = false;
  }
  if (!std::isfinite(anymalModel_->getState().getPositionWorldToBaseInWorldFrame().z())) {
    MELO_WARN("positionWorldToBaseInWorldFrame_.z() is NaN or Inf!");
    isOk = false;
  }
  auto orientationBaseToWorldCopy = anymalModel_->getState().getOrientationBaseToWorld();
  orientationBaseToWorldCopy.fix();
  if (!std::isfinite(orientationBaseToWorldCopy.x())) {
    MELO_WARN("orientationBaseToWorld_.x() is NaN or Inf!");
    isOk = false;
  }
  if (!std::isfinite(orientationBaseToWorldCopy.y())) {
    MELO_WARN("orientationBaseToWorld_.y() is NaN or Inf!");
    isOk = false;
  }
  if (!std::isfinite(orientationBaseToWorldCopy.z())) {
    MELO_WARN("orientationBaseToWorld_.z() is NaN or Inf!");
    isOk = false;
  }
  if (!std::isfinite(orientationBaseToWorldCopy.w())) {
    MELO_WARN("orientationBaseToWorld_.w() is NaN or Inf!");
    isOk = false;
  }

  if (!std::isfinite(anymalModel_->getState().getLinearVelocityBaseInWorldFrame().x())) {
    MELO_WARN("linearVelocityBaseInWorldFrame_.x() is NaN or Inf!");
    isOk = false;
  }
  if (!std::isfinite(anymalModel_->getState().getLinearVelocityBaseInWorldFrame().y())) {
    MELO_WARN("linearVelocityBaseInWorldFrame_.y() is NaN or Inf!");
    isOk = false;
  }
  if (!std::isfinite(anymalModel_->getState().getLinearVelocityBaseInWorldFrame().z())) {
    MELO_WARN("linearVelocityBaseInWorldFrame_.z() is NaN or Inf!");
    isOk = false;
  }
  constexpr double linVelTol = 20.0;
  if (anymalModel_->getState().getLinearVelocityBaseInWorldFrame().norm() > linVelTol) {
    MELO_WARN("||linearVelocityBaseInWorldFrame_|| is larger than %lf!", linVelTol);
    isOk = false;
  }



  if (!std::isfinite(anymalModel_->getState().getAngularVelocityBaseInBaseFrame().x())) {
    MELO_WARN("angularVelocityBaseInBaseFrame_.x() is NaN or Inf!");
    isOk = false;
  }
  if (!std::isfinite(anymalModel_->getState().getAngularVelocityBaseInBaseFrame().y())) {
    MELO_WARN("angularVelocityBaseInBaseFrame_.y() is NaN or Inf!");
    isOk = false;
  }
  if (!std::isfinite(anymalModel_->getState().getAngularVelocityBaseInBaseFrame().z())) {
    MELO_WARN("angularVelocityBaseInBaseFrame_.z() is NaN or Inf!");
    isOk = false;
  }
  constexpr double angVelTol = 4.0;
  if (anymalModel_->getState().getAngularVelocityBaseInBaseFrame().norm() > angVelTol) {
    MELO_WARN("||angularVelocityBaseInBaseFrame_|| is larger than %lf!", angVelTol);
    isOk = false;
  }

  constexpr double jointPosTol = 3.0*M_PI;
  for (int i=0; i<anymalModel_->getState().getJointPositions().Dimension; ++i) {
    if (!std::isfinite(anymalModel_->getState().getJointPositions()(i))) {
      MELO_WARN("jointPositions_(%d) is NaN or Inf!", i);
      isOk = false;
    }
    if (std::abs(anymalModel_->getState().getJointPositions()(i)) > jointPosTol) {
      MELO_WARN("jointPositions_(%d) is larger than %lf!", i, jointPosTol);
      isOk = false;
    }
  }
  constexpr double jointVelTol = 100.0; // [rad/s]
  for (int i=0; i<anymalModel_->getState().getJointVelocities().Dimension; ++i) {
    if (!std::isfinite(anymalModel_->getState().getJointVelocities()(i))) {
      MELO_WARN("jointVelocities_(%d) is NaN or Inf!", i);
      isOk = false;
    }
    if (std::abs(anymalModel_->getState().getJointVelocities()(i)) > jointVelTol) {
      MELO_WARN("jointVelocities_(%d) is larger than %lf!", i, jointVelTol);
      isOk = false;
    }
  }
  return isOk;
}

RocoState::StateStatus RocoState::getStatus() const {
  return status_;
}

void RocoState::setStatus(StateStatus status) {
  status_ = status;
}

robot_utils::Joystick* RocoState::getJoystickPtr() const {
  return const_cast<robot_utils::Joystick*>(&joystick_);
}

kindr::TwistLocalD* RocoState::getDesiredRobotVelocityPtr() const {
  return const_cast<kindr::TwistLocalD*>(&desiredRobotTwist_);
}

kindr::HomTransformQuatD* RocoState::getDesiredRobotPosePtr() const {
  return const_cast<kindr::HomTransformQuatD*>(&desiredRobotPose_);
}

void RocoState::setImu(const any_measurements::Imu& imu) {
  imu_ = imu;
}

const any_measurements::Imu& RocoState::getImu() const {
 return imu_;
}

const anymal_model::ActuatorReadingRobotContainer& RocoState::getActuatorReadings() const {
  return actuatorReadings_;
}

anymal_model::ActuatorReadingRobotContainer& RocoState::getActuatorReadings() {
  return actuatorReadings_;
}

anymal_model::ContactForceCalibratorStatsContainer& RocoState::getForceCalibratorStats() {
  return forceCalibratorStats_;
}

void RocoState::setLocalization(const Localization& localization) {
  localization_ = localization;
}

const RocoState::Localization& RocoState::getLocalization() const {
  return localization_;
}

void RocoState::addVariablesToLog(bool updateLogger) {
  std::string ns{"/state/"};
  signal_logger::add(desiredRobotPose_, std::string{"desRobotPose"}, ns);
  signal_logger::add(desiredRobotTwist_, std::string{"desRobotTwist"}, ns);
  signal_logger::add(getImu().angularVelocity_,"angVelImuInImuFrame", ns+std::string{"/imu/"});
  signal_logger::add(getImu().linearAcceleration_,"linAccImuInImuFrame",ns+std::string{"/imu/"});
  signal_logger::add(status_, std::string{"status"}, ns);

  for (const auto& actuatorKey : AD::getActuatorKeys()) {
    const auto& reading = actuatorReadings_[actuatorKey.getEnum()];
    const auto& name = actuatorKey.getName();
    signal_logger::add(reading.getState().getJointPosition(), std::string{"jointPos_"} + name, ns);
    signal_logger::add(reading.getState().getJointVelocity(), std::string{"jointVel_"} + name, ns);
    signal_logger::add(reading.getState().getJointAcceleration(), std::string{"jointAcc_"} + name, ns);
    signal_logger::add(reading.getState().getJointTorque(), std::string{"jointTor_"} + name, ns);
    signal_logger::add(reading.getState().getCurrent(), std::string{"current_"} + name, ns);
  }

  ns += std::string{"/force_calib/"};
  std::string names[] {"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};
  int i=0;
  for (auto& stat: forceCalibratorStats_) {
    signal_logger::add(stat.numSamples_, std::string{"numSamples_"} + names[i], ns);
    signal_logger::add(stat.numGoodSamples_, std::string{"numGoodSamples_"} + names[i], ns);
    ++i;
  }
}

std::ostream& operator<<(std::ostream& out, const RocoState& state) {
   return out;
}

} /* namespace starlethModel */
