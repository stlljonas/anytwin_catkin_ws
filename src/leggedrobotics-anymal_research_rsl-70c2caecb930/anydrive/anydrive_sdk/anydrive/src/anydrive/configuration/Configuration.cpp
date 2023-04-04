#include "anydrive/configuration/Configuration.hpp"
#include "anydrive/mode/ModeCurrent.hpp"
#include "anydrive/mode/ModeDemoChangeGravity.hpp"
#include "anydrive/mode/ModeDemoSafeJointVelocity.hpp"
#include "anydrive/mode/ModeDisable.hpp"
#include "anydrive/mode/ModeFreeze.hpp"
#include "anydrive/mode/ModeGearPosition.hpp"
#include "anydrive/mode/ModeGearVelocity.hpp"
#include "anydrive/mode/ModeJointPosition.hpp"
#include "anydrive/mode/ModeJointPositionVelocity.hpp"
#include "anydrive/mode/ModeJointPositionVelocityTorque.hpp"
#include "anydrive/mode/ModeJointPositionVelocityTorquePidGains.hpp"
#include "anydrive/mode/ModeJointTorque.hpp"
#include "anydrive/mode/ModeJointVelocity.hpp"
#include "anydrive/mode/ModeMotorPosition.hpp"
#include "anydrive/mode/ModeMotorVelocity.hpp"

namespace anydrive {
namespace configuration {

Configuration::Configuration()
    : errorStateBehavior_(0),
      maxCurrent_(28.0),
      maxMotorVelocity_(680.7),
      direction_(1),
      imuEnable_(true),
      imuAccelerometerRange_(1),
      imuGyroscopeRange_(1) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  addMode(mode::ModeBasePtr(new mode::ModeCurrent()));
  addMode(mode::ModeBasePtr(new mode::ModeDemoChangeGravity()));
  addMode(mode::ModeBasePtr(new mode::ModeDemoSafeJointVelocity()));
  addMode(mode::ModeBasePtr(new mode::ModeDisable()));
  addMode(mode::ModeBasePtr(new mode::ModeFreeze()));
  addMode(mode::ModeBasePtr(new mode::ModeGearPosition()));
  addMode(mode::ModeBasePtr(new mode::ModeGearVelocity()));
  addMode(mode::ModeBasePtr(new mode::ModeJointPosition()));
  addMode(mode::ModeBasePtr(new mode::ModeJointPositionVelocity()));
  addMode(mode::ModeBasePtr(new mode::ModeJointPositionVelocityTorque()));
  addMode(mode::ModeBasePtr(new mode::ModeJointPositionVelocityTorquePidGains()));
  addMode(mode::ModeBasePtr(new mode::ModeJointTorque()));
  addMode(mode::ModeBasePtr(new mode::ModeJointVelocity()));
  addMode(mode::ModeBasePtr(new mode::ModeMotorPosition()));
  addMode(mode::ModeBasePtr(new mode::ModeMotorVelocity()));
}

Configuration::Configuration(const Configuration& other) {
  *this = other;
}

Configuration& Configuration::operator=(const Configuration& other) {
  maxCommandAge_ = other.getMaxCommandAge();
  autoStageLastCommand_ = other.getAutoStageLastCommand();
  setReadingToNanOnDisconnect_ = other.getSetReadingToNanOnDisconnect();
  errorStateBehavior_ = other.getErrorStateBehavior();

  maxCurrent_ = other.getMaxCurrent();
  maxFreezeCurrent_ = other.getMaxFreezeCurrent();
  maxMotorVelocity_ = other.getMaxMotorVelocity();
  maxJointTorque_ = other.getMaxJointTorque();
  currentIntegratorSaturation_ = other.getCurrentIntegratorSaturation();
  jointTorqueIntegratorSaturation_ = other.getJointTorqueIntegratorSaturation();

  direction_ = other.getDirection();

  jointPositionLimitsSdk_ = other.getJointPositionLimitsSdk();
  jointPositionLimitsSoft_ = other.getJointPositionLimitsSoft();
  jointPositionLimitsHard_ = other.getJointPositionLimitsHard();

  for (const auto& jointPositionConfiguration : other.getJointPositionConfigurations()) {
    addJointPositionConfiguration(jointPositionConfiguration.first, jointPositionConfiguration.second);
  }

  imuEnable_ = other.getImuEnable();
  imuAccelerometerRange_ = other.getImuAccelerometerRange();
  imuGyroscopeRange_ = other.getImuGyroscopeRange();

  fanMode_ = other.getFanMode();
  fanIntensity_ = other.getFanIntensity();
  fanLowerTemperature_ = other.getFanLowerTemperature();
  fanUpperTemperature_ = other.getFanUpperTemperature();

  for (const auto& mode : other.getModes()) {
    if (mode.second->getPidGains().isSet()) {
      getMode(mode.first)->setPidGains(mode.second->getPidGains());
    }
  }

  goalStateEnumStartup_ = other.getGoalStateEnumStartup();
  goalStateEnumShutdown_ = other.getGoalStateEnumShutdown();

  gearJointVelocityFilterType_ = other.getGearJointVelocityFilterType();
  gearJointVelocityKfNoiseVariance_ = other.getGearJointVelocityKfNoiseVariance();
  gearJointVelocityKfLambda2_ = other.getGearJointVelocityKfLambda2();
  gearJointVelocityKfGamma_ = other.getGearJointVelocityKfGamma();
  gearJointVelocityEmaAlpha_ = other.getGearJointVelocityEmaAlpha();

  jointVelocityForAccelerationFilterType_ = other.getJointVelocityForAccelerationFilterType();
  jointVelocityForAccelerationKfNoiseVariance_ = other.getJointVelocityForAccelerationKfNoiseVariance();
  jointVelocityForAccelerationKfLambda2_ = other.getJointVelocityForAccelerationKfLambda2();
  jointVelocityForAccelerationKfGamma_ = other.getJointVelocityForAccelerationKfGamma();
  jointVelocityForAccelerationEmaAlpha_ = other.getJointVelocityForAccelerationEmaAlpha();

  jointAccelerationFilterType_ = other.getJointAccelerationFilterType();
  jointAccelerationKfNoiseVariance_ = other.getJointAccelerationKfNoiseVariance();
  jointAccelerationKfLambda2_ = other.getJointAccelerationKfLambda2();
  jointAccelerationKfGamma_ = other.getJointAccelerationKfGamma();
  jointAccelerationEmaAlpha_ = other.getJointAccelerationEmaAlpha();

  return *this;
}

void Configuration::fromFile(const std::string& path) {
  yaml_tools::YamlNode yamlNode = yaml_tools::YamlNode::fromFile(path);
  fromYamlNode(yamlNode);
}

void Configuration::fromYamlNode(const yaml_tools::YamlNode& yamlNode) {
  // Clear the configuration first.
  *this = Configuration();

  if (yamlNode.hasKey("max_command_age")) {
    setMaxCommandAge(yamlNode["max_command_age"].as<double>());
  }
  if (yamlNode.hasKey("auto_stage_last_command")) {
    setAutoStageLastCommand(yamlNode["auto_stage_last_command"].as<bool>());
  }
  if (yamlNode.hasKey("set_reading_to_nan_on_disconnect")) {
    setSetReadingToNanOnDisconnect(yamlNode["set_reading_to_nan_on_disconnect"].as<bool>());
  }
  if (yamlNode.hasKey("error_state_behavior")) {
    setErrorStateBehavior(yamlNode["error_state_behavior"].as<uint16_t>());
  }
  if (yamlNode.hasKey("max_current")) {
    setMaxCurrent(yamlNode["max_current"].as<double>());
  }
  if (yamlNode.hasKey("max_freeze_current")) {
    setMaxFreezeCurrent(yamlNode["max_freeze_current"].as<double>());
  }
  if (yamlNode.hasKey("max_motor_velocity")) {
    setMaxMotorVelocity(yamlNode["max_motor_velocity"].as<double>());
  }
  if (yamlNode.hasKey("max_joint_torque")) {
    setMaxJointTorque(yamlNode["max_joint_torque"].as<double>());
  }
  if (yamlNode.hasKey("current_integrator_saturation")) {
    setCurrentIntegratorSaturation(yamlNode["current_integrator_saturation"].as<double>());
  }
  if (yamlNode.hasKey("joint_torque_integrator_saturation")) {
    setJointTorqueIntegratorSaturation(yamlNode["joint_torque_integrator_saturation"].as<double>());
  }
  if (yamlNode.hasKey("direction")) {
    setDirection(yamlNode["direction"].as<int16_t>());
  }
  if (yamlNode.hasKey("joint_position_limits")) {
    if (yamlNode["joint_position_limits"].hasKey("sdk")) {
      setJointPositionLimitsSdk(common::Limits(yamlNode["joint_position_limits"]["sdk"]["min"].as<double>(),
                                               yamlNode["joint_position_limits"]["sdk"]["max"].as<double>()));
    }
    if (yamlNode["joint_position_limits"].hasKey("soft")) {
      setJointPositionLimitsSoft(common::Limits(yamlNode["joint_position_limits"]["soft"]["min"].as<double>(),
                                                yamlNode["joint_position_limits"]["soft"]["max"].as<double>()));
    }
    if (yamlNode["joint_position_limits"].hasKey("hard")) {
      setJointPositionLimitsHard(common::Limits(yamlNode["joint_position_limits"]["hard"]["min"].as<double>(),
                                                yamlNode["joint_position_limits"]["hard"]["max"].as<double>()));
    }
  }
  if (yamlNode.hasKey("joint_position_configurations")) {
    for (unsigned int i = 0; i < yamlNode["joint_position_configurations"].size(); i++) {
      addJointPositionConfiguration(yamlNode["joint_position_configurations"][i]["name"].as<std::string>(),
                                    yamlNode["joint_position_configurations"][i]["value"].as<double>());
    }
  }
  if (yamlNode.hasKey("imu")) {
    if (yamlNode["imu"].hasKey("enable")) {
      setImuEnable(yamlNode["imu"]["enable"].as<bool>());
    }
    if (yamlNode["imu"].hasKey("accelerometer_range")) {
      setImuAccelerometerRange(yamlNode["imu"]["accelerometer_range"].as<uint32_t>());
    }
    if (yamlNode["imu"].hasKey("gyroscope_range")) {
      setImuGyroscopeRange(yamlNode["imu"]["gyroscope_range"].as<uint32_t>());
    }
  }
  if (yamlNode.hasKey("fan")) {
    if (yamlNode["fan"].hasKey("mode")) {
      setFanMode(yamlNode["fan"]["mode"].as<uint32_t>());
    }
    if (yamlNode["fan"].hasKey("intensity")) {
      setFanIntensity(yamlNode["fan"]["intensity"].as<uint32_t>());
    }
    if (yamlNode["fan"].hasKey("lower_temperature")) {
      setFanLowerTemperature(yamlNode["fan"]["lower_temperature"].as<float>());
    }
    if (yamlNode["fan"].hasKey("upper_temperature")) {
      setFanUpperTemperature(yamlNode["fan"]["upper_temperature"].as<float>());
    }
  }
  if (yamlNode.hasKey("modes")) {
    for (unsigned int i = 0; i < yamlNode["modes"].size(); i++) {
      const std::string modeName = yamlNode["modes"][i]["name"].as<std::string>();
      mode::ModeBasePtr mode = getMode(mode::modeNameToEnum(modeName));
      if (!mode) {
        ANYDRIVE_LOGGED_ERROR("Mode name '" << modeName << "' does not exist, cannot set gains.");
        continue;
      }
      mode->setPidGains(mode::PidGainsF(yamlNode["modes"][i]["gains"]["p"].as<double>(), yamlNode["modes"][i]["gains"]["i"].as<double>(),
                                        yamlNode["modes"][i]["gains"]["d"].as<double>()));
    }
  }
  if (yamlNode.hasKey("goal_states")) {
    if (yamlNode["goal_states"].hasKey("startup")) {
      setGoalStateEnumStartup(fsm::stateNameToEnum(yamlNode["goal_states"]["startup"].as<std::string>()));
    }
    if (yamlNode["goal_states"].hasKey("shutdown")) {
      setGoalStateEnumShutdown(fsm::stateNameToEnum(yamlNode["goal_states"]["shutdown"].as<std::string>()));
    }
  }
  if (yamlNode.hasKey("gear_joint_velocity_filter")) {
    if (yamlNode["gear_joint_velocity_filter"].hasKey("type")) {
      setGearJointVelocityFilterType(yamlNode["gear_joint_velocity_filter"]["type"].as<uint32_t>());
    }
    if (yamlNode["gear_joint_velocity_filter"].hasKey("kf_noise_variance")) {
      setGearJointVelocityKfNoiseVariance(yamlNode["gear_joint_velocity_filter"]["kf_noise_variance"].as<double>());
    }
    if (yamlNode["gear_joint_velocity_filter"].hasKey("kf_lambda_2")) {
      setGearJointVelocityKfLambda2(yamlNode["gear_joint_velocity_filter"]["kf_lambda_2"].as<double>());
    }
    if (yamlNode["gear_joint_velocity_filter"].hasKey("kf_gamma")) {
      setGearJointVelocityKfGamma(yamlNode["gear_joint_velocity_filter"]["kf_gamma"].as<double>());
    }
    if (yamlNode["gear_joint_velocity_filter"].hasKey("ema_alpha")) {
      setGearJointVelocityEmaAlpha(yamlNode["gear_joint_velocity_filter"]["ema_alpha"].as<double>());
    }
  }
  if (yamlNode.hasKey("joint_velocity_filter_for_acceleration")) {
    if (yamlNode["joint_velocity_filter_for_acceleration"].hasKey("type")) {
      setJointVelocityForAccelerationFilterType(yamlNode["joint_velocity_filter_for_acceleration"]["type"].as<uint32_t>());
    }
    if (yamlNode["joint_velocity_filter_for_acceleration"].hasKey("kf_noise_variance")) {
      setJointVelocityForAccelerationKfNoiseVariance(yamlNode["joint_velocity_filter_for_acceleration"]["kf_noise_variance"].as<double>());
    }
    if (yamlNode["joint_velocity_filter_for_acceleration"].hasKey("kf_lambda_2")) {
      setJointVelocityForAccelerationKfLambda2(yamlNode["joint_velocity_filter_for_acceleration"]["kf_lambda_2"].as<double>());
    }
    if (yamlNode["joint_velocity_filter_for_acceleration"].hasKey("kf_gamma")) {
      setJointVelocityForAccelerationKfGamma(yamlNode["joint_velocity_filter_for_acceleration"]["kf_gamma"].as<double>());
    }
    if (yamlNode["joint_velocity_filter_for_acceleration"].hasKey("ema_alpha")) {
      setJointVelocityForAccelerationEmaAlpha(yamlNode["joint_velocity_filter_for_acceleration"]["ema_alpha"].as<double>());
    }
  }
  if (yamlNode.hasKey("joint_acceleration_filter")) {
    if (yamlNode["joint_acceleration_filter"].hasKey("type")) {
      setJointAccelerationFilterType(yamlNode["joint_acceleration_filter"]["type"].as<uint32_t>());
    }
    if (yamlNode["joint_acceleration_filter"].hasKey("kf_noise_variance")) {
      setJointAccelerationKfNoiseVariance(yamlNode["joint_acceleration_filter"]["kf_noise_variance"].as<double>());
    }
    if (yamlNode["joint_acceleration_filter"].hasKey("kf_lambda_2")) {
      setJointAccelerationKfLambda2(yamlNode["joint_acceleration_filter"]["kf_lambda_2"].as<double>());
    }
    if (yamlNode["joint_acceleration_filter"].hasKey("kf_gamma")) {
      setJointAccelerationKfGamma(yamlNode["joint_acceleration_filter"]["kf_gamma"].as<double>());
    }
    if (yamlNode["joint_acceleration_filter"].hasKey("ema_alpha")) {
      setJointAccelerationEmaAlpha(yamlNode["joint_acceleration_filter"]["ema_alpha"].as<double>());
    }
  }
}

void Configuration::setMaxCommandAge(const double maxCommandAge) {
  if (maxCommandAge < 0) {
    ANYDRIVE_ERROR("The max command age must be within [0.0, .inf) s.");
    return;
  }

  std::lock_guard<std::recursive_mutex> lock(mutex_);
  maxCommandAge_ = maxCommandAge;
}

double Configuration::getMaxCommandAge() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return maxCommandAge_;
}

void Configuration::setAutoStageLastCommand(const bool keepSendingLastCommand) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  autoStageLastCommand_ = keepSendingLastCommand;
}

bool Configuration::getAutoStageLastCommand() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return autoStageLastCommand_;
}

void Configuration::setSetReadingToNanOnDisconnect(const bool setReadingToNanOnDisconnect) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  setReadingToNanOnDisconnect_ = setReadingToNanOnDisconnect;
}

bool Configuration::getSetReadingToNanOnDisconnect() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return setReadingToNanOnDisconnect_;
}

void Configuration::setErrorStateBehavior(const uint16_t errorStateBehavior) {
  if (errorStateBehavior > 1) {
    ANYDRIVE_ERROR("The error state behavior must be 0 or 1.");
    return;
  }

  std::lock_guard<std::recursive_mutex> lock(mutex_);
  errorStateBehavior_ = errorStateBehavior;
}

common::Optional<uint16_t> Configuration::getErrorStateBehavior() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return errorStateBehavior_;
}

void Configuration::setMaxCurrent(const double maxCurrent) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  maxCurrent_ = maxCurrent;
}

common::Optional<double> Configuration::getMaxCurrent() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return maxCurrent_;
}

void Configuration::setMaxFreezeCurrent(const double current) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  maxFreezeCurrent_ = current;
}

common::Optional<double> Configuration::getMaxFreezeCurrent() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return maxFreezeCurrent_;
}

void Configuration::setMaxMotorVelocity(const double maxMotorVelocity) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  maxMotorVelocity_ = maxMotorVelocity;
}

common::Optional<double> Configuration::getMaxMotorVelocity() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return maxMotorVelocity_;
}

void Configuration::setMaxJointTorque(const double maxJointTorque) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  maxJointTorque_ = maxJointTorque;
}

common::Optional<double> Configuration::getMaxJointTorque() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return maxJointTorque_;
}

void Configuration::setCurrentIntegratorSaturation(const double saturation) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  currentIntegratorSaturation_ = saturation;
}

common::Optional<double> Configuration::getCurrentIntegratorSaturation() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return currentIntegratorSaturation_;
}

void Configuration::setJointTorqueIntegratorSaturation(const double saturation) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  jointTorqueIntegratorSaturation_ = saturation;
}

common::Optional<double> Configuration::getJointTorqueIntegratorSaturation() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return jointTorqueIntegratorSaturation_;
}

void Configuration::setDirection(const int16_t direction) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  direction_ = direction;
}

common::Optional<int16_t> Configuration::getDirection() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return direction_;
}

void Configuration::setJointPositionLimitsSdk(const common::Limits& limits) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  jointPositionLimitsSdk_ = limits;
}

common::Optional<common::Limits> Configuration::getJointPositionLimitsSdk() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return jointPositionLimitsSdk_;
}

void Configuration::setJointPositionLimitsSoft(const common::Limits& limits) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  jointPositionLimitsSoft_ = limits;
}

common::Optional<common::Limits> Configuration::getJointPositionLimitsSoft() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return jointPositionLimitsSoft_;
}

void Configuration::setJointPositionLimitsHard(const common::Limits& limits) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  jointPositionLimitsHard_ = limits;
}

common::Optional<common::Limits> Configuration::getJointPositionLimitsHard() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return jointPositionLimitsHard_;
}

void Configuration::addJointPositionConfiguration(const std::string& jointPositionConfigurationName,
                                                  double jointPositionConfigurationValue) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  jointPositionConfigurations_.insert({jointPositionConfigurationName, jointPositionConfigurationValue});
}

bool Configuration::getJointPositionConfigurationValue(const std::string& jointPositionConfigurationName,
                                                       double& jointPositionConfigurationValue) const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  auto it = jointPositionConfigurations_.find(jointPositionConfigurationName);
  if (it == jointPositionConfigurations_.end()) {
    return false;
  }
  jointPositionConfigurationValue = it->second;
  return true;
}

std::map<std::string, double> Configuration::getJointPositionConfigurations() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return jointPositionConfigurations_;
}

void Configuration::setImuEnable(const bool enable) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  imuEnable_ = enable;
}

common::Optional<bool> Configuration::getImuEnable() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return imuEnable_;
}

void Configuration::setImuAccelerometerRange(const uint32_t range) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  imuAccelerometerRange_ = range;
}

common::Optional<uint32_t> Configuration::getImuAccelerometerRange() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return imuAccelerometerRange_;
}

void Configuration::setImuGyroscopeRange(const uint32_t range) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  imuGyroscopeRange_ = range;
}

common::Optional<uint32_t> Configuration::getImuGyroscopeRange() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return imuGyroscopeRange_;
}

void Configuration::setFanMode(const uint32_t mode) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  fanMode_ = mode;
}

common::Optional<uint32_t> Configuration::getFanMode() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return fanMode_;
}

void Configuration::setFanIntensity(const uint32_t intensity) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  fanIntensity_ = intensity;
}

common::Optional<uint32_t> Configuration::getFanIntensity() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return fanIntensity_;
}

void Configuration::setFanLowerTemperature(const float temperature) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  fanLowerTemperature_ = temperature;
}

common::Optional<float> Configuration::getFanLowerTemperature() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return fanLowerTemperature_;
}

void Configuration::setFanUpperTemperature(const float temperature) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  fanUpperTemperature_ = temperature;
}

common::Optional<float> Configuration::getFanUpperTemperature() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return fanUpperTemperature_;
}

void Configuration::setGearJointVelocityFilterType(const uint32_t type) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  gearJointVelocityFilterType_ = type;
}

common::Optional<uint32_t> Configuration::getGearJointVelocityFilterType() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return gearJointVelocityFilterType_;
}

void Configuration::setGearJointVelocityKfNoiseVariance(const float variance) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  gearJointVelocityKfNoiseVariance_ = variance;
}

common::Optional<float> Configuration::getGearJointVelocityKfNoiseVariance() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return gearJointVelocityKfNoiseVariance_;
}

void Configuration::setGearJointVelocityKfLambda2(const float lambda) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  gearJointVelocityKfLambda2_ = lambda;
}

common::Optional<float> Configuration::getGearJointVelocityKfLambda2() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return gearJointVelocityKfLambda2_;
}

void Configuration::setGearJointVelocityKfGamma(const float gamma) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  gearJointVelocityKfGamma_ = gamma;
}

common::Optional<float> Configuration::getGearJointVelocityKfGamma() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return gearJointVelocityKfGamma_;
}

void Configuration::setGearJointVelocityEmaAlpha(const float alpha) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  gearJointVelocityEmaAlpha_ = alpha;
}

common::Optional<float> Configuration::getGearJointVelocityEmaAlpha() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return gearJointVelocityEmaAlpha_;
}

void Configuration::setJointVelocityForAccelerationFilterType(const uint32_t type) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  jointVelocityForAccelerationFilterType_ = type;
}

common::Optional<uint32_t> Configuration::getJointVelocityForAccelerationFilterType() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return jointVelocityForAccelerationFilterType_;
}

void Configuration::setJointVelocityForAccelerationKfNoiseVariance(const float variance) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  jointVelocityForAccelerationKfNoiseVariance_ = variance;
}

common::Optional<float> Configuration::getJointVelocityForAccelerationKfNoiseVariance() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return jointVelocityForAccelerationKfNoiseVariance_;
}

void Configuration::setJointVelocityForAccelerationKfLambda2(const float lambda) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  jointVelocityForAccelerationKfLambda2_ = lambda;
}

common::Optional<float> Configuration::getJointVelocityForAccelerationKfLambda2() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return jointVelocityForAccelerationKfLambda2_;
}

void Configuration::setJointVelocityForAccelerationKfGamma(const float gamma) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  jointVelocityForAccelerationKfGamma_ = gamma;
}

common::Optional<float> Configuration::getJointVelocityForAccelerationKfGamma() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return jointVelocityForAccelerationKfGamma_;
}

void Configuration::setJointVelocityForAccelerationEmaAlpha(const float alpha) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  jointVelocityForAccelerationEmaAlpha_ = alpha;
}

common::Optional<float> Configuration::getJointVelocityForAccelerationEmaAlpha() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return jointVelocityForAccelerationEmaAlpha_;
}

void Configuration::setJointAccelerationFilterType(const uint32_t type) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  jointAccelerationFilterType_ = type;
}

common::Optional<uint32_t> Configuration::getJointAccelerationFilterType() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return jointAccelerationFilterType_;
}

void Configuration::setJointAccelerationKfNoiseVariance(const float variance) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  jointAccelerationKfNoiseVariance_ = variance;
}

common::Optional<float> Configuration::getJointAccelerationKfNoiseVariance() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return jointAccelerationKfNoiseVariance_;
}

void Configuration::setJointAccelerationKfLambda2(const float lambda) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  jointAccelerationKfLambda2_ = lambda;
}

common::Optional<float> Configuration::getJointAccelerationKfLambda2() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return jointAccelerationKfLambda2_;
}

void Configuration::setJointAccelerationKfGamma(const float gamma) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  jointAccelerationKfGamma_ = gamma;
}

common::Optional<float> Configuration::getJointAccelerationKfGamma() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return jointAccelerationKfGamma_;
}

void Configuration::setJointAccelerationEmaAlpha(const float alpha) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  jointAccelerationEmaAlpha_ = alpha;
}

common::Optional<float> Configuration::getJointAccelerationEmaAlpha() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return jointAccelerationEmaAlpha_;
}

mode::ModeBasePtr Configuration::getMode(const mode::ModeEnum modeEnum) const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  auto it = modes_.find(modeEnum);
  if (it == modes_.end()) {
    ANYDRIVE_ERROR("Mode does not exist.");
    return mode::ModeBasePtr();
  }
  return it->second;
}

std::map<mode::ModeEnum, mode::ModeBasePtr> Configuration::getModes() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return modes_;
}

void Configuration::setGoalStateEnumStartup(const fsm::StateEnum goalStateEnumStartup) {
  switch (goalStateEnumStartup) {
    case fsm::StateEnum::ColdStart:
    case fsm::StateEnum::DeviceMissing:
    case fsm::StateEnum::Error:
    case fsm::StateEnum::Fatal:
    case fsm::StateEnum::MotorPreOp:
    case fsm::StateEnum::WarmStart:
      ANYDRIVE_ERROR("Invalid startup goal state enum.");
      return;
    default:
      break;
  }

  std::lock_guard<std::recursive_mutex> lock(mutex_);
  goalStateEnumStartup_ = goalStateEnumStartup;
}

fsm::StateEnum Configuration::getGoalStateEnumStartup() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return goalStateEnumStartup_;
}

void Configuration::setGoalStateEnumShutdown(const fsm::StateEnum goalStateEnumShutdown) {
  switch (goalStateEnumShutdown) {
    case fsm::StateEnum::ColdStart:
    case fsm::StateEnum::ControlOp:
    case fsm::StateEnum::DeviceMissing:
    case fsm::StateEnum::Error:
    case fsm::StateEnum::Fatal:
    case fsm::StateEnum::MotorPreOp:
    case fsm::StateEnum::WarmStart:
      ANYDRIVE_ERROR("Invalid shutdown goal state enum.");
      return;
    default:
      break;
  }

  std::lock_guard<std::recursive_mutex> lock(mutex_);
  goalStateEnumShutdown_ = goalStateEnumShutdown;
}

fsm::StateEnum Configuration::getGoalStateEnumShutdown() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return goalStateEnumShutdown_;
}

void Configuration::addMode(const mode::ModeBasePtr& mode) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  modes_.insert({mode->getModeEnum(), mode});
}

}  // namespace configuration
}  // namespace anydrive
