#pragma once

#include <cstdint>
#include <memory>

#include "anydrive/Command.hpp"
#include "anydrive/DriveInfo.hpp"
#include "anydrive/ReadingExtended.hpp"
#include "anydrive/Statusword.hpp"
#include "anydrive/calibration/CalibrationModeEnum.hpp"
#include "anydrive/calibration/CalibrationState.hpp"
#include "anydrive/calibration/CalibrationTypeEnum.hpp"
#include "anydrive/calibration/parameter/Calibration.hpp"
#include "anydrive/common/FirmwareInfo.hpp"
#include "anydrive/common/Limits.hpp"
#include "anydrive/common/ObjectDictionary.hpp"
#include "anydrive/common/Version.hpp"
#include "anydrive/mode/PidGains.hpp"
#include "anydrive/usings.hpp"

namespace anydrive {

class Anydrive;
using AnydriveWeakPtr = std::weak_ptr<Anydrive>;
using AnydrivePtr = std::shared_ptr<Anydrive>;

namespace communication {

class CommunicationInterfaceBase {
 protected:
  AnydriveWeakPtr anydrive_;

 protected:
  explicit CommunicationInterfaceBase(const AnydriveWeakPtr& anydrive);
  virtual ~CommunicationInterfaceBase() = default;

  void printWarnNotImplemented();

 public:
  std::string getName() const;

  virtual bool deviceIsMissing() const = 0;

  virtual bool receivesGearAndJointEncoderTicks() const = 0;

  virtual bool getDriveModel(std::string& model);
  virtual bool getDriveInfoSerialNumber(std::string& serialNumber);
  virtual bool setDriveInfoSerialNumber(const std::string& serialNumber);
  virtual bool getDriveInfoName(std::string& name);
  virtual bool setDriveInfoName(const std::string& name);
  virtual bool getDriveInfoId(uint16_t& id);
  virtual bool setDriveInfoId(const uint16_t id);
  virtual bool getDriveInfoBootloaderVersion(common::Version& bootloaderVersion);
  virtual bool setDriveInfoBootloaderVersion(const common::Version& bootloaderVersion);
  virtual bool getDriveInfoFirmwareVersion(common::Version& firmwareVersion);
  virtual bool getDriveFirmwareInfo(common::FirmwareInfo& firmwareInfo);
  virtual bool getGearboxRatio(uint32_t& ratio);

  // Flash storage.
  virtual bool eraseFlashStorage();
  virtual bool resetFlashStorageSections(const uint16_t flashStorageSections);

  // Calibration.
  virtual bool getCalibrationState(const calibration::CalibrationTypeEnum calibrationTypeEnum,
                                   calibration::CalibrationState& calibrationState);
  virtual bool sendCalibrationGearAndJointEncoderHomingNewJointPosition(const double newJointPosition);
  virtual bool startCalibration(const calibration::CalibrationModeEnum calibrationModeEnum);
  virtual bool calibrationIsRunning(bool& running);
  virtual bool getCalibrationMotorEncoderOffset(const calibration::CalibrationTypeEnum calibrationTypeEnum,
                                                calibration::parameter::MotorEncoderOffset& motorEncoderOffset);
  virtual bool getCalibrationMotorEncoderParameters(const calibration::CalibrationTypeEnum calibrationTypeEnum,
                                                    calibration::parameter::MotorEncoderParameters& motorEncoderParameters);
  virtual bool getCalibrationGearJointEncoderOffset(const calibration::CalibrationTypeEnum calibrationTypeEnum,
                                                    calibration::parameter::GearJointEncoderOffset& gearJointEncoderOffset);
  virtual bool getCalibrationGearJointEncoderOffsetConstant(int32_t& constant) = 0;
  virtual bool setCalibrationGearJointEncoderOffsetConstant(const int32_t constant) = 0;
  virtual bool getCalibrationGearAndJointEncoderHoming(const calibration::CalibrationTypeEnum calibrationTypeEnum,
                                                       calibration::parameter::GearAndJointEncoderHoming& gearAndJointEncoderHoming);
  virtual bool getCalibrationImuGyroscopeDcBias(const calibration::CalibrationTypeEnum calibrationTypeEnum,
                                                calibration::parameter::ImuGyroscopeDcBias& imuGyroscopeDcBias);
  virtual bool setCalibrationSpringStiffness(const calibration::CalibrationTypeEnum calibrationTypeEnum,
                                             const calibration::parameter::SpringStiffness& springStiffness);
  virtual bool getCalibrationSpringStiffness(const calibration::CalibrationTypeEnum calibrationTypeEnum,
                                             calibration::parameter::SpringStiffness& springStiffness);
  virtual bool setCalibrationFrictionEstimation(const calibration::CalibrationTypeEnum calibrationTypeEnum,
                                                const calibration::parameter::FrictionEstimation& frictionEstimation);
  virtual bool getCalibrationFrictionEstimation(const calibration::CalibrationTypeEnum calibrationTypeEnum,
                                                calibration::parameter::FrictionEstimation& frictionEstimation);
  virtual bool resetCustomCalibrationsToFactory(const calibration::CalibrationState calibrationState);
  virtual bool writeFactoryCalibration();

  // Configuration.
  virtual bool getMaxCurrent(double& maxCurrent);
  virtual bool setMaxCurrent(const double maxCurrent);
  virtual bool getMaxFreezeCurrent(double& current);
  virtual bool setMaxFreezeCurrent(const double current);
  virtual bool clearLoggedData();
  virtual bool refreshLoggedData();
  virtual bool getNumHistogramBins(int& numHistogramBins);
  virtual bool readHistograms(std::vector<uint64_t>& histTorque, std::vector<uint64_t>& histVelocity, std::vector<uint64_t>& histCurrent,
                              std::vector<uint64_t>& histTemperature);
  virtual bool readOperatingTimes(uint64_t& timeTotal, uint64_t& timeActive);
  virtual bool readJointTravel(double& jointTravel);
  virtual bool readHistogramEdges(float& torque_upper, float& torque_lower, float& velocity_upper, float& velocity_lower,
                                  float& current_upper, float& current_lower, float& temperature_upper, float& temperature_lower);
  virtual bool getMaxMotorVelocity(double& maxMotorVelocity);
  virtual bool setMaxMotorVelocity(const double maxMotorVelocity);
  virtual bool getMaxJointTorque(double& maxJointTorque);
  virtual bool setMaxJointTorque(const double maxJointTorque);
  virtual bool getCurrentIntegratorSaturation(double& saturation);
  virtual bool setCurrentIntegratorSaturation(const double saturation);
  virtual bool getJointTorqueIntegratorSaturation(double& saturation);
  virtual bool setJointTorqueIntegratorSaturation(const double saturation);
  virtual bool getDirection(int16_t& direction);
  virtual bool setDirection(const int16_t direction);
  virtual bool getJointPositionLimitsSoft(common::Limits& limits);
  virtual bool setJointPositionLimitsSoft(const common::Limits& limits);
  virtual bool getJointPositionLimitsHard(common::Limits& limits);
  virtual bool setJointPositionLimitsHard(const common::Limits& limits);
  virtual bool getControlGains(const mode::ModeEnum mode, mode::PidGainsF& pidGains);
  virtual bool setControlGains(const mode::ModeEnum mode, const mode::PidGainsF& pidGains);
  virtual bool getErrorStateBehavior(uint16_t& errorStateBehavior);
  virtual bool setErrorStateBehavior(const uint16_t errorStateBehavior);
  virtual bool getImuEnabled(bool& enabled);
  virtual bool setImuEnable(const bool enable);
  virtual bool getImuAccelerometerRange(uint32_t& range);
  virtual bool setImuAccelerometerRange(const uint32_t range);
  virtual bool getImuGyroscopeRange(uint32_t& range);
  virtual bool setImuGyroscopeRange(const uint32_t range);
  virtual bool getFanMode(uint32_t& mode);
  virtual bool setFanMode(const uint32_t mode);
  virtual bool getFanIntensity(uint32_t& intensity);
  virtual bool setFanIntensity(const uint32_t intensity);
  virtual bool getFanLowerTemperature(float& temperature);
  virtual bool setFanLowerTemperature(const float temperature);
  virtual bool getFanUpperTemperature(float& temperature);
  virtual bool setFanUpperTemperature(const float temperature);
  virtual bool setBrakeMode(const bool mode);
  virtual bool getBrakeMode(bool& mode);
  virtual bool setBrakeDuty(const float d);
  virtual bool getBrakeDuty(float& d);
  virtual bool getGearJointVelocityFilterType(uint32_t& type);
  virtual bool setGearJointVelocityFilterType(const uint32_t type);
  virtual bool getGearJointVelocityKfNoiseVariance(float& variance);
  virtual bool setGearJointVelocityKfNoiseVariance(const float variance);
  virtual bool getGearJointVelocityKfLambda2(float& lambda);
  virtual bool setGearJointVelocityKfLambda2(const float lambda);
  virtual bool getGearJointVelocityKfGamma(float& gamma);
  virtual bool setGearJointVelocityKfGamma(const float gamma);
  virtual bool getGearJointVelocityEmaAlpha(float& alpha);
  virtual bool setGearJointVelocityEmaAlpha(const float alpha);
  virtual bool getJointVelocityForAccelerationFilterType(uint32_t& type);
  virtual bool setJointVelocityForAccelerationFilterType(const uint32_t type);
  virtual bool getJointVelocityForAccelerationKfNoiseVariance(float& variance);
  virtual bool setJointVelocityForAccelerationKfNoiseVariance(const float variance);
  virtual bool getJointVelocityForAccelerationKfLambda2(float& lambda);
  virtual bool setJointVelocityForAccelerationKfLambda2(const float lambda);
  virtual bool getJointVelocityForAccelerationKfGamma(float& gamma);
  virtual bool setJointVelocityForAccelerationKfGamma(const float gamma);
  virtual bool getJointVelocityForAccelerationEmaAlpha(float& alpha);
  virtual bool setJointVelocityForAccelerationEmaAlpha(const float alpha);
  virtual bool getJointAccelerationFilterType(uint32_t& type);
  virtual bool setJointAccelerationFilterType(const uint32_t type);
  virtual bool getJointAccelerationKfNoiseVariance(float& variance);
  virtual bool setJointAccelerationKfNoiseVariance(const float variance);
  virtual bool getJointAccelerationKfLambda2(float& lambda);
  virtual bool setJointAccelerationKfLambda2(const float lambda);
  virtual bool getJointAccelerationKfGamma(float& gamma);
  virtual bool setJointAccelerationKfGamma(const float gamma);
  virtual bool getJointAccelerationEmaAlpha(float& alpha);
  virtual bool setJointAccelerationEmaAlpha(const float alpha);

  virtual bool writeConfiguration();

  // Status.
  virtual bool requestStatusword() = 0;
  virtual bool getStatusword(Statusword& statusword) = 0;

  // Control.
  virtual void setControlword(const uint16_t controlwordId) = 0;
  virtual void resetControlword() = 0;
  virtual void setCommand(const Command& command) = 0;
  virtual void getReading(ReadingExtended& reading) = 0;

  // RTDL (Real time data logging).
  virtual bool getRtdlEnabled(bool& enabled);
  virtual bool setRtdlEnable(const bool enable);
  virtual bool setRtdlCommand(const uint16_t command);
  virtual bool getRtdlStatus(uint16_t& status);
  virtual bool getRtdlLoggingFrequency(uint16_t& frequency);
  virtual bool setRtdlLoggingFrequency(const uint16_t frequency);
  virtual bool getRtdlStreamingFrequency(uint16_t& frequency);
  virtual bool setRtdlStreamingFrequency(const uint16_t frequency);
  virtual bool getRtdlLastTimestamp(uint64_t& timestamp);

  virtual bool getCalibrationTypeId(uint16_t& calibrationTypeId) = 0;
  virtual bool setCalibrationTypeId(const uint16_t calibrationTypeId) = 0;

  virtual bool getLockStatus(uint32_t& status) = 0;
  virtual bool sendPassword(const std::string& password) = 0;
};

using CommunicationInterfaceBasePtr = std::shared_ptr<CommunicationInterfaceBase>;

}  // namespace communication
}  // namespace anydrive
