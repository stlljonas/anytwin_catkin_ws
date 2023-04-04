#pragma once

#include <atomic>
#include <cstdint>
#include <map>
#include <mutex>

#include <anydrive/communication/CommunicationInterfaceBase.hpp>

// soem_interface
#include <soem_interface/EthercatBusBase.hpp>
#include <soem_interface/EthercatSlaveBase.hpp>

#include "anydrive_ethercat/PdoTypeEnum.hpp"

namespace anydrive_ethercat {

class AnydriveEthercat;

class AnydriveEthercatSlave : public soem_interface::EthercatSlaveBase, public anydrive::communication::CommunicationInterfaceBase {
 protected:
  using RecMutex = std::recursive_mutex;
  using RecLock = std::lock_guard<RecMutex>;

  static constexpr double rpmPerRadS_ = 60.0 / (2.0 * M_PI);

  static constexpr double milliScaling_ = 1000.0;
  static constexpr double kiloScaling_ = 0.001;

  static constexpr double timeScaling_ = milliScaling_;  // ms
  static constexpr double gainScaling_ = 1.0;
  static constexpr double temperatureScaling_ = 100.0;                         // °cC (and offset)
  static constexpr double motorVoltageScaling_ = 100.0;                        // cV
  static constexpr double motorCurrentScaling_ = 1.0;                          // A
  static constexpr double motorPositionScaling_ = 1.0;                         // rad
  static constexpr double motorVelocityScaling_ = kiloScaling_ * rpmPerRadS_;  // krpm
  static constexpr double gearPositionScaling_ = 1.0;                          // rad
  static constexpr double gearVelocityScaling_ = rpmPerRadS_;                  // rpm
  static constexpr double jointPositionScaling_ = 1.0;                         // rad
  static constexpr double jointVelocityScaling_ = rpmPerRadS_;                 // rpm
  static constexpr double jointAccelerationScaling_ = rpmPerRadS_;             // rpm/s
  static constexpr double jointTorqueScaling_ = 1.0;                           // Nm

  static constexpr double timeScalingInv_ = 1.0 / timeScaling_;
  static constexpr double gainScalingInv_ = 1.0 / gainScaling_;
  static constexpr double temperatureScalingInv_ = 1.0 / temperatureScaling_;
  static constexpr double motorVoltageScalingInv_ = 1.0 / motorVoltageScaling_;
  static constexpr double motorCurrentScalingInv_ = 1.0 / motorCurrentScaling_;
  static constexpr double motorPositionScalingInv_ = 1.0 / motorPositionScaling_;
  static constexpr double motorVelocityScalingInv_ = 1.0 / motorVelocityScaling_;
  static constexpr double gearPositionScalingInv_ = 1.0 / gearPositionScaling_;
  static constexpr double gearVelocityScalingInv_ = 1.0 / gearVelocityScaling_;
  static constexpr double jointPositionScalingInv_ = 1.0 / jointPositionScaling_;
  static constexpr double jointVelocityScalingInv_ = 1.0 / jointVelocityScaling_;
  static constexpr double jointAccelerationScalingInv_ = 1.0 / jointAccelerationScaling_;
  static constexpr double jointTorqueScalingInv_ = 1.0 / jointTorqueScaling_;

  static constexpr double temperatureOffset_ = -55.0;

  /*
   * The offset of the data-logging SDO where the histogram-block starts. From this and the size of the histograms, the other sub-indexes
   * are derived.
   */
  static constexpr int loggedDataHistogramsOffset = 4;

  using PdoInfos = std::map<PdoTypeEnum, PdoInfo>;

  static std::map<anydrive::mode::ModeEnum, uint16_t> modeEnumToOdIndex_;

  PdoInfos pdoInfos_;
  std::atomic<PdoTypeEnum> pdoTypeEnum_;
  std::atomic<PdoTypeEnum> currentPdoTypeEnum_;

  mutable RecMutex readingMutex_;
  anydrive::ReadingExtended reading_;

  mutable RecMutex commandMutex_;
  anydrive::Command command_;

  mutable RecMutex controlwordIdMutex_;
  uint16_t controlwordId_ = 0;

  bool isRtdlRunning_ = false;

 public:
  AnydriveEthercatSlave(const anydrive::AnydriveWeakPtr& anydrive, soem_interface::EthercatBusBase* bus, const uint32_t address,
                        const PdoTypeEnum pdoTypeEnum);
  ~AnydriveEthercatSlave() override = default;

  bool startup() override;
  void updateRead() override;
  void updateWrite() override;
  void shutdown() override;

  bool deviceIsMissing() const override;

  void setState(const uint16_t state);
  bool waitForState(const uint16_t state);

  PdoTypeEnum getCurrentPdoTypeEnum() const;
  PdoInfo getCurrentPdoInfo() const override;
  bool receivesGearAndJointEncoderTicks() const override;
  std::string getName() const override;

  // Send SDOs.
  bool sendSdoReadGeneric(const std::string& indexString, const std::string& subindexString, const std::string& valueTypeString,
                          std::string& valueString) override;
  bool sendSdoWriteGeneric(const std::string& indexString, const std::string& subindexString, const std::string& valueTypeString,
                           const std::string& valueString) override;

  // Drive info.
  bool getDriveModel(std::string& model) override;
  bool getDriveInfoSerialNumber(std::string& serialNumber) override;
  bool setDriveInfoSerialNumber(const std::string& serialNumber) override;
  bool getDriveInfoName(std::string& name) override;
  bool setDriveInfoName(const std::string& name) override;
  bool getDriveInfoId(uint16_t& id) override;
  bool setDriveInfoId(const uint16_t id) override;
  bool getDriveInfoBootloaderVersion(anydrive::common::Version& bootloaderVersion) override;
  bool setDriveInfoBootloaderVersion(const anydrive::common::Version& bootloaderVersion) override;
  bool getDriveInfoFirmwareVersion(anydrive::common::Version& firmwareVersion) override;
  bool getDriveFirmwareInfo(anydrive::common::FirmwareInfo& firmwareInfo) override;
  bool getGearboxRatio(uint32_t& ratio) override;

  // Flash storage.
  bool eraseFlashStorage() override;
  bool resetFlashStorageSections(const uint16_t flashStorageSections) override;

  // Calibration.
  bool getCalibrationState(const anydrive::calibration::CalibrationTypeEnum calibrationTypeEnum,
                           anydrive::calibration::CalibrationState& calibrationState) override;
  bool sendCalibrationGearAndJointEncoderHomingNewJointPosition(const double newJointPosition) override;
  bool startCalibration(const anydrive::calibration::CalibrationModeEnum calibrationModeEnum) override;
  bool calibrationIsRunning(bool& running) override;
  bool getCalibrationMotorEncoderOffset(const anydrive::calibration::CalibrationTypeEnum calibrationTypeEnum,
                                        anydrive::calibration::parameter::MotorEncoderOffset& motorEncoderOffset) override;
  bool getCalibrationMotorEncoderParameters(const anydrive::calibration::CalibrationTypeEnum calibrationTypeEnum,
                                            anydrive::calibration::parameter::MotorEncoderParameters& motorEncoderParameters) override;
  bool getCalibrationGearJointEncoderOffset(const anydrive::calibration::CalibrationTypeEnum calibrationTypeEnum,
                                            anydrive::calibration::parameter::GearJointEncoderOffset& gearJointEncoderOffset) override;
  bool getCalibrationGearJointEncoderOffsetConstant(int32_t& constant) override;
  bool setCalibrationGearJointEncoderOffsetConstant(const int32_t constant) override;
  bool getCalibrationGearAndJointEncoderHoming(
      const anydrive::calibration::CalibrationTypeEnum calibrationTypeEnum,
      anydrive::calibration::parameter::GearAndJointEncoderHoming& gearAndJointEncoderHoming) override;
  bool getCalibrationImuGyroscopeDcBias(const anydrive::calibration::CalibrationTypeEnum calibrationTypeEnum,
                                        anydrive::calibration::parameter::ImuGyroscopeDcBias& imuGyroscopeDcBias) override;
  bool setCalibrationSpringStiffness(const anydrive::calibration::CalibrationTypeEnum calibrationTypeEnum,
                                     const anydrive::calibration::parameter::SpringStiffness& springStiffness) override;
  bool getCalibrationSpringStiffness(const anydrive::calibration::CalibrationTypeEnum calibrationTypeEnum,
                                     anydrive::calibration::parameter::SpringStiffness& springStiffness) override;
  bool setCalibrationFrictionEstimation(const anydrive::calibration::CalibrationTypeEnum calibrationTypeEnum,
                                        const anydrive::calibration::parameter::FrictionEstimation& frictionEstimation) override;
  bool getCalibrationFrictionEstimation(const anydrive::calibration::CalibrationTypeEnum calibrationTypeEnum,
                                        anydrive::calibration::parameter::FrictionEstimation& frictionEstimation) override;
  bool resetCustomCalibrationsToFactory(const anydrive::calibration::CalibrationState calibrationState) override;
  bool writeFactoryCalibration() override;

  // Configuration.
  bool getMaxCurrent(double& maxCurrent) override;
  bool setMaxCurrent(const double maxCurrent) override;
  bool getMaxFreezeCurrent(double& current) override;
  bool setMaxFreezeCurrent(const double current) override;
  bool getMaxMotorVelocity(double& maxMotorVelocity) override;
  bool setMaxMotorVelocity(const double maxMotorVelocity) override;
  bool getMaxJointTorque(double& maxJointTorque) override;
  bool setMaxJointTorque(const double maxJointTorque) override;
  bool getCurrentIntegratorSaturation(double& saturation) override;
  bool setCurrentIntegratorSaturation(const double saturation) override;
  bool getJointTorqueIntegratorSaturation(double& saturation) override;
  bool setJointTorqueIntegratorSaturation(const double saturation) override;
  bool getDirection(int16_t& direction) override;
  bool setDirection(const int16_t direction) override;
  bool getJointPositionLimitsSoft(anydrive::common::Limits& limits) override;
  bool setJointPositionLimitsSoft(const anydrive::common::Limits& limits) override;
  bool getJointPositionLimitsHard(anydrive::common::Limits& limits) override;
  bool setJointPositionLimitsHard(const anydrive::common::Limits& limits) override;
  bool getControlGains(const anydrive::mode::ModeEnum mode, anydrive::mode::PidGainsF& pidGains) override;
  bool setControlGains(const anydrive::mode::ModeEnum mode, const anydrive::mode::PidGainsF& pidGains) override;
  bool getErrorStateBehavior(uint16_t& errorStateBehavior) override;
  bool setErrorStateBehavior(const uint16_t errorStateBehavior) override;
  bool getImuEnabled(bool& enabled) override;
  bool setImuEnable(const bool enable) override;
  bool getImuAccelerometerRange(uint32_t& range) override;
  bool setImuAccelerometerRange(const uint32_t range) override;
  bool getImuGyroscopeRange(uint32_t& range) override;
  bool setImuGyroscopeRange(const uint32_t range) override;
  bool getFanMode(uint32_t& mode) override;
  bool setFanMode(const uint32_t mode) override;
  bool getFanIntensity(uint32_t& intensity) override;
  bool setFanIntensity(const uint32_t intensity) override;
  bool getFanLowerTemperature(float& temperature) override;
  bool setFanLowerTemperature(const float temperature) override;
  bool getFanUpperTemperature(float& temperature) override;
  bool setFanUpperTemperature(const float temperature) override;
  bool setBrakeMode(const bool mode) override;
  bool getBrakeMode(bool& mode) override;
  bool setBrakeDuty(const float d) override;
  bool getBrakeDuty(float& d) override;
  bool getGearJointVelocityFilterType(uint32_t& type) override;
  bool setGearJointVelocityFilterType(const uint32_t type) override;
  bool getGearJointVelocityKfNoiseVariance(float& variance) override;
  bool setGearJointVelocityKfNoiseVariance(const float variance) override;
  bool getGearJointVelocityKfLambda2(float& lambda) override;
  bool setGearJointVelocityKfLambda2(const float lambda) override;
  bool getGearJointVelocityKfGamma(float& gamma) override;
  bool setGearJointVelocityKfGamma(const float gamma) override;
  bool getGearJointVelocityEmaAlpha(float& alpha) override;
  bool setGearJointVelocityEmaAlpha(const float alpha) override;
  bool getJointVelocityForAccelerationFilterType(uint32_t& type) override;
  bool setJointVelocityForAccelerationFilterType(const uint32_t type) override;
  bool getJointVelocityForAccelerationKfNoiseVariance(float& variance) override;
  bool setJointVelocityForAccelerationKfNoiseVariance(const float variance) override;
  bool getJointVelocityForAccelerationKfLambda2(float& lambda) override;
  bool setJointVelocityForAccelerationKfLambda2(const float lambda) override;
  bool getJointVelocityForAccelerationKfGamma(float& gamma) override;
  bool setJointVelocityForAccelerationKfGamma(const float gamma) override;
  bool getJointVelocityForAccelerationEmaAlpha(float& alpha) override;
  bool setJointVelocityForAccelerationEmaAlpha(const float alpha) override;
  bool getJointAccelerationFilterType(uint32_t& type) override;
  bool setJointAccelerationFilterType(const uint32_t type) override;
  bool getJointAccelerationKfNoiseVariance(float& variance) override;
  bool setJointAccelerationKfNoiseVariance(const float variance) override;
  bool getJointAccelerationKfLambda2(float& lambda) override;
  bool setJointAccelerationKfLambda2(const float lambda) override;
  bool getJointAccelerationKfGamma(float& gamma) override;
  bool setJointAccelerationKfGamma(const float gamma) override;
  bool getJointAccelerationEmaAlpha(float& alpha) override;
  bool setJointAccelerationEmaAlpha(const float alpha) override;
  bool writeConfiguration() override;

  // Status.
  bool requestStatusword() override;
  bool getStatusword(anydrive::Statusword& statusword) override;

  // Control.
  void setControlword(const uint16_t controlwordId) override;
  void resetControlword() override;
  void setCommand(const anydrive::Command& command) override;
  void getReading(anydrive::ReadingExtended& reading) override;

  // RTDL (Real time data logging).
  bool getRtdlEnabled(bool& enabled) override;
  bool setRtdlEnable(const bool enable) override;
  bool setRtdlCommand(const uint16_t command) override;
  bool getRtdlStatus(uint16_t& status) override;
  bool getRtdlLoggingFrequency(uint16_t& frequency) override;
  bool setRtdlLoggingFrequency(const uint16_t frequency) override;
  bool getRtdlStreamingFrequency(uint16_t& frequency) override;
  bool setRtdlStreamingFrequency(const uint16_t frequency) override;
  bool getRtdlLastTimestamp(uint64_t& timestamp) override;

  // Data logging:
  bool clearLoggedData() override;
  bool refreshLoggedData() override;
  bool getNumHistogramBins(int& numHistogramBins) override;
  bool readHistograms(std::vector<uint64_t>& histTorque, std::vector<uint64_t>& histVelocity, std::vector<uint64_t>& histCurrent,
                      std::vector<uint64_t>& histTemperature) override;
  bool readOperatingTimes(uint64_t& timeTotal, uint64_t& timeActive) override;
  bool readJointTravel(double& jointTravel) override;
  bool readHistogramEdges(float& torque_upper, float& torque_lower, float& velocity_upper, float& velocity_lower, float& current_upper,
                          float& current_lower, float& temperature_upper, float& temperature_lower) override;

  bool getCalibrationTypeId(uint16_t& calibrationTypeId) override;
  bool setCalibrationTypeId(const uint16_t calibrationTypeId) override;

  bool getLockStatus(uint32_t& status) override;
  bool sendPassword(const std::string& password) override;

  bool getCalibrationTypeEnum(anydrive::calibration::CalibrationTypeEnum& calibrationTypeEnum);
  bool setCalibrationTypeEnum(const anydrive::calibration::CalibrationTypeEnum calibrationTypeEnum);

 protected:
  bool configurePdo(const PdoTypeEnum pdoTypeEnum);

  bool sendSdoReadString(const uint16_t index, std::string& string);
  bool sendSdoWriteString(const uint16_t index, const std::string& string);
  bool readHistogram(std::vector<uint64_t>& hist, int subIndexOffset, int size);
};

using AnydriveEthercatSlavePtr = std::shared_ptr<AnydriveEthercatSlave>;

}  // namespace anydrive_ethercat
