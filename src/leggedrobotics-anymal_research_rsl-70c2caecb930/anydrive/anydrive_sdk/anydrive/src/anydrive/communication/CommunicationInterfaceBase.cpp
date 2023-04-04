#include "anydrive/communication/CommunicationInterfaceBase.hpp"
#include "anydrive/Anydrive.hpp"

namespace anydrive {
namespace communication {

CommunicationInterfaceBase::CommunicationInterfaceBase(const AnydriveWeakPtr& anydrive)  // NOLINT
    : anydrive_(anydrive) {}

void CommunicationInterfaceBase::printWarnNotImplemented() {
  ANYDRIVE_NAMED_WARN("Functionality is not implemented.");
}

std::string CommunicationInterfaceBase::getName() const {
  AnydrivePtr anydrive = anydrive_.lock();
  if (anydrive) {
    return anydrive->getName();
  }
  return "";
}

bool CommunicationInterfaceBase::getDriveModel(std::string& /*model*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getDriveInfoSerialNumber(std::string& /*serialNumber*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::setDriveInfoSerialNumber(const std::string& /*serialNumber*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getDriveInfoName(std::string& /*name*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::setDriveInfoName(const std::string& /*name*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getDriveInfoId(uint16_t& /*id*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::setDriveInfoId(const uint16_t /*id*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getDriveInfoBootloaderVersion(common::Version& /*bootloaderVersion*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::setDriveInfoBootloaderVersion(const common::Version& /*bootloaderVersion*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getDriveInfoFirmwareVersion(common::Version& /*firmwareVersion*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getDriveFirmwareInfo(common::FirmwareInfo& /*firmwareInfo*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getGearboxRatio(uint32_t& /*ratio*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::eraseFlashStorage() {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::resetFlashStorageSections(const uint16_t /*flashStorageSections*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getCalibrationState(const calibration::CalibrationTypeEnum /*calibrationTypeEnum*/,
                                                     calibration::CalibrationState& /*calibrationState*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::resetCustomCalibrationsToFactory(const calibration::CalibrationState /*calibrationState*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::sendCalibrationGearAndJointEncoderHomingNewJointPosition(const double /*newJointPosition*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::startCalibration(calibration::CalibrationModeEnum /*calibrationModeEnum*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::calibrationIsRunning(bool& /*running*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getCalibrationMotorEncoderOffset(const calibration::CalibrationTypeEnum /*calibrationTypeEnum*/,
                                                                  calibration::parameter::MotorEncoderOffset& /*motorEncoderOffset*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getCalibrationMotorEncoderParameters(
    const calibration::CalibrationTypeEnum /*calibrationTypeEnum*/,
    calibration::parameter::MotorEncoderParameters& /*motorEncoderParameters*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getCalibrationGearJointEncoderOffset(
    const calibration::CalibrationTypeEnum /*calibrationTypeEnum*/,
    calibration::parameter::GearJointEncoderOffset& /*gearJointEncoderOffset*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getCalibrationGearAndJointEncoderHoming(
    const calibration::CalibrationTypeEnum /*calibrationTypeEnum*/,
    calibration::parameter::GearAndJointEncoderHoming& /*gearAndJointEncoderHoming*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getCalibrationImuGyroscopeDcBias(const calibration::CalibrationTypeEnum /*calibrationTypeEnum*/,
                                                                  calibration::parameter::ImuGyroscopeDcBias& /*imuGyroscopeDcBias*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::setCalibrationSpringStiffness(const calibration::CalibrationTypeEnum /*calibrationTypeEnum*/,
                                                               const calibration::parameter::SpringStiffness& /*springStiffness*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getCalibrationSpringStiffness(const calibration::CalibrationTypeEnum /*calibrationTypeEnum*/,
                                                               calibration::parameter::SpringStiffness& /*springStiffness*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::setCalibrationFrictionEstimation(
    const calibration::CalibrationTypeEnum /*calibrationTypeEnum*/,
    const calibration::parameter::FrictionEstimation& /*frictionEstimation*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getCalibrationFrictionEstimation(const calibration::CalibrationTypeEnum /*calibrationTypeEnum*/,
                                                                  calibration::parameter::FrictionEstimation& /*frictionEstimation*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::writeFactoryCalibration() {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getMaxCurrent(double& /*maxCurrent*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::setMaxCurrent(const double /*maxCurrent*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getMaxFreezeCurrent(double& /*current*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::setMaxFreezeCurrent(const double /*current*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::clearLoggedData() {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::refreshLoggedData() {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getNumHistogramBins(int& /*numHistogramBins*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::readHistograms(std::vector<uint64_t>& /*histTorque*/, std::vector<uint64_t>& /*histVelocity*/,
                                                std::vector<uint64_t>& /*histCurrent*/, std::vector<uint64_t>& /*histTemperature*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::readOperatingTimes(uint64_t& /*timeTotal*/, uint64_t& /*timeActive*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::readJointTravel(double& /*jointTravel*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::readHistogramEdges(float& /*torque_upper*/, float& /*torque_lower*/, float& /*velocity_upper*/,
                                                    float& /*velocity_lower*/, float& /*current_upper*/, float& /*current_lower*/,
                                                    float& /*temperature_upper*/, float& /*temperature_lower*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getMaxMotorVelocity(double& /*maxMotorVelocity*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::setMaxMotorVelocity(const double /*maxMotorVelocity*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getMaxJointTorque(double& /*maxJointTorque*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::setMaxJointTorque(const double /*maxJointTorque*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getCurrentIntegratorSaturation(double& /*saturation*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::setCurrentIntegratorSaturation(const double /*saturation*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getJointTorqueIntegratorSaturation(double& /*saturation*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::setJointTorqueIntegratorSaturation(const double /*saturation*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getDirection(int16_t& /*direction*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::setDirection(const int16_t /*direction*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getJointPositionLimitsSoft(common::Limits& /*limits*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::setJointPositionLimitsSoft(const common::Limits& /*limits*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getJointPositionLimitsHard(common::Limits& /*limits*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::setJointPositionLimitsHard(const common::Limits& /*limits*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getControlGains(const mode::ModeEnum /*mode*/, mode::PidGainsF& /*pidGains*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::setControlGains(const mode::ModeEnum /*mode*/, const mode::PidGainsF& /*pidGains*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getErrorStateBehavior(uint16_t& /*errorStateBehavior*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::setErrorStateBehavior(const uint16_t /*errorStateBehavior*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getImuEnabled(bool& /*enabled*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::setImuEnable(const bool /*enable*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getImuAccelerometerRange(uint32_t& /*range*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::setImuAccelerometerRange(const uint32_t /*range*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getImuGyroscopeRange(uint32_t& /*range*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::setImuGyroscopeRange(const uint32_t /*range*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getFanMode(uint32_t& /*mode*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::setFanMode(const uint32_t /*mode*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getFanIntensity(uint32_t& /*intensity*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::setFanIntensity(const uint32_t /*intensity*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getFanLowerTemperature(float& /*temperature*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::setFanLowerTemperature(const float /*temperature*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getFanUpperTemperature(float& /*temperature*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::setFanUpperTemperature(const float /*temperature*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::setBrakeMode(const bool /*mode*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getBrakeMode(bool& /*mode*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::setBrakeDuty(const float /*d*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getBrakeDuty(float& /*d*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getGearJointVelocityFilterType(uint32_t& /*type*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::setGearJointVelocityFilterType(const uint32_t /*type*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getGearJointVelocityKfNoiseVariance(float& /*variance*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::setGearJointVelocityKfNoiseVariance(const float /*variance*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getGearJointVelocityKfLambda2(float& /*lambda*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::setGearJointVelocityKfLambda2(const float /*lambda*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getGearJointVelocityKfGamma(float& /*gamma*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::setGearJointVelocityKfGamma(const float /*gamma*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getGearJointVelocityEmaAlpha(float& /*alpha*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::setGearJointVelocityEmaAlpha(const float /*alpha*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getJointVelocityForAccelerationFilterType(uint32_t& /*type*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::setJointVelocityForAccelerationFilterType(const uint32_t /*type*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getJointVelocityForAccelerationKfNoiseVariance(float& /*variance*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::setJointVelocityForAccelerationKfNoiseVariance(const float /*variance*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getJointVelocityForAccelerationKfLambda2(float& /*lambda*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::setJointVelocityForAccelerationKfLambda2(const float /*lambda*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getJointVelocityForAccelerationKfGamma(float& /*gamma*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::setJointVelocityForAccelerationKfGamma(const float /*gamma*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getJointVelocityForAccelerationEmaAlpha(float& /*alpha*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::setJointVelocityForAccelerationEmaAlpha(const float /*alpha*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getJointAccelerationFilterType(uint32_t& /*type*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::setJointAccelerationFilterType(const uint32_t /*type*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getJointAccelerationKfNoiseVariance(float& /*variance*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::setJointAccelerationKfNoiseVariance(const float /*variance*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getJointAccelerationKfLambda2(float& /*lambda*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::setJointAccelerationKfLambda2(const float /*lambda*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getJointAccelerationKfGamma(float& /*gamma*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::setJointAccelerationKfGamma(const float /*gamma*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getJointAccelerationEmaAlpha(float& /*alpha*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::setJointAccelerationEmaAlpha(const float /*alpha*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::writeConfiguration() {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getRtdlEnabled(bool& /*enabled*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::setRtdlEnable(const bool /*enable*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::setRtdlCommand(const uint16_t /*command*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getRtdlStatus(uint16_t& /*status*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getRtdlLoggingFrequency(uint16_t& /*frequency*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::setRtdlLoggingFrequency(const uint16_t /*frequency*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getRtdlStreamingFrequency(uint16_t& /*frequency*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::setRtdlStreamingFrequency(const uint16_t /*frequency*/) {
  printWarnNotImplemented();
  return false;
}

bool CommunicationInterfaceBase::getRtdlLastTimestamp(uint64_t& /*timestamp*/) {
  printWarnNotImplemented();
  return false;
}

}  // namespace communication
}  // namespace anydrive
