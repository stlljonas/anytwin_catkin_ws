#include "anydrive/calibration/CalibrationState.hpp"

namespace anydrive {
namespace calibration {

bool CalibrationState::getSingleCalibrationState(const CalibrationModeEnum calibrationModeEnum) const {
  switch (calibrationModeEnum) {
    case CalibrationModeEnum::MotorEncoderOffset:
      return static_cast<bool>(single_.motorEncoderOffset_);
    case CalibrationModeEnum::MotorEncoderParameters:
      return static_cast<bool>(single_.motorEncoderParameters_);
    case CalibrationModeEnum::GearJointEncoderOffset:
      return static_cast<bool>(single_.gearJointEncoderOffset_);
    case CalibrationModeEnum::GearAndJointEncoderHoming:
      return static_cast<bool>(single_.gearAndJointEncoderHoming_);
    case CalibrationModeEnum::ImuGyroscopeDcBias:
      return static_cast<bool>(single_.imuGyroscopeDcBias_);
    case CalibrationModeEnum::SpringStiffness:
      return static_cast<bool>(single_.springStiffness_);
    case CalibrationModeEnum::FrictionEstimation:
      return static_cast<bool>(single_.frictionEstimation_);
    case CalibrationModeEnum::AksimGearCalibState:
      return static_cast<bool>(single_.aksimGear_);
    case CalibrationModeEnum::AksimJointCalibState:
      return static_cast<bool>(single_.aksimJoint_);

    default:
      return false;
  }
}

/*!
 * Set the state of a single calibration.
 * @param calibrationModeEnum Calibration mode enumerator.
 * @param isCalibrated    The new state of the calibration.
 */
void CalibrationState::setSingleCalibrationState(const CalibrationModeEnum calibrationModeEnum, const bool isCalibrated) {
  switch (calibrationModeEnum) {
    case CalibrationModeEnum::MotorEncoderOffset:
      single_.motorEncoderOffset_ = static_cast<uint16_t>(isCalibrated);
      break;
    case CalibrationModeEnum::MotorEncoderParameters:
      single_.motorEncoderParameters_ = static_cast<uint16_t>(isCalibrated);
      break;
    case CalibrationModeEnum::GearJointEncoderOffset:
      single_.gearJointEncoderOffset_ = static_cast<uint16_t>(isCalibrated);
      break;
    case CalibrationModeEnum::GearAndJointEncoderHoming:
      single_.gearAndJointEncoderHoming_ = static_cast<uint16_t>(isCalibrated);
      break;
    case CalibrationModeEnum::ImuGyroscopeDcBias:
      single_.imuGyroscopeDcBias_ = static_cast<uint16_t>(isCalibrated);
      break;
    case CalibrationModeEnum::SpringStiffness:
      single_.springStiffness_ = static_cast<uint16_t>(isCalibrated);
      break;
    case CalibrationModeEnum::FrictionEstimation:
      single_.frictionEstimation_ = static_cast<uint16_t>(isCalibrated);
      break;
    default:
      break;
  }
}

}  // namespace calibration
}  // namespace anydrive
