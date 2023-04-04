#include "anydrive/calibration/CalibrationModeEnum.hpp"
#include "anydrive/common/ObjectDictionary.hpp"

namespace anydrive {
namespace calibration {

uint16_t calibrationModeEnumToId(const CalibrationModeEnum calibrationModeEnum) {
  if (calibrationModeEnum == CalibrationModeEnum::FrictionEstimation) {
    return OD_CALIB_MODE_ID_VAL_FRICTION_ESTIMATION;
  }
  if (calibrationModeEnum == CalibrationModeEnum::GearAndJointEncoderHoming) {
    return OD_CALIB_MODE_ID_VAL_GEAR_AND_JOINT_ENCODER_HOMING;
  }
  if (calibrationModeEnum == CalibrationModeEnum::GearJointEncoderOffset) {
    return OD_CALIB_MODE_ID_VAL_GEAR_JOINT_ENCODER_OFFSET;
  }
  if (calibrationModeEnum == CalibrationModeEnum::ImuGyroscopeDcBias) {
    return OD_CALIB_MODE_ID_VAL_IMU_GYROSCOPE_DC_BIAS;
  }
  if (calibrationModeEnum == CalibrationModeEnum::MotorEncoderOffset) {
    return OD_CALIB_MODE_ID_VAL_MOTOR_ENCODER_OFFSET;
  }
  if (calibrationModeEnum == CalibrationModeEnum::MotorEncoderParameters) {
    return OD_CALIB_MODE_ID_VAL_MOTOR_ENCODER_PARAMETERS;
  }
  if (calibrationModeEnum == CalibrationModeEnum::SpringStiffness) {
    return OD_CALIB_MODE_ID_VAL_SPRING_STIFFNESS;
  }
  if (calibrationModeEnum == CalibrationModeEnum::AksimSelfCalibMode) {
    return OD_CALIB_MODE_ID_VAL_AKSIM_SELF_CALIB;
  }
  return OD_CALIB_MODE_ID_VAL_IDLE;
}

CalibrationModeEnum calibrationModeIdToEnum(const uint16_t calibrationModeId) {
  if (calibrationModeId == OD_CALIB_MODE_ID_VAL_FRICTION_ESTIMATION) {
    return CalibrationModeEnum::FrictionEstimation;
  }
  if (calibrationModeId == OD_CALIB_MODE_ID_VAL_GEAR_AND_JOINT_ENCODER_HOMING) {
    return CalibrationModeEnum::GearAndJointEncoderHoming;
  }
  if (calibrationModeId == OD_CALIB_MODE_ID_VAL_GEAR_JOINT_ENCODER_OFFSET) {
    return CalibrationModeEnum::GearJointEncoderOffset;
  }
  if (calibrationModeId == OD_CALIB_MODE_ID_VAL_IMU_GYROSCOPE_DC_BIAS) {
    return CalibrationModeEnum::ImuGyroscopeDcBias;
  }
  if (calibrationModeId == OD_CALIB_MODE_ID_VAL_MOTOR_ENCODER_OFFSET) {
    return CalibrationModeEnum::MotorEncoderOffset;
  }
  if (calibrationModeId == OD_CALIB_MODE_ID_VAL_MOTOR_ENCODER_PARAMETERS) {
    return CalibrationModeEnum::MotorEncoderParameters;
  }
  if (calibrationModeId == OD_CALIB_MODE_ID_VAL_SPRING_STIFFNESS) {
    return CalibrationModeEnum::SpringStiffness;
  }
  if (calibrationModeId == OD_CALIB_MODE_ID_VAL_AKSIM_SELF_CALIB) {
    return CalibrationModeEnum::AksimSelfCalibMode;
  }
  return CalibrationModeEnum::NA;
}

std::string calibrationModeEnumToName(const CalibrationModeEnum calibrationModeEnum) {
  if (calibrationModeEnum == CalibrationModeEnum::FrictionEstimation) {
    return ANYDRIVE_CALIB_MODE_NAME_SHORT_FRICTION_ESTIMATION;
  }
  if (calibrationModeEnum == CalibrationModeEnum::GearAndJointEncoderHoming) {
    return ANYDRIVE_CALIB_MODE_NAME_SHORT_GEAR_AND_JOINT_ENC_HOMING;
  }
  if (calibrationModeEnum == CalibrationModeEnum::GearJointEncoderOffset) {
    return ANYDRIVE_CALIB_MODE_NAME_SHORT_GEAR_JOINT_ENC_OFFSET;
  }
  if (calibrationModeEnum == CalibrationModeEnum::GravityCompensation) {
    return ANYDRIVE_CALIB_MODE_NAME_SHORT_GRAVITY_COMPENSATION;
  }
  if (calibrationModeEnum == CalibrationModeEnum::ImuGyroscopeDcBias) {
    return ANYDRIVE_CALIB_MODE_NAME_SHORT_IMU_GYRO_DC_BIAS;
  }
  if (calibrationModeEnum == CalibrationModeEnum::MotorEncoderOffset) {
    return ANYDRIVE_CALIB_MODE_NAME_SHORT_MOTOR_ENC_OFFSET;
  }
  if (calibrationModeEnum == CalibrationModeEnum::MotorEncoderParameters) {
    return ANYDRIVE_CALIB_MODE_NAME_SHORT_MOTOR_ENC_PARAMS;
  }
  if (calibrationModeEnum == CalibrationModeEnum::SafeJointVelocity) {
    return ANYDRIVE_CALIB_MODE_NAME_SHORT_SAFE_JOINT_VEL;
  }
  if (calibrationModeEnum == CalibrationModeEnum::SpringStiffness) {
    return ANYDRIVE_CALIB_MODE_NAME_SHORT_SPRING_STIFFNESS;
  }
  if (calibrationModeEnum == CalibrationModeEnum::AksimSelfCalibMode) {
    return ANYDRIVE_CALIB_MODE_NAME_SHORT_AKSIM_SELF_CALIB;
  }
  return ANYDRIVE_CALIB_MODE_NAME_SHORT_NA;
}

CalibrationModeEnum calibrationModeNameToEnum(const std::string& calibrationModeName) {
  if (calibrationModeName == ANYDRIVE_CALIB_MODE_NAME_SHORT_FRICTION_ESTIMATION) {
    return CalibrationModeEnum::FrictionEstimation;
  }
  if (calibrationModeName == ANYDRIVE_CALIB_MODE_NAME_SHORT_GEAR_AND_JOINT_ENC_HOMING) {
    return CalibrationModeEnum::GearAndJointEncoderHoming;
  }
  if (calibrationModeName == ANYDRIVE_CALIB_MODE_NAME_SHORT_GEAR_JOINT_ENC_OFFSET) {
    return CalibrationModeEnum::GearJointEncoderOffset;
  }
  if (calibrationModeName == ANYDRIVE_CALIB_MODE_NAME_SHORT_GRAVITY_COMPENSATION) {
    return CalibrationModeEnum::GravityCompensation;
  }
  if (calibrationModeName == ANYDRIVE_CALIB_MODE_NAME_SHORT_IMU_GYRO_DC_BIAS) {
    return CalibrationModeEnum::ImuGyroscopeDcBias;
  }
  if (calibrationModeName == ANYDRIVE_CALIB_MODE_NAME_SHORT_MOTOR_ENC_OFFSET) {
    return CalibrationModeEnum::MotorEncoderOffset;
  }
  if (calibrationModeName == ANYDRIVE_CALIB_MODE_NAME_SHORT_MOTOR_ENC_PARAMS) {
    return CalibrationModeEnum::MotorEncoderParameters;
  }
  if (calibrationModeName == ANYDRIVE_CALIB_MODE_NAME_SHORT_SAFE_JOINT_VEL) {
    return CalibrationModeEnum::SafeJointVelocity;
  }
  if (calibrationModeName == ANYDRIVE_CALIB_MODE_NAME_SHORT_SPRING_STIFFNESS) {
    return CalibrationModeEnum::SpringStiffness;
  }
  if (calibrationModeName == ANYDRIVE_CALIB_MODE_NAME_SHORT_AKSIM_SELF_CALIB) {
    return CalibrationModeEnum::AksimSelfCalibMode;
  }
  return CalibrationModeEnum::NA;
}

std::ostream& operator<<(std::ostream& out, const CalibrationModeEnum calibrationModeEnum) {
  return out << calibrationModeEnumToName(calibrationModeEnum);
}

}  // namespace calibration
}  // namespace anydrive
