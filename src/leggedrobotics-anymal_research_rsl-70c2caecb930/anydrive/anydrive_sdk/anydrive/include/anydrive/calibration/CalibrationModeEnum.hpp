#pragma once

#include <cstdint>
#include <string>

namespace anydrive {
namespace calibration {

//! Calibration mode short names.
#define ANYDRIVE_CALIB_MODE_NAME_SHORT_FRICTION_ESTIMATION ("FricEst")
#define ANYDRIVE_CALIB_MODE_NAME_SHORT_GEAR_AND_JOINT_ENC_HOMING ("GJEHoming")
#define ANYDRIVE_CALIB_MODE_NAME_SHORT_GEAR_JOINT_ENC_OFFSET ("GJEOffset")
#define ANYDRIVE_CALIB_MODE_NAME_SHORT_GRAVITY_COMPENSATION ("GravComp")
#define ANYDRIVE_CALIB_MODE_NAME_SHORT_IMU_GYRO_DC_BIAS ("GyroBias")
#define ANYDRIVE_CALIB_MODE_NAME_SHORT_MOTOR_ENC_OFFSET ("MEOffset")
#define ANYDRIVE_CALIB_MODE_NAME_SHORT_MOTOR_ENC_PARAMS ("MEParams")
#define ANYDRIVE_CALIB_MODE_NAME_SHORT_NA ("N/A")
#define ANYDRIVE_CALIB_MODE_NAME_SHORT_SAFE_JOINT_VEL ("SafeJVel")
#define ANYDRIVE_CALIB_MODE_NAME_SHORT_SPRING_STIFFNESS ("SprStiff")
#define ANYDRIVE_CALIB_MODE_NAME_SHORT_AKSIM_SELF_CALIB ("AksimSelfCal")

//! Calibration mode enumerators for type safe usage.
enum class CalibrationModeEnum {
  FrictionEstimation,
  GearAndJointEncoderHoming,
  GearJointEncoderOffset,
  GravityCompensation,
  ImuGyroscopeDcBias,
  MotorEncoderOffset,
  MotorEncoderParameters,
  NA,
  SafeJointVelocity,
  SpringStiffness,
  AksimSelfCalibMode,   // For initializing the overall calibration
  AksimGearCalibState,  // For reading the gear-Aksim's calibration state
  AksimJointCalibState  // For reading the joint-Aksim's calibration state
};

/*!
 * Convert a calibration mode enumerator to an ID.
 * @param calibrationModeEnum Calibration mode enumerator.
 * @return Calibration mode ID.
 */
uint16_t calibrationModeEnumToId(const CalibrationModeEnum calibrationModeEnum);

/*!
 * Convert a calibration mode ID to an enumerator.
 * @param calibrationModeId Calibration mode ID.
 * @return Calibration mode enumerator.
 */
CalibrationModeEnum calibrationModeIdToEnum(const uint16_t calibrationModeId);

/*!
 * Convert a calibration mode enumerator to a human readable string (GUI, etc.).
 * @param calibrationModeEnum Calibration mode enumerator.
 * @return Human readable string.
 */
std::string calibrationModeEnumToName(const CalibrationModeEnum calibrationModeEnum);

/*!
 * Convert a human readable string (GUI, etc.) to a calibration mode enumerator.
 * @param calibrationModeName Human readable string.
 * @return Calibration mode enumerator.
 */
CalibrationModeEnum calibrationModeNameToEnum(const std::string& calibrationModeName);

std::ostream& operator<<(std::ostream& out, const CalibrationModeEnum calibrationModeEnum);

}  // namespace calibration
}  // namespace anydrive
