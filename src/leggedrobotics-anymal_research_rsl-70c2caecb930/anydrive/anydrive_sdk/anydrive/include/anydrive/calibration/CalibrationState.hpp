#pragma once

#include <cstdint>

#include "anydrive/calibration/CalibrationModeEnum.hpp"

namespace anydrive {
namespace calibration {

/*!
 * Calibration state bits. Can be used to indicate e.g. which calibrations have been
 * done or which calibrations should be reset.
 * Note: The bit-ordering must be the same as in the firmware; motordrive_types.h
 * Note: The aksim self-calibration states are only supported in ANYdrive 3.
 */
struct CalibrationStateBits {
  uint16_t motorEncoderOffset_ : 1;
  uint16_t motorEncoderParameters_ : 1;
  uint16_t gearJointEncoderOffset_ : 1;
  uint16_t gearAndJointEncoderHoming_ : 1;
  uint16_t frictionEstimation_ : 1;
  uint16_t imuGyroscopeDcBias_ : 1;
  uint16_t springStiffness_ : 1;
  uint16_t aksimGear_ : 1;
  uint16_t aksimJoint_ : 1;
  uint16_t unused_ : 7;
};

//! Calibration state union.
union CalibrationState {
  //! Bitwise access.
  CalibrationStateBits single_;
  //! Complete access.
  uint16_t all_ = 0;

  /*!
   * Check if a calibration has been done.
   * @param calibrationModeEnum Calibration mode enumerator.
   * @return True, iff a calibration has been done.
   */
  bool getSingleCalibrationState(const CalibrationModeEnum calibrationModeEnum) const;

  /*!
   * Set the state of a single calibration.
   * @param calibrationModeEnum Calibration mode enumerator.
   * @param isCalibrated    The new state of the calibration.
   */
  void setSingleCalibrationState(const CalibrationModeEnum calibrationModeEnum, const bool isCalibrated);
};

}  // namespace calibration
}  // namespace anydrive
