#pragma once

#include "anydrive/calibration/parameter/FrictionEstimation.hpp"
#include "anydrive/calibration/parameter/GearAndJointEncoderHoming.hpp"
#include "anydrive/calibration/parameter/GearJointEncoderOffset.hpp"
#include "anydrive/calibration/parameter/ImuGyroscopeDcBias.hpp"
#include "anydrive/calibration/parameter/MotorEncoderOffset.hpp"
#include "anydrive/calibration/parameter/MotorEncoderParameters.hpp"
#include "anydrive/calibration/parameter/SpringStiffness.hpp"

namespace anydrive {
namespace calibration {
namespace parameter {

/*!
 * Calibration class containing all calibration parameters.
 */
struct Calibration {
  //! Motor encoder offset calibration parameters.
  MotorEncoderOffset motorEncoderOffset_;
  //! Motor encoder parameters calibration parameters.
  MotorEncoderParameters motorEncoderParameters_;
  //! Gear/joint encoder offset calibration parameters.
  GearJointEncoderOffset gearJointEncoderOffset_;
  //! Gear and joint encoder calibration parameters.
  GearAndJointEncoderHoming gearAndJointEncoderHoming_;
  //! IMU gyroscope DC bias calibration parameters.
  ImuGyroscopeDcBias imuGyroscopeDcBias_;
  //! Spring stiffness calibration parameters.
  SpringStiffness springStiffness_;
  //! Friction estimation calibration parameters.
  FrictionEstimation frictionEstimation_;

  /*!
   * Comparison operator.
   * @param other Other calibration parameters.
   * @return True if equal.
   */
  bool operator==(const Calibration& other) const;
};

}  // namespace parameter
}  // namespace calibration
}  // namespace anydrive
