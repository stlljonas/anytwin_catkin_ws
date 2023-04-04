#pragma once

// Base class
#include <localization_manager/frame_connectors/FrameConnector.hpp>

namespace localization_manager {

/**
 * @brief      Frame connector which uses the measurement as pose ext to base
 */
class DifferenceFrameConnector : public FrameConnector {
 public:
  using Base = FrameConnector;
  using typename Base::Pose;

  /* Constructor and destructor */
  DifferenceFrameConnector() : Base(false) {}
  ~DifferenceFrameConnector() final = default;

  /*!
   * @copydoc localization_manager::FrameConnector::update(const Pose& poseInOdom, const Pose& poseMeasurement, double time = 0.0)
   * @remark  Computes the external pose in odom as the difference between the robot pose in odom, and the robot pose in external.
   */
  Pose update(const Pose& poseInOdom, const Pose& poseMeasurement, double time = 0.0) final {
    // Compute pose difference between base pose in odom and ext.
    const auto rotationMeasInv = poseMeasurement.getRotation().inverted();
    const auto poseMeasurementInv = Pose{-rotationMeasInv.rotate(poseMeasurement.getPosition()), rotationMeasInv};
    return (poseInOdom * poseMeasurementInv);
  };
};

}  // namespace localization_manager