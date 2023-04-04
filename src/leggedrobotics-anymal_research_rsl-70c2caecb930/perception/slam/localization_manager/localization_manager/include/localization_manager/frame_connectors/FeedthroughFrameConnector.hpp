#pragma once

// Base class
#include <localization_manager/frame_connectors/FrameConnector.hpp>

namespace localization_manager {

/**
 * @brief      Frame connector which uses the measurement as pose odom to ext directly
 */
class FeedthroughFrameConnector : public FrameConnector {
 public:
  using Base = FrameConnector;
  using typename Base::Pose;

  /* Constructor and destructor */
  FeedthroughFrameConnector() : Base(true) {}
  ~FeedthroughFrameConnector() final = default;

  /*!
   * @copydoc localization_manager::FrameConnector::update(const Pose& poseInOdom, const Pose& poseMeasurement, double time = 0.0)
   */
  Pose update(const Pose& poseInOdom, const Pose& poseMeasurement, double time = 0.0) final {
    // Relay the measurement as pose of external in odom.
    return poseMeasurement;
  };
};

}  // namespace localization_manager
