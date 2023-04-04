#pragma once

// kindr
#include <kindr/Core>

namespace localization_manager {

/**
 * @brief      Base class for frame connectors, which compute transforms between the odom frame and
 *             an external frame output by another localization/odometry module.
 */
class FrameConnector {
 public:
  using Pose = kindr::HomTransformQuatD;

  // Struct to be filled in the implementation with the frame connector configuration.
  struct Config {};

  /* Constructor and destructor */
  FrameConnector(bool isFeedthrough = false) : isFeedthrough_(isFeedthrough) {}
  FrameConnector(FrameConnector&& other) noexcept : isFeedthrough_(std::move(other.isFeedthrough_)) {}
  FrameConnector(const FrameConnector& other) = default;
  virtual ~FrameConnector() = default;

  // Delete assignment operators because they don't make sense here.
  FrameConnector& operator=(const FrameConnector& other) = delete;
  FrameConnector& operator=(FrameConnector&& other) noexcept = delete;

  /**
   * @brief Update the pose of the external frame in odometry at a given time.
   *
   * @param poseInOdom        Pose of the robot in the odometry frame.
   * @param poseMeasurement   Pose of the robot in the external frame.
   * @param time              Target time stamp. Default = 0s.
   * @return Pose             Pose of external frame in odom.
   */
  virtual Pose update(const Pose& poseInOdom, const Pose& poseMeasurement, double time = 0.0) = 0;

  /**
   * @brief Configure the frame connector.
   *
   * @param configPtr   Configuration parameters.
   */
  virtual bool configure(std::shared_ptr<Config> configPtr) { return false; }

  /**
   * @brief Reset the frame connector values.
   *
   * @return true
   * @return false
   */
  virtual bool reset() { return false; }

  /**
   * @brief Returns whether the frame connector is feedthrough.
   *
   * @return true  If the frame connector is feedthrough, false otherwise.
   */
  virtual inline bool isFeedthrough() const { return isFeedthrough_; }

 protected:
  // bool to indicate that this frame manager does not use a measurement
  const bool isFeedthrough_{false};
};

}  // namespace localization_manager
