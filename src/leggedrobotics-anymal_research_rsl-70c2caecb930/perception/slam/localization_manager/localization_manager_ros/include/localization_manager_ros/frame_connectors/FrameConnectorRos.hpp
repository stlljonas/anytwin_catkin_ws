#pragma once

// std
#include <string>

// ros
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

// localization manager ros
#include <localization_manager_ros/frame_connectors/FrameConnectorRosBase.hpp>

// conversions
#include <any_measurements_ros/ConvertRosMessages.hpp>

namespace localization_manager_ros {

/**
 * @brief      ROS wrapper for a specific FrameConnectorType.
 */
template <typename FrameConnectorType>
class FrameConnectorRos : public FrameConnectorRosBase {
 public:
  using Base = FrameConnectorRosBase;
  using Base::ImplConfigBase;
  using Base::OdometryRos;
  using Base::OdometryRosPtr;
  using Base::Pose;
  using Base::PoseRos;
  using Base::PoseRosPtr;
  using Base::TransformRos;

  using ImplConfig = typename FrameConnectorType::Config;

  /* Constructor and destructor */
  FrameConnectorRos(ros::NodeHandle& nodeHandle, const std::string& connectorTypeNamespace, const std::string& name);
  ~FrameConnectorRos() final = default;

  /*!
   * @copydoc localization_manager_ros::FrameConnectorRosBase::getTfExtInOdom()
   */
  const TransformRos& getTfExtInOdom() const final;

  /*!
   * @copydoc localization_manager_ros::FrameConnectorRosBase::updatePoseInOdom(const PoseRosPtr& poseInOdom)
   */
  void updatePoseInOdom(const PoseRosPtr& poseInOdom) final;

  /*!
   * @copydoc localization_manager_ros::FrameConnectorRosBase::configureImpl(std::shared_ptr<ImplConfigBase> configPtr)
   */
  void configureImpl(std::shared_ptr<ImplConfigBase> configPtr) final;

  /*!
   * @copydoc localization_manager_ros::FrameConnectorRosBase::hasTfExtInOdom()
   */
  bool hasTfExtInOdom() final;

  /*!
   * @copydoc localization_manager_ros::FrameConnectorRosBase::hasToBroadcastZeroInitialTf()
   */
  bool hasToBroadcastZeroInitialTf() final;

  /*!
   * @copydoc localization_manager_ros::FrameConnectorRosBase::reset()
   */
  void reset();

 protected:
  struct Config {
    // Enum to define various input ros msg types.
    enum class InputMsgType : unsigned int { POSE_WITH_COVARIANCE_STAMPED = 0u, ODOMETRY = 1u };

    // Name that identifies the frame connector.
    std::string name_;
    // Frame IDs.
    std::string odomFrameId_{"odom"};
    std::string extFrameIdIn_{"ext_in"};
    std::string extFrameIdOut_{"ext_out"};
    std::string trackedFrameIdInOdom_{"base"};
    std::string trackedFrameIdInExt_{"base"};
    // Timeout for tf buffer lookups
    double tfLookupTimeout_{2.};
    // Timeout for publishing Tf's from a given external frame pose. If the timeout is triggered, the localization manager stops publishing
    // transforms until a new one arrives.
    double externalTransformTimeout_{5.};
    // Whether an identity transform should be broadcast to iniatialize dependent modules.
    bool broadcastZeroInitTf_ = false;
    // Type of the input ROS msg.
    InputMsgType inputMsgType_{InputMsgType::POSE_WITH_COVARIANCE_STAMPED};
  } config_;

  // Pose offset between the tracked frames.
  Pose poseOdomTrackInExtTrack_{Pose()};
  bool poseOdomTrackInExtTrackInited_{false};

  // Frame connector.
  FrameConnectorType frameConnector_{FrameConnectorType()};

  // Subscriber to the output of a localization/odometry module.
  ros::Subscriber measurementSubscriber_;

  // Service to reset the frame connector
  ros::ServiceServer resetServer_;

  /**
   * @brief Get the pose of the Odometry frame in the external frame.
   *
   */
  void getPoseOdomTrackInExtTrack();

  /**
   * @brief Process a ROS pose message.
   *
   * @param msg   The ROS pose message to process.
   */
  void processRosPoseMeasurement(const PoseRosPtr& msg);

  // ROS callbacks
  void poseMeasurementCallback(const PoseRosPtr& msg);
  void odometryMeasurementCallback(const OdometryRosPtr& msg);
  bool resetServiceCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
};

}  // namespace localization_manager_ros

#include <localization_manager_ros/frame_connectors/FrameConnectorRos.tpp>
