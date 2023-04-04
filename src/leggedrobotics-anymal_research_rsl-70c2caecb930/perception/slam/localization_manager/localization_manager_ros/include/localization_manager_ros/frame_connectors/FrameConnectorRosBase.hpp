#pragma once

// std
#include <mutex>
#include <string>

// geometry msgs
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>

// nav msgs
#include <nav_msgs/Odometry.h>

// tf
#include <tf/transform_datatypes.h>

// kindr
#include <kindr/Core>

// kindr ros
#include <kindr_ros/kindr_ros.hpp>

// localization manager ros
#include <localization_manager_ros/frame_connectors/PoseBuffer.hpp>

namespace localization_manager_ros {

/**
 * @brief      Base class for ros wrappers for the frame connectors
 */
class FrameConnectorRosBase {
 public:
  using Pose = kindr::HomTransformQuatD;
  using PoseRos = geometry_msgs::PoseWithCovarianceStamped;
  using PoseRosPtr = geometry_msgs::PoseWithCovarianceStampedPtr;
  using OdometryRos = nav_msgs::Odometry;
  using OdometryRosPtr = nav_msgs::OdometryPtr;
  using TransformRos = geometry_msgs::TransformStamped;

  using ImplConfigBase = localization_manager::FrameConnector::Config;

  /* Constructor and destructor */
  FrameConnectorRosBase() = default;
  virtual ~FrameConnectorRosBase() = default;

  /**
   * @brief Get a ROS transform message representing the pose of the external frame in odom.
   *
   * @return const TransformRos&  Output ROS transform message.
   */
  virtual const TransformRos& getTfExtInOdom() const = 0;

  /**
   * @brief Update the external frame pose in odom.
   *
   * @param poseRos   Pose of the external frame in odom.
   */
  virtual void updatePoseInOdom(const PoseRosPtr& poseRos) = 0;

  /**
   * @brief Configure the frame connector.
   *
   * @param configPtr   Configuration parameters.
   */
  virtual void configureImpl(std::shared_ptr<ImplConfigBase> configPtr) = 0;

  /**
   * @brief Checks whether the frame connector has received any external frame poses in odom from localization.
   *
   * @return true   If there are localization poses in the buffer, false otherwise.
   */
  virtual bool hasTfExtInOdom() = 0;

  /**
   * @brief Checks whether the frame connector is configure to output an identity transform to the ROS server to bootstrap localization.
   *
   * @return true   If an identity transform is configured as output, false otherwise.
   */
  virtual bool hasToBroadcastZeroInitialTf() = 0;

 protected:
  /**
   * @brief Converts a pose stored in a Kindr::HomTransformQuatD into a ROS geometry_msgs::TransformStamped message.
   *
   * @param pose          Kindr homogeneous transformation representing the pose.
   * @param transformRos  Output transform message.
   */
  void poseToTransformRos(const Pose& pose, TransformRos& transformRos) {
    const auto frameId = transformRos.header.frame_id;
    const auto childFrameId = transformRos.child_frame_id;
    auto transformTf = tf::StampedTransform();
    kindr_ros::convertToRosTf(pose, transformTf);
    tf::transformStampedTFToMsg(transformTf, transformRos);
    transformRos.header.frame_id = frameId;
    transformRos.child_frame_id = childFrameId;
  }

  /**
   * @brief Convert a ROS geometry_msgs::TransformStamped message into a pose stored in a Kindr::HomTransformQuatD.
   *
   * @param transformRos  ROS transform message.
   * @param pose          Output pose.
   */
  void transformRosToPose(const TransformRos& transformRos, Pose& pose) {
    auto transformTf = tf::Transform();
    tf::transformMsgToTF(transformRos.transform, transformTf);
    kindr_ros::convertFromRosTf(transformTf, pose);
  }

  // Mutex for all frame connector resources.
  mutable std::mutex mutexFrameConnectorRos_;

  // Tf to be published.
  TransformRos extInOdomTf_{TransformRos()};

  // Buffer to keep track of received poses in odom.
  std::unique_ptr<PoseBuffer> poseInOdomBufferPtr_;
};

}  // namespace localization_manager_ros