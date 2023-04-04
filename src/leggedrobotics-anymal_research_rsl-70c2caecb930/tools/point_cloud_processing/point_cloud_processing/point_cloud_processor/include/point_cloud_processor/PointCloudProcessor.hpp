/*
 * PointCloudProcessor.hpp
 *
 *  Author: Yoshua Nava
 *  Organization: ANYbotics AG
 *  License: Proprietary
 */
#pragma once

// C++ standard library
#include <memory>

// ros
#include <ros/ros.h>

// msgs and srvs definitions
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>

// tf2
#include <tf2/transform_datatypes.h>

// tf2 ros
#include <tf2_ros/transform_listener.h>

// pointmatcher_ros
#include <pointmatcher_ros/StampedPointCloud.h>

// message logger
#include <message_logger/message_logger.hpp>

// point cloud processor
#include "point_cloud_processor/CloudStream.hpp"
#include "point_cloud_processor/MergedCloudStream.hpp"
#include "point_cloud_processor/usings.hpp"

namespace point_cloud_processor {

/**
 * @brief PointCloudProcessor
 *  The PointCloudProcessor is a ROS node that manages point cloud filtering operations.
 *  It can receive data from different sensors, apply filters and combine it with data
 *  from other sensors in real-time.
 *  The threading model of this class is event-based, with ROS callbacks triggering
 *  processing events. Additionally, the node periodically checks that each output topic
 *  is requested by an external node, to only process data that is required by other nodes.
 */
class PointCloudProcessor {
 public:
  /*!
   * Constructor.
   */
  explicit PointCloudProcessor(ros::NodeHandle& nodeHandle);

  /*!
   * Default Constructor.
   */
  PointCloudProcessor();

  /*!
   * Default Destructor.
   */
  ~PointCloudProcessor() = default;

  /*
   * Load parameters and start ROS communication layer.
   */
  void initialize();

  /*!
   * Reads and verifies the ROS parameters.
   * @return True   if successful.
   */
  bool getParameters();

  /*!
   * Transforms cloud to merged stream target frame
   * @param mergedStream    Merged stream
   * @param targetStamp     Target timestamp
   * @param pmStampedCloud  Cloud to transform, PmStampedCloud type
   * @return True           if successful. False otherwise
   */
  bool transformCloudToMergedFrame(const MergedCloudStream* mergedStream, const ros::Time& targetStamp,
                                   PmStampedPointCloud& pmStampedCloud);

  /*!
   * Combine cloud streams and publish.
   * @param mergedStreamId  The ID of the merged stream cloud to generate.
   */
  void generateAndPublishMergedCloud(const int& mergedStreamId);

  /*!
   * Generic cloud topic callback
   * @param msgInputCloud Input point cloud cloud.
   * @param idStream      ID of the cloud stream.
   */
  void cloudTopicCallback(const sensor_msgs::PointCloud2::ConstPtr& msgInputCloud, const size_t idStream);

  /*!
   * Reload pipeline configuration.
   * @return True   if successful. False otherwise.
   */
  bool reloadPipelineServiceCallback(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response);

  /*!
   * Periodically checks whether output topics have subscribers, to enable/disable subscriptions to sensor data.
   * @param timerEvent  ROS timer event.
   */
  void externalSubscribersCheckCallback(const ros::TimerEvent& timerEvent);

 protected:
  //! ROS node handle.
  ros::NodeHandle nodeHandle_;

  //! TF
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;

  //! Timer for checking if there are subscribers for all topics.
  double externalSubscribersCheckPeriod_;
  ros::Timer externalSubscribersCheckTimer_;

  //! Reset service for reloading the config files
  ros::ServiceServer resetService_;

  //! Cloud streams
  std::string configFileFolder_;
  std::vector<CloudStream::Ptr> cloudStreams_;
  std::vector<MergedCloudStream::Ptr> mergedCloudStreams_;
};

}  // namespace point_cloud_processor
