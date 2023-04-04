/*
 * CloudStream.hpp
 *
 *  Author: Yoshua Nava
 *  Organization: ANYbotics AG
 *  License: Proprietary
 */
#pragma once

// C++ standard library
#include <atomic>
#include <memory>
#include <mutex>

// ros
#include <ros/ros.h>

// pointmatcher_ros
#include <pointmatcher_ros/PointMatcherFilterInterface.h>
#include <pointmatcher_ros/StampedPointCloud.h>

// Self filtering
#include <point_cloud_self_filter/SelfFilter.hpp>

// point cloud processor
#include "point_cloud_processor/usings.hpp"

namespace point_cloud_processor {

/**
 * @brief CloudStream
 *  A CloudStream is a handler of point cloud data. It can apply different types of filters, which are
 *  specified using parameters.
 *  It doesn't run any thread of its own, but responds to external function calls.
 *  To guarantee thread-safety it has a mutex member that is intended to be locked by external users of
 *  the class.
 */
struct CloudStream {
  using Ptr = std::shared_ptr<CloudStream>;

  /*!
   * @brief Constructor. Loads default parameters.
   *
   */
  explicit CloudStream();

  /**
   * @brief Set the point cloud of the cloud stream. Resets flags set while processing previous data.
   *
   * @param pointCloud Input point cloud.
   */
  void setPointCloud(const PmStampedPointCloud& pointCloud);

  /**
   * @brief Get the processed point cloud.
   *
   * @return PmStampedPointCloud& Processed point cloud.
   */
  PmStampedPointCloud& getProcessedCloud();

  /**
   * @brief Processes the current point cloud.
   *
   * @remarks This method is thread-safe and checks if the point cloud has already been processed, to prevent extra CPU usage.
   */
  void processCloud();

  int id_;

  //! Name of source and sensor type (depth, lidar, radar, etc).
  std::string sourceName_;
  std::string sourceType_;

  //! ROS communication layer.
  uint32_t queueSize_;
  bool latch_;
  std::string inputTopic_;
  std::string outputTopic_;
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;

  //! Latest cloud mutex. Prevents data races when the stream's point cloud is accessed by multiple threads.
  std::mutex latestCloudMutex_;
  //! Point cloud data.
  PmStampedPointCloud latestCloud_;

  //! Filters.
  bool applyFilters_;
  std::atomic<bool> filtersAppliedToLatestData_;
  std::string filtersConfigFilename_;
  PmFilterIface pmFiltersIface_;

  //! Self Filtering
  bool applySelfFilter_;
  std::string selfFilterConfigFilename_;
  std::string robotDescriptionContents_;
  std::unique_ptr<point_cloud_self_filter::SelfFilter> selfFilter_;
};

}  // namespace point_cloud_processor
