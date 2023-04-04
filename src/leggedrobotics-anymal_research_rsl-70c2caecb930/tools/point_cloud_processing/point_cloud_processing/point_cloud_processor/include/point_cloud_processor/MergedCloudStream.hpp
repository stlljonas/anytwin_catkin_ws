/*
 * MergedCloudStream.hpp
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

// pointmatcher_ros
#include <pointmatcher_ros/PointMatcherFilterInterface.h>
#include <pointmatcher_ros/StampedPointCloud.h>

// point cloud processor
#include "point_cloud_processor/usings.hpp"

namespace point_cloud_processor {

/**
 * @brief MergedCloudStream
 *  A MergedCloudStream is a struct that can receive references to multiple CloudStreams, and
 *  combine their point clouds into one.
 *  It can compensate for external motion between the point cloud capture times, and apply filters
 *  to already merged clouds.
 *  It doesn't run any thread of its own, but responds to external function calls.
 */
struct MergedCloudStream {
  using Ptr = std::shared_ptr<MergedCloudStream>;

  int id_;

  //! Name of merged stream.
  std::string streamName_;

  //! Target frame.
  std::string targetFrame_;

  //! Master id.
  int masterStreamId_;

  //! Timing parameters.
  double maxTimeOffset_;

  //! Tf.
  double waitTimeTf_;
  bool useFixedFrameTf_;
  std::string fixedFrameTf_;

  //! Descriptors.
  bool keepIntensityDescriptor_;

  //! ROS communication layer.
  bool latch_;
  std::string outputTopic_;
  ros::Publisher publisher_;

  //! Filtering.
  bool applyFilters_;
  PmFilterIface pmFiltersIface_;
  std::string filtersConfigFilename_;

  //! Decimation by skipping N input clouds.
  int cloudDecimation_;
  int cloudsSkippedCounter_;

  //! List of dependent cloud streams.
  std::vector<CloudStream::Ptr> cloudStreams_;

  /*!
   * @brief Constructor. Loads default parameters.
   *
   */
  explicit MergedCloudStream()
      : id_(-1),
        streamName_("merged_range"),
        targetFrame_(""),
        masterStreamId_(-1),
        maxTimeOffset_(0.5),
        waitTimeTf_(0.1),
        useFixedFrameTf_(false),
        fixedFrameTf_("odom"),
        keepIntensityDescriptor_(false),
        latch_(false),
        outputTopic_("/merged_range/points"),
        applyFilters_(false),
        filtersConfigFilename_(""),
        cloudDecimation_(1),
        cloudsSkippedCounter_(0) {}
};

}  // namespace point_cloud_processor
