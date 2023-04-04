#pragma once

// pointmatcher_ros
#include <pointmatcher_ros/PmTf.h>
#include <pointmatcher_ros/StampedPointCloud.h>

// slam common
#include <slam_common/status_codes.hpp>

// slam point cloud mapping
#include <slam_point_cloud_mapping/PointCloudMapBuilderBase.hpp>

namespace icp_tools_global_mapping {

/* PointMatcher_ros */
using PmMatrix = PointMatcher_ros::PmMatrix;
using PmStampedPointCloud = PointMatcher_ros::StampedPointCloud;
using PmTf = PointMatcher_ros::PmTf;

/* slam_common */
using OperationalStatusCode = slam_common::OperationalStatusCode;

/* slam_point_cloud_mapping */
using PointCloudMapBuilderBase = slam_point_cloud_mapping::PointCloudMapBuilderBase;

}  // namespace icp_tools_global_mapping