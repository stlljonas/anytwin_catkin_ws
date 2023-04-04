#pragma once

// pointmatcher_ros
#include <pointmatcher_ros/PmTf.h>
#include <pointmatcher_ros/StampedPointCloud.h>

// slam common
#include <slam_common/status_codes.hpp>

// slam common ros
#include <slam_common_ros/SlamInterface.hpp>

// slam point cloud mapping
#include <slam_point_cloud_mapping/PointCloudMapCache.hpp>

// kindr
#include <kindr/Core>

namespace icp_tools_common {

/* Kindr */
using HomTransformQuat = kindr::HomogeneousTransformationPosition3RotationQuaternionD;

/* PointMatcher_ros */
using PmIcp = PointMatcher_ros::PmIcp;
using PmMatches = PointMatcher_ros::PmMatches;
using PmMatrix = PointMatcher_ros::PmMatrix;
using PmStampedPointCloud = PointMatcher_ros::StampedPointCloud;
using PmTf = PointMatcher_ros::PmTf;
using PmTfParameters = PointMatcher_ros::PmTfParameters;

/* slam_common */
using OperationalStatusCode = slam_common::OperationalStatusCode;

/* slam_common_ros */
template <typename MapBuilderType>
using SlamInterface = slam_common_ros::SlamInterface<MapBuilderType>;

/* slam_point_cloud_mapping */
using PointCloudMapCache = slam_point_cloud_mapping::PointCloudMapCache;

}  // namespace icp_tools_common