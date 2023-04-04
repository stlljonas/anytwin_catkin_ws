/*
 * usings.hpp
 *
 *  Author: Yoshua Nava
 *  Organization: ANYbotics AG
 *  License: Proprietary
 */
#pragma once

// pointmatcher_ros
#include <pointmatcher_ros/PmTf.h>
#include <pointmatcher_ros/PointMatcherFilterInterface.h>
#include <pointmatcher_ros/StampedPointCloud.h>
#include <pointmatcher_ros/usings.h>

namespace point_cloud_processor {

/* PointMatcher_ros */
using PmFilterIface = PointMatcher_ros::PointMatcherFilterInterface;
using PmStampedPointCloud = PointMatcher_ros::StampedPointCloud;
using PmTf = PointMatcher_ros::PmTf;

}  // namespace point_cloud_processor