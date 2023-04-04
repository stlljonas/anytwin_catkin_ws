/**
 * @authors     Gabriel Hottiger
 * @affiliation ANYbotics
 * @brief       Ros conversions for geometry_utils common classes.
 */

#pragma once

#include <geometry_utils/PoseStamped.hpp>
#include <geometry_utils/TransformStamped.hpp>
#include <geometry_utils/TwistStamped.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>

namespace geometry_utils_ros {

geometry_utils::PoseStamped fromRos(const geometry_msgs::PoseStamped& poseStampedMsg);

geometry_utils::TwistStamped fromRos(const geometry_msgs::TwistStamped& twistStampedMsg);

geometry_utils::TransformStamped fromRos(const geometry_msgs::TransformStamped& transformStampedMsg);

geometry_msgs::PoseStamped toRos(const geometry_utils::PoseStamped& poseStamped);

geometry_msgs::TwistStamped toRos(const geometry_utils::TwistStamped& twistStamped);

geometry_msgs::TransformStamped toRos(const geometry_utils::TransformStamped& transformStamped);

}  // namespace geometry_utils_ros
