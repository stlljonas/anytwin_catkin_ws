/**
 * @authors     Prajish Sankar
 * @affiliation ANYbotics
 * @date        Jun 29, 2020
 * @brief       Clipping commanded velocities before sending the velocities to the motion planner.
 */

#pragma once

// eigen
#include "Eigen/Core"

namespace motion_generation_utils {

/*! Clips the desired torso velocity within a cuboid (velocities along axes are independently ensured that they don't cross their limits).
 *
 * @param desiredVelocity         The torso velocity to be modified (or clipped).
 * @param velocityLimit           The velocity limits specified; here, they are the dimensions of the cuboid.
 * @param velocityLimitOffset     The amount by which both max and min velocities in x,y are increased (optional).
 *
 * @note The @p velocityLimitOffset could be used to reduce the maximum speed of the robot when going down on a slope, for instance.
 *
 * This is equivalent to clipping @p desiredVelocity to a cuboid of half-dimensions @p velocityLimit, centered around @p velocityLimitOffset.
 */
void clipToBox(Eigen::Vector3d& desiredVelocity,
               const Eigen::Vector3d& velocityLimit,
               const Eigen::Vector2d& velocityLimitOffset = Eigen::Vector2d::Zero());

/*! Clips the desired torso velocity within an ellipsoid (ensures resulting velocity magnitude does not get higher than max limits specified).
 *
 * @param desiredVelocity         The torso velocity to be modified (or clipped).
 * @param velocityLimit           The velocity limits specified; here, they are the semi-axes lengths of the ellipsoid.
 * @param velocityLimitOffset     The amount by which both max and min velocities in x,y are increased (optional).
 *
 * @note The @p velocityLimitOffset could be used to reduce the maximum speed of the robot when going down on a slope, for instance.
 *
 * This is equivalent to clipping @p desiredVelocity to an ellipsoid of half-dimensions @p velocityLimit, centered around @p velocityLimitOffset.
 */
void clipToEllipsoid(Eigen::Vector3d& desiredVelocity,
                     const Eigen::Vector3d& velocityLimit,
                     const Eigen::Vector2d& velocityLimitOffset = Eigen::Vector2d::Zero());

}  /* namespace motion_generation_utils */
