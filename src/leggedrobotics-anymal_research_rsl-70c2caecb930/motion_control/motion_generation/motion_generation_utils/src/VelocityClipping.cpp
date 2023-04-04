/**
 * @authors     Prajish Sankar
 * @affiliation ANYbotics
 * @date        Jun 29, 2020
 */

// motion generation utils
#include "motion_generation_utils/VelocityClipping.hpp"

// robot_utils
#include <robot_utils/math/math.hpp>

namespace motion_generation_utils {

void clipToBox(Eigen::Vector3d& desiredVelocity, const Eigen::Vector3d& velocityLimit, const Eigen::Vector2d& velocityLimitOffset) {
  using namespace std;
  // Clip the desired velocity to within the box.
  // Need to ensure that the velocity lower bound does not become positive and the upper bound does not become negative.
  robot_utils::boundToRange(&desiredVelocity.x(),
                            min(-abs(velocityLimit.x()) + velocityLimitOffset.x(), 0.0),
                            max(abs(velocityLimit.x()) + velocityLimitOffset.x(), 0.0));
  robot_utils::boundToRange(&desiredVelocity.y(),
                            min(-abs(velocityLimit.y()) + velocityLimitOffset.y(), 0.0),
                            max(abs(velocityLimit.y()) + velocityLimitOffset.y(), 0.0));
  robot_utils::boundToRange(&desiredVelocity.z(),
                            -abs(velocityLimit.z()),
                            abs(velocityLimit.z()));
}

void clipToEllipsoid(Eigen::Vector3d& desiredVelocity, const Eigen::Vector3d& velocityLimit, const Eigen::Vector2d& velocityLimitOffset) {
  using namespace std;
  const double& a = velocityLimit.x();
  const double& b = velocityLimit.y();
  const double& c = velocityLimit.z();

  // Azimuthal angle (in the xy plane)
  const double phi = atan2(desiredVelocity.y(), desiredVelocity.x());

  // Polar angle
  const double normXY = robot_utils::computeNorm(desiredVelocity.x(), desiredVelocity.y());
  const double theta = atan2(normXY, desiredVelocity.z());

  // Clip the desired velocity to within the ellipsoid.
  if (sin(theta) != 0.0) {
    /* 1. Need to ensure that the velocity lower bound does not become positive and the upper bound does not become negative.
     *
     * 2. The following formulae work only when sin(theta) != 0. Otherwise, the x,y velocities will be clipped so that they become equal to
     * that of velocityLimitOffset, even when the commanded velocities are zero. Therefore, for the case of zero commanded velocity, the
     * implementation is different (just clip the z velocity and let x.y component be as is, i.e. equal to 0.0).
     */
    robot_utils::boundToRange(&desiredVelocity.x(),
                              min(-abs(a * sin(theta) * cos(phi)) + velocityLimitOffset.x(), 0.0),
                              max(abs(a * sin(theta) * cos(phi)) + velocityLimitOffset.x(), 0.0));
    robot_utils::boundToRange(&desiredVelocity.y(),
                              min(-abs(b * sin(theta) * sin(phi)) + velocityLimitOffset.y(), 0.0),
                              max(abs(b * sin(theta) * sin(phi)) + velocityLimitOffset.y(), 0.0));
    robot_utils::boundToRange(&desiredVelocity.z(),
                              -abs(c * cos(theta)),
                              abs(c * cos(theta)));
  } else {
    // A separate implementation for the case of zero commanded velocity (normXY = 0 ==> sin(theta) = 0).
    robot_utils::boundToRange(&desiredVelocity.z(), -abs(c * cos(theta)), abs(c * cos(theta)));
  }
}

}  /* namespace motion_generation_utils */

