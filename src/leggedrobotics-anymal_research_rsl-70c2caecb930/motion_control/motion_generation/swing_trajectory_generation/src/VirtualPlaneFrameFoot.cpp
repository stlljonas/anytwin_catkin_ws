/*
 * VirtualPlaneFrameFoot.hpp
 *
 *  Created on: Jan. 19, 2018
 *      Author: Fabian Jenelten
 */

#include "swing_trajectory_generation/VirtualPlaneFrameFoot.hpp"

// logger
#include "message_logger/message_logger.hpp"

namespace sto {

//bool VirtualPlaneFrameFoot::computeVirtualPlaneFrame(
//    const motion_generation::Position& positionWorldToPreviousStanceFootholdInWorldFrame,
//    const motion_generation::Position& positionWorldToDesiredFootholdInWorldFrame
//    ) {
//  /*
//   * Definition of the virtual plane frame:
//   *  > Center: Previous stance foot location.
//   *  > x axis aligned with the direction of the desired foothold.
//   *  > z axis aligned with local terrain.
//   */
//
//
//  const Eigen::Vector3d vectorPreviousToDesiredFoothold = (
//      positionWorldToDesiredFootholdInWorldFrame-
//      positionWorldToPreviousStanceFootholdInWorldFrame
//   ).toImplementation();
//
//  const double dist = vectorPreviousToDesiredFoothold.norm();
//
//  if (robot_utils::areNear(dist, 0.0, 1e-4)) {
//    MELO_WARN_STREAM("[VirtualPlaneFrameFoot::computeVirtualPlaneFrame] Footholds are close to each other! Use previous VPF");
//    return true;
//  }
//
//  // Set origin of the virtual plane frame.
//  poseVirtualPlaneToWorld_.getPosition() = positionWorldToPreviousStanceFootholdInWorldFrame;
//
//  const double pitch = std::asin(vectorPreviousToDesiredFoothold.z() / dist);
//  const double yaw = std::asin(vectorPreviousToDesiredFoothold.y() / dist);
//
//  poseVirtualPlaneToWorld_.getRotation() = motion_generation::RotationQuaternion(motion_generation::EulerAnglesZyx(yaw, pitch, 0.0).setUnique()).inverted();
//
//  return true;
//}

bool VirtualPlaneFrameFoot::computeVirtualPlaneFrame(
    const motion_generation::Position& positionWorldToPreviousStanceFootholdInWorldFrame,
    const motion_generation::RotationQuaternion& orientationWorldToControl
    ) {
  poseVirtualPlaneToWorld_.getPosition() = positionWorldToPreviousStanceFootholdInWorldFrame;
  poseVirtualPlaneToWorld_.getRotation() = orientationWorldToControl.inverted();
  return true;
}


} /* namespace sto */
