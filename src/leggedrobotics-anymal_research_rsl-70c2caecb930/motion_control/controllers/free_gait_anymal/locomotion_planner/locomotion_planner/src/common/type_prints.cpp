/*
 * type_prints.cpp
 *
 *  Created on: Mar 6, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include "locomotion_planner/common/type_prints.hpp"

namespace locomotion_planner {

std::ostream& operator<< (std::ostream& out, const PlanarPose& pose)
{
  out << "[x: " << pose.x()
      << ", y: " << pose.y()
      << ", yaw: " << pose.z() << "]";
  return out;
}

} // namespace
