

/*
 * type_prints.hpp
 *
 *  Created on: Mar 6, 2017
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#pragma once

#include "locomotion_planner/common/type_defs.hpp"

#include <iostream>

namespace locomotion_planner {

std::ostream& operator<< (std::ostream& out, const PlanarPose& pose);

} // namespace
