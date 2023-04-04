/*
 * WholeBodyController.hpp
 *
 *  Created on: Jun 5, 2015
 *      Author: Dario Bellicoso
 */

#pragma once

// model
#include <anymal_model/AnymalModel.hpp>

// whole body control
#include <whole_body_control_romo/motion_control/WholeBodyController.hpp>

namespace loco_anymal {
using WholeBodyController = whole_body_control_romo::WholeBodyController<anymal_description::ConcreteAnymalDescription, anymal_model::AnymalState>;
} /* namespace loco_anymal */
