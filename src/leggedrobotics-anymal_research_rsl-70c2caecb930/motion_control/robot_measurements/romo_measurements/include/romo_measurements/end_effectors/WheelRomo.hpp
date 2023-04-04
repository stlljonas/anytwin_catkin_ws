/*!
 * @file	 WheelRomo.hpp
 * @author Gabriel Hottiger
 * @date	 Nov, 2017
 */

#pragma once

// romo_measurements
#include "romo_measurements/end_effectors/FootRomo.hpp"

// loco
#include "loco/common/end_effectors/Wheel.hpp"

namespace romo_measurements {

template <typename ConcreteDescription_, typename RobotState_, typename EndeffectorBase_ = loco::Wheel>
using WheelRomo = romo_measurements::FootRomo<ConcreteDescription_, RobotState_, EndeffectorBase_>;

}  // namespace romo_measurements