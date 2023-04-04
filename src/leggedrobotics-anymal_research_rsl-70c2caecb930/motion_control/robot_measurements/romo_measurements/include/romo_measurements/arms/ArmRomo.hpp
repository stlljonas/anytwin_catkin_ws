/*!
 * @file	 ArmRomo.hpp
 * @author Gabriel Hottiger
 * @date	 Nov, 2017
 */

#pragma once

// romo_measurements
#include "romo_measurements/limbs/LimbRomo.hpp"

// loco
#include "loco/common/arms/ArmBase.hpp"
#include "loco/common/typedefs.hpp"

namespace romo_measurements {

template <typename ConcreteDescription_, typename RobotState_, typename LimbBase_ = loco::ArmBase>
using ArmRomo = romo_measurements::LimbRomo<ConcreteDescription_, RobotState_, LimbBase_>;

}  // namespace romo_measurements