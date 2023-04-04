/*!
 * @file	 LegPropertiesRomo.hpp
 * @author Gabriel Hottiger
 * @date	 Nov, 2017
 */

#pragma once

// loco
#include "loco/common/legs/LegProperties.hpp"

// romo_measurements
#include "romo_measurements/limbs/LimbPropertiesRomo.hpp"

namespace romo_measurements {

template <typename ConcreteDescription_, typename RobotState_, typename LimbPropertiesBase_ = loco::LegProperties>
class LegPropertiesRomo : public romo_measurements::LimbPropertiesRomo<ConcreteDescription_, RobotState_, LimbPropertiesBase_> {

  static_assert(std::is_base_of<loco::LegProperties, LimbPropertiesBase_>::value,
                "[LegPropertiesRomo]: LimbPropertiesBase_ must derive from loco::LegProperties");

 public:
  //! Forward all constructors
  using LimbPropertiesRomo<ConcreteDescription_, RobotState_, LimbPropertiesBase_>::LimbPropertiesRomo;
};

}  // namespace romo_measurements