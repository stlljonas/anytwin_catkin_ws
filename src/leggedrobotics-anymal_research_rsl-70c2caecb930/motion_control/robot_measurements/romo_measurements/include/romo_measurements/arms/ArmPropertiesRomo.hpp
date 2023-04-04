/*!
 * @file	 ArmPropertiesRomo.hpp
 * @author Koen Krämer
 * @date	 Mar, 2018
 */

#pragma once

// loco
#include "loco/common/arms/ArmProperties.hpp"

// romo_measurements
#include "romo_measurements/limbs/LimbPropertiesRomo.hpp"

namespace romo_measurements {

template <typename ConcreteDescription_, typename RobotState_, typename LimbPropertiesBase_ = loco::ArmProperties>
class ArmPropertiesRomo : public romo_measurements::LimbPropertiesRomo<ConcreteDescription_, RobotState_, LimbPropertiesBase_> {

  static_assert(std::is_base_of<loco::ArmProperties, LimbPropertiesBase_>::value,
                "[ArmPropertiesRomo]: LimbPropertiesBase_ must derive from loco::ArmProperties");

 public:
  //! Forward all constructors
  using LimbPropertiesRomo<ConcreteDescription_, RobotState_, LimbPropertiesBase_>::LimbPropertiesRomo;
};

}  // namespace romo_measurements
