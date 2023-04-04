/*!
 * @file    ArmRomo.tpp
 * @author  Koen Krämer
 * @date	  Mar, 2018
 */

#pragma once

// romo_measurements
#include "romo_measurements/arms/ArmRomo.hpp"

namespace romo_measurements {

// stateLiftoff_, stateTouchdown_?
template <typename ConcreteDescription_, typename RobotState_, typename LimbBase_>
bool ArmRomo<ConcreteDescription_, RobotState_, LimbBase_>::initialize(double dt) {


  // Advance arm
  if( !LimbRomo::initialize(dt) ) {
    MELO_WARN_STREAM("[ArmRomo]: Arm " << std::string( RD::mapKeyEnumToKeyName(this->limbEnum_) )
                                       << " could not initialize LimbRomo!");
    return false;
  }

  return true;
}

template <typename ConcreteDescription_, typename RobotState_, typename LimbBase_>
bool ArmRomo<ConcreteDescription_, RobotState_, LimbBase_>::advance(double dt) {
  if( !LimbRomo::advance(dt) ) {
    MELO_WARN_STREAM("[ArmRomo]: Arm " << std::string( RD::mapKeyEnumToKeyName(this->limbEnum_) )
                                       << " could not advance LimbRomo!");
    return false;
  }

  return true;
}

}  // namespace romo_measurements
