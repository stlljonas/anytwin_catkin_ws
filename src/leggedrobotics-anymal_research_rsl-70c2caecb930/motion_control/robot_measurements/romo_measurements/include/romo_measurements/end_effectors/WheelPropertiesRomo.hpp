/*!
 * @file	 WheelPropertiesRomo.hpp
 * @author Gabriel Hottiger
 * @date	 Nov, 2017
 */

#pragma once

// romo_measurements
#include "romo_measurements/end_effectors/EndEffectorPropertiesRomo.hpp"

// loco
#include "loco/common/end_effectors/WheelProperties.hpp"

// romo
#include "romo/RobotModel.hpp"

namespace romo_measurements {

template <typename ConcreteDescription_, typename RobotState_,
  typename EndEffectorPropertiesBase_ = loco::WheelProperties>
class WheelPropertiesRomo : public romo_measurements::EndEffectorPropertiesRomo<ConcreteDescription_,
                                                                        RobotState_,
                                                                        EndEffectorPropertiesBase_> {

  static_assert(std::is_base_of<loco::WheelProperties, EndEffectorPropertiesBase_>::value,
                "[WheelPropertiesRomo]: EndEffectorPropertiesBase_ must derive from loco::WheelProperties");

 protected:
  using EndEffectorPropertiesRomo = romo_measurements::EndEffectorPropertiesRomo<ConcreteDescription_,
                                                                         RobotState_,
                                                                         EndEffectorPropertiesBase_>;
  using RobotModel                = typename EndEffectorPropertiesRomo::RobotModel;
  using RD                        = typename EndEffectorPropertiesRomo::RD;
  using BodyEnum                  = typename RD::BodyEnum;

 public:
  WheelPropertiesRomo(BodyEnum body, const RobotModel& model)
    : EndEffectorPropertiesRomo(body, model)
  {

  }

  ~WheelPropertiesRomo() override = default;

  bool initialize(double dt) override {
    this->setDiameter(RD::getWheelDiameter(this->limbEnum_));
    return EndEffectorPropertiesRomo::initialize(dt);
  }

};

}  // namespace romo_measurements

