/*!
 * @file	 EndEffectorPropertiesRomo.hpp
 * @author Gabriel Hottiger
 * @date	 Nov, 2017
 */

#pragma once

// loco
#include "loco/common/typedefs.hpp"
#include "loco/common/end_effectors/EndEffectorProperties.hpp"

// romo
#include "romo/RobotModel.hpp"

// stl
#include <type_traits>

namespace romo_measurements {

template <typename ConcreteDescription_, typename RobotState_,
  typename EndEffectorPropertiesBase_ = loco::EndEffectorProperties>
class EndEffectorPropertiesRomo : public EndEffectorPropertiesBase_ {

  static_assert(std::is_base_of<loco::EndEffectorProperties, EndEffectorPropertiesBase_>::value,
                "[EndEffectorPropertiesRomo]: EndEffectorPropertiesBase_ must derive from loco::EndEffectorProperties");

 protected:
  using RobotModel          = romo::RobotModel<ConcreteDescription_, RobotState_>;
  using RD                  = typename RobotModel::RD;
  using BodyEnum            = typename RD::BodyEnum;
  using LimbEnum            = typename RD::LimbEnum;
  using CoordinateFrameEnum = typename RD::CoordinateFrameEnum;

 public:
  EndEffectorPropertiesRomo(const BodyEnum bodyEnum, const RobotModel& model)
    : EndEffectorPropertiesBase_(),
      model_(model),
      bodyEnum_(bodyEnum),
      limbEnum_(RD::template mapEnums<LimbEnum>(bodyEnum)) {
  }

  // constructor to pass the number of contact constraints to EndEffectorProperties
  EndEffectorPropertiesRomo(const BodyEnum bodyEnum, const RobotModel& model, unsigned int numberOfContactConstraints)
      : EndEffectorPropertiesBase_(numberOfContactConstraints),
        model_(model),
        bodyEnum_(bodyEnum),
        limbEnum_(RD::template mapEnums<LimbEnum>(bodyEnum)) {
    }

  ~EndEffectorPropertiesRomo() override = default;

  bool initialize(double dt) override {
    return this->advance(dt);
  }

  bool advance(double dt) override {
    return true;
  }

 protected:
  const RobotModel& model_;
  const BodyEnum bodyEnum_;
  const LimbEnum limbEnum_;
};

}  // namespace romo_measurements
