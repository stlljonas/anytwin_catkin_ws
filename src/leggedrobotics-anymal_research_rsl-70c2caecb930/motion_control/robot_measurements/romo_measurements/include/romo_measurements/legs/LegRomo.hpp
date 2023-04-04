/*!
 * @file    LegRomo.hpp
 * @author  Gabriel Hottiger
 * @date	  Nov, 2017
 */

#pragma once

// loco
#include "loco/common/legs/LegBase.hpp"
#include "loco/common/end_effectors/FootBase.hpp"

// romo_measurements
#include "romo_measurements/limbs/LimbRomo.hpp"
#include "romo_measurements/end_effectors/FootRomo.hpp"

namespace romo_measurements {

template <typename ConcreteDescription_, typename RobotState_, typename LimbBase_ = loco::LegBase>
class LegRomo: public romo_measurements::LimbRomo<ConcreteDescription_, RobotState_, LimbBase_> {

  static_assert(std::is_base_of<loco::LegBase, LimbBase_>::value,
                "[LegRomo]: LimbBase_ must derive from loco::LegBase");

 protected:
  using LimbRomo            = romo_measurements::LimbRomo<ConcreteDescription_, RobotState_, LimbBase_>;
  using RobotModel          = typename LimbRomo::RobotModel;
  using RD                  = typename RobotModel::RD;
  using LimbEnum            = typename RD::LimbEnum;

 public:
  using LimbRomo::LimbRomo;

  ~LegRomo() override = default;

  bool initialize(double dt) override;
  bool advance(double dt) override;
};

}  // namespace romo_measurements

#include "romo_measurements/legs/LegRomo.tpp"
