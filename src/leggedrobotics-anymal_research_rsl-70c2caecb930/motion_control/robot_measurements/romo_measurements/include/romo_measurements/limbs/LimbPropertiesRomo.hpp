/*!
 * @file	 LimbPropertiesRomo.hpp
 * @author Gabriel Hottiger
 * @date	 Nov, 2017
 */

#pragma once

// loco
#include "loco/common/typedefs.hpp"
#include "loco/common/limbs/LimbProperties.hpp"

// romo
#include "romo/RobotModel.hpp"

namespace romo_measurements {

template <typename ConcreteDescription_, typename RobotState_, typename LimbPropertiesBase_ = loco::LimbProperties>
class LimbPropertiesRomo: public LimbPropertiesBase_ {

  static_assert(std::is_base_of<loco::LimbProperties, LimbPropertiesBase_>::value,
                "[LimbPropertiesRomo]: LimbPropertiesBase_ must derive from loco::LimbProperties");

 protected:
  using RobotModel          = romo::RobotModel<ConcreteDescription_, RobotState_>;
  using RD                  = typename RobotModel::RD;
  using LimbEnum            = typename RD::LimbEnum;
  using BranchEnum          = typename RD::BranchEnum;
  using BodyEnum            = typename RD::BodyEnum;
  using CoordinateFrameEnum = typename RD::CoordinateFrameEnum;

 public:
  LimbPropertiesRomo(LimbEnum limbEnum,
                     const RobotModel & model);

  ~LimbPropertiesRomo() override = default;

  bool initialize(double dt) override;
  bool advance(double dt) override;

  double getMaximumLimbExtension() const override;
  double getMinimumLimbExtension() const override;

 protected:
  const RobotModel & model_;
  const LimbEnum limbEnum_;
  const BranchEnum branchEnum_;
};

}  // namespace romo_measurements

#include "romo_measurements/limbs/LimbPropertiesRomo.tpp"