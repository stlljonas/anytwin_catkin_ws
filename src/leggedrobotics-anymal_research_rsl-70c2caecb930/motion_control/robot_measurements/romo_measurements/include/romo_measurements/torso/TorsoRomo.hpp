/*!
 * @file	  TorsoRomo.hpp
 * @author	Gabriel Hottiger
 * @date	  Nov, 2017
 */

#pragma once

//romo_measurements
#include "romo_measurements/torso/TorsoPropertiesRomo.hpp"

// loco
#include "loco/common/torso/TorsoBase.hpp"
#include "loco/common/typedefs.hpp"

// romo
#include "romo/RobotModel.hpp"

namespace romo_measurements {

template <typename ConcreteDescription_, typename RobotState_>
class TorsoRomo : public loco::TorsoBase {

 protected:
  using RobotModel             = romo::RobotModel<ConcreteDescription_, RobotState_>;
  using RD                     = typename RobotModel::RD;
  using BodyEnum               = typename RD::BodyEnum;
  using CoordinateFrameEnum    = typename RD::CoordinateFrameEnum ;

 public:
  TorsoRomo(const std::string& name, const RobotModel& model);
  TorsoRomo(const std::string& name, const RobotModel& model, loco::TorsoPropertiesPtr&& properties);

  ~TorsoRomo() override = default;

  bool initialize(double dt) override;

  // Todo who advances members stridePhase_ / strideDuration_ ?
  bool advance(double dt) override;

 protected:
  const RobotModel& model_;

};

}  // namespace romo_measurements

#include "romo_measurements/torso/TorsoRomo.tpp"