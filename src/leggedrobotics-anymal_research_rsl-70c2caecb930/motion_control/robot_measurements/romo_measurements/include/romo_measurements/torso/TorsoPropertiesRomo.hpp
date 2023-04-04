/*!
 * @file	 TorsoPropertiesRomo.hpp
 * @author Gabriel Hottiger
 * @date	 Nov, 2017
 */

#pragma once

// loco
#include "loco/common/typedefs.hpp"
#include "loco/common/torso/TorsoProperties.hpp"

// romo
#include "romo/RobotModel.hpp"

namespace romo_measurements {

template <typename ConcreteDescription_, typename RobotState_>
class TorsoPropertiesRomo : public loco::TorsoProperties
{
 protected:
  using RobotModel          = romo::RobotModel<ConcreteDescription_, RobotState_>;
  using RD                  = typename RobotModel::RD;
  using BodyEnum            = typename RD::BodyEnum;
  using CoordinateFrameEnum = typename RD::CoordinateFrameEnum ;

 public:
  explicit TorsoPropertiesRomo(const RobotModel& model)
    : loco::TorsoProperties(),
      model_(model)
  {
    setHeadingAxisInBaseFrame( loco::Vector(1.0, 0.0, 0.0));
    setLateralAxisInBaseFrame( loco::Vector(0.0, 1.0, 0.0));
    setVerticalAxisInBaseFrame(loco::Vector(0.0, 0.0, 1.0));
  }

  ~TorsoPropertiesRomo() override = default;

  bool initialize(double dt) override {
    setMass(model_.getRootMass());
    setGravity(loco::LinearAcceleration(model_.getGravityVectorInWorldFrame()));
    setInertiaTensorInBaseFrame(model_.getBodyInertiaMatrix(BodyEnum::BASE));
    return this->advance(dt);
  }

  // Todo who advances member maximumBaseTwistInControlFrame_?
  bool advance(double /* dt */) override {
    setBaseToCenterOfMassPositionInBaseFrame( loco::Position(
      model_.getPositionBodyToBodyCom( BodyEnum::BASE, BodyEnum::BASE, CoordinateFrameEnum::BASE) ) );
    return true;
  }

 protected:
  const RobotModel& model_;
};

}  // namespace romo_measurements