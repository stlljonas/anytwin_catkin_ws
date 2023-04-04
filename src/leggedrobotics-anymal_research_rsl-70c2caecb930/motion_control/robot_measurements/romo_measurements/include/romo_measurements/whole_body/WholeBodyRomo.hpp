/*!
 * @file	  WholeBodyRomo.hpp
 * @author	Gabriel Hottiger
 * @date	  Nov, 2017
 */

#pragma once

// romo_measurements
#include "romo_measurements/whole_body/WholeBodyPropertiesRomo.hpp"

// loco
#include "loco/common/WholeBody.hpp"
#include "loco/common/WholeBodyProperties.hpp"

// romo
#include "romo/RobotModel.hpp"

namespace romo_measurements {

template <typename ConcreteDescription_, typename RobotState_>
class WholeBodyRomo : public loco::WholeBody
{
 protected:
  using PropertiesRomo          = romo_measurements::WholeBodyPropertiesRomo<ConcreteDescription_, RobotState_>;
  using RobotModel              = romo::RobotModel<ConcreteDescription_, RobotState_>;
  using RD                      = typename RobotModel::RD;
  using BranchEnum              = typename RD::BranchEnum;
  using CoordinateFrameEnum     = typename RD::CoordinateFrameEnum ;

 public:
  template<typename LegGroup_ = loco::Legs, typename ArmGroup_ = loco::Arms>
  WholeBodyRomo(const RobotModel & model,
                loco::TorsoBase& torso,
                loco::WholeBodyPropertiesPtr&& wholeBodyProperties,
                const LegGroup_& legs = LegGroup_(),
                const ArmGroup_& arms = ArmGroup_(),
                bool updateDynamics = false)
    : loco::WholeBody(torso, std::move(wholeBodyProperties), legs, arms, updateDynamics),
      model_(model)
  {

  }

  template<typename LegGroup_ = loco::Legs, typename ArmGroup_ = loco::Arms>
  WholeBodyRomo(const RobotModel & model,
                loco::TorsoBase& torso,
                const LegGroup_& legs = LegGroup_(),
                const ArmGroup_& arms = ArmGroup_(),
                bool updateDynamics = false)
    : WholeBodyRomo(model, torso, loco::WholeBodyPropertiesPtr(new PropertiesRomo(model)),
                    legs, arms, updateDynamics)
  {

  }

  ~WholeBodyRomo() override = default;

  bool initialize(double dt) override;
  bool advance(double dt) override;

 protected:
  const RobotModel& model_;
};

}  // namespace romo_measurements

#include "romo_measurements/whole_body/WholeBodyRomo.tpp"