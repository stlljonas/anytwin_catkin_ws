/*
 * WholeBodyState.hpp
 *
 *  Created on: May 2, 2018
 *      Author: Gabriel Hottiger
 */

#pragma once

// wholebody_romo
#include "whole_body_control_romo/BaseTaskStateRomo.hpp"
#include "whole_body_control_romo/support_jacobian/SupportJacobianRomo.hpp"
// romo
#include <romo/RobotModel.hpp>

// loco
#include <loco/common/TerrainModelBase.hpp>
#include <loco/common/WholeBody.hpp>

namespace whole_body_control_romo {

template <typename ConcreteDescription_, typename RobotState_>
class WholeBodyStateRomo {
 protected:
  using SupportJacobian = SupportJacobianRomo<ConcreteDescription_, RobotState_>;
  using ContactFlags = typename SupportJacobian::ContactFlags;
  using RobotModel = romo::RobotModel<ConcreteDescription_, RobotState_>;
  using RD = typename RobotModel::RD;
  using BaseTaskState = BaseTaskStateRomo<ConcreteDescription_, RobotState_>;

 public:
  WholeBodyStateRomo(const RobotModel& model, const loco::WholeBody& wholeBody, const SupportJacobian& supportJacobian,
                     const loco::TerrainModelBase& terrain, const ContactFlags& contactFlags);

  const RobotModel& getRobotModel() const { return model_; }
  const loco::WholeBody& getWholeBody() const { return wholeBody_; }
  const SupportJacobian& getSupportJacobian() const { return supportJacobian_; }
  const loco::TerrainModelBase& getTerrain() const { return terrain_; }
  const ContactFlags& getContactFlags() const { return contactFlags_; }
  const BaseTaskState& getBaseTaskState() const { return basetaskState_; }

  //! Advance internal states
  bool advance(double dt);
  bool addVariablesToLog(const std::string& ns) const;

 protected:
  //! Robot model
  const RobotModel& model_;
  //! Whole body
  const loco::WholeBody& wholeBody_;
  //! Support Jacobian
  const SupportJacobian& supportJacobian_;
  //! Terrain
  const loco::TerrainModelBase& terrain_;
  //! Contact Flags
  const ContactFlags& contactFlags_;
  //! Base task state
  BaseTaskState basetaskState_;
};

}  // namespace whole_body_control_romo

#include "whole_body_control_romo/WholeBodyStateRomo.tpp"
