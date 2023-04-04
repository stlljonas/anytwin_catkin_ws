/*
 * PointFootAnymal.hpp
 *
 *  Created on: Jan 10, 2017
 *      Author: Dario Bellicoso, Gabriel Hottiger
 */

#pragma once

// loco
#include <loco/common/typedefs.hpp>

// anymal model
#include "anymal_model/AnymalModel.hpp"

// loco anymal
#include <loco_anymal/typedefs.hpp>

namespace loco_anymal {

class PointFootAnymal : public FootRomo {
 public:
  using AD = anymal_description::AnymalDescription;

  // Use this constructor if you want to use the default end effector properties object.
  PointFootAnymal(ContactEnum contactEnum,
                     const std::string& name,
                     anymal_model::AnymalModel& anymalModel,
                     anymal_model::AnymalModel& anymalModelDesired);

  // Use this constructor to pass in an end effector properties object.
  PointFootAnymal(ContactEnum conatctEnum,
                     const std::string& name,
                     anymal_model::AnymalModel& anymalModel,
                     anymal_model::AnymalModel& anymalModelDesired,
                     loco::EndEffectorPropertiesPtr&& endEffectorProperties);

  ~PointFootAnymal() override = default;

  loco::Position getPositionWorldToEndEffectorInWorldFrame(const JointPositions& jointPositions) override;
  loco::Position getPositionWorldToEndEffectorInBaseFrame(const JointPositions& jointPositions) override;
  loco::Position getPositionBaseToEndEffectorInBaseFrame(const JointPositions& jointPositions) override;

  JointPositions getJointPositionsFromPositionBaseToEndEffectorInBaseFrameIteratively(const loco::Position& positionBaseToEndEffectorInBaseFrame) override;
  JointPositions getJointPositionsFromPositionBaseToEndEffectorInBaseFrame(const loco::Position& positionBaseToEndEffectorInBaseFrame) override;
  JointVelocities getJointVelocitiesFromLinearVelocityBaseToEndEffectorInBaseFrame(const loco::LinearVelocity& velocity) override;

  /*! Computes the 3xDOF Jacobian matrix, given a set of joint angles of the leg
   *
   * @param jointPositions The joint angles of the leg in consideration
   * @return The Jacobian of size 3xN, where N = size of generalized velocity vector 'u'
   */
  Eigen::MatrixXd getTranslationJacobianBaseToEndEffectorInBaseFrameFromJointAngles(const JointPositions& jointPositions) override;

 protected:
  anymal_model::AnymalModel& anymalModel_;
  anymal_model::AnymalModel& anymalModelDesired_;
};

using PointFootAnymalPtr = std::unique_ptr<PointFootAnymal>;

} /* namespace loco_anymal */


