/*
 * EndEffectorStateMeasured.hpp
 *
 *  Created on: Dec 5, 2016
 *      Author: Dario Bellicoso
 */

#pragma once

// loco
#include <loco/common/end_effectors/EndEffectorStateBase.hpp>
#include <loco/common/joints/MeasuredJointStates.hpp>
#include <loco/common/typedefs.hpp>

// STL
#include <memory>

namespace loco {

class EndEffectorStateMeasured : public EndEffectorStateBase {
 public:
  EndEffectorStateMeasured();
  ~EndEffectorStateMeasured() override = default;

  void setMeasuredJointStatesLimb(MeasuredJointStates* const measuredJointStatesLimb);

  void setPositionBaseToEndEffectorInBaseFrame(const Position& positionBaseToEndEffectorInBaseFrame);
  const Position& getPositionBaseToEndEffectorInBaseFrame() const;

  const TranslationJacobianLimb& getTranslationJacobianBaseToEndEffectorInBaseFrame() const;
  void setTranslationJacobianBaseToEndEffectorInBaseFrame(const TranslationJacobianLimb& translationJacobianBaseToEndEffectorInBaseFrame);

  const TranslationJacobian& getTranslationJacobianWorldToEndEffectorInWorldFrame() const;
  void setTranslationJacobianWorldToEndEffectorInWorldFrame(const TranslationJacobian& translationJacobianWorldToEndEffectorInWorldFrame);

  const RotationJacobianLimb& getRotationJacobianBaseToEndEffectorInBaseFrame() const;
  void setRotationJacobianBaseToEndEffectorInBaseFrame(const RotationJacobianLimb& rotationJacobianBaseToEndEffectorInBaseFrame);

  const RotationJacobian& getRotationJacobianWorldToEndEffectorInWorldFrame() const;
  void setRotationJacobianWorldToEndEffectorInWorldFrame(const RotationJacobian& rotationJacobianWorldToEndEffectorInWorldFrame);

  const TranslationJacobianLimb& getTranslationJacobianTimeDerivativeBaseToEndEffectorInBaseFrame() const;
  void setTranslationJacobianTimeDerivativeBaseToEndEffectorInBaseFrame(
      const TranslationJacobianLimb& translationJacobianTimeDerivativeBaseToEndEffectorInBaseFrame);

  const TranslationJacobian& getTranslationJacobianTimeDerivativeWorldToEndEffectorInWorldFrame() const;
  void setTranslationJacobianTimeDerivativeWorldToEndEffectorInWorldFrame(
      const TranslationJacobian& translationJacobianTimeDerivativeWorldToEndEffectorInWorldFrame);

  const RotationJacobianLimb& getRotationJacobianTimeDerivativeBaseToEndEffectorInBaseFrame() const;
  void setRotationJacobianTimeDerivativeBaseToEndEffectorInBaseFrame(
      const RotationJacobianLimb& rotationJacobianTimeDerivativeBaseToEndEffectorInBaseFrame);

  const RotationJacobian& getRotationJacobianTimeDerivativeWorldToEndEffectorInWorldFrame() const;
  void setRotationJacobianTimeDerivativeWorldToEndEffectorInWorldFrame(
      const RotationJacobian& rotationJacobianTimeDerivativeWorldToEndEffectorInWorldFrame);

 protected:
  //! Measured joint states
  MeasuredJointStates* measuredJointStatesLimb_;

  /*! The translation jacobian from base to end effector in base frame is a matrix J which projects
   *  the limb joint velocities to the linear velocity of the limb end-effector in base frame
   *  relative to the base linear velocity.
   */
  TranslationJacobianLimb translationJacobianBaseToEndEffectorInBaseFrame_;

  /*! The translation jacobian from world to endeffector in world frame is a matrix J which projects
   *  the whole-body joint velocities to the absolute linear velocity of the limb end-effector.
   */
  TranslationJacobian translationJacobianWorldToEndEffectorInWorldFrame_;

  /*! The rotation jacobian from base to end effector in base frame is a matrix J which projects
   *  the limb joint velocities to the angular velocity of the limb end-effector in base frame
   *  relative to the base angular velocity.
   */
  RotationJacobianLimb rotationJacobianBaseToEndEffectorInBaseFrame_;

  /*! The rotation jacobian from world to endeffector in world frame is a matrix J which projects
   *  the whole-body joint velocities to the absolute angular velocity of the limb end-effector.
   */
  RotationJacobian rotationJacobianWorldToEndEffectorInWorldFrame_;

  // Time Derivative Jacobians
  TranslationJacobianLimb translationJacobianTimeDerivativeBaseToEndEffectorInBaseFrame_;
  TranslationJacobian translationJacobianTimeDerivativeWorldToEndEffectorInWorldFrame_;
  RotationJacobianLimb rotationJacobianTimeDerivativeBaseToEndEffectorInBaseFrame_;
  RotationJacobian rotationJacobianTimeDerivativeWorldToEndEffectorInWorldFrame_;

  Position positionBaseToEndEffectorInBaseFrame_;
};

using EndEffectorStateMeasuredPtr = std::unique_ptr<EndEffectorStateMeasured>;

} /* namespace loco */
