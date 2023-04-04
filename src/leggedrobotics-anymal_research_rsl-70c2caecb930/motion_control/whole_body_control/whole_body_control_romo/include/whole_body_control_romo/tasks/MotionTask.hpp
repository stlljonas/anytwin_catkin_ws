/*
 * MotionTask.hpp
 *
 *  Created on: May 7, 2018
 *      Author: Gabriel Hottiger
 */

#pragma once

// whole_body_control_romo
#include "whole_body_control_romo/WholeBodyStateRomo.hpp"
#include "whole_body_control_romo/tasks/TaskRomo.hpp"
#include "whole_body_control_romo/typedefs.hpp"

// pid control
#include <basic_controllers/PIDGains.hpp>

namespace whole_body_control_romo {

enum class MotionTaskFrame {
  WORLD = 0,  //! World aligned frame
  CONTROL,    //! Terrain aligned frame
  BASE,       //! Floating base frame
  SIZE
};

template <typename ConcreteDescription_, typename RobotState_>
class MotionTask : public TaskRomo<ConcreteDescription_, RobotState_> {
 protected:
  using PIDGains = basic_controllers::PIDGains3D;

  using WholeBodyState = WholeBodyStateRomo<ConcreteDescription_, RobotState_>;
  using RobotModel = romo::RobotModel<ConcreteDescription_, RobotState_>;
  using RD = romo::RobotDescription<ConcreteDescription_>;

 public:
  // Motion task frame keys
  static constexpr std_utils::KeyArray<MotionTaskFrame> motionTaskFrameKeys_ = {{std_utils::make_key(MotionTaskFrame::WORLD, "WORLD"),
                                                                                 std_utils::make_key(MotionTaskFrame::CONTROL, "CONTROL"),
                                                                                 std_utils::make_key(MotionTaskFrame::BASE, "BASE")}};

 public:
  explicit MotionTask(WholeBodyState& wholeBodyState)
  : TaskRomo<ConcreteDescription_, RobotState_>(wholeBodyState),
    motionTaskFrameEnum_(MotionTaskFrame::SIZE)
    { }

  //! Getters
  MotionTaskFrame getMotionTaskFrameEnum() const { return motionTaskFrameEnum_; }
  const PIDGains& getGainsInMotionTaskFrame() const { return gainsInTaskFrame_; }
  const Eigen::Vector3d& getRelativeWeightsInMotionTaskFrame() const { return relativeWeightsInTaskFrame_.getValue(); }

  //! Load parameters
  bool loadParameters(TiXmlHandle taskHandle) override;

  //! Add parameters to handler
  bool addParametersToHandler() override;

  //! Add signals to logger.
  bool addVariablesToLog(const std::string& ns) const override;

  //! Initialize the task.
  bool initialize(double dt) override;

  bool setOptimizationTaskPositionTrackingInSourceFrame(
      int solutionSpaceDimension, MotionTaskFrame sourceFrame, Vector3dConstRef desiredPositionWorldToPointOnBodyInSourceFrame,
      Vector3dConstRef desiredLinearVelocityPointOnBodyInSourceFrame, Vector3dConstRef desiredLinearAccelerationPointOnBodyInSourceFrame,
      Vector3dConstRef measuredPositionWorldToPointOnBodyInSourceFrame, Vector3dConstRef measuredLinearVelocityPointOnBodyInSourceFrame,
      JacobianTranslationConstRef jacobianTranslationWorldToPointOnBodyInSourceFrame,
      JacobianTranslationConstRef jacobianTranslationTimeDerivativeWorldToPointOnBodyInSourceFrame, bool appendToCurrentTask = false);

  bool setOptimizationTaskOrientationTrackingInSourceFrame(
      int solutionSpaceDimension, MotionTaskFrame sourceFrame, const RotationQuaternion& desiredOrientationSourceFrameToPointOnBody,
      Vector3dConstRef desiredAngularVelocityPointOnBodyInSourceFrame, Vector3dConstRef desiredAngularAccelerationPointOnBodyInSourceFrame,
      const RotationQuaternion& measuredOrientationSourceFrameToPointOnBody,
      Vector3dConstRef measuredAngularVelocityPointOnBodyInSourceFrame,
      JacobianTranslationConstRef jacobianRotationWorldToPointOnBodyInSourceFrame,
      JacobianTranslationConstRef jacobianRotationTimeDerivativeWorldToPointOnBodyInSourceFrame, bool appendToCurrentTask = false);

  RotationQuaternion getOrientationSourceFrameToTaskFrame(MotionTaskFrame sourceFrame) const;
  RotationQuaternion getOrientationFrameAToFrameB(MotionTaskFrame frameA, MotionTaskFrame frameB) const;

 protected:
  //! Frame in which task should be added to optimization, the gains are expressed and the relative weights are acting.
  MotionTaskFrame motionTaskFrameEnum_;

  //! PID gains in the task frame.
  PIDGains gainsInTaskFrame_;

  //! Relative weights between x-y-z in the task frame.
  parameter_handler::Parameter<Eigen::Vector3d> relativeWeightsInTaskFrame_;

  //! The position tracking error. Used for control and logging.
  Eigen::Vector3d linearPositionErrorInTaskFrame_;

  //! The orientation tracking error. Used for control and logging.
  Eigen::Vector3d angularPositionErrorInTaskFrame_;

  //! The linear velocity tracking error. Used for control and logging.
  Eigen::Vector3d linearVelocityErrorInTaskFrame_;

  //! The angular velocity tracking error. Used for control and logging.
  Eigen::Vector3d angularVelocityErrorInTaskFrame_;
};

template <typename ConcreteDescription_, typename RobotState_>
constexpr std_utils::KeyArray<MotionTaskFrame> MotionTask<ConcreteDescription_, RobotState_>::motionTaskFrameKeys_;

}  // namespace whole_body_control_romo

#include "whole_body_control_romo/tasks/MotionTask.tpp"
