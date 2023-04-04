/*
 * EndEffectorMotionTask.hpp
 *
 *  Created on: May 11, 2018
 *      Author: Gabriel Hottiger
 */

#pragma once

#include "whole_body_control_romo/tasks/MotionTask.hpp"
#include "whole_body_control_romo/typedefs.hpp"

#include <tinyxml_tools/tinyxml_tools.hpp>

#include <type_traits>

namespace whole_body_control_romo {

template <typename ConcreteDescription_, typename RobotState_>
class BodyMotionTask : public MotionTask<ConcreteDescription_, RobotState_> {
 protected:
  using Base = MotionTask<ConcreteDescription_, RobotState_>;
  using RD = typename Base::RD;
  using BranchEnum = typename RD::BranchEnum;
  using BodyNodeEnum = typename RD::BodyNodeEnum;
  using CoordinateFrameEnum = typename RD::CoordinateFrameEnum;
  using WholeBodyState = typename Base::WholeBodyState;

 public:
  BodyMotionTask(WholeBodyState& wholeBodyState) : Base(wholeBodyState), branchEnum_(BranchEnum::SIZE), bodyNodeEnum_(BodyNodeEnum::SIZE) {}

  //! Load parameters
  bool loadParameters(TiXmlHandle taskHandle) override {
    TiXmlHandle motionTaskHandle = taskHandle;
    if (!tinyxml_tools::getChildHandle(motionTaskHandle, taskHandle, "Motion")) {
      return false;
    }

    std::string branchString;
    if (!tinyxml_tools::loadParameter(branchString, motionTaskHandle.ToElement(), "branch")) {
      return false;
    }
    try {
      // Get branch
      branchEnum_ = RD::template mapKeyNameToKeyEnum<BranchEnum>(branchString);
    } catch (std::out_of_range& e) {
      MELO_WARN_STREAM("Branch " << branchString << " does not exist." << e.what());
      return false;
    }

    std::string bodyNodeString;
    if (!tinyxml_tools::loadParameter(bodyNodeString, motionTaskHandle.ToElement(), "node")) {
      return false;
    }
    try {
      bodyNodeEnum_ = RD::template mapKeyNameToKeyEnum<BodyNodeEnum>(bodyNodeString);
    } catch (std::out_of_range& e) {
      MELO_WARN_STREAM("Body node " << bodyNodeString << " does not exist." << e.what());
      return false;
    }

    return Base::loadParameters(taskHandle);
  }

  void setOptimizationTaskPositionTrackingInSourceFrame(int solutionSpaceDimension, MotionTaskFrame sourceFrame,
                                                        Vector3dConstRef desiredPositionWorldToPointOnBodyInSourceFrame,
                                                        Vector3dConstRef desiredLinearVelocityPointOnBodyInSourceFrame,
                                                        Vector3dConstRef desiredLinearAccelerationPointOnBodyInSourceFrame,
                                                        Vector3dConstRef positionBodyToPointOnBodyInBodyFrame) {
    // Get the jacobian from world to point on body.
    Eigen::MatrixXd jacobianTranslationWorldToPointOnBodyInWorldFrame = Eigen::MatrixXd::Zero(3, RD::getNumDof());
    this->getWholeBodyState().getRobotModel().getJacobianTranslationWorldToPointOnBody(jacobianTranslationWorldToPointOnBodyInWorldFrame,
                                                                                       positionBodyToPointOnBodyInBodyFrame, branchEnum_,
                                                                                       bodyNodeEnum_, CoordinateFrameEnum::WORLD);

    // Get the time derivative of the world to point on body.
    Eigen::MatrixXd jacobianTranslationTimeDerivativeWorldToPointOnBodyInWorldFrame = Eigen::MatrixXd::Zero(3, RD::getNumDof());
    this->getWholeBodyState().getRobotModel().getJacobianTranslationTimeDerivativeWorldToPointOnBody(
        jacobianTranslationTimeDerivativeWorldToPointOnBodyInWorldFrame, positionBodyToPointOnBodyInBodyFrame, branchEnum_, bodyNodeEnum_,
        CoordinateFrameEnum::WORLD);

    // Get the position of the world to point on body.
    Eigen::Vector3d positionWorldToPointOnBodyInWorldFrame;
    this->getWholeBodyState().getRobotModel().getPositionWorldToPointOnBody(positionWorldToPointOnBodyInWorldFrame,
                                                                            positionBodyToPointOnBodyInBodyFrame, branchEnum_,
                                                                            bodyNodeEnum_, CoordinateFrameEnum::WORLD);

    // Get the linear velocity of the point on body.
    const Eigen::Vector3d linearVelocityPointOnBodyInWorldFrame = this->getWholeBodyState().getRobotModel().getLinearVelocityWorldToPointOnBody(
        branchEnum_, bodyNodeEnum_, positionBodyToPointOnBodyInBodyFrame, CoordinateFrameEnum::WORLD);

    const RotationMatrix orientationWorldToSourceFrame(this->getOrientationFrameAToFrameB(MotionTaskFrame::WORLD, sourceFrame));

    Base::setOptimizationTaskPositionTrackingInSourceFrame(
        solutionSpaceDimension, sourceFrame, desiredPositionWorldToPointOnBodyInSourceFrame, desiredLinearVelocityPointOnBodyInSourceFrame,
        desiredLinearAccelerationPointOnBodyInSourceFrame, orientationWorldToSourceFrame.rotate(positionWorldToPointOnBodyInWorldFrame),
        orientationWorldToSourceFrame.rotate(linearVelocityPointOnBodyInWorldFrame),
        orientationWorldToSourceFrame.matrix() * jacobianTranslationWorldToPointOnBodyInWorldFrame,
        orientationWorldToSourceFrame.matrix() * jacobianTranslationTimeDerivativeWorldToPointOnBodyInWorldFrame);
  }

  void setOptimizationTaskOrientationTrackingInSourceFrame(int solutionSpaceDimension, MotionTaskFrame sourceFrame,
                                                           const RotationQuaternion& desiredOrientationSourceFrameToPointOnBody,
                                                           Vector3dConstRef desiredAngularVelocityBodyInSourceFrame,
                                                           Vector3dConstRef desiredAngularAccelerationBodyInSourceFrame,
                                                           Vector3dConstRef positionBodyToPointOnBodyInBodyFrame,
                                                           const RotationQuaternion& orientationBodyToPointOnBody) {
    // Get the jacobian from world to point on body.
    Eigen::MatrixXd jacobianRotationWorldToPointOnBodyInWorldFrame = Eigen::MatrixXd::Zero(3, RD::getNumDof());
    this->getWholeBodyState().getRobotModel().getJacobianRotationWorldToPointOnBody(jacobianRotationWorldToPointOnBodyInWorldFrame,
                                                                                    positionBodyToPointOnBodyInBodyFrame, branchEnum_,
                                                                                    bodyNodeEnum_, CoordinateFrameEnum::WORLD);

    // Get the time derivative of the world to point on body.
    Eigen::MatrixXd jacobianRotationTimeDerivativeWorldToPointOnBodyInWorldFrame = Eigen::MatrixXd::Zero(3, RD::getNumDof());
    this->getWholeBodyState().getRobotModel().getJacobianRotationTimeDerivativeWorldToPointOnBody(
        jacobianRotationTimeDerivativeWorldToPointOnBodyInWorldFrame, positionBodyToPointOnBodyInBodyFrame, branchEnum_, bodyNodeEnum_,
        CoordinateFrameEnum::WORLD);

    // Get the position of the world to point on body.
    const Eigen::Matrix3d& orientationWorldFrameToBody =
        this->getWholeBodyState().getRobotModel().getOrientationWorldToBody(branchEnum_, bodyNodeEnum_);
    const RotationQuaternion orientationWorldFrameToPointOnBody = orientationBodyToPointOnBody * RotationMatrix(orientationWorldFrameToBody);

    // Get the linear velocity of the point on body.
    const Eigen::Vector3d angularVelocityPointOnBodyInWorldFrame =
        this->getWholeBodyState().getRobotModel().getAngularVelocityWorldToBody(branchEnum_, bodyNodeEnum_, CoordinateFrameEnum::WORLD);

    const RotationMatrix orientationWorldToSourceFrame(this->getOrientationFrameAToFrameB(MotionTaskFrame::WORLD, sourceFrame));

    Base::setOptimizationTaskOrientationTrackingInSourceFrame(
        solutionSpaceDimension, sourceFrame, desiredOrientationSourceFrameToPointOnBody, desiredAngularVelocityBodyInSourceFrame,
        desiredAngularAccelerationBodyInSourceFrame, orientationWorldFrameToPointOnBody * orientationWorldToSourceFrame.inverted(),
        orientationWorldToSourceFrame.rotate(angularVelocityPointOnBodyInWorldFrame),
        orientationWorldToSourceFrame.matrix() * jacobianRotationWorldToPointOnBodyInWorldFrame,
        orientationWorldToSourceFrame.matrix() * jacobianRotationTimeDerivativeWorldToPointOnBodyInWorldFrame);
  }

 protected:
  BranchEnum branchEnum_;
  BodyNodeEnum bodyNodeEnum_;
};

template <typename ConcreteDescription_, typename RobotState_>
class BodyRotationTask : public BodyMotionTask<ConcreteDescription_, RobotState_> {
 protected:
  using Base = BodyMotionTask<ConcreteDescription_, RobotState_>;
  using RD = typename Base::RD;
  using WholeBodyState = typename Base::WholeBodyState;

 public:
  BodyRotationTask(WholeBodyState& wholeBodyState) : Base(wholeBodyState) {}

  static std::unique_ptr<whole_body_control::Task> createTask(WholeBodyState& wholeBodyState) {
    return std_utils::make_unique<BodyRotationTask<ConcreteDescription_, RobotState_>>(wholeBodyState);
  }
  static std::string getTaskTypeName() { return "body_rotation"; }


  bool update(double dt, int solutionSpaceDimension) override {
    this->setOptimizationTaskOrientationTrackingInSourceFrame(solutionSpaceDimension, MotionTaskFrame::WORLD, RotationQuaternion(),
                                                              Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                                              RotationQuaternion());

    return true;
  }
};

template <typename ConcreteDescription_, typename RobotState_>
class BodyTranslationTask : public BodyMotionTask<ConcreteDescription_, RobotState_> {
 protected:
  using Base = BodyMotionTask<ConcreteDescription_, RobotState_>;
  using RD = typename Base::RD;
  using WholeBodyState = typename Base::WholeBodyState;

 public:
  BodyTranslationTask(WholeBodyState& wholeBodyState) : Base(wholeBodyState) {}

  static std::unique_ptr<whole_body_control::Task> createTask(WholeBodyState& wholeBodyState) {
    return std_utils::make_unique<BodyTranslationTask<ConcreteDescription_, RobotState_>>(wholeBodyState);
  }
  static std::string getTaskTypeName() { return "body_translation"; }

  bool update(double dt, int solutionSpaceDimension) override {
    this->setOptimizationTaskPositionTrackingInSourceFrame(solutionSpaceDimension, MotionTaskFrame::WORLD, Eigen::Vector3d::Zero(),
                                                           Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    return true;
  }
};

}  // namespace whole_body_control_romo
