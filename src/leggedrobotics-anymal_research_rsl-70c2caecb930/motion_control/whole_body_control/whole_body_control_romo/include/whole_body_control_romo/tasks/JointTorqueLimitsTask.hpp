/*
 * JointTorqueLimitsTask.hpp
 *
 *  Created on: July 30, 2018
 *      Author: Gabriel Hottiger
 */

#pragma once

// whole_body_control_romo
#include "whole_body_control_romo/WholeBodyStateRomo.hpp"
#include "whole_body_control_romo/tasks/TaskRomo.hpp"
#include "whole_body_control_romo/typedefs.hpp"

namespace whole_body_control_romo {

    template <typename ConcreteDescription_, typename RobotState_>
    class JointTorqueLimitsTask : public TaskRomo<ConcreteDescription_, RobotState_> {
    protected:
        using Base = TaskRomo<ConcreteDescription_, RobotState_>;
        using WholeBodyState = typename Base::WholeBodyState;
        using RD = typename Base::RD;
        using ActuatorNodeEnum = typename RD::ActuatorNodeEnum;
        using ActuatorEnum = typename RD::ActuatorEnum;

    public:
        explicit JointTorqueLimitsTask(WholeBodyState& wholeBodyState) : Base(wholeBodyState) {
            this->equalityConstraintJacobian_ = Eigen::MatrixXd();
            this->equalityConstraintTargetValues_ = Eigen::VectorXd();
            this->equalityConstraintRelativeWeights_ = Eigen::VectorXd();
            this->constraintType_ = hopt::ConstraintType::Inequality;
            maxTorqueVector_ = Eigen::VectorXd::Zero(RD::getJointsDimension());
        }

        static std::unique_ptr<whole_body_control::Task> createTask(WholeBodyState& wholeBodyState) {
            return std_utils::make_unique<JointTorqueLimitsTask<ConcreteDescription_, RobotState_>>(wholeBodyState);
        }
        static std::string getTaskTypeName() { return "joint_torque_limits"; }

        bool loadParameters(TiXmlHandle taskHandle) override {
            double maxTorque = 0.0;
            TiXmlHandle maxTorqueHandle = taskHandle;
            bool loadLimitsFromXML  = false;
            if (!tinyxml_tools::loadParameter(loadLimitsFromXML, taskHandle, "override_model", false)) {
              return false;
            }
            if (!loadLimitsFromXML) {
              unsigned int index = 0;
              for (const auto jointKey : RD::getJointKeys()) {
                const auto jointEnum = jointKey.getEnum();
                const auto jointName = jointKey.getName();
                try {
                  const auto& actuatorEnum = RD::mapJointEnumToActuatorEnum::at(jointEnum);
                  const auto actuatorString = RD::template mapKeyEnumToKeyName<ActuatorEnum>(actuatorEnum);

                  maxTorqueVector_[index] =
                      this->getWholeBodyState().getRobotModel().getLimits()->getActuatorMaxCommandEffort(actuatorEnum);
                  MELO_DEBUG_STREAM("[JointTorqueLimitsTask] Actuator Name: " << actuatorString
                                                                            << " Max Torque: " << maxTorqueVector_[index])
                  index++;
                } catch (const std::out_of_range& ex) {
                  MELO_WARN_STREAM("[JointTorqueLimitsTask] Out of range warning: " << ex.what() << "\n"
                                                                                    << jointName << " is an unactuated joint")
                }
              }
              maxTorqueVectorSize_ = index;
            } else {
              if (tinyxml_tools::getChildHandle(maxTorqueHandle, taskHandle, "MaxTorque", false)) {
                if (!tinyxml_tools::loadParameter(maxTorque, maxTorqueHandle, "all", 0.0)) {
                  return false;
                }
                unsigned int index = 0;
                for (const auto jointKey : RD::getJointKeys()) {
                  const auto jointEnum = jointKey.getEnum();
                  const auto jointName = jointKey.getName();
                  try {
                    // The call to mapJointEnumToActuatorEnum with unused result is necessary,
                    // as this will throw the exception we are catching.
                    RD::mapJointEnumToActuatorEnum::at(jointEnum);
                    index++;
                  } catch (const std::out_of_range& ex) {
                    MELO_WARN_STREAM("[JointTorqueLimitsTask] Out of range warning: " << ex.what() << "\n"
                                                                                      << jointName << " is an unactuated joint");
                  }
                }
                maxTorqueVectorSize_ = index;
                maxTorqueVector_ = Eigen::VectorXd::Constant(maxTorqueVectorSize_, maxTorque);
              } else if (tinyxml_tools::getChildHandle(maxTorqueHandle, taskHandle, "MaxTorqueActuatorNode", false)) {
                unsigned int index = 0;
                for (const auto jointKey : RD::getJointKeys()) {
                  const auto jointEnum = jointKey.getEnum();
                  const auto jointName = jointKey.getName();
                  try {
                    const auto& actuatorEnum = RD::mapJointEnumToActuatorEnum::at(jointEnum);
                    const auto actuatorNodeEnum = RD::template mapEnums<ActuatorNodeEnum>(actuatorEnum);
                    const auto actuatorNodeString = RD::template mapKeyEnumToKeyName<ActuatorNodeEnum>(actuatorNodeEnum);
                    if (!tinyxml_tools::loadParameter(maxTorque, maxTorqueHandle, actuatorNodeString, 0.0)) {
                      return false;
                    }
                    maxTorqueVector_[index] = maxTorque;
                    index++;
                  } catch (const std::out_of_range& ex) {
                    MELO_WARN_STREAM("[JointTorqueLimitsTask] Out of range warning: " << ex.what() << "\n"
                                                                                      << jointName << " is an unactuated joint");
                  }
                }
                maxTorqueVectorSize_ = index;
              } else if (tinyxml_tools::getChildHandle(maxTorqueHandle, taskHandle, "MaxTorqueActuator", false)) {
                unsigned int index = 0;
                for (const auto jointKey : RD::getJointKeys()) {
                  const auto jointEnum = jointKey.getEnum();
                  const auto jointName = jointKey.getName();
                  try {
                    const auto& actuatorEnum = RD::mapJointEnumToActuatorEnum::at(jointEnum);
                    const auto actuatorString = RD::template mapKeyEnumToKeyName<ActuatorEnum>(actuatorEnum);
                    if (!tinyxml_tools::loadParameter(maxTorque, maxTorqueHandle, actuatorString, 0.0)) {
                      return false;
                    }
                    maxTorqueVector_[index] = maxTorque;
                    index++;
                  } catch (const std::out_of_range& ex) {
                    MELO_WARN_STREAM("[JointTorqueLimitsTask] Out of range warning: " << ex.what() << "\n"
                                                                                      << jointName << " is an unactuated joint");
                  }
                }
                maxTorqueVectorSize_ = index;
              } else {
                MELO_ERROR_STREAM("[JointTorqueLimitsTask] No torque limit type found in task "
                                  << this->getName() << "; you need a tag with one of MaxTorque, MaxTorqueActuatorNode, MaxTorqueActuator");
                return false;
              }
            }

            this->inequalityConstraintMaxValues_.resize(2 * maxTorqueVectorSize_);
            this->inequalityConstraintRelativeWeights_ = Eigen::VectorXd::Ones(2 * maxTorqueVectorSize_);

            return Base::loadParameters(taskHandle);
        }

        bool update(double dt, int solutionSpaceDimension) override {
            this->inequalityConstraintJacobian_.resize(2 * maxTorqueVectorSize_, solutionSpaceDimension);

            Eigen::MatrixXd Dsub(maxTorqueVectorSize_, solutionSpaceDimension);
            Dsub << this->getWholeBodyState().getWholeBody().getWholeBodyMassMatrix().bottomRows(maxTorqueVectorSize_),
                    -this->getWholeBodyState().getSupportJacobian().getSupportJacobianTransposeInForceFrame().bottomRows(maxTorqueVectorSize_);

            this->inequalityConstraintJacobian_.topRows(maxTorqueVectorSize_) = Dsub;
            this->inequalityConstraintJacobian_.bottomRows(maxTorqueVectorSize_) = -Dsub;

            this->inequalityConstraintMaxValues_.topRows(maxTorqueVectorSize_) =
                    maxTorqueVector_.topRows(maxTorqueVectorSize_) - this->getWholeBodyState().getWholeBody().getWholeBodyNonlinearEffects().bottomRows(maxTorqueVectorSize_);
            this->inequalityConstraintMaxValues_.bottomRows(maxTorqueVectorSize_) =
                    maxTorqueVector_.topRows(maxTorqueVectorSize_) + this->getWholeBodyState().getWholeBody().getWholeBodyNonlinearEffects().bottomRows(maxTorqueVectorSize_);

            return true;
        }

    private:
        Eigen::VectorXd maxTorqueVector_;
        unsigned int maxTorqueVectorSize_ = 0;
    };

}  // namespace whole_body_control_romo
