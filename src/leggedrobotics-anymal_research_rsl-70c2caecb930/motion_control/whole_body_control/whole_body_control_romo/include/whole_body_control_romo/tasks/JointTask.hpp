/*
 * JointTask.hpp
 *
 *  Created on: May 14, 2018
 *      Author: Gabriel Hottiger
 */

#pragma once

// whole_body_control_romo
#include "whole_body_control_romo/WholeBodyStateRomo.hpp"
#include "whole_body_control_romo/tasks/TaskRomo.hpp"
#include "whole_body_control_romo/typedefs.hpp"

// pid control
#include <basic_controllers/PIDGains.hpp>

// parameter_handler
#include <parameter_handler/tinyxml_tools_traits.hpp>

namespace whole_body_control_romo {

template <typename ConcreteDescription_, typename RobotState_>
class JointTask : public TaskRomo<ConcreteDescription_, RobotState_> {
 protected:
  using Base = TaskRomo<ConcreteDescription_, RobotState_>;
  using WholeBodyState = WholeBodyStateRomo<ConcreteDescription_, RobotState_>;
  using RobotModel = romo::RobotModel<ConcreteDescription_, RobotState_>;
  using RD = romo::RobotDescription<ConcreteDescription_>;
  using ConcreteTopology = typename RD::ConcreteTopology;
  using JointEnum = typename RD::JointEnum;
  using JointNodeEnum = typename RD::JointNodeEnum;
  using LimbEnum = typename RD::LimbEnum;
  //! Struct with necessary info for joint task
  struct JointTaskInfo {
    const loco::LimbBase* limb;
    LimbEnum limbEnum;
    int idInLimbJ;
    JointEnum jointEnum;
  };

 public:
  //! Constructor.
  explicit JointTask(WholeBodyState& wholeBodyState) :
    Base(wholeBodyState),
    model_(wholeBodyState.getRobotModel())
  {
  }

  //! Destructor.
  ~JointTask() override = default;

  //! Load parameters
  bool loadParameters(TiXmlHandle taskHandle) override {
    // Clear limbs
    jointTaskInfos_.clear();

    // Get toplevel handle
    TiXmlHandle jointTaskHandle = taskHandle;
    if (!tinyxml_tools::getChildHandle(jointTaskHandle, taskHandle, "Joint")) {
      return false;
    }

    // Get limbs from comma separated list
    std::string limbList;
    if (!tinyxml_tools::loadParameter(limbList, jointTaskHandle.ToElement(), "limbs")) {
      return false;
    }
    std::vector<std::string> limbNames = this->getLimbNamesFromCommaSeperatedString(limbList);

    // Get joint nodes from comma separated list
    std::string jointNodeList;
    if (!tinyxml_tools::loadParameter(jointNodeList, jointTaskHandle.ToElement(), "joint_nodes")) {
      return false;
    }
    std::vector<std::string> jointNodeNames = this->getListFromCommaSeparatedString(jointNodeList);
    const bool allJointNodes = (jointNodeNames.front() == "all");

    for (auto limbName : limbNames) {
      JointTaskInfo jointTaskInfo;

      // Get limbEnum
      try {
        jointTaskInfo.limbEnum = RD::template mapKeyNameToKeyEnum<LimbEnum>(limbName);
      } catch (std::out_of_range& e) {
        MELO_WARN_STREAM("[JointVelocityTask] Limb " << limbName << " does not exist." << e.what());
        return false;
      }

      // Get limb from limbId
      auto& limbs = this->getWholeBodyState().getWholeBody().getLimbs();
      auto limbId = static_cast<unsigned int>(RD::mapKeyEnumToKeyId(jointTaskInfo.limbEnum));
      auto limbIt = std::find_if(limbs.begin(), limbs.end(),
                                 [limbId](loco::LimbBase const* const limb) { return limb->getLimbUInt() == limbId; });
      if (limbIt != limbs.end()) {
        jointTaskInfo.limb = *limbIt;
      } else {
        MELO_WARN_STREAM("[JointTask] No Limb " << limbName << " with Id " << limbId
                                                << " in limbs group! Skipping it.");
        continue;
      }

      // Handle special case "all" nodes
      if (allJointNodes) {
        std::vector<JointEnum> jointsFromLimb = RD::mapJointEnumToLimbEnum::findVector(jointTaskInfo.limbEnum);
        jointNodeNames.resize(jointsFromLimb.size());
        std::transform(jointsFromLimb.begin(), jointsFromLimb.end(), jointNodeNames.begin(),
                       [](const JointEnum j) { return RD::template mapKeyEnumToKeyName<JointEnum, JointNodeEnum>(j); });
      }

      // Add every node-limb combination
      for (auto jointNodeName : jointNodeNames) {
        // Transform to jointnode enum
        JointNodeEnum jointNodeEnum;
        try {
          jointNodeEnum = RD::template mapKeyNameToKeyEnum<JointNodeEnum>(jointNodeName);
        } catch (std::out_of_range& e) {
          MELO_WARN_STREAM("[JointVelocityTask] JointNode " << jointNodeName << " does not exist." << e.what());
          return false;
        }

        // Get joint from limb and node
        std::vector<JointEnum> jointsFromNode =
            ConcreteTopology::mapJointEnumToJointNodeEnum::findVector(jointNodeEnum);
        auto jointEnumIt =
            std::find_if(jointsFromNode.begin(), jointsFromNode.end(), [&jointTaskInfo](const JointEnum joint) {
              return RD::template mapEnums<LimbEnum>(joint) == jointTaskInfo.limbEnum;
            });
        if (jointEnumIt == jointsFromNode.end()) {
          MELO_WARN_STREAM("[JointTask] Limb " << limbName << " does not have a joint node " << jointNodeName
                                               << "! Skipping it.");
          continue;
        }

        // Save jointEnum
        jointTaskInfo.jointEnum = RD::getJointKeys().at(*jointEnumIt).getEnum();

        // Save idInLimbJ
        jointTaskInfo.idInLimbJ =RD::getJointKeys().at(*jointEnumIt).getId() - RD::getLimbStartIndexInJ(jointTaskInfo.limbEnum);

        // Add info on success
        jointTaskInfos_.push_back(jointTaskInfo);
      }
    }

    return Base::loadParameters(taskHandle);
  }

 protected:
  std::vector<JointTaskInfo> jointTaskInfos_;
  const RobotModel& model_;
};

template <typename ConcreteDescription_, typename RobotState_>
class JointMotionTask : public JointTask<ConcreteDescription_, RobotState_> {
 protected:
  using Base = JointTask<ConcreteDescription_, RobotState_>;
  using PIDGainsD = basic_controllers::PIDGainsD;
  using WholeBodyState = WholeBodyStateRomo<ConcreteDescription_, RobotState_>;

 public:
  explicit JointMotionTask(WholeBodyState& wholeBodyState) : Base(wholeBodyState) {}

 protected:
  //! Load parameters
  bool loadParameters(TiXmlHandle taskHandle) override {
    // Get joint task handle
    TiXmlHandle jointTaskHandle = taskHandle;
    if (!tinyxml_tools::getChildHandle(jointTaskHandle, taskHandle, "Joint")) {
      return false;
    }

    // Get gain handle
    TiXmlHandle gainHandle = taskHandle;
    if (!tinyxml_tools::getChildHandle(gainHandle, jointTaskHandle, "Gains")) {
      return false;
    }

    // Read gains
    if (!tinyxml_tools::loadParameter(pidGains_.proportionalGains_, gainHandle.ToElement(), "kp", 0.0) ||
        !tinyxml_tools::loadParameter(pidGains_.integralGains_, gainHandle.ToElement(), "ki", 0.0) ||
        !tinyxml_tools::loadParameter(pidGains_.derivativeGains_, gainHandle.ToElement(), "kd", 0.0) ||
        !tinyxml_tools::loadParameter(pidGains_.maxIntegral_, gainHandle.ToElement(), "maxI", 0.0)) {
      return false;
    }

    return Base::loadParameters(taskHandle);
  }

 protected:
  PIDGainsD pidGains_;
};

template <typename ConcreteDescription_, typename RobotState_>
class JointLimitsTask : public JointTask<ConcreteDescription_, RobotState_> {
 protected:
  using Base = JointTask<ConcreteDescription_, RobotState_>;
  using PIDGainsD = basic_controllers::PIDGainsD;
  using WholeBodyState = WholeBodyStateRomo<ConcreteDescription_, RobotState_>;

 public:
  explicit JointLimitsTask(WholeBodyState& wholeBodyState) : Base(wholeBodyState), timestepScaling_(0.0) {}

 protected:
  //! Load parameters
  bool loadParameters(TiXmlHandle taskHandle) override {
    // Get joint task handle
    TiXmlHandle jointTaskHandle = taskHandle;
    if (!tinyxml_tools::getChildHandle(jointTaskHandle, taskHandle, "Joint")) {
      return false;
    }

    // Get lookahead handle
    TiXmlHandle limitHandle = taskHandle;
    if (!tinyxml_tools::getChildHandle(limitHandle, jointTaskHandle, "LimitStiffness")) {
      return false;
    }

    // Read timestep scaling
    if (!tinyxml_tools::loadParameter(timestepScaling_, limitHandle.ToElement(), "timestepScaling", 1.0)) {
      return false;
    }

    return Base::loadParameters(taskHandle);
  }

 protected:
  double timestepScaling_;
};

}  // namespace whole_body_control_romo
