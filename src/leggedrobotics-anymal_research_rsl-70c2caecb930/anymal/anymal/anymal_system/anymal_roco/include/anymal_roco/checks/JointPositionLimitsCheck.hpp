/**
 * @authors      Alexander Reske
 * @affiliation  ANYbotics AG
 * @brief        Class for checking if joint position limits are respected.
 */

#pragma once

// std
#include <memory>
#include <vector>

// anymal description
#include <anymal_description/AnymalDescription.hpp>

// anymal_roco
#include "anymal_roco/checks/LimitCheckBase.hpp"
#include "anymal_roco/checks/StateCheckBase.hpp"

namespace anymal_roco {

class JointPositionLimitsCheck : public StateCheckBase {

 public:
  using AD = anymal_description::AnymalDescription;
  using LimitCheckBaseConstPtr = std::shared_ptr<const anymal_roco::LimitCheckBase>;
  using JointPositionLimits = std::array<LimitCheckBaseConstPtr, AD::getJointsDimension()>;

  //! Constructor
  JointPositionLimitsCheck(JointPositionLimits jointPositionLimits) : jointPositionLimits_(jointPositionLimits) {};

  //! Default Destructor
  ~JointPositionLimitsCheck() override = default;

  //! Check state
  bool check(const RocoState& state, boost::shared_mutex& stateMutex) const override;

 private:
  JointPositionLimits jointPositionLimits_;
};

} /* namespace anymal_roco */
