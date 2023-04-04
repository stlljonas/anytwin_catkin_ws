/*
 * ImpedanceAndVirtualModelControllerFreeGait.cpp
 *
 *  Created on: Feb 5, 2018
 *      Author: Dario Bellicoso
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include <anymal_ctrl_free_gait/base/ImpedanceAndVirtualModelControllerFreeGait.hpp>

namespace loco {

bool ImpedanceAndVirtualModelControllerFreeGait::setControlModeForLimbs() {
  // Only the stance control mode is set here. The swing control mode is set in FootPlacementStrategyFreeGait.
  // The stance control mode for this motion tracker should always be set to MODE_JOINT_POSITION_VELOCITY_TORQUE_PID_GAINS.
  for (auto limb : limbs_) {
    if ((limb->getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::Support) ||
        (limb->getLimbStrategy().getLimbStrategyEnum() == LimbStrategyEnum::ContactInvariant)) {
      loco::JointControlModes desiredJointControlModes = limb->getInitializedJointControlModes();
      desiredJointControlModes.setConstant(loco::ControlMode::MODE_JOINT_POSITION_VELOCITY_TORQUE_PID_GAINS);
      limb->getLimbStateDesiredPtr()->setJointControlModes(desiredJointControlModes);
    }
  }

  return true;
}

}  // namespace loco
