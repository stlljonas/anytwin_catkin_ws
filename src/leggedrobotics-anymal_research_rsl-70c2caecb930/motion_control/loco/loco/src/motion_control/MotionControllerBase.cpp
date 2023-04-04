/*!
 * @file     MotionControllerBase.hpp
 * @author   Péter Fankhauser, Christian Gehring, Dario Bellicoso
 * @date     March 6, 2014
 * @brief
 */

// loco
#include "loco/motion_control/MotionControllerBase.hpp"

namespace loco {

MotionControllerBase::MotionControllerBase(WholeBody& wholeBody)
    : ModuleBase("motion_controller"), wholeBody_(wholeBody), limbs_(*wholeBody.getLimbsPtr()), torso_(*wholeBody.getTorsoPtr()) {}

bool MotionControllerBase::setToInterpolated(const MotionControllerBase& motionController1, const MotionControllerBase& motionController2,
                                             double t) {
  return false;
}

} /* namespace loco */
