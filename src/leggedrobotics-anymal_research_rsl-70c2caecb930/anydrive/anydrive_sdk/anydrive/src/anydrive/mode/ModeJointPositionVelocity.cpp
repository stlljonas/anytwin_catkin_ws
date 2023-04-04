#include "anydrive/mode/ModeJointPositionVelocity.hpp"

namespace anydrive {
namespace mode {

ModeJointPositionVelocity::ModeJointPositionVelocity() : ModeBase(ModeEnum::JointPositionVelocity) {
  controlJointPosition_ = true;
  controlJointVelocity_ = true;
}

}  // namespace mode
}  // namespace anydrive
