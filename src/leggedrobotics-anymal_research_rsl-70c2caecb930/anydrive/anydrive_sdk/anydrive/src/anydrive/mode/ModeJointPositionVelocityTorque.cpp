#include "anydrive/mode/ModeJointPositionVelocityTorque.hpp"

namespace anydrive {
namespace mode {

ModeJointPositionVelocityTorque::ModeJointPositionVelocityTorque() : ModeBase(ModeEnum::JointPositionVelocityTorque) {
  controlJointPosition_ = true;
  controlJointVelocity_ = true;
  controlJointTorque_ = true;
}

}  // namespace mode
}  // namespace anydrive
