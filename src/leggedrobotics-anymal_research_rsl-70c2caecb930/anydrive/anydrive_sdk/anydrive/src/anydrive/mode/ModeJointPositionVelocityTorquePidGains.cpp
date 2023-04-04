#include "anydrive/mode/ModeJointPositionVelocityTorquePidGains.hpp"

namespace anydrive {
namespace mode {

ModeJointPositionVelocityTorquePidGains::ModeJointPositionVelocityTorquePidGains()
    : ModeBase(ModeEnum::JointPositionVelocityTorquePidGains) {
  controlJointPosition_ = true;
  controlJointVelocity_ = true;
  controlJointTorque_ = true;
  customGains_ = true;
}

}  // namespace mode
}  // namespace anydrive
