#include "anydrive/mode/ModeJointVelocity.hpp"

namespace anydrive {
namespace mode {

ModeJointVelocity::ModeJointVelocity() : ModeBase(ModeEnum::JointVelocity) {
  controlJointVelocity_ = true;
}

}  // namespace mode
}  // namespace anydrive
