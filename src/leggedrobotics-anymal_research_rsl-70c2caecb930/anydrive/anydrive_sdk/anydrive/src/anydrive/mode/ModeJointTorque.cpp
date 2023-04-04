#include "anydrive/mode/ModeJointTorque.hpp"

namespace anydrive {
namespace mode {

ModeJointTorque::ModeJointTorque() : ModeBase(ModeEnum::JointTorque) {
  controlJointTorque_ = true;
}

}  // namespace mode
}  // namespace anydrive
