#include "anydrive/mode/ModeJointPosition.hpp"

namespace anydrive {
namespace mode {

ModeJointPosition::ModeJointPosition() : ModeBase(ModeEnum::JointPosition) {
  controlJointPosition_ = true;
}

}  // namespace mode
}  // namespace anydrive
