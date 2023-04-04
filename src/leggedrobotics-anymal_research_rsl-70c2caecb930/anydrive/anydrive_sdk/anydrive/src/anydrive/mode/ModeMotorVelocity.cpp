#include "anydrive/mode/ModeMotorVelocity.hpp"

namespace anydrive {
namespace mode {

ModeMotorVelocity::ModeMotorVelocity() : ModeBase(ModeEnum::MotorVelocity) {
  controlMotorVelocity_ = true;
}

}  // namespace mode
}  // namespace anydrive
