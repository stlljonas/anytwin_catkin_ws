#include "anydrive/mode/ModeMotorPosition.hpp"

namespace anydrive {
namespace mode {

ModeMotorPosition::ModeMotorPosition() : ModeBase(ModeEnum::MotorPosition) {
  controlMotorPosition_ = true;
}

}  // namespace mode
}  // namespace anydrive
