#include "anydrive/mode/ModeGearPosition.hpp"

namespace anydrive {
namespace mode {

ModeGearPosition::ModeGearPosition() : ModeBase(ModeEnum::GearPosition) {
  controlGearPosition_ = true;
}

}  // namespace mode
}  // namespace anydrive
