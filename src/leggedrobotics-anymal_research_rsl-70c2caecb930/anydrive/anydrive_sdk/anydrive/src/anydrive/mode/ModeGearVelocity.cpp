#include "anydrive/mode/ModeGearVelocity.hpp"

namespace anydrive {
namespace mode {

ModeGearVelocity::ModeGearVelocity() : ModeBase(ModeEnum::GearVelocity) {
  controlGearVelocity_ = true;
}

}  // namespace mode
}  // namespace anydrive
