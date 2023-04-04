#include "anydrive/mode/ModeDemoChangeGravity.hpp"

namespace anydrive {
namespace mode {

ModeDemoChangeGravity::ModeDemoChangeGravity() : ModeBase(ModeEnum::DemoChangeGravity) {
  controlJointTorque_ = true;
}

}  // namespace mode
}  // namespace anydrive
