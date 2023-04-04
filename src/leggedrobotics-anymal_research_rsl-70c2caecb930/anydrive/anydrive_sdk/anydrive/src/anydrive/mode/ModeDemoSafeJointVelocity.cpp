#include "anydrive/mode/ModeDemoSafeJointVelocity.hpp"

namespace anydrive {
namespace mode {

ModeDemoSafeJointVelocity::ModeDemoSafeJointVelocity() : ModeBase(ModeEnum::DemoSafeJointVelocity) {
  controlJointVelocity_ = true;
}

}  // namespace mode
}  // namespace anydrive
