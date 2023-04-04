#pragma once

#include <anydrive/mode/ModeBase.hpp>

namespace anydrive {
namespace mode {

class ModeDemoSafeJointVelocity : public ModeBase {
 public:
  ModeDemoSafeJointVelocity();
  ~ModeDemoSafeJointVelocity() override = default;
};

}  // namespace mode
}  // namespace anydrive
