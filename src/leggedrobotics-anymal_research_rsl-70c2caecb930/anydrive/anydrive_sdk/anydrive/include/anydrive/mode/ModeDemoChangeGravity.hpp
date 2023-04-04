#pragma once

#include <anydrive/mode/ModeBase.hpp>

namespace anydrive {
namespace mode {

class ModeDemoChangeGravity : public ModeBase {
 public:
  ModeDemoChangeGravity();
  ~ModeDemoChangeGravity() override = default;
};

}  // namespace mode
}  // namespace anydrive
