#pragma once

#include <anydrive/mode/ModeBase.hpp>

namespace anydrive {
namespace mode {

class ModeGearPosition : public ModeBase {
 public:
  ModeGearPosition();
  ~ModeGearPosition() override = default;
};

}  // namespace mode
}  // namespace anydrive
