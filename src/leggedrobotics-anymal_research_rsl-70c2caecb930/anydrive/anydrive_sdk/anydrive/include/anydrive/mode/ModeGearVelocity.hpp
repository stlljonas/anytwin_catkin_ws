#pragma once

#include <anydrive/mode/ModeBase.hpp>

namespace anydrive {
namespace mode {

class ModeGearVelocity : public ModeBase {
 public:
  ModeGearVelocity();
  ~ModeGearVelocity() override = default;
};

}  // namespace mode
}  // namespace anydrive
