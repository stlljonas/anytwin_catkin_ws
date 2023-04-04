#pragma once

#include <anydrive/mode/ModeBase.hpp>

namespace anydrive {
namespace mode {

class ModeMotorVelocity : public ModeBase {
 public:
  ModeMotorVelocity();
  ~ModeMotorVelocity() override = default;
};

}  // namespace mode
}  // namespace anydrive
