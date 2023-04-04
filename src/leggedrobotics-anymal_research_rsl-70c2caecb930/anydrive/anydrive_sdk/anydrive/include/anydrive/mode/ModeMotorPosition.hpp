#pragma once

#include <anydrive/mode/ModeBase.hpp>

namespace anydrive {
namespace mode {

class ModeMotorPosition : public ModeBase {
 public:
  ModeMotorPosition();
  ~ModeMotorPosition() override = default;
};

}  // namespace mode
}  // namespace anydrive
