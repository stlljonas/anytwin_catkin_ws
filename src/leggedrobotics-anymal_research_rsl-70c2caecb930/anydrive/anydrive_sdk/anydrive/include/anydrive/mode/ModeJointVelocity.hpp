#pragma once

#include <anydrive/mode/ModeBase.hpp>

namespace anydrive {
namespace mode {

class ModeJointVelocity : public ModeBase {
 public:
  ModeJointVelocity();
  ~ModeJointVelocity() override = default;
};

}  // namespace mode
}  // namespace anydrive
