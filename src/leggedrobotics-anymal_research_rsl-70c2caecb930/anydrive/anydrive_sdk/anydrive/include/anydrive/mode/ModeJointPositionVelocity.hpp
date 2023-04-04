#pragma once

#include <anydrive/mode/ModeBase.hpp>

namespace anydrive {
namespace mode {

class ModeJointPositionVelocity : public ModeBase {
 public:
  ModeJointPositionVelocity();
  ~ModeJointPositionVelocity() override = default;
};

}  // namespace mode
}  // namespace anydrive
