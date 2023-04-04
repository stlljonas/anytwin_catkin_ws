#pragma once

#include <anydrive/mode/ModeBase.hpp>

namespace anydrive {
namespace mode {

class ModeJointPositionVelocityTorque : public ModeBase {
 public:
  ModeJointPositionVelocityTorque();
  ~ModeJointPositionVelocityTorque() override = default;
};

}  // namespace mode
}  // namespace anydrive
