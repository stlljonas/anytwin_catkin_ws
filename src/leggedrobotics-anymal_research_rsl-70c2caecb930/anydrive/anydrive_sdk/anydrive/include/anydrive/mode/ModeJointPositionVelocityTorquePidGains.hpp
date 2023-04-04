#pragma once

#include <anydrive/mode/ModeBase.hpp>

namespace anydrive {
namespace mode {

class ModeJointPositionVelocityTorquePidGains : public ModeBase {
 public:
  ModeJointPositionVelocityTorquePidGains();
  ~ModeJointPositionVelocityTorquePidGains() override = default;
};

}  // namespace mode
}  // namespace anydrive
