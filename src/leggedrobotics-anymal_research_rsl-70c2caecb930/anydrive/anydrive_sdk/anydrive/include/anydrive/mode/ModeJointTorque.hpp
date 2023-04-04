#pragma once

#include <anydrive/mode/ModeBase.hpp>

namespace anydrive {
namespace mode {

class ModeJointTorque : public ModeBase {
 public:
  ModeJointTorque();
  ~ModeJointTorque() override = default;
};

}  // namespace mode
}  // namespace anydrive
