#pragma once

#include <anydrive/mode/ModeBase.hpp>

namespace anydrive {
namespace mode {

class ModeJointPosition : public ModeBase {
 public:
  ModeJointPosition();
  ~ModeJointPosition() override = default;
};

}  // namespace mode
}  // namespace anydrive
