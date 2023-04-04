#pragma once

#include <anydrive/mode/ModeBase.hpp>

namespace anydrive {
namespace mode {

class ModeFreeze : public ModeBase {
 public:
  ModeFreeze();
  ~ModeFreeze() override = default;
};

}  // namespace mode
}  // namespace anydrive
