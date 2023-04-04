#pragma once

#include <anydrive/mode/ModeBase.hpp>

namespace anydrive {
namespace mode {

class ModeDisable : public ModeBase {
 public:
  ModeDisable();
  ~ModeDisable() override = default;
};

}  // namespace mode
}  // namespace anydrive
