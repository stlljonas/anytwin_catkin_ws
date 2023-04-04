#pragma once

#include <anydrive/mode/ModeBase.hpp>

namespace anydrive {
namespace mode {

class ModeCurrent : public ModeBase {
 public:
  ModeCurrent();
  ~ModeCurrent() override = default;
};

}  // namespace mode
}  // namespace anydrive
