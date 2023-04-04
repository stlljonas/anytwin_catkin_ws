#include "anydrive/mode/ModeCurrent.hpp"

namespace anydrive {
namespace mode {

ModeCurrent::ModeCurrent() : ModeBase(ModeEnum::Current) {
  controlCurrent_ = true;
}

}  // namespace mode
}  // namespace anydrive
