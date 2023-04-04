#include "anydrive/mode/ModeBase.hpp"

namespace anydrive {
namespace mode {

ModeBase::ModeBase(const ModeEnum modeEnum) : modeEnum_(modeEnum) {}

ModeEnum ModeBase::getModeEnum() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return modeEnum_;
}

std::string ModeBase::getName() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return modeEnumToName(modeEnum_);
}

void ModeBase::setPidGains(const PidGainsF& pidGains) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  pidGains_ = pidGains;
}

common::Optional<PidGainsF> ModeBase::getPidGains() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return pidGains_;
}

}  // namespace mode
}  // namespace anydrive
