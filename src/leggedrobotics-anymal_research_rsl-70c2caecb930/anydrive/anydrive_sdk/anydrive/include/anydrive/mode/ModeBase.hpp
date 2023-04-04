#pragma once

#include <memory>
#include <mutex>

#include "anydrive/common/Optional.hpp"
#include "anydrive/mode/ModeEnum.hpp"
#include "anydrive/mode/PidGains.hpp"

namespace anydrive {
namespace mode {

class ModeBase {
 protected:
  mutable std::recursive_mutex mutex_;

  ModeEnum modeEnum_;

  common::Optional<PidGainsF> pidGains_;

 public:
  bool controlCurrent_ = false;
  bool controlMotorPosition_ = false;
  bool controlMotorVelocity_ = false;
  bool controlGearPosition_ = false;
  bool controlGearVelocity_ = false;
  bool controlJointPosition_ = false;
  bool controlJointVelocity_ = false;
  bool controlJointTorque_ = false;
  bool customGains_ = false;

 protected:
  explicit ModeBase(const ModeEnum modeEnum);
  virtual ~ModeBase() = default;

 public:
  ModeEnum getModeEnum() const;
  std::string getName() const;

  void setPidGains(const PidGainsF& pidGains);
  common::Optional<PidGainsF> getPidGains() const;
};

using ModeBasePtr = std::shared_ptr<ModeBase>;

}  // namespace mode
}  // namespace anydrive
