// anydrive
#include "anydrive/Statusword.hpp"

namespace anydrive {

Statusword::Data::Data(const uint32_t data) : all_(data) {}

bool Statusword::Data::operator==(const Data& other) {
  return all_ == other.all_;  // NOLINT(cppcoreguidelines-pro-type-union-access)
}

bool Statusword::Data::operator!=(const Data& other) {
  return !(*this == other);
}

Statusword::Statusword(const Statusword& statusword) : stamp_(statusword.getStamp()), data_(statusword.getData()) {}

Statusword::Statusword(uint32_t data) {
  setData(data);
}

Statusword& Statusword::operator=(const Statusword& statusword) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  stamp_ = statusword.getStamp();
  data_ = statusword.getData();
  return *this;
}

bool Statusword::isEmpty() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return stamp_ == TimePoint();
}

double Statusword::getAge() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  const Duration age = std::chrono::system_clock::now() - stamp_;
  return age.count();
}

Statusword::TimePoint Statusword::getStamp() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return stamp_;
}

void Statusword::setData(const uint32_t data) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  stamp_ = std::chrono::system_clock::now();
  data_ = Data(data);
}

uint32_t Statusword::getData() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.all_;  // NOLINT(cppcoreguidelines-pro-type-union-access)
}

fsm::StateEnum Statusword::getStateEnum() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return fsm::stateIdToEnum(data_.bits_.stateId_);  // NOLINT(cppcoreguidelines-pro-type-union-access)
}

void Statusword::setStateEnum(const fsm::StateEnum stateEnum) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  stamp_ = std::chrono::system_clock::now();
  data_.bits_.stateId_ = fsm::stateEnumToId(stateEnum);  // NOLINT(cppcoreguidelines-pro-type-union-access)
}

mode::ModeEnum Statusword::getModeEnum() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return mode::modeIdToEnum(data_.bits_.modeId_);  // NOLINT(cppcoreguidelines-pro-type-union-access)
}

void Statusword::setModeEnum(const mode::ModeEnum modeEnum) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  stamp_ = std::chrono::system_clock::now();
  data_.bits_.modeId_ = mode::modeEnumToId(modeEnum);  // NOLINT(cppcoreguidelines-pro-type-union-access)
}

void Statusword::getMessages(std::vector<std::string>& infos, std::vector<std::string>& warnings, std::vector<std::string>& errors,
                             std::vector<std::string>& fatals) const {
  Statusword previousStatusword;
  return getMessagesDiff(previousStatusword, infos, warnings, errors, fatals);
}

void Statusword::getMessagesDiff(Statusword& previousStatusword, std::vector<std::string>& infos, std::vector<std::string>& warnings,
                                 std::vector<std::string>& errors, std::vector<std::string>& fatals) const {
  // Warnings.
  if (!previousStatusword.hasWarningHighTemperatureBridge() && hasWarningHighTemperatureBridge()) {
    warnings.emplace_back("Warning: High power stage temperature.");
  } else if (previousStatusword.hasWarningHighTemperatureBridge() && !hasWarningHighTemperatureBridge()) {
    infos.emplace_back("Warning disappeared: High power stage temperature.");
  }
  if (!previousStatusword.hasWarningHighTemperatureStator() && hasWarningHighTemperatureStator()) {
    warnings.emplace_back("Warning: High electric motor temperature.");
  } else if (previousStatusword.hasWarningHighTemperatureStator() && !hasWarningHighTemperatureStator()) {
    infos.emplace_back("Warning disappeared: High electric motor temperature.");
  }
  if (!previousStatusword.hasWarningHighTemperatureCpu() && hasWarningHighTemperatureCpu()) {
    warnings.emplace_back("Warning: High CPU temperature.");
  } else if (previousStatusword.hasWarningHighTemperatureCpu() && !hasWarningHighTemperatureCpu()) {
    infos.emplace_back("Warning disappeared: High CPU temperature.");
  }
  if (!previousStatusword.hasWarningEncoderOutlierMotor() && hasWarningEncoderOutlierMotor()) {
    warnings.emplace_back("Warning: Motor encoder outlier.");
  } else if (previousStatusword.hasWarningEncoderOutlierMotor() && !hasWarningEncoderOutlierMotor()) {
    infos.emplace_back("Warning disappeared: Motor encoder outlier.");
  }
  if (!previousStatusword.hasWarningEncoderOutlierGear() && hasWarningEncoderOutlierGear()) {
    warnings.emplace_back("Warning: Gear encoder outlier.");
  } else if (previousStatusword.hasWarningEncoderOutlierGear() && !hasWarningEncoderOutlierGear()) {
    infos.emplace_back("Warning disappeared: Gear encoder outlier.");
  }
  if (!previousStatusword.hasWarningEncoderOutlierJoint() && hasWarningEncoderOutlierJoint()) {
    warnings.emplace_back("Warning: Joint encoder outlier.");
  } else if (previousStatusword.hasWarningEncoderOutlierJoint() && !hasWarningEncoderOutlierJoint()) {
    infos.emplace_back("Warning disappeared: Joint encoder outlier.");
  }
  if (!previousStatusword.hasWarningIncompleteCalibration() && hasWarningIncompleteCalibration()) {
    warnings.emplace_back("Warning: Incomplete calibration.");
  } else if (previousStatusword.hasWarningIncompleteCalibration() && !hasWarningIncompleteCalibration()) {
    infos.emplace_back("Warning disappeared: Incomplete calibration.");
  }
  if (!previousStatusword.hasWarningEncoderCrcGear() && hasWarningEncoderCrcGear()) {
    warnings.emplace_back("Warning: Invalid gear encoder CRC.");
  } else if (previousStatusword.hasWarningEncoderCrcGear() && !hasWarningEncoderCrcGear()) {
    infos.emplace_back("Warning disappeared: Invalid gear encoder CRC.");
  }
  if (!previousStatusword.hasWarningEncoderCrcJoint() && hasWarningEncoderCrcJoint()) {
    warnings.emplace_back("Warning: Invalid joint encoder CRC.");
  } else if (previousStatusword.hasWarningEncoderCrcJoint() && !hasWarningEncoderCrcJoint()) {
    infos.emplace_back("Warning disappeared: Invalid joint encoder CRC.");
  }

  // Errors.
  if (!previousStatusword.hasErrorInvalidJointTorque() && hasErrorInvalidJointTorque()) {
    errors.emplace_back("Error: Invalid joint torque.");
  } else if (previousStatusword.hasErrorInvalidJointTorque() && !hasErrorInvalidJointTorque()) {
    infos.emplace_back("Error disappeared: Invalid joint torque.");
  }
  if (!previousStatusword.hasErrorPdoTimeout() && hasErrorPdoTimeout()) {
    errors.emplace_back("Error: PDO timeout.");
  } else if (previousStatusword.hasErrorPdoTimeout() && !hasErrorPdoTimeout()) {
    infos.emplace_back("Error disappeared: PDO timeout.");
  }
  if (!previousStatusword.hasErrorInvalidGearPosition() && hasErrorInvalidGearPosition()) {
    errors.emplace_back("Error: Invalid gear position.");
  } else if (previousStatusword.hasErrorInvalidGearPosition() && !hasErrorInvalidGearPosition()) {
    infos.emplace_back("Error disappeared: Invalid gear position.");
  }
  if (!previousStatusword.hasErrorInvalidJointPosition() && hasErrorInvalidJointPosition()) {
    errors.emplace_back("Error: Invalid joint position.");
  } else if (previousStatusword.hasErrorInvalidJointPosition() && !hasErrorInvalidJointPosition()) {
    infos.emplace_back("Error disappeared: Invalid joint position.");
  }
  if (!previousStatusword.hasErrorJointPositionLimitsSoft() && hasErrorJointPositionLimitsSoft()) {
    errors.emplace_back("Error: Reached soft joint position limits.");
  } else if (previousStatusword.hasErrorJointPositionLimitsSoft() && !hasErrorJointPositionLimitsSoft()) {
    infos.emplace_back("Error disappeared: Reached soft joint position limits.");
  }

  // Fatals.
  if (!previousStatusword.hasFatalOvertemperatureBridge() && hasFatalOvertemperatureBridge()) {
    fatals.emplace_back("Fatal: Power stage overtemperature.");
  } else if (previousStatusword.hasFatalOvertemperatureBridge() && !hasFatalOvertemperatureBridge()) {
    infos.emplace_back("Fatal disappeared: Power stage overtemperature.");
  }
  if (!previousStatusword.hasFatalOvertemperatureStator() && hasFatalOvertemperatureStator()) {
    fatals.emplace_back("Fatal: Electric motor overtemperature.");
  } else if (previousStatusword.hasFatalOvertemperatureStator() && !hasFatalOvertemperatureStator()) {
    infos.emplace_back("Fatal disappeared: Electric motor overtemperature.");
  }
  if (!previousStatusword.hasFatalOvertemperatureCpu() && hasFatalOvertemperatureCpu()) {
    fatals.emplace_back("Fatal: CPU overtemperature.");
  } else if (previousStatusword.hasFatalOvertemperatureCpu() && !hasFatalOvertemperatureCpu()) {
    infos.emplace_back("Fatal disappeared: CPU overtemperature.");
  }
  if (!previousStatusword.hasFatalJointPositionLimitsHard() && hasFatalJointPositionLimitsHard()) {
    fatals.emplace_back("Fatal: Reached hard joint position limits.");
  } else if (previousStatusword.hasFatalJointPositionLimitsHard() && !hasFatalJointPositionLimitsHard()) {
    infos.emplace_back("Fatal disappeared: Reached hard joint position limits.");
  }
  if (!previousStatusword.hasFatalMotorEncoder() && hasFatalMotorEncoder()) {
    fatals.emplace_back("Fatal: Motor encoder not working.");
  } else if (previousStatusword.hasFatalMotorEncoder() && !hasFatalMotorEncoder()) {
    infos.emplace_back("Fatal disappeared: Motor encoder not working.");
  }
  if (!previousStatusword.hasFatalCurrentSensor() && hasFatalCurrentSensor()) {
    fatals.emplace_back("Fatal: Current sensor not working.");
  } else if (previousStatusword.hasFatalCurrentSensor() && !hasFatalCurrentSensor()) {
    infos.emplace_back("Fatal disappeared: Current sensor not working.");
  }
  if (!previousStatusword.hasFatalOvervoltage() && hasFatalOvervoltage()) {
    fatals.emplace_back("Fatal: Drive input overvoltage.");
  } else if (previousStatusword.hasFatalOvervoltage() && !hasFatalOvervoltage()) {
    infos.emplace_back("Fatal disappeared: Drive input overvoltage.");
  }
  if (!previousStatusword.hasFatalUndervoltage() && hasFatalUndervoltage()) {
    fatals.emplace_back("Fatal: Drive input undervoltage.");
  } else if (previousStatusword.hasFatalUndervoltage() && !hasFatalUndervoltage()) {
    infos.emplace_back("Fatal disappeared: Drive input undervoltage.");
  }
  if (!previousStatusword.hasFatalInvalidMotion() && hasFatalInvalidMotion()) {
    fatals.emplace_back("Fatal: Invalid motion.");
  } else if (previousStatusword.hasFatalInvalidMotion() && !hasFatalInvalidMotion()) {
    infos.emplace_back("Fatal disappeared: Invalid motion.");
  }
}

bool Statusword::hasWarningHighTemperatureBridge() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.warningOvertemperatureBridge_ != 0u;  // NOLINT(cppcoreguidelines-pro-type-union-access)
}

bool Statusword::hasWarningHighTemperatureStator() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.warningOvertemperatureStator_ != 0u;  // NOLINT(cppcoreguidelines-pro-type-union-access)
}

bool Statusword::hasWarningHighTemperatureCpu() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.warningOvertemperatureCpu_ != 0u;  // NOLINT(cppcoreguidelines-pro-type-union-access)
}

bool Statusword::hasWarningEncoderOutlierMotor() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.warningEncoderOutlierMotor_ != 0u;  // NOLINT(cppcoreguidelines-pro-type-union-access)
}

bool Statusword::hasWarningEncoderOutlierGear() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.warningEncoderOutlierGear_ != 0u;  // NOLINT(cppcoreguidelines-pro-type-union-access)
}

bool Statusword::hasWarningEncoderOutlierJoint() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.warningEncoderOutlierJoint_ != 0u;  // NOLINT(cppcoreguidelines-pro-type-union-access)
}

bool Statusword::hasErrorInvalidJointTorque() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.errorInvalidJointTorque_ != 0u;  // NOLINT(cppcoreguidelines-pro-type-union-access)
}

bool Statusword::hasErrorPdoTimeout() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.errorPdoTimeout_ != 0u;  // NOLINT(cppcoreguidelines-pro-type-union-access)
}

bool Statusword::hasFatalOvertemperatureBridge() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.fatalOvertemperatureBridge_ != 0u;  // NOLINT(cppcoreguidelines-pro-type-union-access)
}

bool Statusword::hasFatalOvertemperatureStator() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.fatalOvertemperatureStator_ != 0u;  // NOLINT(cppcoreguidelines-pro-type-union-access)
}

bool Statusword::hasFatalOvertemperatureCpu() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.fatalOvertemperatureCpu_ != 0u;  // NOLINT(cppcoreguidelines-pro-type-union-access)
}

bool Statusword::hasErrorInvalidGearPosition() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.errorInvalidGearPosition_ != 0u;  // NOLINT(cppcoreguidelines-pro-type-union-access)
}

bool Statusword::hasErrorInvalidJointPosition() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.errorInvalidJointPosition_ != 0u;  // NOLINT(cppcoreguidelines-pro-type-union-access)
}

bool Statusword::hasErrorJointPositionLimitsSoft() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.errorJointPositionLimitsSoft_ != 0u;  // NOLINT(cppcoreguidelines-pro-type-union-access)
}

bool Statusword::hasFatalJointPositionLimitsHard() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.errorJointPositionLimitsHard_ != 0u;  // NOLINT(cppcoreguidelines-pro-type-union-access)
}

bool Statusword::hasWarningIncompleteCalibration() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.warningIncompleteCalibration_ != 0u;  // NOLINT(cppcoreguidelines-pro-type-union-access)
}

bool Statusword::hasWarningEncoderCrcGear() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.warningEncoderCrcGear_ != 0u;  // NOLINT(cppcoreguidelines-pro-type-union-access)
}

bool Statusword::hasWarningEncoderCrcJoint() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.warningEncoderCrcJoint_ != 0u;  // NOLINT(cppcoreguidelines-pro-type-union-access)
}

bool Statusword::hasFatalMotorEncoder() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.fatalMotorEncoder_ != 0u;  // NOLINT(cppcoreguidelines-pro-type-union-access)
}

bool Statusword::hasFatalCurrentSensor() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.fatalCurrentSensor_ != 0u;  // NOLINT(cppcoreguidelines-pro-type-union-access)
}

bool Statusword::hasFatalOvervoltage() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.fatalOvervoltage_ != 0u;  // NOLINT(cppcoreguidelines-pro-type-union-access)
}

bool Statusword::hasFatalUndervoltage() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.fatalUndervoltage_ != 0u;  // NOLINT(cppcoreguidelines-pro-type-union-access)
}

bool Statusword::hasFatalInvalidMotion() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  return data_.bits_.fatalInvalidMotion_ != 0u;  // NOLINT(cppcoreguidelines-pro-type-union-access)
}

std::ostream& operator<<(std::ostream& os, const Statusword& statusword) {
  for (uint32_t i = 8 * sizeof(uint32_t); i > uint32_t(0); i--) {
    os << ((statusword.getData() & (uint32_t(1) << (i - uint32_t(1)))) != 0u ? "1" : "0");
  }
  return os;
}

}  // namespace anydrive
