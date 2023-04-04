#pragma once

#include <chrono>
#include <mutex>
#include <vector>

#include "anydrive/common/Macros.hpp"
#include "anydrive/fsm/StateEnum.hpp"
#include "anydrive/mode/ModeEnum.hpp"

namespace anydrive {

class Statusword {
 public:
  struct DataBits  // See drive firmware: motordrive_def.h
  {
    uint32_t stateId_ : 4;  // FSM Status: Bits 0-3
    uint32_t modeId_ : 4;   // Operation Mode: 4-7

    uint32_t warningOvertemperatureBridge_ : 1;
    uint32_t warningOvertemperatureStator_ : 1;
    uint32_t warningOvertemperatureCpu_ : 1;
    uint32_t warningEncoderOutlierMotor_ : 1;
    uint32_t warningEncoderOutlierGear_ : 1;
    uint32_t warningEncoderOutlierJoint_ : 1;

    uint32_t errorInvalidJointTorque_ : 1;
    uint32_t errorPdoTimeout_ : 1;

    uint32_t fatalOvertemperatureBridge_ : 1;
    uint32_t fatalOvertemperatureStator_ : 1;
    uint32_t fatalOvertemperatureCpu_ : 1;

    uint32_t errorInvalidGearPosition_ : 1;
    uint32_t errorInvalidJointPosition_ : 1;
    uint32_t errorJointPositionLimitsSoft_ : 1;
    uint32_t errorJointPositionLimitsHard_ : 1;

    uint32_t warningIncompleteCalibration_ : 1;

    uint32_t warningEncoderCrcGear_ : 1;
    uint32_t warningEncoderCrcJoint_ : 1;

    uint32_t fatalMotorEncoder_ : 1;
    uint32_t fatalCurrentSensor_ : 1;

    uint32_t fatalOvervoltage_ : 1;
    uint32_t fatalUndervoltage_ : 1;

    uint32_t unused_ : 1;

    uint32_t fatalInvalidMotion_ : 1;
  };

  union Data {
    DataBits bits_;
    uint32_t all_ = 0;

    Data() = default;
    Data(const uint32_t data);  // NOLINT

    bool operator==(const Data& other);
    bool operator!=(const Data& other);
  };

 protected:
  using TimePoint = std::chrono::system_clock::time_point;
  using Duration = std::chrono::duration<double>;

  mutable std::recursive_mutex mutex_;
  TimePoint stamp_;
  Data data_;

 public:
  Statusword() = default;
  Statusword(const Statusword& statusword);
  explicit Statusword(const uint32_t data);
  virtual ~Statusword() = default;

  Statusword& operator=(const Statusword& statusword);

  bool isEmpty() const;

  double getAge() const;
  TimePoint getStamp() const;

  void setData(const uint32_t data);
  uint32_t getData() const;

  fsm::StateEnum getStateEnum() const;
  void setStateEnum(const fsm::StateEnum stateEnum);

  mode::ModeEnum getModeEnum() const;
  void setModeEnum(const mode::ModeEnum modeEnum);

  void getMessages(std::vector<std::string>& infos, std::vector<std::string>& warnings, std::vector<std::string>& errors,
                   std::vector<std::string>& fatals) const;
  void getMessagesDiff(Statusword& previousStatusword, std::vector<std::string>& infos, std::vector<std::string>& warnings,
                       std::vector<std::string>& errors, std::vector<std::string>& fatals) const;

  bool hasWarningHighTemperatureBridge() const;
  bool hasWarningHighTemperatureStator() const;
  bool hasWarningHighTemperatureCpu() const;
  bool hasWarningEncoderOutlierMotor() const;
  bool hasWarningEncoderOutlierGear() const;
  bool hasWarningEncoderOutlierJoint() const;
  bool hasErrorInvalidJointTorque() const;
  bool hasErrorPdoTimeout() const;
  bool hasFatalOvertemperatureBridge() const;
  bool hasFatalOvertemperatureStator() const;
  bool hasFatalOvertemperatureCpu() const;
  bool hasErrorInvalidGearPosition() const;
  bool hasErrorInvalidJointPosition() const;
  bool hasErrorJointPositionLimitsSoft() const;
  bool hasFatalJointPositionLimitsHard() const;
  bool hasWarningIncompleteCalibration() const;
  bool hasWarningEncoderCrcGear() const;
  bool hasWarningEncoderCrcJoint() const;
  bool hasFatalMotorEncoder() const;
  bool hasFatalCurrentSensor() const;
  bool hasFatalOvervoltage() const;
  bool hasFatalUndervoltage() const;
  bool hasFatalInvalidMotion() const;
};

std::ostream& operator<<(std::ostream& os, const Statusword& statusword);

}  // namespace anydrive
