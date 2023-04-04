#include <anydrive/Anydrive.hpp>
#include <anydrive/thread_sleep.hpp>

#include "anydrive_ethercat/AnydriveEthercatSlave.hpp"
#include "anydrive_ethercat/RxPdo.hpp"
#include "anydrive_ethercat/TxPdoA.hpp"
#include "anydrive_ethercat/TxPdoB.hpp"
#include "anydrive_ethercat/TxPdoC.hpp"
#include "anydrive_ethercat/TxPdoD.hpp"

namespace anydrive_ethercat {

std::map<anydrive::mode::ModeEnum, uint16_t> AnydriveEthercatSlave::modeEnumToOdIndex_ = {
    {anydrive::mode::ModeEnum::Current, OD_GAINS_CURRENT_CTRL_ID},
    {anydrive::mode::ModeEnum::MotorPosition, OD_GAINS_MOTOR_POSITION_CTRL_ID},
    {anydrive::mode::ModeEnum::MotorVelocity, OD_GAINS_MOTOR_VELOCITY_CTRL_ID},
    {anydrive::mode::ModeEnum::GearPosition, OD_GAINS_GEAR_POSITION_CTRL_ID},
    {anydrive::mode::ModeEnum::GearVelocity, OD_GAINS_GEAR_VELOCITY_CTRL_ID},
    {anydrive::mode::ModeEnum::JointPosition, OD_GAINS_JOINT_POSITION_CTRL_ID},
    {anydrive::mode::ModeEnum::JointVelocity, OD_GAINS_JOINT_VELOCITY_CTRL_ID},
    {anydrive::mode::ModeEnum::JointTorque, OD_GAINS_JOINT_TORQUE_CTRL_ID},
    {anydrive::mode::ModeEnum::JointPositionVelocity, OD_GAINS_JOINT_POSITION_VELOCITY_CTRL_ID},
    {anydrive::mode::ModeEnum::JointPositionVelocityTorque, OD_GAINS_JOINT_POSITION_VELOCITY_TORQUE_CTRL_ID},
    {anydrive::mode::ModeEnum::JointPositionVelocityTorquePidGains, OD_GAINS_JOINT_POSITION_VELOCITY_TORQUE_PID_GAINS_CTRL_ID}};

AnydriveEthercatSlave::AnydriveEthercatSlave(const anydrive::AnydriveWeakPtr& anydrive, soem_interface::EthercatBusBase* bus,
                                             const uint32_t address, const PdoTypeEnum pdoTypeEnum)
    : soem_interface::EthercatSlaveBase(bus, address),
      anydrive::communication::CommunicationInterfaceBase(anydrive),
      pdoTypeEnum_(pdoTypeEnum),
      currentPdoTypeEnum_(PdoTypeEnum::NA) {
  PdoInfo pdoInfo;

  pdoInfo.rxPdoId_ = OD_DSP402_RX_PDO_SID_VAL_A;
  pdoInfo.txPdoId_ = OD_DSP402_TX_PDO_SID_VAL_A;
  pdoInfo.rxPdoSize_ = sizeof(RxPdo);
  pdoInfo.txPdoSize_ = sizeof(TxPdoA);
  pdoInfo.moduleId_ = 0x00119800;
  pdoInfos_.insert({PdoTypeEnum::A, pdoInfo});

  pdoInfo.rxPdoId_ = OD_DSP402_RX_PDO_SID_VAL_B;
  pdoInfo.txPdoId_ = OD_DSP402_TX_PDO_SID_VAL_B;
  pdoInfo.rxPdoSize_ = sizeof(RxPdo);
  pdoInfo.txPdoSize_ = sizeof(TxPdoB);
  pdoInfo.moduleId_ = 0x00219800;
  pdoInfos_.insert({PdoTypeEnum::B, pdoInfo});

  pdoInfo.rxPdoId_ = OD_DSP402_RX_PDO_SID_VAL_C;
  pdoInfo.txPdoId_ = OD_DSP402_TX_PDO_SID_VAL_C;
  pdoInfo.rxPdoSize_ = sizeof(RxPdo);
  pdoInfo.txPdoSize_ = sizeof(TxPdoC);
  pdoInfo.moduleId_ = 0x00319800;
  pdoInfos_.insert({PdoTypeEnum::C, pdoInfo});

  pdoInfo.rxPdoId_ = OD_DSP402_RX_PDO_SID_VAL_D;
  pdoInfo.txPdoId_ = OD_DSP402_TX_PDO_SID_VAL_D;
  pdoInfo.rxPdoSize_ = sizeof(RxPdo);
  pdoInfo.txPdoSize_ = sizeof(TxPdoD);
  pdoInfo.moduleId_ = 0x00419800;
  pdoInfos_.insert({PdoTypeEnum::D, pdoInfo});
}

std::string AnydriveEthercatSlave::getName() const {
  RecLock lock(mutex_);
  auto anydrive = anydrive_.lock();
  if (!anydrive) {
    ANYDRIVE_NAMED_ERROR("The anydrive pointer is empty.");
    return "";
  }

  return anydrive->getName();
}

bool AnydriveEthercatSlave::startup() {
  RecLock lock(mutex_);

  // Configure PDO setup
  return configurePdo(pdoTypeEnum_);
}

void AnydriveEthercatSlave::updateRead() {
  TxPdoA txPdoA;
  TxPdoB txPdoB;
  TxPdoC txPdoC;
  TxPdoD txPdoD;

  const any_measurements::Time updateStamp = bus_->getUpdateReadStamp();
  switch (getCurrentPdoTypeEnum()) {
    case PdoTypeEnum::A:
      bus_->readTxPdo(address_, txPdoA);
      {
        RecLock lockReading(readingMutex_);
        reading_.getState().setStamp(updateStamp);
        reading_.getState().setStatusword(anydrive::Statusword(txPdoA.statusword_));
        reading_.getState().setTemperature(temperatureScalingInv_ * txPdoA.measuredTemperature_ + temperatureOffset_);
        reading_.getState().setVoltage(motorVoltageScalingInv_ * txPdoA.measuredMotorVoltage_);
        reading_.getState().setMotorPosition(motorPositionScalingInv_ * txPdoA.measuredMotorPosition_);
        reading_.getState().setGearPosition(gearPositionScalingInv_ * txPdoA.measuredGearPosition_);
        reading_.getState().setJointPosition(jointPositionScalingInv_ * txPdoA.measuredJointPosition_);
        reading_.getState().setCurrent(motorCurrentScalingInv_ * txPdoA.measuredMotorCurrent_);
        reading_.getState().setMotorVelocity(motorVelocityScalingInv_ * txPdoA.measuredMotorVelocity_);
        reading_.getState().setGearVelocity(gearVelocityScalingInv_ * txPdoA.measuredGearVelocity_);
        reading_.getState().setJointVelocity(jointVelocityScalingInv_ * txPdoA.measuredJointVelocity_);
        reading_.getState().setJointAcceleration(jointAccelerationScalingInv_ * txPdoA.measuredJointAcceleration_);
        reading_.getState().setJointTorque(jointTorqueScalingInv_ * txPdoA.measuredJointTorque_);
      }
      break;
    case PdoTypeEnum::B:
      bus_->readTxPdo(address_, txPdoB);
      {
        RecLock lockReading(readingMutex_);
        reading_.getState().setStamp(updateStamp);
        reading_.getState().setStatusword(anydrive::Statusword(txPdoB.statusword_));
        reading_.getState().setTemperature(temperatureScalingInv_ * txPdoB.measuredTemperature_ + temperatureOffset_);
        reading_.getState().setVoltage(motorVoltageScalingInv_ * txPdoB.measuredMotorVoltage_);
        reading_.getState().setMotorPosition(motorPositionScalingInv_ * txPdoB.measuredMotorPosition_);
        reading_.getState().setGearPosition(gearPositionScalingInv_ * txPdoB.measuredGearPosition_);
        reading_.getState().setJointPosition(jointPositionScalingInv_ * txPdoB.measuredJointPosition_);
        reading_.getState().setCurrent(motorCurrentScalingInv_ * txPdoB.measuredMotorCurrent_);
        reading_.getState().setMotorVelocity(motorVelocityScalingInv_ * txPdoB.measuredMotorVelocity_);
        reading_.getState().setGearVelocity(gearVelocityScalingInv_ * txPdoB.measuredGearVelocity_);
        reading_.getState().setJointVelocity(jointVelocityScalingInv_ * txPdoB.measuredJointVelocity_);
        reading_.getState().setJointAcceleration(jointAccelerationScalingInv_ * txPdoB.measuredJointAcceleration_);
        reading_.getState().setJointTorque(jointTorqueScalingInv_ * txPdoB.measuredJointTorque_);
        any_measurements::Imu imu;
        imu.time_ = updateStamp;
        imu.linearAcceleration_.x() = txPdoB.measuredImuLinearAccelerationX_;
        imu.linearAcceleration_.y() = txPdoB.measuredImuLinearAccelerationY_;
        imu.linearAcceleration_.z() = txPdoB.measuredImuLinearAccelerationZ_;
        imu.angularVelocity_.x() = txPdoB.measuredImuAngularVelocityX_;
        imu.angularVelocity_.y() = txPdoB.measuredImuAngularVelocityY_;
        imu.angularVelocity_.z() = txPdoB.measuredImuAngularVelocityZ_;
        // 7th IMU value is not read out at the moment.
        reading_.getState().setImu(imu);
      }
      break;
    case PdoTypeEnum::C:
      bus_->readTxPdo(address_, txPdoC);
      {
        RecLock lockReading(readingMutex_);
        reading_.getState().setStamp(updateStamp);
        reading_.getState().setStatusword(anydrive::Statusword(txPdoC.statusword_));
        reading_.getState().setTemperature(temperatureScalingInv_ * txPdoC.measuredTemperature_ + temperatureOffset_);
        reading_.getState().setVoltage(motorVoltageScalingInv_ * txPdoC.measuredMotorVoltage_);
        reading_.getState().setMotorPosition(motorPositionScalingInv_ * txPdoC.measuredMotorPosition_);
        reading_.getState().setGearPosition(gearPositionScalingInv_ * txPdoC.measuredGearPosition_);
        reading_.getState().setJointPosition(jointPositionScalingInv_ * txPdoC.measuredJointPosition_);
        reading_.getState().setCurrent(motorCurrentScalingInv_ * txPdoC.measuredMotorCurrent_);
        reading_.getState().setMotorVelocity(motorVelocityScalingInv_ * txPdoC.measuredMotorVelocity_);
        reading_.getState().setGearVelocity(gearVelocityScalingInv_ * txPdoC.measuredGearVelocity_);
        reading_.getState().setJointVelocity(jointVelocityScalingInv_ * txPdoC.measuredJointVelocity_);
        reading_.getState().setJointAcceleration(jointAccelerationScalingInv_ * txPdoC.measuredJointAcceleration_);
        reading_.getState().setJointTorque(jointTorqueScalingInv_ * txPdoC.measuredJointTorque_);
        reading_.getState().setGearPositionTicks(txPdoC.measuredGearPositionTicks_);
        reading_.getState().setJointPositionTicks(txPdoC.measuredJointPositionTicks_);
        reading_.getState().setTimestamp(txPdoC.timestamp_);
        reading_.getState().setDesiredCurrentD(txPdoC.desiredCurrentD_);
        reading_.getState().setMeasuredCurrentD(txPdoC.measuredCurrentD_);
        reading_.getState().setDesiredCurrentQ(txPdoC.desiredCurrentQ_);
        reading_.getState().setMeasuredCurrentQ(txPdoC.measuredCurrentQ_);
        reading_.getState().setAlpha(txPdoC.alpha_);
        reading_.getState().setBeta(txPdoC.beta_);
        reading_.getState().setDutyCycleU(txPdoC.dutyCycleU_);
        reading_.getState().setDutyCycleV(txPdoC.dutyCycleV_);
        reading_.getState().setDutyCycleW(txPdoC.dutyCycleW_);
        reading_.getState().setMeasuredCurrentPhaseU(txPdoC.measuredCurrentPhaseU_);
        reading_.getState().setMeasuredCurrentPhaseV(txPdoC.measuredCurrentPhaseV_);
        reading_.getState().setMeasuredCurrentPhaseW(txPdoC.measuredCurrentPhaseW_);
        reading_.getState().setMeasuredVoltagePhaseU(txPdoC.measuredVoltagePhaseU_);
        reading_.getState().setMeasuredVoltagePhaseV(txPdoC.measuredVoltagePhaseV_);
        reading_.getState().setMeasuredVoltagePhaseW(txPdoC.measuredVoltagePhaseW_);
        reading_.getCommanded().setStamp(updateStamp);
        reading_.getCommanded().setCurrent(txPdoC.desiredCurrentQ_);
        reading_.getCommanded().setMotorVelocity(motorVelocityScalingInv_ * txPdoC.desiredMotorVelocity_);
        reading_.getCommanded().setGearPosition(gearPositionScalingInv_ * txPdoC.desiredGearPosition_);
        reading_.getCommanded().setGearVelocity(gearVelocityScalingInv_ * txPdoC.desiredGearVelocity_);
        reading_.getCommanded().setJointPosition(jointPositionScalingInv_ * txPdoC.desiredJointPosition_);
        reading_.getCommanded().setJointVelocity(jointVelocityScalingInv_ * txPdoC.desiredJointVelocity_);
        reading_.getCommanded().setJointTorque(jointTorqueScalingInv_ * txPdoC.desiredJointTorque_);
        any_measurements::Imu imu;
        imu.time_ = updateStamp;
        imu.linearAcceleration_.x() = txPdoC.measuredImuLinearAccelerationX_;
        imu.linearAcceleration_.y() = txPdoC.measuredImuLinearAccelerationY_;
        imu.linearAcceleration_.z() = txPdoC.measuredImuLinearAccelerationZ_;
        imu.angularVelocity_.x() = txPdoC.measuredImuAngularVelocityX_;
        imu.angularVelocity_.y() = txPdoC.measuredImuAngularVelocityY_;
        imu.angularVelocity_.z() = txPdoC.measuredImuAngularVelocityZ_;
        // 7th IMU value is not read out at the moment.
        reading_.getState().setImu(imu);
      }
      break;
    case PdoTypeEnum::D:
      bus_->readTxPdo(address_, txPdoD);
      {
        RecLock lockReading(readingMutex_);
        reading_.getState().setStamp(updateStamp);
        reading_.getState().setStatusword(anydrive::Statusword(txPdoD.statusword_));
        reading_.getState().setTemperature(temperatureScalingInv_ * txPdoD.measuredTemperature_ + temperatureOffset_);
        reading_.getState().setVoltage(motorVoltageScalingInv_ * txPdoD.measuredMotorVoltage_);
        reading_.getState().setMotorPosition(motorPositionScalingInv_ * txPdoD.measuredMotorPosition_);
        reading_.getState().setGearPosition(gearPositionScalingInv_ * txPdoD.measuredGearPosition_);
        reading_.getState().setJointPosition(jointPositionScalingInv_ * txPdoD.measuredJointPosition_);
        reading_.getState().setCurrent(motorCurrentScalingInv_ * txPdoD.measuredMotorCurrent_);
        reading_.getState().setMotorVelocity(motorVelocityScalingInv_ * txPdoD.measuredMotorVelocity_);
        reading_.getState().setGearVelocity(gearVelocityScalingInv_ * txPdoD.measuredGearVelocity_);
        reading_.getState().setJointVelocity(jointVelocityScalingInv_ * txPdoD.measuredJointVelocity_);
        reading_.getState().setJointAcceleration(jointAccelerationScalingInv_ * txPdoD.measuredJointAcceleration_);
        reading_.getState().setJointTorque(jointTorqueScalingInv_ * txPdoD.measuredJointTorque_);
        reading_.getState().setGearPositionTicks(txPdoD.measuredGearPositionTicks_);
        reading_.getState().setJointPositionTicks(txPdoD.measuredJointPositionTicks_);
        reading_.getState().setTimestamp(txPdoD.timestamp_);
        reading_.getState().setDesiredCurrentD(txPdoD.desiredCurrentD_);
        reading_.getState().setMeasuredCurrentD(txPdoD.measuredCurrentD_);
        reading_.getState().setDesiredCurrentQ(txPdoD.desiredCurrentQ_);
        reading_.getState().setMeasuredCurrentQ(txPdoD.measuredCurrentQ_);
        reading_.getState().setMeasuredCurrentPhaseU(txPdoD.measuredCurrentPhaseU_);
        reading_.getState().setMeasuredCurrentPhaseV(txPdoD.measuredCurrentPhaseV_);
        reading_.getState().setMeasuredCurrentPhaseW(txPdoD.measuredCurrentPhaseW_);
        reading_.getState().setMeasuredVoltagePhaseU(txPdoD.measuredVoltagePhaseU_);
        reading_.getState().setMeasuredVoltagePhaseV(txPdoD.measuredVoltagePhaseV_);
        reading_.getState().setMeasuredVoltagePhaseW(txPdoD.measuredVoltagePhaseW_);
      }
      break;
    default:
      break;
  }

  // Save the command in the reading.
  if (!isRtdlRunning_) {
    RecLock lockReading(readingMutex_);
    RecLock lockCommand(commandMutex_);
    reading_.setCommanded(command_);
  }
}

void AnydriveEthercatSlave::updateWrite() {
  std::shared_ptr<anydrive::Anydrive> anydrive = anydrive_.lock();
  if (!anydrive) {
    ANYDRIVE_NAMED_ERROR("The anydrive pointer is empty.");
    return;
  }

  RecLock lock(commandMutex_);
  const anydrive::mode::ModeEnum modeEnum = command_.getModeEnum();
  const uint16_t modeOfOperation = anydrive::mode::modeEnumToId(modeEnum);
  const auto& mode = anydrive->getConfiguration().getMode(modeEnum);
  if (!mode) {
    return;
  }

  RxPdo rxPdo;
  {
    RecLock lock(controlwordIdMutex_);
    rxPdo.controlword_ = controlwordId_;
  }
  rxPdo.modeOfOperation_ = modeOfOperation;
  rxPdo.desiredMotorCurrent_ = motorCurrentScaling_ * command_.getCurrent();
  if (mode->controlMotorVelocity_) {
    rxPdo.desiredVelocity_ = motorVelocityScaling_ * command_.getMotorVelocity();
  } else if (mode->controlGearVelocity_) {
    rxPdo.desiredVelocity_ = gearVelocityScaling_ * command_.getGearVelocity();
  } else if (mode->controlJointVelocity_) {
    rxPdo.desiredVelocity_ = jointVelocityScaling_ * command_.getJointVelocity();
  }
  rxPdo.desiredJointTorque_ = jointTorqueScaling_ * command_.getJointTorque();
  if (mode->controlMotorPosition_) {
    rxPdo.desiredPosition_ = motorPositionScaling_ * command_.getMotorPosition();
  } else if (mode->controlGearPosition_) {
    rxPdo.desiredPosition_ = gearPositionScaling_ * command_.getGearPosition();
  } else if (mode->controlJointPosition_) {
    rxPdo.desiredPosition_ = jointPositionScaling_ * command_.getJointPosition();
  }
  rxPdo.controlParameterA_ = gainScaling_ * command_.getPidGains().getP();
  rxPdo.controlParameterB_ = gainScaling_ * command_.getPidGains().getI();
  rxPdo.controlParameterC_ = gainScaling_ * command_.getPidGains().getD();
  rxPdo.controlParameterD_ = gainScaling_ * 0.0;
  bus_->writeRxPdo(address_, rxPdo);
}

void AnydriveEthercatSlave::shutdown() {}

bool AnydriveEthercatSlave::deviceIsMissing() const {
  return false;  // TODO(remo)
}

void AnydriveEthercatSlave::setState(const uint16_t state) {
  RecLock lock(mutex_);
  bus_->setState(state, address_);
}

bool AnydriveEthercatSlave::waitForState(const uint16_t state) {
  RecLock lock(mutex_);
  return bus_->waitForState(state, address_);
}

PdoTypeEnum AnydriveEthercatSlave::getCurrentPdoTypeEnum() const {
  return currentPdoTypeEnum_;
}

AnydriveEthercatSlave::PdoInfo AnydriveEthercatSlave::getCurrentPdoInfo() const {
  return pdoInfos_.at(currentPdoTypeEnum_);
}

bool AnydriveEthercatSlave::receivesGearAndJointEncoderTicks() const {
  return (getCurrentPdoTypeEnum() == PdoTypeEnum::C || getCurrentPdoTypeEnum() == PdoTypeEnum::D);
}

bool AnydriveEthercatSlave::sendSdoReadGeneric(const std::string& indexString, const std::string& subindexString,
                                               const std::string& valueTypeString, std::string& valueString) {
  // Check the arguments.
  if (indexString.empty()) {
    ANYDRIVE_NAMED_ERROR("The index string is empty.");
    return false;
  }
  if (subindexString.empty() && valueTypeString != "string") {
    ANYDRIVE_NAMED_ERROR("The subindex string is empty.");
    return false;
  }
  if (valueTypeString.empty()) {
    ANYDRIVE_NAMED_ERROR("The value type string is empty.");
    return false;
  }

  // Parse the index.
  std::size_t indexPos = 0;
  const uint16_t index = std::stoi(indexString, &indexPos, 0);
  if (indexPos == 0) {
    ANYDRIVE_NAMED_ERROR("The index string '" << indexString << "' could not be parsed.");
    return false;
  }

  // Parse the subindex.
  std::size_t subindexPos = 0;
  uint8_t subindex = 0;
  if (valueTypeString != "string") {
    subindex = std::stoi(subindexString, &subindexPos, 0);
    if (subindexPos == 0) {
      ANYDRIVE_NAMED_ERROR("The subindex string '" << subindexString << "' could not be parsed.");
      return false;
    }
  }

  // Compare the value type and send the appropriate SDO.
  RecLock lock(mutex_);
  if (valueTypeString == "int8") {
    int8_t value;
    if (!sendSdoRead(index, subindex, false, value)) {
      return false;
    }
    valueString = std::to_string(value);
  } else if (valueTypeString == "int16") {
    int16_t value;
    if (!sendSdoRead(index, subindex, false, value)) {
      return false;
    }
    valueString = std::to_string(value);
  } else if (valueTypeString == "int32") {
    int32_t value;
    if (!sendSdoRead(index, subindex, false, value)) {
      return false;
    }
    valueString = std::to_string(value);
  } else if (valueTypeString == "int64") {
    int64_t value;
    if (!sendSdoRead(index, subindex, false, value)) {
      return false;
    }
    valueString = std::to_string(value);
  } else if (valueTypeString == "uint8") {
    uint8_t value;
    if (!sendSdoRead(index, subindex, false, value)) {
      return false;
    }
    valueString = std::to_string(value);
  } else if (valueTypeString == "uint16") {
    uint16_t value;
    if (!sendSdoRead(index, subindex, false, value)) {
      return false;
    }
    valueString = std::to_string(value);
  } else if (valueTypeString == "uint32") {
    uint32_t value;
    if (!sendSdoRead(index, subindex, false, value)) {
      return false;
    }
    valueString = std::to_string(value);
  } else if (valueTypeString == "uint64") {
    uint64_t value;
    if (!sendSdoRead(index, subindex, false, value)) {
      return false;
    }
    valueString = std::to_string(value);
  } else if (valueTypeString == "float") {
    float value;
    if (!sendSdoRead(index, subindex, false, value)) {
      return false;
    }
    valueString = std::to_string(value);
  } else if (valueTypeString == "double") {
    double value;
    if (!sendSdoRead(index, subindex, false, value)) {
      return false;
    }
    valueString = std::to_string(value);
  } else if (valueTypeString == "string") {
    if (!sendSdoReadString(index, valueString)) {
      return false;
    }
  } else {
    ANYDRIVE_NAMED_ERROR("Type must be int8, int16, int32, int64, uint8, uint16, uint32, uint64, float, double or string.");
    return false;
  }
  return true;
}

bool AnydriveEthercatSlave::sendSdoWriteGeneric(const std::string& indexString, const std::string& subindexString,
                                                const std::string& valueTypeString, const std::string& valueString) {
  // Check the arguments.
  if (indexString.empty()) {
    ANYDRIVE_NAMED_ERROR("The index string is empty.");
    return false;
  }
  if (subindexString.empty() && valueTypeString != "string") {
    ANYDRIVE_NAMED_ERROR("The subindex string is empty.");
    return false;
  }
  if (valueTypeString.empty()) {
    ANYDRIVE_NAMED_ERROR("The value type string is empty.");
    return false;
  }
  if (valueString.empty() && valueTypeString != "string") {
    ANYDRIVE_NAMED_ERROR("The value string is empty.");
    return false;
  }

  // Parse the index.
  std::size_t indexPos = 0;
  const uint16_t index = std::stoi(indexString, &indexPos, 0);
  if (indexPos == 0) {
    ANYDRIVE_NAMED_ERROR("The index string '" << indexString << "' could not be parsed.");
    return false;
  }

  // Parse the subindex.
  std::size_t subindexPos = 0;
  uint8_t subindex = 0;
  if (valueTypeString != "string") {
    subindex = std::stoi(subindexString, &subindexPos, 0);
    if (subindexPos == 0) {
      ANYDRIVE_NAMED_ERROR("The subindex string '" << subindexString << "' could not be parsed.");
      return false;
    }
  }

  // Compare the value type and send the appropriate SDO.
  RecLock lock(mutex_);
  if (valueTypeString == "int8") {
    int8_t value = std::stoi(valueString);
    return sendSdoWrite(index, subindex, false, value);
  } else if (valueTypeString == "int16") {
    int16_t value = std::stoi(valueString);
    return sendSdoWrite(index, subindex, false, value);
  } else if (valueTypeString == "int32") {
    int32_t value = std::stoi(valueString);
    return sendSdoWrite(index, subindex, false, value);
  } else if (valueTypeString == "int64") {
    int64_t value = std::stoi(valueString);
    return sendSdoWrite(index, subindex, false, value);
  } else if (valueTypeString == "uint8") {
    uint8_t value = std::stoi(valueString);
    return sendSdoWrite(index, subindex, false, value);
  } else if (valueTypeString == "uint16") {
    uint16_t value = std::stoi(valueString);
    return sendSdoWrite(index, subindex, false, value);
  } else if (valueTypeString == "uint32") {
    uint32_t value = std::stoi(valueString);
    return sendSdoWrite(index, subindex, false, value);
  } else if (valueTypeString == "uint64") {
    uint64_t value = std::stoi(valueString);
    return sendSdoWrite(index, subindex, false, value);
  } else if (valueTypeString == "float") {
    float value = std::stof(valueString);
    return sendSdoWrite(index, subindex, false, value);
  } else if (valueTypeString == "double") {
    double value = std::stod(valueString);
    return sendSdoWrite(index, subindex, false, value);
  } else if (valueTypeString == "string") {
    return sendSdoWriteString(index, valueString);
  } else {
    ANYDRIVE_NAMED_ERROR("Type must be int8, int16, int32, int64, uint8, uint16, uint32, uint64, float, double or string.");
    return false;
  }
}

bool AnydriveEthercatSlave::getDriveModel(std::string& model) {
  return sendSdoReadVisibleString(OD_DSP402_MODEL_ID, 0, model);
}

bool AnydriveEthercatSlave::getDriveInfoSerialNumber(std::string& serialNumber) {
  return sendSdoReadString(OD_DRIVE_INFO_HARDWARE_SERIAL_NUMBER_ID, serialNumber);
}

bool AnydriveEthercatSlave::setDriveInfoSerialNumber(const std::string& serialNumber) {
  return sendSdoWriteString(OD_DRIVE_INFO_HARDWARE_SERIAL_NUMBER_ID, serialNumber);
}

bool AnydriveEthercatSlave::getDriveInfoName(std::string& name) {
  return sendSdoReadString(OD_DRIVE_INFO_DRIVE_NAME_ID, name);
}

bool AnydriveEthercatSlave::setDriveInfoName(const std::string& name) {
  return sendSdoWriteString(OD_DRIVE_INFO_DRIVE_NAME_ID, name);
}

bool AnydriveEthercatSlave::getDriveInfoId(uint16_t& id) {
  RecLock lock(mutex_);
  return sendSdoRead(OD_DRIVE_INFO_DRIVE_ID_ID, 0x00, false, id);
}

bool AnydriveEthercatSlave::setDriveInfoId(const uint16_t id) {
  RecLock lock(mutex_);
  return sendSdoWrite(OD_DRIVE_INFO_DRIVE_ID_ID, 0x00, false, id);
}

bool AnydriveEthercatSlave::getDriveInfoBootloaderVersion(anydrive::common::Version& bootloaderVersion) {
  std::string bootloaderVersionString;
  if (!sendSdoReadString(OD_DRIVE_INFO_BOOTLOADER_VERSION_ID, bootloaderVersionString)) {
    return false;
  }
  bootloaderVersion.fromString(bootloaderVersionString);
  return true;
}

bool AnydriveEthercatSlave::setDriveInfoBootloaderVersion(const anydrive::common::Version& bootloaderVersion) {
  return sendSdoWriteString(OD_DRIVE_INFO_BOOTLOADER_VERSION_ID, bootloaderVersion.toString());
}

bool AnydriveEthercatSlave::getDriveInfoFirmwareVersion(anydrive::common::Version& firmwareVersion) {
  std::string firmwareVersionString;
  if (!sendSdoReadString(OD_DRIVE_INFO_FIRMWARE_VERSION_ID, firmwareVersionString)) {
    return false;
  }
  firmwareVersion.fromString(firmwareVersionString);
  return true;
}

bool AnydriveEthercatSlave::getDriveFirmwareInfo(anydrive::common::FirmwareInfo& firmwareInfo) {
  uint8_t infoRaw[128];
  uint8_t numberOfSubindexes = 0;
  bool ok = false;

  for (int j = 0; j < 20; ++j) {
    if (sendSdoRead(OD_DRIVE_FIRMWARE_INFO_ID, OD_DRIVE_FIRMWARE_INFO_SID_0, false, numberOfSubindexes)) {
      ok = true;
      break;
    }
  }
  if (!ok) {
    ANYDRIVE_ERROR("Could not read subindex 0 of index 0x7075.");
    return false;
  }

  const uint8_t nChars = numberOfSubindexes * sizeof(uint32_t) / sizeof(char);
  if (nChars != 128) {
    ANYDRIVE_ERROR("Firmware info array has the wrong length (" << static_cast<int>(nChars) << ").");
    return false;
  }

  for (uint8_t i = 0; i < numberOfSubindexes; i++) {
    ok = false;
    for (int j = 0; j < 20; ++j) {
      anydrive::threadSleep(0.0001);
      if (sendSdoRead(OD_DRIVE_FIRMWARE_INFO_ID, OD_DRIVE_FIRMWARE_INFO_SID_DATA + i, false, ((uint32_t*)infoRaw)[i])) {  // NOLINT
        ok = true;
        break;
      }
    }
    if (!ok) {
      ANYDRIVE_ERROR("Error during read of the firmware info.");
      return false;
    }
  }

  firmwareInfo.infoVersion_ = infoRaw[0];

  if (firmwareInfo.infoVersion_ == 3) {
    // Get version.
    std::stringstream versionStream;
    for (int i = 0; i < 3; ++i) {
      versionStream << static_cast<int>(infoRaw[1 + i]);
      if (i < 2) {
        versionStream << ".";
      }
    }
    firmwareInfo.version_ = versionStream.str();

    // Get fw hash.
    std::stringstream hashStream;
    for (int i = 0; i < 16; ++i) {
      if (infoRaw[4 + i] == '\0') {
        break;
      }
      hashStream << std::setfill('0') << std::setw(sizeof(uint8_t) * 2) << std::hex << static_cast<int>(infoRaw[4 + i]);
    }
    firmwareInfo.fwHash_ = hashStream.str();

    // Get channel id.
    std::stringstream channelIdStream;
    for (int i = 0; i < 16; ++i) {
      if (infoRaw[20 + i] == '\0') {
        break;
      }
      channelIdStream << std::setfill('0') << std::setw(sizeof(uint8_t) * 2) << std::hex << static_cast<int>(infoRaw[20 + i]);
    }
    firmwareInfo.channelId_ = channelIdStream.str();

    // Get channel tid.
    std::stringstream channelTidStream;
    for (int i = 0; i < 16; ++i) {
      if (infoRaw[36 + i] == '\0') {
        break;
      }
      channelTidStream << infoRaw[36 + i];
    }
    firmwareInfo.channelTid_ = channelTidStream.str();

    // Get serial number.
    std::stringstream serialNumberStream;
    for (int i = 0; i < 16; ++i) {
      if (infoRaw[52 + i] == '\0') {
        break;
      }
      serialNumberStream << infoRaw[52 + i];
    }
    firmwareInfo.serialNumber_ = serialNumberStream.str();

    // Get key id.
    std::stringstream keyIdStream;
    for (int i = 0; i < 16; ++i) {
      keyIdStream << std::setfill('0') << std::setw(sizeof(uint8_t) * 2) << std::hex << static_cast<int>(infoRaw[68 + i]);
      if (i == 3 || i == 5 || i == 7 || i == 9) {
        keyIdStream << "-";
      }
    }
    firmwareInfo.keyId_ = keyIdStream.str();
  } else {
    ANYDRIVE_ERROR("Firmware info version (" << static_cast<int>(firmwareInfo.infoVersion_)
                                             << ") is not compatible with this SDK version. "
                                                "Please update the SDK.")
    return false;
  }

  return true;
}

bool AnydriveEthercatSlave::getGearboxRatio(uint32_t& ratio) {
  return sendSdoReadUInt32(OD_GEARBOX_RATIO_ID, 0x00, false, ratio);
}

bool AnydriveEthercatSlave::eraseFlashStorage() {
  RecLock lock(mutex_);
  ANYDRIVE_NAMED_INFO("Erasing flash storage.");
  return sendSdoWrite(OD_FLASH_ERASE_ID, 0x00, false, static_cast<uint16_t>(OD_FLASH_ERASE_ID_VAL_RUN));
}

bool AnydriveEthercatSlave::resetFlashStorageSections(const uint16_t flashStorageSections) {
  RecLock lock(mutex_);
  ANYDRIVE_NAMED_INFO("Resetting flash storage sections (" << flashStorageSections << ").");
  return sendSdoWrite(OD_FLASH_RESET_ID, 0x00, false, flashStorageSections);
}

bool AnydriveEthercatSlave::getCalibrationState(const anydrive::calibration::CalibrationTypeEnum calibrationTypeEnum,
                                                anydrive::calibration::CalibrationState& calibrationState) {
  RecLock lock(mutex_);
  anydrive::calibration::CalibrationTypeEnum previousCalibrationTypeEnum = anydrive::calibration::CalibrationTypeEnum::NA;
  if (!getCalibrationTypeEnum(previousCalibrationTypeEnum)) {
    return false;
  }
  if (!setCalibrationTypeEnum(calibrationTypeEnum)) {
    return false;
  }
  const bool success =
      sendSdoRead(OD_CALIB_STATES_ID, 0x00, false, calibrationState.all_);  // NOLINT(cppcoreguidelines-pro-type-union-access)
  if (!setCalibrationTypeEnum(previousCalibrationTypeEnum)) {
    return false;
  }
  return success;
}

bool AnydriveEthercatSlave::sendCalibrationGearAndJointEncoderHomingNewJointPosition(const double newJointPosition) {
  RecLock lock(mutex_);
  ANYDRIVE_NAMED_DEBUG("Sending homing calibration new joint position.");
  return sendSdoWrite(OD_CALIB_GEAR_AND_JOINT_ENCODER_HOMING_NEW_JOINT_POSITION_ID, 0x00, false,
                      static_cast<float>(jointPositionScaling_ * newJointPosition));
}

bool AnydriveEthercatSlave::startCalibration(const anydrive::calibration::CalibrationModeEnum calibrationModeEnum) {
  RecLock lock(mutex_);
  ANYDRIVE_NAMED_INFO("Starting calibration '" << anydrive::calibration::calibrationModeEnumToName(calibrationModeEnum) << "'.");
  return sendSdoWrite(OD_CALIB_MODE_ID, 0x00, false, anydrive::calibration::calibrationModeEnumToId(calibrationModeEnum));
}

bool AnydriveEthercatSlave::calibrationIsRunning(bool& running) {
  RecLock lock(mutex_);
  uint16_t calibrationMode = 0;
  if (!sendSdoRead(OD_CALIB_MODE_ID, 0x00, false, calibrationMode)) {
    return false;
  }
  running = (calibrationMode != OD_CALIB_MODE_ID_VAL_IDLE);
  return true;
}

bool AnydriveEthercatSlave::getCalibrationMotorEncoderOffset(const anydrive::calibration::CalibrationTypeEnum calibrationTypeEnum,
                                                             anydrive::calibration::parameter::MotorEncoderOffset& motorEncoderOffset) {
  RecLock lock(mutex_);
  anydrive::calibration::CalibrationTypeEnum previousCalibrationTypeEnum = anydrive::calibration::CalibrationTypeEnum::NA;
  if (!getCalibrationTypeEnum(previousCalibrationTypeEnum)) {
    return false;
  }
  if (!setCalibrationTypeEnum(calibrationTypeEnum)) {
    return false;
  }
  const bool success =
      sendSdoRead(OD_CALIB_MOTOR_ENCODER_PARAMS_ID, OD_CALIB_MOTOR_ENCODER_PARAMS_SID_OFFSET, false, motorEncoderOffset.value_);
  if (!setCalibrationTypeEnum(previousCalibrationTypeEnum)) {
    return false;
  }
  return success;
}

bool AnydriveEthercatSlave::getCalibrationMotorEncoderParameters(
    const anydrive::calibration::CalibrationTypeEnum calibrationTypeEnum,
    anydrive::calibration::parameter::MotorEncoderParameters& motorEncoderParameters) {
  RecLock lock(mutex_);
  anydrive::calibration::CalibrationTypeEnum previousCalibrationTypeEnum = anydrive::calibration::CalibrationTypeEnum::NA;
  if (!getCalibrationTypeEnum(previousCalibrationTypeEnum)) {
    return false;
  }
  if (!setCalibrationTypeEnum(calibrationTypeEnum)) {
    return false;
  }
  bool success = true;
  success &= sendSdoRead(OD_CALIB_MOTOR_ENCODER_PARAMS_ID, OD_CALIB_MOTOR_ENCODER_PARAMS_SID_DGAIN, false, motorEncoderParameters.dGain_);
  success &= sendSdoRead(OD_CALIB_MOTOR_ENCODER_PARAMS_ID, OD_CALIB_MOTOR_ENCODER_PARAMS_SID_DOFFS, false, motorEncoderParameters.dOffs_);
  success &= sendSdoRead(OD_CALIB_MOTOR_ENCODER_PARAMS_ID, OD_CALIB_MOTOR_ENCODER_PARAMS_SID_DOFFC, false, motorEncoderParameters.dOffc_);
  success &= sendSdoRead(OD_CALIB_MOTOR_ENCODER_PARAMS_ID, OD_CALIB_MOTOR_ENCODER_PARAMS_SID_DPH, false, motorEncoderParameters.dPh_);
  success &= sendSdoRead(OD_CALIB_MOTOR_ENCODER_PARAMS_ID, OD_CALIB_MOTOR_ENCODER_PARAMS_SID_AGAIN, false, motorEncoderParameters.aGain_);
  success &= sendSdoRead(OD_CALIB_MOTOR_ENCODER_PARAMS_ID, OD_CALIB_MOTOR_ENCODER_PARAMS_SID_AOFFS, false, motorEncoderParameters.aOffs_);
  success &= sendSdoRead(OD_CALIB_MOTOR_ENCODER_PARAMS_ID, OD_CALIB_MOTOR_ENCODER_PARAMS_SID_AOFFC, false, motorEncoderParameters.aOffc_);
  if (!setCalibrationTypeEnum(previousCalibrationTypeEnum)) {
    return false;
  }
  return success;
}

bool AnydriveEthercatSlave::getCalibrationGearJointEncoderOffset(
    const anydrive::calibration::CalibrationTypeEnum calibrationTypeEnum,
    anydrive::calibration::parameter::GearJointEncoderOffset& gearJointEncoderOffset) {
  RecLock lock(mutex_);
  anydrive::calibration::CalibrationTypeEnum previousCalibrationTypeEnum = anydrive::calibration::CalibrationTypeEnum::NA;
  if (!getCalibrationTypeEnum(previousCalibrationTypeEnum)) {
    return false;
  }
  if (!setCalibrationTypeEnum(calibrationTypeEnum)) {
    return false;
  }
  bool success = true;
  success &= sendSdoRead(OD_CALIB_GEAR_JOINT_ENCODER_OFFSET_ID, OD_CALIB_GEAR_JOINT_ENCODER_OFFSET_SID_CONSTANT, false,
                         gearJointEncoderOffset.constant_);
  success &= sendSdoRead(OD_CALIB_GEAR_JOINT_ENCODER_OFFSET_ID, OD_CALIB_GEAR_JOINT_ENCODER_OFFSET_SID_SIN1_AMPLITUDE, false,
                         gearJointEncoderOffset.sin1Amplitude_);
  success &= sendSdoRead(OD_CALIB_GEAR_JOINT_ENCODER_OFFSET_ID, OD_CALIB_GEAR_JOINT_ENCODER_OFFSET_SID_SIN1_PHASESHIFT, false,
                         gearJointEncoderOffset.sin1Phaseshift_);
  success &= sendSdoRead(OD_CALIB_GEAR_JOINT_ENCODER_OFFSET_ID, OD_CALIB_GEAR_JOINT_ENCODER_OFFSET_SID_SIN2_AMPLITUDE, false,
                         gearJointEncoderOffset.sin2Amplitude_);
  success &= sendSdoRead(OD_CALIB_GEAR_JOINT_ENCODER_OFFSET_ID, OD_CALIB_GEAR_JOINT_ENCODER_OFFSET_SID_SIN2_PHASESHIFT, false,
                         gearJointEncoderOffset.sin2Phaseshift_);
  if (!setCalibrationTypeEnum(previousCalibrationTypeEnum)) {
    return false;
  }
  return success;
}

bool AnydriveEthercatSlave::getCalibrationGearAndJointEncoderHoming(
    const anydrive::calibration::CalibrationTypeEnum calibrationTypeEnum,
    anydrive::calibration::parameter::GearAndJointEncoderHoming& gearAndJointEncoderHoming) {
  RecLock lock(mutex_);
  anydrive::calibration::CalibrationTypeEnum previousCalibrationTypeEnum = anydrive::calibration::CalibrationTypeEnum::NA;
  if (!getCalibrationTypeEnum(previousCalibrationTypeEnum)) {
    return false;
  }
  if (!setCalibrationTypeEnum(calibrationTypeEnum)) {
    return false;
  }
  bool success = true;
  success &= sendSdoRead(OD_CALIB_GEAR_AND_JOINT_ENCODER_HOMING_TICKS_ID, OD_CALIB_GEAR_AND_JOINT_ENCODER_HOMING_TICKS_SID_GEAR, false,
                         gearAndJointEncoderHoming.gearEncoderRawTicks_);
  success &= sendSdoRead(OD_CALIB_GEAR_AND_JOINT_ENCODER_HOMING_TICKS_ID, OD_CALIB_GEAR_AND_JOINT_ENCODER_HOMING_TICKS_SID_JOINT, false,
                         gearAndJointEncoderHoming.jointEncoderRawTicks_);
  if (!setCalibrationTypeEnum(previousCalibrationTypeEnum)) {
    return false;
  }
  return success;
}

bool AnydriveEthercatSlave::getCalibrationImuGyroscopeDcBias(const anydrive::calibration::CalibrationTypeEnum calibrationTypeEnum,
                                                             anydrive::calibration::parameter::ImuGyroscopeDcBias& imuGyroscopeDcBias) {
  RecLock lock(mutex_);
  anydrive::calibration::CalibrationTypeEnum previousCalibrationTypeEnum = anydrive::calibration::CalibrationTypeEnum::NA;
  if (!getCalibrationTypeEnum(previousCalibrationTypeEnum)) {
    return false;
  }
  if (!setCalibrationTypeEnum(calibrationTypeEnum)) {
    return false;
  }
  bool success = true;
  success &= sendSdoRead(OD_CALIB_IMU_GYROSCOPE_DC_BIAS_ID, OD_CALIB_IMU_GYROSCOPE_DC_BIAS_SID_X, false, imuGyroscopeDcBias.x_);
  success &= sendSdoRead(OD_CALIB_IMU_GYROSCOPE_DC_BIAS_ID, OD_CALIB_IMU_GYROSCOPE_DC_BIAS_SID_Y, false, imuGyroscopeDcBias.y_);
  success &= sendSdoRead(OD_CALIB_IMU_GYROSCOPE_DC_BIAS_ID, OD_CALIB_IMU_GYROSCOPE_DC_BIAS_SID_Z, false, imuGyroscopeDcBias.z_);
  if (!setCalibrationTypeEnum(previousCalibrationTypeEnum)) {
    return false;
  }
  return success;
}

bool AnydriveEthercatSlave::setCalibrationSpringStiffness(const anydrive::calibration::CalibrationTypeEnum /*calibrationTypeEnum*/,
                                                          const anydrive::calibration::parameter::SpringStiffness& springStiffness) {
  RecLock lock(mutex_);
  bool success = true;
  success &= sendSdoWrite(OD_CALIB_SPRING_STIFFNESS_ID, OD_CALIB_SPRING_STIFFNESS_SID_NEG, false, springStiffness.neg_);
  success &= sendSdoWrite(OD_CALIB_SPRING_STIFFNESS_ID, OD_CALIB_SPRING_STIFFNESS_SID_POS, false, springStiffness.pos_);
  return success;
}

bool AnydriveEthercatSlave::getCalibrationSpringStiffness(const anydrive::calibration::CalibrationTypeEnum calibrationTypeEnum,
                                                          anydrive::calibration::parameter::SpringStiffness& springStiffness) {
  RecLock lock(mutex_);
  anydrive::calibration::CalibrationTypeEnum previousCalibrationTypeEnum = anydrive::calibration::CalibrationTypeEnum::NA;
  if (!getCalibrationTypeEnum(previousCalibrationTypeEnum)) {
    return false;
  }
  if (!setCalibrationTypeEnum(calibrationTypeEnum)) {
    return false;
  }
  bool success = true;
  success &= sendSdoRead(OD_CALIB_SPRING_STIFFNESS_ID, OD_CALIB_SPRING_STIFFNESS_SID_NEG, false, springStiffness.neg_);
  success &= sendSdoRead(OD_CALIB_SPRING_STIFFNESS_ID, OD_CALIB_SPRING_STIFFNESS_SID_POS, false, springStiffness.pos_);
  if (!setCalibrationTypeEnum(previousCalibrationTypeEnum)) {
    return false;
  }
  return success;
}

bool AnydriveEthercatSlave::setCalibrationFrictionEstimation(
    const anydrive::calibration::CalibrationTypeEnum /*calibrationTypeEnum*/,
    const anydrive::calibration::parameter::FrictionEstimation& frictionEstimation) {
  RecLock lock(mutex_);
  bool success = true;
  success &= sendSdoWrite(OD_CALIB_FRICTION_ESTIMATION_ID, OD_CALIB_FRICTION_ESTIMATION_SID_BREAK_AWAY_FRICTION, false,
                          frictionEstimation.breakAwayFriction_);
  success &= sendSdoWrite(OD_CALIB_FRICTION_ESTIMATION_ID, OD_CALIB_FRICTION_ESTIMATION_SID_BREAK_AWAY_FRICTION_BAND, false,
                          frictionEstimation.breakAwayFrictionBand_);
  success &= sendSdoWrite(OD_CALIB_FRICTION_ESTIMATION_ID, OD_CALIB_FRICTION_ESTIMATION_SID_VISCOUS_FRICTION_COEFF_NEG, false,
                          frictionEstimation.viscousFrictionCoeffNeg_);
  success &= sendSdoWrite(OD_CALIB_FRICTION_ESTIMATION_ID, OD_CALIB_FRICTION_ESTIMATION_SID_VISCOUS_FRICTION_COEFF_POS, false,
                          frictionEstimation.viscousFrictionCoeffPos_);
  return success;
}

bool AnydriveEthercatSlave::getCalibrationFrictionEstimation(const anydrive::calibration::CalibrationTypeEnum calibrationTypeEnum,
                                                             anydrive::calibration::parameter::FrictionEstimation& frictionEstimation) {
  RecLock lock(mutex_);
  anydrive::calibration::CalibrationTypeEnum previousCalibrationTypeEnum = anydrive::calibration::CalibrationTypeEnum::NA;
  if (!getCalibrationTypeEnum(previousCalibrationTypeEnum)) {
    return false;
  }
  if (!setCalibrationTypeEnum(calibrationTypeEnum)) {
    return false;
  }
  bool success = true;
  success &= sendSdoRead(OD_CALIB_FRICTION_ESTIMATION_ID, OD_CALIB_FRICTION_ESTIMATION_SID_BREAK_AWAY_FRICTION, false,
                         frictionEstimation.breakAwayFriction_);
  success &= sendSdoRead(OD_CALIB_FRICTION_ESTIMATION_ID, OD_CALIB_FRICTION_ESTIMATION_SID_BREAK_AWAY_FRICTION_BAND, false,
                         frictionEstimation.breakAwayFrictionBand_);
  success &= sendSdoRead(OD_CALIB_FRICTION_ESTIMATION_ID, OD_CALIB_FRICTION_ESTIMATION_SID_VISCOUS_FRICTION_COEFF_NEG, false,
                         frictionEstimation.viscousFrictionCoeffNeg_);
  success &= sendSdoRead(OD_CALIB_FRICTION_ESTIMATION_ID, OD_CALIB_FRICTION_ESTIMATION_SID_VISCOUS_FRICTION_COEFF_POS, false,
                         frictionEstimation.viscousFrictionCoeffPos_);
  if (!setCalibrationTypeEnum(previousCalibrationTypeEnum)) {
    return false;
  }
  return success;
}

bool AnydriveEthercatSlave::getCalibrationGearJointEncoderOffsetConstant(int32_t& constant) {
  return sendSdoRead(OD_CALIB_GEAR_JOINT_ENCODER_OFFSET_ID, OD_CALIB_GEAR_JOINT_ENCODER_OFFSET_SID_CONSTANT, false, constant);
}

bool AnydriveEthercatSlave::setCalibrationGearJointEncoderOffsetConstant(const int32_t constant) {
  return sendSdoWrite(OD_CALIB_GEAR_JOINT_ENCODER_OFFSET_ID, OD_CALIB_GEAR_JOINT_ENCODER_OFFSET_SID_CONSTANT, false, constant);
}

bool AnydriveEthercatSlave::resetCustomCalibrationsToFactory(const anydrive::calibration::CalibrationState calibrationState) {
  RecLock lock(mutex_);
  ANYDRIVE_NAMED_INFO(
      "Resetting custom calibrations to factory. Fields: " << calibrationState.all_);  // NOLINT(cppcoreguidelines-pro-type-union-access)
  return sendSdoWrite(OD_CALIB_FACTORY_TO_CUSTOM_ID, 0x00, false,
                      calibrationState.all_);  // NOLINT(cppcoreguidelines-pro-type-union-access)
}

bool AnydriveEthercatSlave::writeFactoryCalibration() {
  RecLock lock(mutex_);
  ANYDRIVE_NAMED_INFO("Writing factory calibration to flash.");
  return sendSdoWrite(OD_CALIB_CUSTOM_TO_FACTORY_ID, 0x00, false, static_cast<uint16_t>(OD_CALIB_CUSTOM_TO_FACTORY_ID_VAL_RUN));
}

bool AnydriveEthercatSlave::getMaxCurrent(double& maxCurrent) {
  RecLock lock(mutex_);
  float maxCurrentFloat = 0.0;
  if (!sendSdoRead(OD_DSP402_CURRENT_MAX_ID, 0x00, false, maxCurrentFloat)) {
    return false;
  }
  maxCurrent = motorCurrentScalingInv_ * maxCurrentFloat;
  return true;
}

bool AnydriveEthercatSlave::setMaxCurrent(const double maxCurrent) {
  ANYDRIVE_NAMED_DEBUG("Setting max current (" << maxCurrent << " A).");
  RecLock lock(mutex_);
  return sendSdoWrite(OD_DSP402_CURRENT_MAX_ID, 0x00, false, static_cast<float>(motorCurrentScaling_ * maxCurrent));
}

bool AnydriveEthercatSlave::getMaxFreezeCurrent(double& current) {
  RecLock lock(mutex_);
  float currentFloat = 0.0;
  if (!sendSdoRead(OD_CONFIG_MAX_FREEZE_CURRENT, 0x00, false, currentFloat)) {
    return false;
  }
  current = motorCurrentScalingInv_ * currentFloat;
  return true;
}

bool AnydriveEthercatSlave::setMaxFreezeCurrent(const double current) {
  RecLock lock(mutex_);
  ANYDRIVE_NAMED_DEBUG("Setting freeze current limit (" << current << ").");
  return sendSdoWrite(OD_CONFIG_MAX_FREEZE_CURRENT, 0x00, false, static_cast<float>(motorCurrentScaling_ * current));
}

bool AnydriveEthercatSlave::clearLoggedData() {
  ANYDRIVE_NAMED_DEBUG("Erasing logged data.");
  // Subindex 2 erases the data
  return sendSdoWriteUInt64(OD_DATALOGGING, 2, false, static_cast<uint64_t>(1));
}

bool AnydriveEthercatSlave::refreshLoggedData() {
  ANYDRIVE_NAMED_DEBUG("Refreshing logged data.");
  // Subindex 1 refreshes the data
  return sendSdoWriteUInt64(OD_DATALOGGING, 1, false, static_cast<uint64_t>(1));
}

bool AnydriveEthercatSlave::getNumHistogramBins(int& numHistogramBins) {
  ANYDRIVE_NAMED_DEBUG("Getting number of histogram bins of datalogger");
  // Subindex 3 reads the number of histogram bins:
  uint64_t n;
  if (!sendSdoReadUInt64(OD_DATALOGGING, 3, false, n)) {
    return false;
  }
  numHistogramBins = static_cast<int>(n);
  return true;
}

bool AnydriveEthercatSlave::readHistograms(std::vector<uint64_t>& histTorque, std::vector<uint64_t>& histVelocity,
                                           std::vector<uint64_t>& histCurrent, std::vector<uint64_t>& histTemperature) {
  ANYDRIVE_NAMED_DEBUG("Reading logged histograms");
  // Get the number of histogram bins from the drive:
  int numHistogramBins;
  if (!getNumHistogramBins(numHistogramBins)) {
    ANYDRIVE_NAMED_ERROR("Could not read number of histogram bins.");
    return false;
  }
  // Sanity-check:
  if (numHistogramBins < 2) {
    ANYDRIVE_NAMED_ERROR("The histogram size is < 2, which seems not to make sense.");
    return false;
  }
  // The sub-indices of the histograms in the SDO. The sequence of the histograms is hardcoded in the firmware.
  int torqueSubIdxOffs = loggedDataHistogramsOffset;  // The torque histogram is the first.
  int velocitySubIdxOffs = loggedDataHistogramsOffset + numHistogramBins;
  int currentSubIdxOffs = loggedDataHistogramsOffset + (2 * numHistogramBins);
  int temperatureSubIdxOffs = loggedDataHistogramsOffset + (3 * numHistogramBins);
  // Read the four histograms:
  if (!readHistogram(histTorque, torqueSubIdxOffs, numHistogramBins)) {
    ANYDRIVE_NAMED_ERROR("Could not read torque histogram.");
    return false;
  }
  if (!readHistogram(histVelocity, velocitySubIdxOffs, numHistogramBins)) {
    ANYDRIVE_NAMED_ERROR("Could not read velocity histogram.");
    return false;
  }
  if (!readHistogram(histCurrent, currentSubIdxOffs, numHistogramBins)) {
    ANYDRIVE_NAMED_ERROR("Could not read q-current histogram.");
    return false;
  }
  if (!readHistogram(histTemperature, temperatureSubIdxOffs, numHistogramBins)) {
    ANYDRIVE_NAMED_ERROR("Could not read temperature histogram.");
    return false;
  }
  return true;
}

bool AnydriveEthercatSlave::readHistogram(std::vector<uint64_t>& hist, int subIndexOffset, int size) {
  hist.resize(size);
  uint8_t subindex;
  for (int i = 0; i < size; i++) {
    subindex = static_cast<uint8_t>(i + subIndexOffset);
    if (!sendSdoReadUInt64(OD_DATALOGGING, subindex, false, hist[i])) {
      ANYDRIVE_NAMED_ERROR("Could not read logged data SDO at subindex " << static_cast<int>(subindex) << ". Aborting.");
      return false;
    }
  }
  return true;
}

bool AnydriveEthercatSlave::readOperatingTimes(uint64_t& timeTotal, uint64_t& timeActive) {
  ANYDRIVE_NAMED_DEBUG("Reading logged operating times");
  // Get the number of histogram bins from the drive, as this defines the desired subindex in the data logging SDO.
  int numHistogramBins;
  if (!getNumHistogramBins(numHistogramBins)) {
    ANYDRIVE_NAMED_ERROR("Could not read number of histogram bins.");
    return false;
  }
  // Sanity-check:
  if (numHistogramBins < 2) {
    ANYDRIVE_NAMED_ERROR("The histogram size is < 2, which seems not to make sense.");
    return false;
  }
  // The total operating time is the first subindex after the temperature histogram:
  int subIdxTotal = loggedDataHistogramsOffset + (4 * numHistogramBins);
  // The active time subindex is one after:
  int subIdxActive = subIdxTotal + 1;
  if (!sendSdoReadUInt64(OD_DATALOGGING, static_cast<uint8_t>(subIdxTotal), false, timeTotal)) {
    ANYDRIVE_NAMED_ERROR("Could not read logged data SDO at subindex " << subIdxTotal << ". Aborting.");
    return false;
  }
  if (!sendSdoReadUInt64(OD_DATALOGGING, static_cast<uint8_t>(subIdxActive), false, timeActive)) {
    ANYDRIVE_NAMED_ERROR("Could not read logged data SDO at subindex " << subIdxActive << ". Aborting.");
    return false;
  }
  return true;
}

bool AnydriveEthercatSlave::readJointTravel(double& jointTravel) {
  ANYDRIVE_NAMED_DEBUG("Reading logged joint travel distance");
  // Get the number of histogram bins from the drive, as this defines the desired subindex in the data logging SDO.
  int numHistogramBins;
  if (!getNumHistogramBins(numHistogramBins)) {
    ANYDRIVE_NAMED_ERROR("Could not read number of histogram bins.");
    return false;
  }
  // Sanity-check:
  if (numHistogramBins < 2) {
    ANYDRIVE_NAMED_ERROR("The histogram size is < 2, which seems not to make sense.");
    return false;
  }
  // The joint travel is the third subindex in the data logging SDO:
  int subIdx = loggedDataHistogramsOffset + (4 * numHistogramBins) + 2;

  if (!sendSdoReadDouble(OD_DATALOGGING, static_cast<uint8_t>(subIdx), false, jointTravel)) {
    ANYDRIVE_NAMED_ERROR("Could not read logged data SDO at subindex " << subIdx << ". Aborting.");
    return false;
  }
  return true;
}

bool AnydriveEthercatSlave::readHistogramEdges(float& torque_upper, float& torque_lower, float& velocity_upper, float& velocity_lower,
                                               float& current_upper, float& current_lower, float& temperature_upper,
                                               float& temperature_lower) {
  ANYDRIVE_NAMED_DEBUG("Reading logged joint travel distance");
  // Torque Histogram Edges:
  if (!sendSdoReadFloat(OD_DATALOGGING_HIST_EDGES, static_cast<uint8_t>(1), false, torque_upper)) {
    ANYDRIVE_NAMED_ERROR("Could not read torque histogram upper edge. Aborting.");
    return false;
  }
  if (!sendSdoReadFloat(OD_DATALOGGING_HIST_EDGES, static_cast<uint8_t>(2), false, torque_lower)) {
    ANYDRIVE_NAMED_ERROR("Could not read torque histogram lower edge. Aborting.");
    return false;
  }
  // Velocity Histogram Edges:
  if (!sendSdoReadFloat(OD_DATALOGGING_HIST_EDGES, static_cast<uint8_t>(3), false, velocity_upper)) {
    ANYDRIVE_NAMED_ERROR("Could not read velocity histogram upper edge. Aborting.");
    return false;
  }
  if (!sendSdoReadFloat(OD_DATALOGGING_HIST_EDGES, static_cast<uint8_t>(4), false, velocity_lower)) {
    ANYDRIVE_NAMED_ERROR("Could not read velocity histogram lower edge. Aborting.");
    return false;
  }
  // q-Current Histogram Edges:
  if (!sendSdoReadFloat(OD_DATALOGGING_HIST_EDGES, static_cast<uint8_t>(5), false, current_upper)) {
    ANYDRIVE_NAMED_ERROR("Could not read q-current histogram upper edge. Aborting.");
    return false;
  }
  if (!sendSdoReadFloat(OD_DATALOGGING_HIST_EDGES, static_cast<uint8_t>(6), false, current_lower)) {
    ANYDRIVE_NAMED_ERROR("Could not read q-current histogram lower edge. Aborting.");
    return false;
  }
  // Temperature Histogram Edges:
  if (!sendSdoReadFloat(OD_DATALOGGING_HIST_EDGES, static_cast<uint8_t>(7), false, temperature_upper)) {
    ANYDRIVE_NAMED_ERROR("Could not read temperature histogram upper edge. Aborting.");
    return false;
  }
  if (!sendSdoReadFloat(OD_DATALOGGING_HIST_EDGES, static_cast<uint8_t>(8), false, temperature_lower)) {
    ANYDRIVE_NAMED_ERROR("Could not read temperature histogram lower edge. Aborting.");
    return false;
  }
  return true;
}

bool AnydriveEthercatSlave::getMaxMotorVelocity(double& maxMotorVelocity) {
  RecLock lock(mutex_);
  float maxMotorVelocityFloat = 0.0;
  if (!sendSdoRead(OD_DSP402_MOTOR_VELOCITY_MAX_ID, 0x00, false, maxMotorVelocityFloat)) {
    return false;
  }
  maxMotorVelocity = motorVelocityScalingInv_ * maxMotorVelocityFloat;
  return true;
}

bool AnydriveEthercatSlave::setMaxMotorVelocity(const double maxMotorVelocity) {
  RecLock lock(mutex_);
  ANYDRIVE_NAMED_DEBUG("Setting max motor velocity (" << maxMotorVelocity << " rad/s).");
  return sendSdoWrite(OD_DSP402_MOTOR_VELOCITY_MAX_ID, 0x00, false, static_cast<float>(motorVelocityScaling_ * maxMotorVelocity));
}

bool AnydriveEthercatSlave::getMaxJointTorque(double& maxJointTorque) {
  RecLock lock(mutex_);
  float maxJointTorqueFloat = 0.0;
  if (!sendSdoRead(OD_DSP402_JOINT_TORQUE_MAX_ID, 0x00, false, maxJointTorqueFloat)) {
    return false;
  }
  maxJointTorque = jointTorqueScalingInv_ * maxJointTorqueFloat;
  return true;
}

bool AnydriveEthercatSlave::setMaxJointTorque(const double maxJointTorque) {
  ANYDRIVE_NAMED_DEBUG("Setting max joint torque (" << maxJointTorque << " Nm).");
  RecLock lock(mutex_);
  return sendSdoWrite(OD_DSP402_JOINT_TORQUE_MAX_ID, 0x00, false, static_cast<float>(jointTorqueScaling_ * maxJointTorque));
}

bool AnydriveEthercatSlave::getCurrentIntegratorSaturation(double& saturation) {
  RecLock lock(mutex_);
  float saturationFloat = 0.0;
  if (!sendSdoRead(OD_CONFIG_CURRENT_INTEGRATOR_SATURATION, 0x00, false, saturationFloat)) {
    return false;
  }
  saturation = saturationFloat;
  return true;
}

bool AnydriveEthercatSlave::setCurrentIntegratorSaturation(const double saturation) {
  ANYDRIVE_NAMED_DEBUG("Setting current integrator saturation (" << saturation << " A).");
  RecLock lock(mutex_);
  return sendSdoWrite(OD_CONFIG_CURRENT_INTEGRATOR_SATURATION, 0x00, false, static_cast<float>(saturation));
}

bool AnydriveEthercatSlave::getJointTorqueIntegratorSaturation(double& saturation) {
  RecLock lock(mutex_);
  float saturationFloat = 0.0;
  if (!sendSdoRead(OD_CONFIG_JOINT_TORQUE_INTEGRATOR_SATURATION, 0x00, false, saturationFloat)) {
    return false;
  }
  saturation = saturationFloat;
  return true;
}

bool AnydriveEthercatSlave::setJointTorqueIntegratorSaturation(const double saturation) {
  ANYDRIVE_NAMED_DEBUG("Setting joint torque integrator saturation (" << saturation << " Nm).");
  RecLock lock(mutex_);
  return sendSdoWrite(OD_CONFIG_JOINT_TORQUE_INTEGRATOR_SATURATION, 0x00, false, static_cast<float>(saturation));
}

bool AnydriveEthercatSlave::getDirection(int16_t& direction) {
  RecLock lock(mutex_);
  return sendSdoRead(OD_CONTROL_DIRECTION_ID, 0x00, false, direction);
}

bool AnydriveEthercatSlave::setDirection(const int16_t direction) {
  RecLock lock(mutex_);
  ANYDRIVE_NAMED_DEBUG("Setting direction (" << direction << ").");
  return sendSdoWrite(OD_CONTROL_DIRECTION_ID, 0x00, false, direction);
}

bool AnydriveEthercatSlave::getJointPositionLimitsSoft(anydrive::common::Limits& limits) {
  RecLock lock(mutex_);
  bool success = true;
  success &= sendSdoRead(OD_DSP402_SOFT_JOINT_POSITION_LIMIT_ID, OD_DSP402_SOFT_JOINT_POSITION_LIMIT_SID_MIN, false, limits.min());
  success &= sendSdoRead(OD_DSP402_SOFT_JOINT_POSITION_LIMIT_ID, OD_DSP402_SOFT_JOINT_POSITION_LIMIT_SID_MAX, false, limits.max());
  limits *= jointPositionScalingInv_;
  return success;
}

bool AnydriveEthercatSlave::setJointPositionLimitsSoft(const anydrive::common::Limits& limits) {
  RecLock lock(mutex_);
  std::shared_ptr<anydrive::Anydrive> anydrive = anydrive_.lock();
  if (!anydrive) {
    ANYDRIVE_NAMED_ERROR("The anydrive pointer is empty.");
    return false;
  }
  ANYDRIVE_NAMED_DEBUG("Setting soft joint position limits ([" << limits.min() << ", " << limits.max() << "] rad).");
  anydrive::common::Limits adjustedLimits = limits * anydrive->getConfiguration().getDirection();
  bool success = true;
  success &= sendSdoWrite(OD_DSP402_SOFT_JOINT_POSITION_LIMIT_ID, OD_DSP402_SOFT_JOINT_POSITION_LIMIT_SID_MIN, false,
                          static_cast<double>(jointPositionScaling_ * adjustedLimits.min()));
  success &= sendSdoWrite(OD_DSP402_SOFT_JOINT_POSITION_LIMIT_ID, OD_DSP402_SOFT_JOINT_POSITION_LIMIT_SID_MAX, false,
                          static_cast<double>(jointPositionScaling_ * adjustedLimits.max()));
  return success;
}

bool AnydriveEthercatSlave::getJointPositionLimitsHard(anydrive::common::Limits& limits) {
  RecLock lock(mutex_);
  bool success = true;
  success &= sendSdoRead(OD_DSP402_HARD_JOINT_POSITION_LIMIT_ID, OD_DSP402_HARD_JOINT_POSITION_LIMIT_SID_MIN, false, limits.min());
  success &= sendSdoRead(OD_DSP402_HARD_JOINT_POSITION_LIMIT_ID, OD_DSP402_HARD_JOINT_POSITION_LIMIT_SID_MAX, false, limits.max());
  limits *= jointPositionScalingInv_;
  return success;
}

bool AnydriveEthercatSlave::setJointPositionLimitsHard(const anydrive::common::Limits& limits) {
  RecLock lock(mutex_);
  std::shared_ptr<anydrive::Anydrive> anydrive = anydrive_.lock();
  if (!anydrive) {
    ANYDRIVE_NAMED_ERROR("The anydrive pointer is empty.");
    return false;
  }
  ANYDRIVE_NAMED_DEBUG("Setting hard joint position limits ([" << limits.min() << ", " << limits.max() << "] rad).");
  anydrive::common::Limits adjustedLimits = limits * anydrive->getConfiguration().getDirection();
  bool success = true;
  success &= sendSdoWrite(OD_DSP402_HARD_JOINT_POSITION_LIMIT_ID, OD_DSP402_HARD_JOINT_POSITION_LIMIT_SID_MIN, false,
                          static_cast<double>(jointPositionScaling_ * adjustedLimits.min()));
  success &= sendSdoWrite(OD_DSP402_HARD_JOINT_POSITION_LIMIT_ID, OD_DSP402_HARD_JOINT_POSITION_LIMIT_SID_MAX, false,
                          static_cast<double>(jointPositionScaling_ * adjustedLimits.max()));
  return success;
}

bool AnydriveEthercatSlave::getControlGains(const anydrive::mode::ModeEnum mode, anydrive::mode::PidGainsF& pidGains) {
  auto odIndex = modeEnumToOdIndex_.find(mode);
  if (odIndex == modeEnumToOdIndex_.end()) {
    ANYDRIVE_NAMED_ERROR("Getting control gains is not supported for mode '" << anydrive::mode::modeEnumToName(mode) << "'.");
    return false;
  }

  RecLock lock(mutex_);
  bool success = true;
  success &= sendSdoRead(odIndex->second, OD_GAINS_COMMON_SID_P, false, pidGains.getP());
  success &= sendSdoRead(odIndex->second, OD_GAINS_COMMON_SID_I, false, pidGains.getI());
  success &= sendSdoRead(odIndex->second, OD_GAINS_COMMON_SID_D, false, pidGains.getD());
  pidGains *= gainScalingInv_;
  return success;
}

bool AnydriveEthercatSlave::setControlGains(anydrive::mode::ModeEnum mode, const anydrive::mode::PidGainsF& pidGains) {
  auto odIndex = modeEnumToOdIndex_.find(mode);
  if (odIndex == modeEnumToOdIndex_.end()) {
    ANYDRIVE_NAMED_ERROR("Setting control gains is not supported for mode '" << anydrive::mode::modeEnumToName(mode) << "'.");
    return false;
  }

  RecLock lock(mutex_);
  ANYDRIVE_NAMED_DEBUG("Setting control gains for mode '" << anydrive::mode::modeEnumToName(mode) << "'.");
  bool success = true;
  success &= sendSdoWrite(odIndex->second, OD_GAINS_COMMON_SID_P, false, static_cast<float>(gainScaling_ * pidGains.getP()));
  success &= sendSdoWrite(odIndex->second, OD_GAINS_COMMON_SID_I, false, static_cast<float>(gainScaling_ * pidGains.getI()));
  success &= sendSdoWrite(odIndex->second, OD_GAINS_COMMON_SID_D, false, static_cast<float>(gainScaling_ * pidGains.getD()));
  return success;
}

bool AnydriveEthercatSlave::getErrorStateBehavior(uint16_t& errorStateBehavior) {
  RecLock lock(mutex_);
  return sendSdoRead(OD_FSM_ERROR_BEHAVIOR_ID, 0x00, false, errorStateBehavior);
}

bool AnydriveEthercatSlave::setErrorStateBehavior(const uint16_t errorStateBehavior) {
  RecLock lock(mutex_);
  ANYDRIVE_NAMED_DEBUG("Setting error state behavior (" << errorStateBehavior << ").");
  return sendSdoWrite(OD_FSM_ERROR_BEHAVIOR_ID, 0x00, false, errorStateBehavior);
}

bool AnydriveEthercatSlave::getImuEnabled(bool& enabled) {
  RecLock lock(mutex_);
  uint32_t enabledInt = 0;
  bool isOk = sendSdoRead(OD_CONTROL_IMU_CONFIG_ID, OD_CONTROL_IMU_CONFIG_SID_ENABLE, false, enabledInt);
  enabled = (enabledInt == 1);
  return isOk;
}

bool AnydriveEthercatSlave::setImuEnable(const bool enable) {
  RecLock lock(mutex_);
  ANYDRIVE_NAMED_DEBUG((enable ? "En" : "Dis") << "abling IMU.");
  return sendSdoWrite(OD_CONTROL_IMU_CONFIG_ID, OD_CONTROL_IMU_CONFIG_SID_ENABLE, false, static_cast<uint32_t>(enable ? 1 : 0));
}

bool AnydriveEthercatSlave::getImuAccelerometerRange(uint32_t& range) {
  RecLock lock(mutex_);
  return sendSdoRead(OD_CONTROL_IMU_CONFIG_ID, OD_CONTROL_IMU_CONFIG_SID_ACCELEROMETER_RANGE, false, range);
}

bool AnydriveEthercatSlave::setImuAccelerometerRange(const uint32_t range) {
  RecLock lock(mutex_);
  ANYDRIVE_NAMED_DEBUG("Setting IMU accelerometer range (" << range << ").");
  return sendSdoWrite(OD_CONTROL_IMU_CONFIG_ID, OD_CONTROL_IMU_CONFIG_SID_ACCELEROMETER_RANGE, false, range);
}

bool AnydriveEthercatSlave::getImuGyroscopeRange(uint32_t& range) {
  RecLock lock(mutex_);
  return sendSdoRead(OD_CONTROL_IMU_CONFIG_ID, OD_CONTROL_IMU_CONFIG_SID_GYROSCOPE_RANGE, false, range);
}

bool AnydriveEthercatSlave::setImuGyroscopeRange(const uint32_t range) {
  RecLock lock(mutex_);
  ANYDRIVE_NAMED_DEBUG("Setting IMU gyroscope range (" << range << ").");
  return sendSdoWrite(OD_CONTROL_IMU_CONFIG_ID, OD_CONTROL_IMU_CONFIG_SID_GYROSCOPE_RANGE, false, range);
}

bool AnydriveEthercatSlave::getFanMode(uint32_t& mode) {
  RecLock lock(mutex_);
  return sendSdoRead(OD_CONTROL_FAN_ID, OD_CONTROL_FAN_SID_MODE, false, mode);
}

bool AnydriveEthercatSlave::setFanMode(const uint32_t mode) {
  RecLock lock(mutex_);
  ANYDRIVE_NAMED_DEBUG("Setting fan mode (" << mode << ").");
  return sendSdoWrite(OD_CONTROL_FAN_ID, OD_CONTROL_FAN_SID_MODE, false, mode);
}

bool AnydriveEthercatSlave::getFanIntensity(uint32_t& intensity) {
  RecLock lock(mutex_);
  return sendSdoRead(OD_CONTROL_FAN_ID, OD_CONTROL_FAN_SID_INTENSITY, false, intensity);
}

bool AnydriveEthercatSlave::setFanIntensity(const uint32_t intensity) {
  RecLock lock(mutex_);
  ANYDRIVE_NAMED_DEBUG("Setting fan intensity (" << intensity << ").");
  return sendSdoWrite(OD_CONTROL_FAN_ID, OD_CONTROL_FAN_SID_INTENSITY, false, intensity);
}

bool AnydriveEthercatSlave::getFanLowerTemperature(float& temperature) {
  RecLock lock(mutex_);
  return sendSdoRead(OD_CONTROL_FAN_ID, OD_CONTROL_FAN_SID_LOWER_TEMPERATURE, false, temperature);
}

bool AnydriveEthercatSlave::setFanLowerTemperature(const float temperature) {
  RecLock lock(mutex_);
  ANYDRIVE_NAMED_DEBUG("Setting fan lower temperature (" << temperature << ").");
  return sendSdoWrite(OD_CONTROL_FAN_ID, OD_CONTROL_FAN_SID_LOWER_TEMPERATURE, false, temperature);
}

bool AnydriveEthercatSlave::getFanUpperTemperature(float& temperature) {
  RecLock lock(mutex_);
  return sendSdoRead(OD_CONTROL_FAN_ID, OD_CONTROL_FAN_SID_UPPER_TEMPERATURE, false, temperature);
}

bool AnydriveEthercatSlave::setFanUpperTemperature(const float temperature) {
  RecLock lock(mutex_);
  ANYDRIVE_NAMED_DEBUG("Setting fan upper temperature (" << temperature << ").");
  return sendSdoWrite(OD_CONTROL_FAN_ID, OD_CONTROL_FAN_SID_UPPER_TEMPERATURE, false, temperature);
}

bool AnydriveEthercatSlave::setBrakeMode(const bool mode) {
  RecLock lock(mutex_);
  if (mode) {
    ANYDRIVE_NAMED_DEBUG("Enabling Brake Mode")
    if (!sendSdoWrite(OD_CONTROL_BRAKE_ID, OD_CONTROL_BRAKE_SID_CTRL, false, OD_CONTROL_BRAKE_SID_CTRL_VAL_ENABLE)) {
      ANYDRIVE_ERROR("Could not enable brake. Fan enabled?");
      return false;
    } else {
      ANYDRIVE_NAMED_DEBUG("Brake output active.")
      return true;
    }
  } else {
    ANYDRIVE_NAMED_DEBUG("Disabling Brake Mode")
    if (!sendSdoWrite(OD_CONTROL_BRAKE_ID, OD_CONTROL_BRAKE_SID_CTRL, false, OD_CONTROL_BRAKE_SID_CTRL_VAL_DISABLE)) {
      ANYDRIVE_ERROR("Could not disable brake.");
      return false;
    } else {
      ANYDRIVE_NAMED_DEBUG("Brake output disabled.")
      return true;
    }
  }
}

bool AnydriveEthercatSlave::getBrakeMode(bool& mode) {
  RecLock lock(mutex_);
  uint16_t m;
  if (!sendSdoRead(OD_CONTROL_BRAKE_ID, OD_CONTROL_BRAKE_SID_CTRL, false, m)) {
    return false;
  }
  if (m == 0) {
    mode = false;
  } else if (m == 1) {
    mode = true;
  } else {
    ANYDRIVE_ERROR("Brake mode not recognized. Received: " << m)
    mode = false;
    return false;
  }
  return true;
}

bool AnydriveEthercatSlave::setBrakeDuty(const float d) {
  // Expected duty cycle range: 0...1
  RecLock lock(mutex_);
  float d_lim;
  if (d > 1.0f) {
    d_lim = 1.0f;
  } else if (d < 0.0f) {
    d_lim = 0.0f;
  } else {
    d_lim = d;
  }
  auto d_disc = static_cast<uint16_t>(65535.0f * d_lim);
  return sendSdoWrite(OD_CONTROL_BRAKE_ID, OD_CONTROL_BRAKE_SID_DUTY, false, d_disc);
}

bool AnydriveEthercatSlave::getBrakeDuty(float& d) {
  RecLock lock(mutex_);
  uint16_t d_disc;
  if (!sendSdoRead(OD_CONTROL_BRAKE_ID, OD_CONTROL_BRAKE_SID_DUTY, false, d_disc)) {
    return false;
  }
  d = static_cast<float>(d_disc) / 65535.0f;
  return true;
}

bool AnydriveEthercatSlave::getGearJointVelocityFilterType(uint32_t& type) {
  RecLock lock(mutex_);
  return sendSdoRead(OD_FILTER_GEAR_JOINT_VELOCITY_ID, OD_FILTER_GEAR_JOINT_VELOCITY_SID_FILTER_TYPE, false, type);
}

bool AnydriveEthercatSlave::setGearJointVelocityFilterType(const uint32_t type) {
  RecLock lock(mutex_);
  ANYDRIVE_NAMED_DEBUG("Setting gear joint velicity filter type (" << type << ").");
  return sendSdoWrite(OD_FILTER_GEAR_JOINT_VELOCITY_ID, OD_FILTER_GEAR_JOINT_VELOCITY_SID_FILTER_TYPE, false, type);
}

bool AnydriveEthercatSlave::getGearJointVelocityKfNoiseVariance(float& variance) {
  RecLock lock(mutex_);
  return sendSdoRead(OD_FILTER_GEAR_JOINT_VELOCITY_ID, OD_FILTER_GEAR_JOINT_VELOCITY_SID_KF_NOISE_VARIANCE, false, variance);
}

bool AnydriveEthercatSlave::setGearJointVelocityKfNoiseVariance(const float variance) {
  RecLock lock(mutex_);
  ANYDRIVE_NAMED_DEBUG("Setting gear joint velicity KF noies variance (" << variance << ").");
  return sendSdoWrite(OD_FILTER_GEAR_JOINT_VELOCITY_ID, OD_FILTER_GEAR_JOINT_VELOCITY_SID_KF_NOISE_VARIANCE, false, variance);
}

bool AnydriveEthercatSlave::getGearJointVelocityKfLambda2(float& lambda) {
  RecLock lock(mutex_);
  return sendSdoRead(OD_FILTER_GEAR_JOINT_VELOCITY_ID, OD_FILTER_GEAR_JOINT_VELOCITY_SID_KF_LAMBDA_2, false, lambda);
}

bool AnydriveEthercatSlave::setGearJointVelocityKfLambda2(const float lambda) {
  RecLock lock(mutex_);
  ANYDRIVE_NAMED_DEBUG("Setting gear joint velicity KF lambda^2 (" << lambda << ").");
  return sendSdoWrite(OD_FILTER_GEAR_JOINT_VELOCITY_ID, OD_FILTER_GEAR_JOINT_VELOCITY_SID_KF_LAMBDA_2, false, lambda);
}

bool AnydriveEthercatSlave::getGearJointVelocityKfGamma(float& gamma) {
  RecLock lock(mutex_);
  return sendSdoRead(OD_FILTER_GEAR_JOINT_VELOCITY_ID, OD_FILTER_GEAR_JOINT_VELOCITY_SID_KF_GAMMA, false, gamma);
}

bool AnydriveEthercatSlave::setGearJointVelocityKfGamma(const float gamma) {
  RecLock lock(mutex_);
  ANYDRIVE_NAMED_DEBUG("Setting gear joint velicity KF gamma (" << gamma << ").");
  return sendSdoWrite(OD_FILTER_GEAR_JOINT_VELOCITY_ID, OD_FILTER_GEAR_JOINT_VELOCITY_SID_KF_GAMMA, false, gamma);
}

bool AnydriveEthercatSlave::getGearJointVelocityEmaAlpha(float& alpha) {
  RecLock lock(mutex_);
  return sendSdoRead(OD_FILTER_GEAR_JOINT_VELOCITY_ID, OD_FILTER_GEAR_JOINT_VELOCITY_SID_EMA_ALPHA, false, alpha);
}

bool AnydriveEthercatSlave::setGearJointVelocityEmaAlpha(const float alpha) {
  RecLock lock(mutex_);
  ANYDRIVE_NAMED_DEBUG("Setting gear joint velicity EMA alpha (" << alpha << ").");
  return sendSdoWrite(OD_FILTER_GEAR_JOINT_VELOCITY_ID, OD_FILTER_GEAR_JOINT_VELOCITY_SID_EMA_ALPHA, false, alpha);
}

bool AnydriveEthercatSlave::getJointVelocityForAccelerationFilterType(uint32_t& type) {
  RecLock lock(mutex_);
  return sendSdoRead(OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_ID, OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_SID_FILTER_TYPE, false, type);
}

bool AnydriveEthercatSlave::setJointVelocityForAccelerationFilterType(const uint32_t type) {
  RecLock lock(mutex_);
  ANYDRIVE_NAMED_DEBUG("Setting joint velocity for acceleration filter type (" << type << ").");
  return sendSdoWrite(OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_ID, OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_SID_FILTER_TYPE, false, type);
}

bool AnydriveEthercatSlave::getJointVelocityForAccelerationKfNoiseVariance(float& variance) {
  RecLock lock(mutex_);
  return sendSdoRead(OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_ID, OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_SID_KF_NOISE_VARIANCE, false,
                     variance);
}

bool AnydriveEthercatSlave::setJointVelocityForAccelerationKfNoiseVariance(const float variance) {
  RecLock lock(mutex_);
  ANYDRIVE_NAMED_DEBUG("Setting joint velocity for acceleration KF noies variance (" << variance << ").");
  return sendSdoWrite(OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_ID, OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_SID_KF_NOISE_VARIANCE, false,
                      variance);
}

bool AnydriveEthercatSlave::getJointVelocityForAccelerationKfLambda2(float& lambda) {
  RecLock lock(mutex_);
  return sendSdoRead(OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_ID, OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_SID_KF_LAMBDA_2, false,
                     lambda);
}

bool AnydriveEthercatSlave::setJointVelocityForAccelerationKfLambda2(const float lambda) {
  RecLock lock(mutex_);
  ANYDRIVE_NAMED_DEBUG("Setting joint velocity for acceleration KF lambda^2 (" << lambda << ").");
  return sendSdoWrite(OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_ID, OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_SID_KF_LAMBDA_2, false,
                      lambda);
}

bool AnydriveEthercatSlave::getJointVelocityForAccelerationKfGamma(float& gamma) {
  RecLock lock(mutex_);
  return sendSdoRead(OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_ID, OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_SID_KF_GAMMA, false, gamma);
}

bool AnydriveEthercatSlave::setJointVelocityForAccelerationKfGamma(const float gamma) {
  RecLock lock(mutex_);
  ANYDRIVE_NAMED_DEBUG("Setting joint velocity for acceleration KF gamma (" << gamma << ").");
  return sendSdoWrite(OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_ID, OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_SID_KF_GAMMA, false, gamma);
}

bool AnydriveEthercatSlave::getJointVelocityForAccelerationEmaAlpha(float& alpha) {
  RecLock lock(mutex_);
  return sendSdoRead(OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_ID, OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_SID_EMA_ALPHA, false, alpha);
}

bool AnydriveEthercatSlave::setJointVelocityForAccelerationEmaAlpha(const float alpha) {
  RecLock lock(mutex_);
  ANYDRIVE_NAMED_DEBUG("Setting joint velocity for acceleration EMA alpha (" << alpha << ").");
  return sendSdoWrite(OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_ID, OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_SID_EMA_ALPHA, false, alpha);
}

bool AnydriveEthercatSlave::getJointAccelerationFilterType(uint32_t& type) {
  RecLock lock(mutex_);
  return sendSdoRead(OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_ID, OD_FILTER_JOINT_VELOCITY_FOR_ACCELERATION_SID_FILTER_TYPE, false, type);
}

bool AnydriveEthercatSlave::setJointAccelerationFilterType(const uint32_t type) {
  RecLock lock(mutex_);
  ANYDRIVE_NAMED_DEBUG("Setting joint acceleration filter type (" << type << ").");
  return sendSdoWrite(OD_FILTER_JOINT_ACCELERATION_ID, OD_FILTER_JOINT_ACCELERATION_SID_FILTER_TYPE, false, type);
}

bool AnydriveEthercatSlave::getJointAccelerationKfNoiseVariance(float& variance) {
  RecLock lock(mutex_);
  return sendSdoRead(OD_FILTER_JOINT_ACCELERATION_ID, OD_FILTER_JOINT_ACCELERATION_SID_KF_NOISE_VARIANCE, false, variance);
}

bool AnydriveEthercatSlave::setJointAccelerationKfNoiseVariance(const float variance) {
  RecLock lock(mutex_);
  ANYDRIVE_NAMED_DEBUG("Setting joint acceleration KF noies variance (" << variance << ").");
  return sendSdoWrite(OD_FILTER_JOINT_ACCELERATION_ID, OD_FILTER_JOINT_ACCELERATION_SID_KF_NOISE_VARIANCE, false, variance);
}

bool AnydriveEthercatSlave::getJointAccelerationKfLambda2(float& lambda) {
  RecLock lock(mutex_);
  return sendSdoRead(OD_FILTER_JOINT_ACCELERATION_ID, OD_FILTER_JOINT_ACCELERATION_SID_KF_LAMBDA_2, false, lambda);
}

bool AnydriveEthercatSlave::setJointAccelerationKfLambda2(const float lambda) {
  RecLock lock(mutex_);
  ANYDRIVE_NAMED_DEBUG("Setting joint acceleration KF lambda^2 (" << lambda << ").");
  return sendSdoWrite(OD_FILTER_JOINT_ACCELERATION_ID, OD_FILTER_JOINT_ACCELERATION_SID_KF_LAMBDA_2, false, lambda);
}

bool AnydriveEthercatSlave::getJointAccelerationKfGamma(float& gamma) {
  RecLock lock(mutex_);
  return sendSdoRead(OD_FILTER_JOINT_ACCELERATION_ID, OD_FILTER_JOINT_ACCELERATION_SID_KF_GAMMA, false, gamma);
}

bool AnydriveEthercatSlave::setJointAccelerationKfGamma(const float gamma) {
  RecLock lock(mutex_);
  ANYDRIVE_NAMED_DEBUG("Setting joint acceleration KF gamma (" << gamma << ").");
  return sendSdoWrite(OD_FILTER_JOINT_ACCELERATION_ID, OD_FILTER_JOINT_ACCELERATION_SID_KF_GAMMA, false, gamma);
}

bool AnydriveEthercatSlave::getJointAccelerationEmaAlpha(float& alpha) {
  RecLock lock(mutex_);
  return sendSdoRead(OD_FILTER_JOINT_ACCELERATION_ID, OD_FILTER_JOINT_ACCELERATION_SID_EMA_ALPHA, false, alpha);
}

bool AnydriveEthercatSlave::setJointAccelerationEmaAlpha(const float alpha) {
  RecLock lock(mutex_);
  ANYDRIVE_NAMED_DEBUG("Setting joint acceleration EMA alpha (" << alpha << ").");
  return sendSdoWrite(OD_FILTER_JOINT_ACCELERATION_ID, OD_FILTER_JOINT_ACCELERATION_SID_EMA_ALPHA, false, alpha);
}

bool AnydriveEthercatSlave::writeConfiguration() {
  RecLock lock(mutex_);
  ANYDRIVE_NAMED_INFO("Writing configuration to flash.");
  return sendSdoWrite(OD_FLASH_WRITE_CONFIGURATION_ID, 0x00, false, static_cast<uint16_t>(OD_FLASH_WRITE_CONFIGURATION_ID_VAL_RUN));
}

bool AnydriveEthercatSlave::requestStatusword() {
  // The statusword does not have to be requested in advance, therefore it can be read in getStatusword(..) directly.
  return true;
}

bool AnydriveEthercatSlave::getStatusword(anydrive::Statusword& statusword) {
  RecLock lock(mutex_);
  uint32_t data = 0;
  if (!sendSdoRead(OD_DSP402_STATUS_WORD_ID, 0x00, false, data)) {
    return false;
  }
  statusword.setData(data);
  return true;
}

void AnydriveEthercatSlave::setControlword(const uint16_t controlwordId) {
  ANYDRIVE_NAMED_DEBUG("Setting controlword (" << controlwordId << ").");
  RecLock lock(controlwordIdMutex_);
  controlwordId_ = controlwordId;
}

void AnydriveEthercatSlave::resetControlword() {
  ANYDRIVE_NAMED_DEBUG("Resetting controlword.");
  RecLock lock(controlwordIdMutex_);
  controlwordId_ = 0;
}

void AnydriveEthercatSlave::setCommand(const anydrive::Command& command) {
  RecLock lock(commandMutex_);
  command_ = command;
}

void AnydriveEthercatSlave::getReading(anydrive::ReadingExtended& reading) {
  RecLock lock(readingMutex_);
  reading = reading_;
}

bool AnydriveEthercatSlave::getRtdlEnabled(bool& enabled) {
  RecLock lock(mutex_);
  uint16_t value;
  bool isOk = sendSdoRead(OD_CONTROL_RTDL_CONTROL_ID, OD_CONTROL_RTDL_CONTROL_SID_ENABLE, false, value);
  enabled = (value == OD_CONTROL_RTDL_CONTROL_SID_ENABLE_VAL_ENABLE);
  return isOk;
}

bool AnydriveEthercatSlave::setRtdlEnable(const bool enable) {
  RecLock lock(mutex_);
  ANYDRIVE_NAMED_INFO((enable ? "En" : "Dis") << "abling RTDL.");
  bool isOk = sendSdoWrite(OD_CONTROL_RTDL_CONTROL_ID, OD_CONTROL_IMU_CONFIG_SID_ENABLE, false,
                           (enable ? OD_CONTROL_RTDL_CONTROL_SID_ENABLE_VAL_ENABLE : OD_CONTROL_RTDL_CONTROL_SID_ENABLE_VAL_DISABLE));
  if (isOk) {
    isRtdlRunning_ = enable;
  }
  return isOk;
}

bool AnydriveEthercatSlave::setRtdlCommand(const uint16_t command) {
  RecLock lock(mutex_);
  if (command > 3) {
    ANYDRIVE_NAMED_ERROR("Invalid RTDL command (" << command << ").");
    return false;
  }
  ANYDRIVE_NAMED_DEBUG("Setting RTDL command (" << command << ").");
  return sendSdoWrite(OD_CONTROL_RTDL_CONTROL_ID, OD_CONTROL_RTDL_CONTROL_SID_COMMAND, false, command);
}

bool AnydriveEthercatSlave::getRtdlStatus(uint16_t& status) {
  RecLock lock(mutex_);
  return sendSdoRead(OD_CONTROL_RTDL_CONTROL_ID, OD_CONTROL_RTDL_CONTROL_SID_STATUS, false, status);
}

bool AnydriveEthercatSlave::getRtdlLoggingFrequency(uint16_t& frequency) {
  RecLock lock(mutex_);
  return sendSdoRead(OD_CONTROL_RTDL_CONTROL_ID, OD_CONTROL_RTDL_CONTROL_SID_LOGGING_FREQUENCY, false, frequency);
}

bool AnydriveEthercatSlave::setRtdlLoggingFrequency(const uint16_t frequency) {
  RecLock lock(mutex_);
  if (frequency > 1) {
    ANYDRIVE_NAMED_ERROR("Invalid RTDL logging frequency (" << frequency << ").");
    return false;
  }
  ANYDRIVE_NAMED_DEBUG("Setting RTDL logging frequency (" << frequency << ").");
  return sendSdoWrite(OD_CONTROL_RTDL_CONTROL_ID, OD_CONTROL_RTDL_CONTROL_SID_LOGGING_FREQUENCY, false, frequency);
}

bool AnydriveEthercatSlave::getRtdlStreamingFrequency(uint16_t& frequency) {
  RecLock lock(mutex_);
  return sendSdoRead(OD_CONTROL_RTDL_CONTROL_ID, OD_CONTROL_RTDL_CONTROL_SID_STREAMING_FREQUENCY, false, frequency);
}

bool AnydriveEthercatSlave::setRtdlStreamingFrequency(const uint16_t frequency) {
  RecLock lock(mutex_);
  ANYDRIVE_NAMED_DEBUG("Setting RTDL streaming frequency (" << frequency << " Hz).");
  return sendSdoWrite(OD_CONTROL_RTDL_CONTROL_ID, OD_CONTROL_RTDL_CONTROL_SID_STREAMING_FREQUENCY, false, frequency);
}

bool AnydriveEthercatSlave::getRtdlLastTimestamp(uint64_t& timestamp) {
  RecLock lock(mutex_);
  return sendSdoRead(OD_CONTROL_RTDL_LAST_TIMESTAMP_ID, 0x0, false, timestamp);
}

bool AnydriveEthercatSlave::configurePdo(const PdoTypeEnum pdoTypeEnum) {
  if (pdoTypeEnum == PdoTypeEnum::NA) {
    ANYDRIVE_NAMED_ERROR("Invalid EtherCAT PDO Type.");
    return false;
  }

  // If PDO setup is already active, return.
  if (pdoTypeEnum == getCurrentPdoTypeEnum()) {
    return true;
  }

  {
    RecLock lock(mutex_);
    if (!sendSdoWrite(OD_SWITCH_PDO_ID, OD_SWITCH_PDO_SID, false, pdoInfos_.at(pdoTypeEnum).moduleId_)) {
      return false;
    }
  }

  currentPdoTypeEnum_ = pdoTypeEnum;
  return true;
}

bool AnydriveEthercatSlave::sendSdoReadString(const uint16_t index, std::string& string) {
  RecLock lock(mutex_);
  const uint8_t lenSubindex = 0;
  const uint8_t dataSubindex = 1;

  // Read number of subindices.
  uint8_t nSubindex = 0;
  if (!sendSdoRead(index, lenSubindex, false, nSubindex)) {
    return false;
  }

  // Read char casted as an uint32 array.
  const uint8_t nChars = nSubindex * sizeof(uint32_t) / sizeof(char);
  // TODO(sam) Find better solution instead of disabling -Wvla.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wvla"
  char arrayChar[nChars + 1];
#pragma GCC diagnostic pop

  for (uint8_t i = 0; i < nSubindex; i++) {
    if (!sendSdoRead(index, dataSubindex + i, false, ((uint32_t*)arrayChar)[i])) {  // NOLINT
      return false;
    }
  }
  arrayChar[nChars] = '\0';

  // Transform char array to string.
  string = std::string(arrayChar);
  return true;
}

bool AnydriveEthercatSlave::sendSdoWriteString(const uint16_t index, const std::string& string) {
  RecLock lock(mutex_);
  const uint8_t lenSubindex = 0;
  const uint8_t dataSubindex = 1;

  // Read number of subindices.
  uint8_t nSubindex = 0;
  if (!sendSdoRead(index, lenSubindex, false, nSubindex)) {
    return false;
  }

  // Check if the string is not too big.
  const uint8_t nChars = nSubindex * sizeof(uint32_t) / sizeof(char);
  if (string.size() > nChars) {
    ANYDRIVE_NAMED_WARN("String is too big (" << string.size() << " > " << (uint16_t)nChars << "), cannot be written.");
    return false;
  }

  // Transform string to char array.
  // TODO(sam) Find better solution instead of disabling -Wvla.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wvla"
  char arrayChar[nChars];
#pragma GCC diagnostic pop
  for (uint8_t i = 0; i < nChars; i++) {
    arrayChar[i] = (i < string.size()) ? string[i] : '\0';
  }

  // Write char casted as an uint32 array.
  for (uint8_t i = 0; i < nSubindex; i++) {
    if (!sendSdoWrite(index, dataSubindex + i, false, ((uint32_t*)arrayChar)[i])) {  // NOLINT
      return false;
    }
  }

  return true;
}

bool AnydriveEthercatSlave::getCalibrationTypeEnum(anydrive::calibration::CalibrationTypeEnum& calibrationTypeEnum) {
  uint16_t id;
  if (getCalibrationTypeId(id)) {
    calibrationTypeEnum = anydrive::calibration::calibrationTypeIdToEnum(id);
    return true;
  }
  return false;
}

bool AnydriveEthercatSlave::setCalibrationTypeEnum(const anydrive::calibration::CalibrationTypeEnum calibrationTypeEnum) {
  return setCalibrationTypeId(anydrive::calibration::calibrationTypeEnumToId(calibrationTypeEnum));
}

bool AnydriveEthercatSlave::getCalibrationTypeId(uint16_t& calibrationTypeId) {
  RecLock lock(mutex_);
  return sendSdoRead(OD_CALIB_SELECTION_ID, 0x00, false, calibrationTypeId);
}

bool AnydriveEthercatSlave::setCalibrationTypeId(const uint16_t calibrationTypeId) {
  RecLock lock(mutex_);
  return sendSdoWrite(OD_CALIB_SELECTION_ID, 0x00, false, calibrationTypeId);
}

bool AnydriveEthercatSlave::getLockStatus(uint32_t& status) {
  RecLock lock(mutex_);
  return sendSdoRead(OD_VARIOUS_PASSWORD_ID, 3, false, status);
}

bool AnydriveEthercatSlave::sendPassword(const std::string& password) {
  RecLock lock(mutex_);
  return sendSdoWriteGeneric(std::to_string(OD_VARIOUS_PASSWORD_ID), "1", "string", password);
}

}  // namespace anydrive_ethercat
