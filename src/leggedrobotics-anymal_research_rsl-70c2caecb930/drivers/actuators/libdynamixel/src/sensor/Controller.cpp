#include "libdynamixel/sensor/Controller.h"

#include <bitset>
#include <sstream>

#include "libdynamixel/com/SerialPort.h"
#include "libdynamixel/sensor/Packet.h"
#include "libdynamixel/sensor/Instructions.h"
#include "libdynamixel/sensor/Addresses.h"
#include "libdynamixel/exceptions/IOException.h"
#include "libdynamixel/exceptions/BadArgumentException.h"

namespace dynamixel {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

  Controller::Controller(const std::shared_ptr<SerialPort>& serialPort) :
      serialPort_(serialPort) {
  }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

  void Controller::writePacket(const std::shared_ptr<Packet>& packet) {
    *this << *packet;
  }

  std::shared_ptr<Packet> Controller::readPacket(uint8_t id) {
    auto packet = std::make_shared<Packet>();
    packet->setId(id);
    *this >> *packet;
    return packet;
  }

  bool Controller::ping(uint8_t id) {
    auto packet = std::make_shared<Packet>();
    packet->setId(id);
    packet->setLength(2);
    packet->setInstructionOrError(Instructions::PING);
    packet->setChecksum(packet->computeChecksum());
    try {
      writePacket(packet);
      auto status = readPacket(id);
      if (status->getInstructionOrError())
        return false;
      else
        return true;
    }
    catch (const IOException& e) {
      return false;
    }
  }

  bool Controller::reset(uint8_t id) {
    auto packet = std::make_shared<Packet>();
    packet->setId(id);
    packet->setLength(2);
    packet->setInstructionOrError(Instructions::RESET);
    packet->setChecksum(packet->computeChecksum());
    try {
      writePacket(packet);
      auto status = readPacket(id);
      if (status->getInstructionOrError())
        return false;
      else
        return true;
    }
    catch (const IOException& e) {
      return false;
    }
  }

  bool Controller::action(uint8_t id) {
    auto packet = std::make_shared<Packet>();
    packet->setId(id);
    packet->setLength(2);
    packet->setInstructionOrError(Instructions::ACTION);
    packet->setChecksum(packet->computeChecksum());
    try {
      writePacket(packet);
      auto status = readPacket(id);
      if (status->getInstructionOrError())
        return false;
      else
        return true;
    }
    catch (const IOException& e) {
      return false;
    }
  }

  std::shared_ptr<Packet> Controller::writeData(uint8_t id, uint8_t address,
      const std::vector<uint8_t>& data) {
    auto packet = std::make_shared<Packet>();
    packet->setId(id);
    packet->setLength(data.size() + 3);
    packet->setInstructionOrError(Instructions::WRITE_DATA);
    std::vector<uint8_t> parameters;
    parameters.reserve(data.size() + 1);
    parameters.push_back(address);
    parameters.insert(parameters.end(), data.cbegin(), data.cend());
    packet->setParameters(parameters);
    packet->setChecksum(packet->computeChecksum());
    writePacket(packet);
    return readPacket(id);
  }

  std::shared_ptr<Packet> Controller::regWriteData(uint8_t id, uint8_t address,
      const std::vector<uint8_t>& data) {
    auto packet = std::make_shared<Packet>();
    packet->setId(id);
    packet->setLength(data.size() + 3);
    packet->setInstructionOrError(Instructions::REG_WRITE);
    std::vector<uint8_t> parameters;
    parameters.reserve(data.size() + 1);
    parameters.push_back(address);
    parameters.insert(parameters.end(), data.cbegin(), data.cend());
    packet->setParameters(parameters);
    packet->setChecksum(packet->computeChecksum());
    writePacket(packet);
    return readPacket(id);
  }

  std::shared_ptr<Packet> Controller::syncWriteData(uint8_t address, const
      std::unordered_map<uint8_t, std::vector<uint8_t> >& data) {
    if (data.empty())
      throw BadArgumentException<size_t>(data.size(),
        "Controller::syncWriteData: data is empty");
    const auto dataLength = data.begin()->second.size();
    const auto numIds = data.size();
    auto packet = std::make_shared<Packet>();
    packet->setId(broadcastingId);
    packet->setLength((dataLength + 1) * numIds + 4);
    packet->setInstructionOrError(Instructions::SYNC_WRITE);
    std::vector<uint8_t> parameters;
    parameters.push_back(address);
    parameters.push_back(dataLength);
    for (const auto& id : data) {
      if (id.second.size() != dataLength)
        throw BadArgumentException<size_t>(id.second.size(),
          "Controller::syncWriteData: every IDs must have same data length");
      parameters.push_back(id.first);
      parameters.insert(parameters.end(), id.second.cbegin(), id.second.cend());
    }
    packet->setParameters(parameters);
    packet->setChecksum(packet->computeChecksum());
    writePacket(packet);
    return readPacket(broadcastingId);
  }

  std::shared_ptr<Packet> Controller::readData(uint8_t id, uint8_t address,
      uint8_t numBytes) {
    auto packet = std::make_shared<Packet>();
    packet->setId(id);
    packet->setLength(4);
    packet->setInstructionOrError(Instructions::READ_DATA);
    std::vector<uint8_t> parameters;
    parameters.reserve(2);
    parameters.push_back(address);
    parameters.push_back(numBytes);
    packet->setParameters(parameters);
    packet->setChecksum(packet->computeChecksum());
    writePacket(packet);
    return readPacket(id);
  }

  uint16_t Controller::getModelNumber(uint8_t id) {
    auto status = readData(id, Addresses::modelNumberLow, 2);
    if (status->getInstructionOrError())
      throw IOException("Controller::getModelNumber(): \n" +
        getErrorString(status->getInstructionOrError()));
    return static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
  }

  uint8_t Controller::getFirmwareVersion(uint8_t id) {
    auto status = readData(id, Addresses::firmwareVersion, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getFirmwareVersion(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  uint8_t Controller::getId(uint8_t id) {
    auto status = readData(id, Addresses::id, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getId(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  void Controller::setId(uint8_t id, uint8_t newId, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(newId);
    auto status = registered ? regWriteData(id, Addresses::id, data) :
      writeData(id, Addresses::id, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setId(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Controller::getBaudRate(uint8_t id) {
    auto status = readData(id, Addresses::baudRate, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getBaudRate(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  void Controller::setBaudRate(uint8_t id, uint8_t baudRate, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(baudRate);
    auto status = registered ? regWriteData(id, Addresses::baudRate, data) :
      writeData(id, Addresses::baudRate, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setBaudRate(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Controller::getReturnDelayTime(uint8_t id) {
    auto status = readData(id, Addresses::returnDelayTime, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getReturnDelayTime(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  void Controller::setReturnDelayTime(uint8_t id, uint8_t returnDelayTime, bool
      registered) {
    std::vector<uint8_t> data;
    data.push_back(returnDelayTime);
    auto status = registered ? regWriteData(id, Addresses::returnDelayTime,
      data) : writeData(id, Addresses::returnDelayTime, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setReturnDelayTime(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint16_t Controller::getCwAngleLimit(uint8_t id) {
    auto status = readData(id, Addresses::CWAngleLimitLow, 2);
    if (status->getInstructionOrError())
      throw IOException("Controller::getCwAngleLimit(): \n" +
        getErrorString(status->getInstructionOrError()));
    return static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
  }

  void Controller::setCwAngleLimit(uint8_t id, uint16_t cwAngleLimit, bool
      registered) {
    std::vector<uint8_t> data;
    data.push_back(reinterpret_cast<uint8_t*>(&cwAngleLimit)[0]);
    data.push_back(reinterpret_cast<uint8_t*>(&cwAngleLimit)[1]);
    auto status = registered ? regWriteData(id, Addresses::CWAngleLimitLow,
      data) : writeData(id, Addresses::CWAngleLimitLow, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setCwAngleLimit(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint16_t Controller::getCcwAngleLimit(uint8_t id) {
    auto status = readData(id, Addresses::CCWAngleLimitLow, 2);
    if (status->getInstructionOrError())
      throw IOException("Controller::getCcwAngleLimit(): \n" +
        getErrorString(status->getInstructionOrError()));
    return static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
  }

  void Controller::setCcwAngleLimit(uint8_t id, uint16_t ccwAngleLimit, bool
      registered) {
    std::vector<uint8_t> data;
    data.push_back(reinterpret_cast<uint8_t*>(&ccwAngleLimit)[0]);
    data.push_back(reinterpret_cast<uint8_t*>(&ccwAngleLimit)[1]);
    auto status = registered ? regWriteData(id, Addresses::CCWAngleLimitLow,
      data) : writeData(id, Addresses::CCWAngleLimitLow, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setCcwAngleLimit(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  void Controller::getAngleLimits(uint8_t id, uint16_t& cwAngleLimit, uint16_t&
      ccwAngleLimit) {
    auto status = readData(id, Addresses::CWAngleLimitLow, 4);
    if (status->getInstructionOrError())
      throw IOException("Controller::getAngleLimits(): \n" +
        getErrorString(status->getInstructionOrError()));
    cwAngleLimit = static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
    ccwAngleLimit = static_cast<uint16_t>(status->getParameters()[3]) << 8 |
      static_cast<uint16_t>(status->getParameters()[2]);
  }

  void Controller::setAngleLimits(uint8_t id, uint16_t cwAngleLimit, uint16_t
      ccwAngleLimit, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(reinterpret_cast<uint8_t*>(&cwAngleLimit)[0]);
    data.push_back(reinterpret_cast<uint8_t*>(&cwAngleLimit)[1]);
    data.push_back(reinterpret_cast<uint8_t*>(&ccwAngleLimit)[0]);
    data.push_back(reinterpret_cast<uint8_t*>(&ccwAngleLimit)[1]);
    auto status = registered ? regWriteData(id, Addresses::CWAngleLimitLow,
      data) : writeData(id, Addresses::CWAngleLimitLow, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setAngleLimits(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Controller::getHighestLimitTemperature(uint8_t id) {
    auto status = readData(id, Addresses::highestLimitTemperature, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getHighestLimitTemperature(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  void Controller::setHighestLimitTemperature(uint8_t id, uint8_t temperature,
      bool registered) {
    std::vector<uint8_t> data;
    data.push_back(temperature);
    auto status = registered ? regWriteData(id,
      Addresses::highestLimitTemperature, data) : writeData(id,
      Addresses::highestLimitTemperature, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setHighestLimitTemperature(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Controller::getHighestLimitVoltage(uint8_t id) {
    auto status = readData(id, Addresses::highestLimitVoltage, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getHighestLimitVoltage(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  void Controller::setHighestLimitVoltage(uint8_t id, uint8_t voltage, bool
      registered) {
    std::vector<uint8_t> data;
    data.push_back(voltage);
    auto status = registered ? regWriteData(id, Addresses::highestLimitVoltage,
      data) : writeData(id, Addresses::highestLimitVoltage, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setHighestLimitVoltage(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Controller::getLowestLimitVoltage(uint8_t id) {
    auto status = readData(id, Addresses::lowestLimitVoltage, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getLowestLimitVoltage(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  void Controller::setLowestLimitVoltage(uint8_t id, uint8_t voltage, bool
      registered) {
    std::vector<uint8_t> data;
    data.push_back(voltage);
    auto status = registered ? regWriteData(id, Addresses::lowestLimitVoltage,
      data) : writeData(id, Addresses::lowestLimitVoltage, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setLowestLimitVoltage(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint16_t Controller::getMaxTorque(uint8_t id) {
    auto status = readData(id, Addresses::maxTorqueLow, 2);
    if (status->getInstructionOrError())
      throw IOException("Controller::getMaxTorque(): \n" +
        getErrorString(status->getInstructionOrError()));
    return static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
  }

  void Controller::setMaxTorque(uint8_t id, uint16_t torque, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(reinterpret_cast<uint8_t*>(&torque)[0]);
    data.push_back(reinterpret_cast<uint8_t*>(&torque)[1]);
    auto status = registered ? regWriteData(id, Addresses::maxTorqueLow, data) :
      writeData(id, Addresses::maxTorqueLow, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setMaxTorque(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Controller::getStatusReturnLevel(uint8_t id) {
    auto status = readData(id, Addresses::statusReturnLevel, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getStatusReturnLevel(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  void Controller::setStatusReturnLevel(uint8_t id, uint8_t level, bool
      registered) {
    std::vector<uint8_t> data;
    data.push_back(level);
    auto status = registered ? regWriteData(id, Addresses::statusReturnLevel,
      data) : writeData(id, Addresses::statusReturnLevel, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setStatusReturnLevel(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Controller::getAlarmLed(uint8_t id) {
    auto status = readData(id, Addresses::alarmLED, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getAlarmLed(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  void Controller::setAlarmLed(uint8_t id, uint8_t code, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(code);
    auto status = registered ? regWriteData(id, Addresses::alarmLED, data) :
      writeData(id, Addresses::alarmLED, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setAlarmLed(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Controller::getAlarmShutdown(uint8_t id) {
    auto status = readData(id, Addresses::alarmShutdown, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getAlarmShutdown(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  void Controller::setAlarmShutdown(uint8_t id, uint8_t code, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(code);
    auto status = registered ? regWriteData(id, Addresses::alarmShutdown,
      data) : writeData(id, Addresses::alarmShutdown, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setAlarmShutdown(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint16_t Controller::getMultiTurnOffset(uint8_t id) {
    auto status = readData(id, Addresses::multiTurnOffsetLow, 2);
    if (status->getInstructionOrError())
      throw IOException("Controller::getMultiTurnOffset(): \n" +
        getErrorString(status->getInstructionOrError()));
    return static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
  }

  void Controller::setMultiTurnOffset(uint8_t id, uint16_t offset, bool
      registered) {
    std::vector<uint8_t> data;
    data.push_back(reinterpret_cast<uint8_t*>(&offset)[0]);
    data.push_back(reinterpret_cast<uint8_t*>(&offset)[1]);
    auto status = registered ? regWriteData(id, Addresses::multiTurnOffsetLow,
      data) : writeData(id, Addresses::multiTurnOffsetLow, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setMultiTurnOffset(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Controller::getResolutionDivider(uint8_t id) {
    auto status = readData(id, Addresses::resolutionDivider, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getResolutionDivider(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  void Controller::setResolutionDivider(uint8_t id, uint8_t divider, bool
      registered) {
    std::vector<uint8_t> data;
    data.push_back(divider);
    auto status = registered ? regWriteData(id, Addresses::resolutionDivider,
      data) : writeData(id, Addresses::resolutionDivider, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setResolutionDivider(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  bool Controller::isTorqueEnable(uint8_t id) {
    auto status = readData(id, Addresses::torqueEnable, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::isTorqueEnable(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0] ? true : false;
  }

  void Controller::setTorqueEnable(uint8_t id, bool enable, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(enable ? 1 : 0);
    auto status = registered ? regWriteData(id, Addresses::torqueEnable, data) :
      writeData(id, Addresses::torqueEnable, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setTorqueEnable(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  bool Controller::isLed(uint8_t id) {
    auto status = readData(id, Addresses::led, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::isLed(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0] ? true : false;
  }

  void Controller::setLed(uint8_t id, bool enable, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(enable ? 1 : 0);
    auto status = registered ? regWriteData(id, Addresses::led, data) :
      writeData(id, Addresses::led, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setLed(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Controller::getDGain(uint8_t id) {
    auto status = readData(id, Addresses::dGain, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getDGain(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  void Controller::setDGain(uint8_t id, uint8_t gain, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(gain);
    auto status = registered ? regWriteData(id, Addresses::dGain, data) :
      writeData(id, Addresses::dGain, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setDGain(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Controller::getIGain(uint8_t id) {
    auto status = readData(id, Addresses::iGain, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getIGain(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  void Controller::setIGain(uint8_t id, uint8_t gain, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(gain);
    auto status = registered ? regWriteData(id, Addresses::iGain, data) :
      writeData(id, Addresses::iGain, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setIGain(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Controller::getPGain(uint8_t id) {
    auto status = readData(id, Addresses::pGain, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getPGain(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  void Controller::setPGain(uint8_t id, uint8_t gain, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(gain);
    auto status = registered ? regWriteData(id, Addresses::pGain, data) :
      writeData(id, Addresses::pGain, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setPGain(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  void Controller::getPIDGains(uint8_t id, uint8_t& pGain, uint8_t& iGain,
      uint8_t& dGain) {
    auto status = readData(id, Addresses::dGain, 3);
    if (status->getInstructionOrError())
      throw IOException("Controller::getPIDGains(): \n" +
        getErrorString(status->getInstructionOrError()));
    dGain = status->getParameters()[0];
    iGain = status->getParameters()[1];
    pGain = status->getParameters()[2];
  }

  void Controller::setPIDGains(uint8_t id, uint8_t pGain, uint8_t iGain, uint8_t
      dGain, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(dGain);
    data.push_back(iGain);
    data.push_back(pGain);
    auto status = registered ? regWriteData(id, Addresses::dGain, data) :
      writeData(id, Addresses::dGain, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setPIDGains(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint16_t Controller::getGoalPosition(uint8_t id) {
    auto status = readData(id, Addresses::goalPositionLow, 2);
    if (status->getInstructionOrError())
      throw IOException("Controller::getGoalPosition(): \n" +
        getErrorString(status->getInstructionOrError()));
    return static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
  }

  void Controller::setGoalPosition(uint8_t id, uint16_t position, bool
      registered) {
    std::vector<uint8_t> data;
    data.push_back(reinterpret_cast<uint8_t*>(&position)[0]);
    data.push_back(reinterpret_cast<uint8_t*>(&position)[1]);
    auto status = registered ? regWriteData(id, Addresses::goalPositionLow,
      data) : writeData(id, Addresses::goalPositionLow, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setGoalPosition(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint16_t Controller::getMovingSpeed(uint8_t id) {
    auto status = readData(id, Addresses::movingSpeedLow, 2);
    if (status->getInstructionOrError())
      throw IOException("Controller::getMovingSpeed(): \n" +
        getErrorString(status->getInstructionOrError()));
    return static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
  }

  void Controller::setMovingSpeed(uint8_t id, uint16_t speed, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(reinterpret_cast<uint8_t*>(&speed)[0]);
    data.push_back(reinterpret_cast<uint8_t*>(&speed)[1]);
    auto status = registered ? regWriteData(id, Addresses::movingSpeedLow,
      data) : writeData(id, Addresses::movingSpeedLow, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setMovingSpeed(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  void Controller::getGoalPositionSpeed(uint8_t id, uint16_t& position,
      uint16_t& speed) {
    auto status = readData(id, Addresses::goalPositionLow, 4);
    if (status->getInstructionOrError())
      throw IOException("Controller::getGoalPositionSpeed(): \n" +
        getErrorString(status->getInstructionOrError()));
    position = static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
    speed = static_cast<uint16_t>(status->getParameters()[3]) << 8 |
      static_cast<uint16_t>(status->getParameters()[2]);
  }

  void Controller::setGoalPositionSpeed(uint8_t id, uint16_t position, uint16_t
      speed, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(reinterpret_cast<uint8_t*>(&position)[0]);
    data.push_back(reinterpret_cast<uint8_t*>(&position)[1]);
    data.push_back(reinterpret_cast<uint8_t*>(&speed)[0]);
    data.push_back(reinterpret_cast<uint8_t*>(&speed)[1]);
    auto status = registered ? regWriteData(id, Addresses::goalPositionLow,
      data) : writeData(id, Addresses::goalPositionLow, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setGoalPositionSpeed(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint16_t Controller::getTorqueLimit(uint8_t id) {
    auto status = readData(id, Addresses::torqueLimitLow, 2);
    if (status->getInstructionOrError())
      throw IOException("Controller::getTorqueLimit(): \n" +
        getErrorString(status->getInstructionOrError()));
    return static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
  }

  void Controller::setTorqueLimit(uint8_t id, uint16_t torque, bool
      registered) {
    std::vector<uint8_t> data;
    data.push_back(reinterpret_cast<uint8_t*>(&torque)[0]);
    data.push_back(reinterpret_cast<uint8_t*>(&torque)[1]);
    auto status = registered ? regWriteData(id, Addresses::torqueLimitLow,
      data) : writeData(id, Addresses::torqueLimitLow, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setTorqueLimit(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  void Controller::getGoalPositionSpeedTorque(uint8_t id, uint16_t& position,
      uint16_t& speed, uint16_t& torque) {
    auto status = readData(id, Addresses::goalPositionLow, 6);
    if (status->getInstructionOrError())
      throw IOException("Controller::getGoalPositionSpeedTorque(): \n" +
        getErrorString(status->getInstructionOrError()));
    position = static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
    speed = static_cast<uint16_t>(status->getParameters()[3]) << 8 |
      static_cast<uint16_t>(status->getParameters()[2]);
    torque = static_cast<uint16_t>(status->getParameters()[5]) << 8 |
      static_cast<uint16_t>(status->getParameters()[4]);
  }

  void Controller::setGoalPositionSpeedTorque(uint8_t id, uint16_t position,
      uint16_t speed, uint16_t torque, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(reinterpret_cast<uint8_t*>(&position)[0]);
    data.push_back(reinterpret_cast<uint8_t*>(&position)[1]);
    data.push_back(reinterpret_cast<uint8_t*>(&speed)[0]);
    data.push_back(reinterpret_cast<uint8_t*>(&speed)[1]);
    data.push_back(reinterpret_cast<uint8_t*>(&torque)[0]);
    data.push_back(reinterpret_cast<uint8_t*>(&torque)[1]);
    auto status = registered ? regWriteData(id, Addresses::goalPositionLow,
      data) : writeData(id, Addresses::goalPositionLow, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setGoalPositionSpeedTorque(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint16_t Controller::getPresentPosition(uint8_t id) {
    auto status = readData(id, Addresses::presentPositionLow, 2);
    if (status->getInstructionOrError())
      throw IOException("Controller::getPresentPosition(): \n" +
        getErrorString(status->getInstructionOrError()));
    return static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
  }

  uint16_t Controller::getPresentSpeed(uint8_t id) {
    auto status = readData(id, Addresses::presentSpeedLow, 2);
    if (status->getInstructionOrError())
      throw IOException("Controller::getPresentSpeed(): \n" +
        getErrorString(status->getInstructionOrError()));
    return static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
  }

  uint16_t Controller::getPresentLoad(uint8_t id) {
    auto status = readData(id, Addresses::presentLoadLow, 2);
    if (status->getInstructionOrError())
      throw IOException("Controller::getPresentLoad(): \n" +
        getErrorString(status->getInstructionOrError()));
    return static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
  }

  void Controller::getPresentPositionSpeedLoad(uint8_t id, uint16_t& position,
      uint16_t& speed, uint16_t& load) {
    auto status = readData(id, Addresses::presentPositionLow, 6);
    if (status->getInstructionOrError())
      throw IOException("Controller::getPresentPositionSpeedLoad(): \n" +
        getErrorString(status->getInstructionOrError()));
    position = static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
    speed = static_cast<uint16_t>(status->getParameters()[3]) << 8 |
      static_cast<uint16_t>(status->getParameters()[2]);
    load = static_cast<uint16_t>(status->getParameters()[5]) << 8 |
      static_cast<uint16_t>(status->getParameters()[4]);
  }

  uint8_t Controller::getPresentVoltage(uint8_t id) {
    auto status = readData(id, Addresses::presentVoltage, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getPresentVoltage(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  uint8_t Controller::getPresentTemperature(uint8_t id) {
    auto status = readData(id, Addresses::presentTemperature, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getPresentTemperature(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  bool Controller::isInstructionRegistered(uint8_t id) {
    auto status = readData(id, Addresses::registered, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::isInstructionRegistered(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0] ? true : false;
  }

  bool Controller::isMoving(uint8_t id) {
    auto status = readData(id, Addresses::moving, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::isMoving(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0] ? true : false;
  }

  void Controller::getState(uint8_t id, uint16_t& position, uint16_t& speed,
      uint16_t& load, uint8_t& voltage, uint8_t& temperature, bool& registered,
      bool& moving) {
    auto status = readData(id, Addresses::presentPositionLow, 10);
    if (status->getInstructionOrError())
      throw IOException("Controller::getState(): \n" +
        getErrorString(status->getInstructionOrError()));
    position = static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
    speed = static_cast<uint16_t>(status->getParameters()[3]) << 8 |
      static_cast<uint16_t>(status->getParameters()[2]);
    load = static_cast<uint16_t>(status->getParameters()[5]) << 8 |
      static_cast<uint16_t>(status->getParameters()[4]);
    voltage = status->getParameters()[6];
    temperature = status->getParameters()[7];
    registered = status->getParameters()[8] ? true : false;
    moving = status->getParameters()[9] ? true : false;
  }

  bool Controller::isEEPROMLock(uint8_t id) {
    auto status = readData(id, Addresses::lock, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::isEEPROMLock(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0] ? true : false;
  }

  void Controller::setEEPROMLock(uint8_t id, bool enable, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(enable ? 1 : 0);
    auto status = registered ? regWriteData(id, Addresses::lock, data) :
      writeData(id, Addresses::lock, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setEEPROMLock(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint16_t Controller::getPunch(uint8_t id) {
    auto status = readData(id, Addresses::punchLow, 2);
    if (status->getInstructionOrError())
      throw IOException("Controller::getPunch(): \n" +
        getErrorString(status->getInstructionOrError()));
    return static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
  }

  void Controller::setPunch(uint8_t id, uint16_t punch, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(reinterpret_cast<uint8_t*>(&punch)[0]);
    data.push_back(reinterpret_cast<uint8_t*>(&punch)[1]);
    auto status = registered ? regWriteData(id, Addresses::punchLow, data) :
      writeData(id, Addresses::punchLow, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setPunch(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint16_t Controller::getCurrent(uint8_t id) {
    auto status = readData(id, Addresses::currentLow, 2);
    if (status->getInstructionOrError())
      throw IOException("Controller::getCurrent(): \n" +
        getErrorString(status->getInstructionOrError()));
    return static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
  }

  bool Controller::isTorqueControlModeEnable(uint8_t id) {
    auto status = readData(id, Addresses::torqueControlModeEnable, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getTorqueControlModeEnable(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0] ? true : false;
  }

  void Controller::setTorqueControlModeEnable(uint8_t id, bool enable, bool
      registered) {
    std::vector<uint8_t> data;
    data.push_back(enable ? 1 : 0);
    auto status = registered ? regWriteData(id,
      Addresses::torqueControlModeEnable, data) : writeData(id,
      Addresses::torqueControlModeEnable, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setTorqueControlModeEnable(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint16_t Controller::getGoalTorque(uint8_t id) {
    auto status = readData(id, Addresses::goalTorqueLow, 2);
    if (status->getInstructionOrError())
      throw IOException("Controller::getGoalTorque(): \n" +
        getErrorString(status->getInstructionOrError()));
    return static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
  }

  void Controller::setGoalTorque(uint8_t id, uint16_t torque, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(reinterpret_cast<uint8_t*>(&torque)[0]);
    data.push_back(reinterpret_cast<uint8_t*>(&torque)[1]);
    auto status = registered ? regWriteData(id, Addresses::goalTorqueLow,
      data) : writeData(id, Addresses::goalTorqueLow, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setGoalTorque(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Controller::getGoalAcceleration(uint8_t id) {
    auto status = readData(id, Addresses::goalAcceleration, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getGoalAcceleration(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  void Controller::setGoalAcceleration(uint8_t id, uint8_t acceleration, bool
      registered) {
    std::vector<uint8_t> data;
    data.push_back(acceleration);
    auto status = registered ? regWriteData(id, Addresses::goalAcceleration,
      data) : writeData(id, Addresses::goalAcceleration, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setGoalAcceleration(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Controller::getCwComplianceMargin(uint8_t id) {
    auto status = readData(id, Addresses::cwComplianceMargin, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getCwComplianceMargin(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  void Controller::setCwComplianceMargin(uint8_t id, uint8_t margin, bool
      registered) {
    std::vector<uint8_t> data;
    data.push_back(margin);
    auto status = registered ? regWriteData(id, Addresses::cwComplianceMargin,
      data) : writeData(id, Addresses::cwComplianceMargin, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setCwComplianceMargin(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Controller::getCcwComplianceMargin(uint8_t id) {
    auto status = readData(id, Addresses::ccwComplianceMargin, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getCcwComplianceMargin(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  void Controller::setCcwComplianceMargin(uint8_t id, uint8_t margin, bool
      registered) {
    std::vector<uint8_t> data;
    data.push_back(margin);
    auto status = registered ? regWriteData(id, Addresses::ccwComplianceMargin,
      data) : writeData(id, Addresses::ccwComplianceMargin, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setCcwComplianceMargin(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Controller::getCwComplianceSlope(uint8_t id) {
    auto status = readData(id, Addresses::cwComplianceSlope, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getCwComplianceSlope(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  void Controller::setCwComplianceSlope(uint8_t id, uint8_t slope, bool
      registered) {
    std::vector<uint8_t> data;
    data.push_back(slope);
    auto status = registered ? regWriteData(id, Addresses::cwComplianceSlope,
      data) : writeData(id, Addresses::cwComplianceSlope, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setCwComplianceSlope(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Controller::getCcwComplianceSlope(uint8_t id) {
    auto status = readData(id, Addresses::ccwComplianceSlope, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getCcwComplianceSlope(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  void Controller::setCcwComplianceSlope(uint8_t id, uint8_t slope, bool
    registered) {
    std::vector<uint8_t> data;
    data.push_back(slope);
    auto status = registered ? regWriteData(id, Addresses::ccwComplianceSlope,
      data) : writeData(id, Addresses::ccwComplianceSlope, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setCcwComplianceSlope(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  void Controller::getCompliance(uint8_t id, uint8_t& cwComplianceMargin,
      uint8_t& ccwComplianceMargin, uint8_t& cwComplianceSlope, uint8_t&
      ccwComplianceSlope) {
    auto status = readData(id, Addresses::cwComplianceMargin, 4);
    if (status->getInstructionOrError())
      throw IOException("Controller::getCompliance(): \n" +
        getErrorString(status->getInstructionOrError()));
    cwComplianceMargin = status->getParameters()[0];
    ccwComplianceMargin = status->getParameters()[1];
    cwComplianceSlope = status->getParameters()[2];
    ccwComplianceSlope = status->getParameters()[3];
  }

  void Controller::setCompliance(uint8_t id, uint8_t cwComplianceMargin, uint8_t
      ccwComplianceMargin, uint8_t cwComplianceSlope, uint8_t
      ccwComplianceSlope, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(cwComplianceMargin);
    data.push_back(ccwComplianceMargin);
    data.push_back(cwComplianceSlope);
    data.push_back(ccwComplianceSlope);
    auto status = registered ? regWriteData(id, Addresses::cwComplianceMargin,
      data) : writeData(id, Addresses::cwComplianceMargin, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setCompliance(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint8_t Controller::getDriveMode(uint8_t id) {
    auto status = readData(id, Addresses::driveMode, 1);
    if (status->getInstructionOrError())
      throw IOException("Controller::getDriveMode(): \n" +
        getErrorString(status->getInstructionOrError()));
    return status->getParameters()[0];
  }

  void Controller::setDriveMode(uint8_t id, uint8_t mode, bool registered) {
    std::vector<uint8_t> data;
    data.push_back(mode);
    auto status = registered ? regWriteData(id, Addresses::driveMode,
      data) : writeData(id, Addresses::driveMode, data);
    if (status->getInstructionOrError())
      throw IOException("Controller::setDriveMode(): \n" +
        getErrorString(status->getInstructionOrError()));
  }

  uint16_t Controller::getSensedCurrent(uint8_t id) {
    auto status = readData(id, Addresses::sensedCurrentLow, 2);
    if (status->getInstructionOrError())
      throw IOException("Controller::getSensedCurrent(): \n" +
        getErrorString(status->getInstructionOrError()));
    return static_cast<uint16_t>(status->getParameters()[1]) << 8 |
      static_cast<uint16_t>(status->getParameters()[0]);
  }

  void Controller::write(const char* buffer, size_t numBytes) {
    serialPort_->write(buffer, numBytes);
  }

  void Controller::read(char* buffer, size_t numBytes) {
    serialPort_->read(buffer, numBytes);
  }

  std::string Controller::getErrorString(uint8_t errorCode) const {
    std::bitset<8> bitset(errorCode);
    std::stringstream errorStringStream;
    if (bitset.test(7))
      errorStringStream << std::string("-") << std::endl;
    if (bitset.test(6))
      errorStringStream << std::string("Instruction Error") << std::endl;
    if (bitset.test(5))
      errorStringStream << std::string("Overload Error") << std::endl;
    if (bitset.test(4))
      errorStringStream << std::string("Checksum Error") << std::endl;
    if (bitset.test(3))
      errorStringStream << std::string("Range Error") << std::endl;
    if (bitset.test(2))
      errorStringStream << std::string("Overheating Error") << std::endl;
    if (bitset.test(1))
      errorStringStream << std::string("Angle Limit Error") << std::endl;
    if (bitset.test(0))
      errorStringStream << std::string("Input Voltage Error") << std::endl;
    return errorStringStream.str();
  }

  Model Controller::getModelInformation(uint16_t modelNumber) {
    if (isModelSupported(modelNumber))
      return Models::table.at(modelNumber);
    else
      throw BadArgumentException<size_t>(modelNumber,
        "Controller::getModelInformation: invalid model number");
  }

}
