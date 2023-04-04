/** \file serialTest.cpp
    \brief This file is a testing binary for serial port.
  */

#include <iostream>
#include <string>
#include <cstdlib>
#include <memory>

#include "libdynamixel/com/SerialPort.h"
#include "libdynamixel/sensor/Controller.h"

using namespace dynamixel;

int main(int argc, char **argv) {
  if (argc != 4) {
    std::cerr << "Usage: " << argv[0] << " <device> <baudrate> <id>"
      << std::endl;
    return -1;
  }
  Controller controller(std::make_shared<SerialPort>(std::string(argv[1]),
    atoi(argv[2])));
  const auto id = atoi(argv[3]);
  const auto modelNumber = controller.getModelNumber(id);
  std::cout << "Model number: " << modelNumber << std::endl;
  controller.getModelInformation(modelNumber).ostream(std::cout);
  std::cout << "Firmware version: "
    << (unsigned)controller.getFirmwareVersion(id) << std::endl;
  std::cout << "Device ID: " << (unsigned)controller.getId(id) << std::endl;
  std::cout << "Baud rate: " << (unsigned)controller.getBaudRate(id)
    << std::endl;
  std::cout << "Return delay time (us): "
    << Controller::raw2us(controller.getReturnDelayTime(id)) << std::endl;
  std::cout << "Clockwise angle limit (rad): "
    << Controller::tick2angle(controller.getCwAngleLimit(id)) << std::endl;
  std::cout << "Counterclockwise angle limit (rad): "
    << Controller::tick2angle(controller.getCcwAngleLimit(id)) << std::endl;
  std::cout << "Highest limit temperature (C): "
    << (unsigned)controller.getHighestLimitTemperature(id) << std::endl;
  std::cout << "Highest limit voltage (V): "
    << Controller::raw2volt(controller.getHighestLimitVoltage(id)) << std::endl;
  std::cout << "Lowest limit voltage (V): "
    << Controller::raw2volt(controller.getLowestLimitVoltage(id)) << std::endl;
  std::cout << "Maximum torque (%): "
    << Controller::raw2torqueRatio(controller.getMaxTorque(id)) * 100.0
    << std::endl;
  std::cout << "Status return level: "
    << (unsigned)controller.getStatusReturnLevel(id) << std::endl;
  std::cout << "Alarm LED: " << (unsigned)controller.getAlarmLed(id)
    << std::endl;
  std::cout << "Alarm shutdown: " << (unsigned)controller.getAlarmShutdown(id)
    << std::endl;
  if (Controller::isModelMX(modelNumber)) {
    std::cout << "Multi-turn offset: " << controller.getMultiTurnOffset(id)
      << std::endl;
    std::cout << "Resolution divider: "
      << (unsigned)controller.getResolutionDivider(id) << std::endl;
  }
  std::cout << "Torque enable: " << (unsigned)controller.isTorqueEnable(id)
    << std::endl;
  std::cout << "LED: " << (unsigned)controller.isLed(id) << std::endl;
  if (controller.isLed(id))
    controller.setLed(id, false, true);
  else
    controller.setLed(id, true, true);
  controller.action(id);
  if (Controller::isModelMX(modelNumber)) {
    std::cout << "D gain: " << Controller::raw2Kd(controller.getDGain(id))
      << std::endl;
    std::cout << "I gain: " << Controller::raw2Ki(controller.getIGain(id))
      << std::endl;
    std::cout << "P gain: " << Controller::raw2Kp(controller.getPGain(id))
      << std::endl;
  }
  else {
    std::cout << "Clockwise compliance margin: "
      << controller.getCwComplianceMargin(id) << std::endl;
    std::cout << "Counterclockwise compliance margin: "
      << controller.getCcwComplianceMargin(id) << std::endl;
    std::cout << "Clockwise compliance slope: "
      << controller.getCwComplianceSlope(id) << std::endl;
    std::cout << "Counterclockwise compliance slope: "
      << controller.getCcwComplianceSlope(id) << std::endl;
  }
  std::cout << "Goal position (rad): "
    << Controller::tick2angle(controller.getGoalPosition(id)) << std::endl;
  std::cout << "Moving speed (rad/s): "
    << Controller::rpm2rps(Controller::raw2rpm(controller.getMovingSpeed(id)))
    << std::endl;
  std::cout << "Torque limit (%): "
    << Controller::raw2torqueRatio(controller.getTorqueLimit(id)) * 100.0
    << std::endl;
  std::cout << "Present position (rad): "
    << Controller::tick2angle(controller.getPresentPosition(id)) << std::endl;
  std::cout << "Present speed (rad/s): "
    << Controller::rpm2rps(Controller::raw2rpm(controller.getPresentSpeed(id)))
    << std::endl;
  std::cout << "Present load (%): "
    << Controller::raw2torqueRatio(controller.getPresentLoad(id)) * 100.0
    << std::endl;
  std::cout << "Present voltage (V): "
    << Controller::raw2volt(controller.getPresentVoltage(id)) << std::endl;
  std::cout << "Present temperature (C): "
    << (unsigned)controller.getPresentTemperature(id) << std::endl;
  std::cout << "Instruction registered: "
    << (unsigned)controller.isInstructionRegistered(id) << std::endl;
  std::cout << "Moving: " << (unsigned)controller.isMoving(id) << std::endl;
  std::cout << "EEPROM lock: " << (unsigned)controller.isEEPROMLock(id)
    << std::endl;
  std::cout << "Punch: " << controller.getPunch(id) << std::endl;
  std::cout << "Current (A): " << Controller::raw2amp(controller.getCurrent(id))
    << std::endl;
  if (Controller::isModelTorqueControl(modelNumber)) {
    std::cout << "Torque control mode enable: "
      << (unsigned)controller.isTorqueControlModeEnable(id) << std::endl;
    std::cout << "Goal torque (A): "
      << Controller::rawtorque2amp(controller.getGoalTorque(id)) << std::endl;
  }
  if (Controller::isModelMX(modelNumber))
    std::cout << "Goal acceleration (rad/sec^2): "
      << Controller::raw2rps2(controller.getGoalAcceleration(id)) << std::endl;
  controller.setMovingSpeed(1, 1);
  controller.setGoalPosition(1, 0);
  return 0;
}
