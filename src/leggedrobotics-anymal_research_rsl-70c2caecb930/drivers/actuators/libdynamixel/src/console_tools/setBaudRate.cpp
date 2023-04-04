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

bool baudRateIsValid(unsigned int baudRate)
{
  return (baudRate == 0) ||
         (baudRate == 50) ||
         (baudRate == 75) ||
         (baudRate == 110) ||
         (baudRate == 134) ||
         (baudRate == 150) ||
         (baudRate == 200) ||
         (baudRate == 300) ||
         (baudRate == 600) ||
         (baudRate == 1200) ||
         (baudRate == 1800) ||
         (baudRate == 2400) ||
         (baudRate == 4800) ||
         (baudRate == 9600) ||
         (baudRate == 19200) ||
         (baudRate == 38400) ||
         (baudRate == 57600) ||
         (baudRate == 115200) ||
         (baudRate == 230400) ||
         (baudRate == 1000000);
}

int main(int argc, char **argv)
{
  if (argc != 5)
  {
    std::cerr << "Usage: " << argv[0] << " <device> <baudrate> <id>  <newbaudrate>" << std::endl;
    return -1;
  }
  const std::string device = std::string(argv[1]);
  const unsigned int baudRate = atoi(argv[2]);
  const unsigned int id = atoi(argv[3]);
  const unsigned int newBaudRate = atoi(argv[4]);
  if (baudRateIsValid(newBaudRate))
  {
    Controller controller(std::make_shared<SerialPort>(device, baudRate));
    controller.setBaudRate(id, 2000000/newBaudRate-1, false);
    std::cout << "Baud rate has been set." << std::endl;
  }
  else
  {
    std::cout << "New baud rate is invalid. Available are:" << std::endl;
    std::cout << "0" << std::endl;
    std::cout << "50" << std::endl;
    std::cout << "75" << std::endl;
    std::cout << "110" << std::endl;
    std::cout << "134" << std::endl;
    std::cout << "150" << std::endl;
    std::cout << "200" << std::endl;
    std::cout << "300" << std::endl;
    std::cout << "600" << std::endl;
    std::cout << "1200" << std::endl;
    std::cout << "1800" << std::endl;
    std::cout << "2400" << std::endl;
    std::cout << "4800" << std::endl;
    std::cout << "9600" << std::endl;
    std::cout << "19200" << std::endl;
    std::cout << "38400" << std::endl;
    std::cout << "57600" << std::endl;
    std::cout << "115200" << std::endl;
    std::cout << "230400" << std::endl;
    std::cout << "1000000" << std::endl;
    return -1;
  }
  Controller controller(std::make_shared<SerialPort>(device, newBaudRate));
  std::cout << "New baud rate is " << 2000000/(controller.getBaudRate(id)+1) << std::endl;
  return 0;
}
