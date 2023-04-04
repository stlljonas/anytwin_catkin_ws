/** \file: ImuStatus.h
    \brief: define custom struct that holds IMU status
  */

#ifndef IMU_STATUS_H
#define IMU_STATUS_H

#include <memory>
#include <string>
#include <utility>
#include <cstdint>
#include <chrono>

struct ImuStatus {
  bool configParamReceived_ = false;
  bool connected_ = false;
  bool configured_ = false;
  bool isRunning_ = false;

  // Error Code
  int errCode_ = 0;
};


/**
 *　　Definition of Error Code
 *　
 *      0     no error
 *      10    connect: no port found
 *      11    connect: cannot open port
 *      12    connect: cannot create device
 *      13    device_config: device not created
 *      14    device_config: cannot switch to configure mode
 *      15    device_config: unknown or unsupported device
 *      16    device_config: cannot configure device
 *      17    cannot put device into measurement mode
 *      18    cannot set baudrate
 *      19    invalid serial key
 *      21    connection lost
 */

#endif
