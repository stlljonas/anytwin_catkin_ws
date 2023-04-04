/** \file: ImuConfig.h
    \brief: define custom struct that holds IMU driver configuration data
  */

#ifndef IMU_CONFIG_H
#define IMU_CONFIG_H

#include <string>
#include <utility>
#include <cstdint>

struct ImuConfig {

  // name of Xsens port
  // don't need to provide, will auto-detect and update
  std::string serialDev_;
  // serial key
  std::string serialKey_;
  // baudrate (MUST HAVE for timing correction)
  int serialBaudrate_ = 0;
  // sensor data output frequency (Hz)
  int outputFrequency_ = 0;
  // size of FIFO buffer to temporarily store data
  int bufferSize_ = 0;

  // wait time between attempts to connect (ms)
  int retryInterval_ = 500;
  // max interval between consecutive pkgs (ms)
  int maxInterPacketTime_ = 500;
  // whether the sensor should output orientation
  bool outputOrientation_ = false;

  /* synchronization parameters */
  // whether we should correct timing by delay over serial port
  bool correctTiming_ = true;
  // whether xsense should send the status word too
  bool sendStatusWord_ = true;
  // whether to send the sync out signal or not
  bool sendSync_ = false;
  // whether to listen for external sync trigger or not
  bool receiveSync_ = false;
  //skip factor, 0 means no skipping, 1 means skip 1 pulse etc.
  int skipFactor_ = 9;
  // pulse width in microseconds
  int pulseWidth_ = 2000;

};

#endif
