/** \file: XsensMTImuDriver.cpp
    \brief: defines a ROS-independent driver class for Xsens MTi series IMU
  */

#include <message_logger/message_logger.hpp>

#include <xsensmt_interface/XsensMTInterface.h>

using namespace std;

namespace xsensmt {

/**
 *  Initialize internal parameters of the XsensMTInterface object
 *  MUST be called first
 */
bool XsensMTInterface::init() {
  xsController_ = XsControl::construct();
  return true;
}

/**
 *  Connect to and configure the sensor, start measurement
 */
bool XsensMTInterface::configure(const ImuConfig& configParam) {
  configParam_ = configParam;
  status_.configParamReceived_ = true;

  if (configParam_.bufferSize_ > 0) callbackHandler_.dataBuffer_.setCapacity(configParam_.bufferSize_);

  // configure timing correction parameters
  callbackHandler_.correctTiming_ = configParam_.correctTiming_;
  if (configParam_.serialBaudrate_ > 0) callbackHandler_.baudRate_ = configParam_.serialBaudrate_;

  if (!connect()) {
    return false;
  }

  if (!device_config()) {
    return false;
  }

  /* put device into measurement mode */
  if (!xsDevice_->gotoMeasurement()) {
    std::unique_lock<std::mutex> lock(mutexStatus_);
    status_.errCode_ = 17;  // cannot switch to measurement mode
    return false;
  }

  {
    std::unique_lock<std::mutex> lock(mutexStatus_);
    status_.isRunning_ = true;
    status_.errCode_ = 0;
  }

  xsDevice_->addCallbackHandler(&callbackHandler_);
  statusMonitorSpinnerThread_ = new std::thread(&XsensMTInterface::statusMonitorSpinner, this);

  return true;
}

/**
 *  When the XsensMTInterface object is no longer needed, clean up resources
 */
bool XsensMTInterface::cleanup() {
  statusMonitorSpinnerStop_ = true;

  statusMonitorSpinnerThread_->join();

  MELO_INFO("XsensMTInterface: Status monitor thread terminated.");

  if (xsDevice_) {
    xsController_->closePort(xsDevice_->portName().toStdString());
  }
  return true;
}

/**
 *  Update measurement results
 */
bool XsensMTInterface::update() { return true; }

/**
 *  Retrive measurement results
 */
bool XsensMTInterface::getMeasurement(ImuMeasurement& data) {
  /* retrieve data */
  if (!callbackHandler_.dataBuffer_.isEmpty()) {
    auto pkg = callbackHandler_.dataBuffer_.dequeue();
    XsDataPacket xspkg = pkg.first;

    if (xspkg.containsCalibratedAcceleration() && xspkg.containsCalibratedGyroscopeData()) {
      data.timestamp_ = pkg.second;
      data.counter_ = xspkg.packetCounter();

      XsVector acceleration = xspkg.calibratedAcceleration();
      data.linearAcceleration_[0] = acceleration.at(0);
      data.linearAcceleration_[1] = acceleration.at(1);
      data.linearAcceleration_[2] = acceleration.at(2);

      XsVector gyro = xspkg.calibratedGyroscopeData();
      data.angularVelocity_[0] = gyro.at(0);
      data.angularVelocity_[1] = gyro.at(1);
      data.angularVelocity_[2] = gyro.at(2);

      // trigger marker is on the bit 22 (starting from 0)
      data.triggerIndicator_ = bool((xspkg.status() >> 22) & 1);

    } else {
      MELO_ERROR("XsensMTInterface: IMU data does not contain calibrated acc. and gyroscope data!");
      return false;
    }
  } else {
    return false;
  }
  return true;
}

/**
 *  Retrive XsensMTInterface object and sensor status
 */
ImuStatus XsensMTInterface::getStatus() {
  std::unique_lock<std::mutex> lock(mutexStatus_);
  return status_;
}

/**
 *  Try to connect to sensor
 */
bool XsensMTInterface::connect() {
  /* clear device */
  xsDevice_ = 0;

  /* scan for sensor ports */
  XsPortInfo portInfo = XsScanner::scanPort(configParam_.serialDev_, XsBaud::numericToRate(configParam_.serialBaudrate_));
  /* try to open a port*/
  if (xsController_->openPort(portInfo.portName().toStdString(), portInfo.baudrate())) {
    xsDevice_ = xsController_->device(portInfo.deviceId());
    if (xsDevice_) {
      /* identify whether this is a xsens MTi device*/
      if (xsDevice_->deviceId().isMtMk4() | xsDevice_->deviceId().isMtx()) {
        MELO_INFO("XsensMTInterface: Sucessully connected xsens MTi device at port '%s' with baud rate '%d'!",
                  portInfo.portName().toStdString().c_str(), XsBaud::rateToNumeric(portInfo.baudrate()));

      } else {
        MELO_ERROR("XsensMTInterface: Device at port '%s' with baud rate '%d' is no xsens MTi device!",
                   portInfo.portName().toStdString().c_str(), XsBaud::rateToNumeric(portInfo.baudrate()));
        return false;
      }
    } else {
      MELO_ERROR("XsensMTInterface: Could not create device at port '%s' with baud rate '%d'!", portInfo.portName().toStdString().c_str(),
                 XsBaud::rateToNumeric(portInfo.baudrate()));
      return false;
    }
  } else {
    MELO_ERROR("XsensMTInterface: Could not open port '%s' with baud rate '%d'!", portInfo.portName().toStdString().c_str(),
               XsBaud::rateToNumeric(portInfo.baudrate()));
    return false;
  }

  status_.connected_ = true;
  return true;
}

/**
 *  Try to configure sensor
 */
bool XsensMTInterface::device_config() {
  if (!xsDevice_) {
    std::unique_lock<std::mutex> lock(mutexStatus_);
    status_.errCode_ = 13;  // device not created
    return false;
  }

  if (!xsDevice_->gotoConfig()) {
    xsDevice_ = 0;
    std::unique_lock<std::mutex> lock(mutexStatus_);
    status_.errCode_ = 14;  // cannot switch to configure mode
    return false;
  }

  /* write configuration */
  if (xsDevice_->deviceId().isMtx()) {
    XsOutputMode outputMode = XOM_Calibrated;
    if (configParam_.outputFrequency_) outputMode = outputMode | XsOutputMode(XOM_Orientation);

    XsOutputSettings outputSettings = XOS_Timestamp_PacketCounter | XOS_Dataformat_Double;
    XsDeviceMode deviceMode(configParam_.outputFrequency_);
    deviceMode.setModeFlag(outputMode);
    deviceMode.setSettingsFlag(outputSettings);

    if (!xsDevice_->setDeviceMode(deviceMode)) {
      xsDevice_ = 0;
      std::unique_lock<std::mutex> lock(mutexStatus_);
      status_.errCode_ = 16;  // cannot configure device
      return false;
    }
  } else if (xsDevice_->deviceId().isMtMk4()) {
    XsOutputConfigurationArray configArray;

    XsOutputConfiguration acc(XDI_Acceleration | XDI_SubFormatDouble, configParam_.outputFrequency_);
    configArray.push_back(acc);

    XsOutputConfiguration gyro(XDI_RateOfTurn | XDI_SubFormatDouble, configParam_.outputFrequency_);
    configArray.push_back(gyro);

    XsOutputConfiguration packetCounter(XDI_PacketCounter, configParam_.outputFrequency_);
    configArray.push_back(packetCounter);

    XsOutputConfiguration sampleTimeFine(XDI_SampleTimeFine, configParam_.outputFrequency_);
    configArray.push_back(sampleTimeFine);

    XsOutputConfiguration utcTime(XDI_UtcTime, configParam_.outputFrequency_);
    configArray.push_back(utcTime);

    if (configParam_.outputOrientation_) {
      XsOutputConfiguration orientation(XDI_Quaternion | XDI_SubFormatDouble, configParam_.outputFrequency_);
      configArray.push_back(orientation);
    }

    if (configParam_.sendStatusWord_) {
      XsOutputConfiguration status_word(XDI_StatusWord, configParam_.outputFrequency_);
      configArray.push_back(status_word);
    }

    if (!xsDevice_->setOutputConfiguration(configArray)) {
      xsDevice_ = 0;
      std::unique_lock<std::mutex> lock(mutexStatus_);
      status_.errCode_ = 16;  // cannot configure device
      return false;
    }

    XsSyncSettingArray syncSettingsArray;
    configureSyncSettings(syncSettingsArray);

    if (!xsDevice_->setSyncSettings(syncSettingsArray)) {
      xsDevice_ = 0;
      std::unique_lock<std::mutex> lock(mutexStatus_);
      status_.errCode_ = 16;  // cannot configure device
      std::cerr << "XsensMTIInferface: Cannot configure the sync." << std::endl;
      return false;
    }

  } else {
    xsDevice_ = 0;
    std::unique_lock<std::mutex> lock(mutexStatus_);
    status_.errCode_ = 15;  // unknown or unsupported device
    return false;
  }

  status_.configured_ = true;
  return true;
}

/**
 *  constantly monitors the status of sensor
 *  retry connecting if connection breaks
 */
void XsensMTInterface::statusMonitorSpinner() {
  bool measurementStarted_ = false;
  bool retry_ = false;

  chrono::steady_clock::time_point t_start;
  chrono::steady_clock::time_point t_end;
  chrono::steady_clock::time_point lastPkgTimeStamp_;
  int duration;

  while (true) {
    t_start = chrono::steady_clock::now();
    {
      /* check for termination condition */
      if (statusMonitorSpinnerStop_) return;
    }

    /* retrieve last pkg timestamp */
    if (!callbackHandler_.pkgTimeStampBuffer_.isEmpty()) {
      lastPkgTimeStamp_ = callbackHandler_.pkgTimeStampBuffer_.dequeue();
      if (!measurementStarted_) measurementStarted_ = true;
    }

    /* check for connection status */
    if (measurementStarted_ &&
        std::chrono::duration_cast<std::chrono::milliseconds>(t_start - lastPkgTimeStamp_).count() > configParam_.maxInterPacketTime_) {
      {
        std::unique_lock<std::mutex> lock(mutexStatus_);
        status_.connected_ = false;
        status_.configured_ = false;
        status_.isRunning_ = false;
        status_.errCode_ = 21;  // connection is lost
      }

      xsDevice_ = 0;
      measurementStarted_ = false;
      retry_ = true;
    }

    /* try to reconnect to sensor */
    if (retry_) {
      xsController_->close();
      if (!connect()) {
        usleep(configParam_.retryInterval_ * 1000);
        continue;
      }

      if (!device_config()) {
        usleep(configParam_.retryInterval_ * 1000);
        continue;
      }

      /* put device into measurement mode */
      if (!xsDevice_->gotoMeasurement()) {
        std::unique_lock<std::mutex> lock(mutexStatus_);
        status_.errCode_ = 17;  // cannot switch to measurement mode
        usleep(configParam_.retryInterval_ * 1000);
        continue;
      }

      {
        std::unique_lock<std::mutex> lock(mutexStatus_);
        status_.isRunning_ = true;
        status_.errCode_ = 0;
      }

      /* successfully reconnected */
      retry_ = false;
      xsDevice_->addCallbackHandler(&callbackHandler_);
      continue;
    }

    t_end = chrono::steady_clock::now();
    duration = chrono::duration_cast<chrono::microseconds>(t_end - t_start).count();
    if (duration < 10000) usleep(10000 - duration);
  }
}

/**
 *  return configuration parameters
 */
ImuConfig XsensMTInterface::getConfigParam() { return configParam_; }

std::string XsensMTInterface::errToString(int err) {
  std::string errStr;

  switch (err) {
    case 0:
      errStr = "";
      break;
    case 10:
      errStr = "cannot find sensor port";
      break;
    case 11:
      errStr = "cannot open sensor port";
      break;
    case 12:
      errStr = "cannot create xsDevice";
      break;
    case 13:
      errStr = "device not created";
      break;
    case 14:
      errStr = "cannot put device into configure mode";
      break;
    case 15:
      errStr = "unknown or unsupported device";
      break;
    case 16:
      errStr = "cannot configure device";
      break;
    case 17:
      errStr = "cannot put device into measurement mode";
      break;
    case 18:
      errStr = "cannot set baudrate";
      break;
    case 19:
      errStr = "invalid serial key";
      break;
    case 21:
      errStr = "connection to device lost";
      break;

    default:
      errStr = "unknown errCode";
      break;
  }

  return errStr;
}

void XsensMTInterface::configureSyncSettings(XsSyncSettingArray& settingsArray) {
  XsSyncSetting settings;

  // for complete reference of this see:
  // MT_mangaer user manual from xsens or API docs

  if (configParam_.receiveSync_) {
    if (configParam_.sendSync_) {
      // Can't both send and receive trigger
      settings.m_polarity = XSP_None;
      MELO_ERROR("XsensMTInterface: Syncin and Syncout cannot both be true. Ignoring Triggers.");
    } else {
      // Input pin for external trigger
      settings.m_line = XSL_In1;
      // Function to wait for external trigger
      settings.m_function = XSF_SendLatest;
      // Depends on Arduino code
      settings.m_polarity = XSP_RisingEdge;
    }
  } else if (configParam_.sendSync_) {
    // the only out pin that we have
    settings.m_line = XSL_Bi1Out;
    // Function for outputing signal
    settings.m_function = XSF_IntervalTransitionMeasurement;
    // could also be falling edge
    settings.m_polarity = XSP_RisingEdge;
    // puls width in us
    settings.m_pulseWidth = configParam_.pulseWidth_;
    // skip factor, 0 means don't skip anything, 1 means skip 1 pulse etc.
    settings.m_skipFactor = configParam_.skipFactor_;

  } else {
    settings.m_polarity = XSP_None;  // don't send or react to trigger changes
  }

  settingsArray.push_back(settings);
}

}  // namespace xsensmt
