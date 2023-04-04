/** \file: XsensMTImuDriver.h
    \brief: defines a ROS-independent driver class for Xsens MTi series IMU
 */

#ifndef XSENS_MT_INTERFACE_H
#define XSENS_MT_INTERFACE_H

#include <memory>
#include <string>
#include <utility>
#include <cstdint>
#include <chrono>
#include <thread>
#include <iostream>
#include <atomic>
#include <mutex>

#include <libxsensmt/xsensdeviceapi.h>

#include <xsensmt_interface/SafeQueue.h>

#include <xsensmt_interface/ImuMeasurement.h>
#include <xsensmt_interface/ImuConfig.h>
#include <xsensmt_interface/ImuStatus.h>

namespace xsensmt {

class XsensMTInterface {

public:
  /* Derive Xsens data handler callback */
  class CallbackHandler : public XsCallback {

  public:
    /* Define thread-safe queue to hold sensor data */
    typedef std::pair<XsDataPacket, std::chrono::time_point<std::chrono::steady_clock>> QueueContent;
    typedef std::chrono::time_point<std::chrono::steady_clock> TimeStamp;

    /* Constructor & Destructor */
    CallbackHandler(size_t queueCapacity = 3)
    : dataBuffer_(queueCapacity),
      pkgTimeStampBuffer_(1),
      correctTiming_(true),
      baudRate_(921600) {}
    virtual ~CallbackHandler() {}

    CallbackHandler(const CallbackHandler& other) = delete;
    CallbackHandler& operator = (const CallbackHandler& other) = delete;
    CallbackHandler(CallbackHandler&& other) = delete;
    CallbackHandler& operator = (CallbackHandler&& other) = delete;

    /* queue (FIFO) to hold sensor data */
    SafeQueue<QueueContent> dataBuffer_;
    SafeQueue<TimeStamp> pkgTimeStampBuffer_;

    /* whether we should consider delay over serial port */
    std::atomic<bool> correctTiming_;

    /* baud rate */
    std::atomic<int> baudRate_;

  protected:
    /* Override virtual method callback*/
    virtual void onMessageReceivedFromDevice(XsDevice*, const XsMessage* msg) {

      XsDataPacket packet(msg);
      if( packet.containsCalibratedAcceleration() &&
          packet.containsCalibratedGyroscopeData() ){
        timespec start;
        clock_gettime(CLOCK_REALTIME, &start);
        //                                 sec  = start.tv_sec;
        //                                 nsec = start.tv_nsec;
        auto d = std::chrono::seconds{start.tv_sec} + std::chrono::nanoseconds{start.tv_nsec};

        //                            std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        std::chrono::steady_clock::time_point now(d);

        // correct delay over serial transmission
        if(correctTiming_){
          std::chrono::microseconds transmission_delay(msg->getTotalMessageSize()*10*1000000/baudRate_);
          now -= transmission_delay;
        }
        pkgTimeStampBuffer_.enqueue(now);
        dataBuffer_.enqueue(std::make_pair(packet, now));
      }
    }
  };

  /* Constructor & Destructor */
  XsensMTInterface()
  : statusMonitorSpinnerStop_(false) {}
  ~XsensMTInterface() {}

  // Initialize internal parameters of the XsensMTInterface object
  bool init();

  // Connect to and configure the sensor, start measurement
  bool configure(const ImuConfig& configParam);

  // When the XsensMTInterface object is no longer needed, clean up resources
  bool cleanup();

  // Update measurement results
  bool update();

  // Retrive measurement results
  bool getMeasurement(ImuMeasurement& data);  // pass measurement by reference

  // get configuration parameters
  ImuConfig getConfigParam();

  // Retrive XsensMTInterface object and sensor status
  ImuStatus getStatus();

  // translate status errCode_ into a readable string
  std::string errToString(int err);

protected:
  bool connect();
  bool device_config();
  void statusMonitorSpinner();
  void configureSyncSettings(XsSyncSettingArray &settingsArray);

  ImuConfig configParam_;
  ImuStatus status_;
  XsControl* xsController_ = nullptr;
  XsDevice* xsDevice_ = nullptr;
  CallbackHandler callbackHandler_;
  std::thread* statusMonitorSpinnerThread_ = nullptr;

  std::atomic<bool> statusMonitorSpinnerStop_;

  mutable std::mutex mutexStatus_;
  //mutable std::mutex mutexStop_;
};

}


#endif

