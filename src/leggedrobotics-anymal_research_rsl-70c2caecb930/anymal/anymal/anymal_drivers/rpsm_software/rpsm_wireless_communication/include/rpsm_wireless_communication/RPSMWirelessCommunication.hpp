/*!
 * @file    RPSMWirelessCommunication.hpp
 * @author  Russell Buchanan
 * @date    Dec 1, 2016
 */
#pragma once

// readXbee.c
#include "rpsm_wireless_communication/readXbee.h"

// ros
#include <ros/ros.h>
#include <rpsm_wireless_communication/WirelessCommunicationStatus.h>
#include <rpsm_wireless_communication/RPSMStatus.h>
#include <rpsm_wireless_communication/RPSMService.h>

// nodewrap
#include "any_node/Node.hpp"

//For creating USB device handle
#include <termios.h> 
#include <vector>

namespace rpsm_wireless_communication {

class RPSMWirelessCommunication : public any_node::Node {

public:
  RPSMWirelessCommunication() = delete;
  RPSMWirelessCommunication(NodeHandlePtr nh);
  virtual ~RPSMWirelessCommunication();

  bool init() override;
  void cleanup() override;
  virtual bool update(const any_worker::WorkerEvent& event);


protected:
  // USB port handles
  char *port = "/dev/ttyUSB0";
  struct termios tty;
  int fd;

  int recBytes, sendBytes;
  uint8_t *sendBuffer;

  // Counters to check for new message
  uint64_t msgCount = 0;
  uint64_t oldCount = 0;

  // Counters for error checking
  uint64_t validMsgCount = 0;
  uint64_t corruptMsgCount = 0;

  uint8_t buttonState[2];


  void publish(uint8_t *message);
  void initializePublishers();

  bool receiveWirelessData();
  bool sendWirelessData();

  bool sendWorker(const any_worker::WorkerEvent& event);

  bool serviceCallback(rpsm_wireless_communication::RPSMService::Request  &req,
   rpsm_wireless_communication::RPSMService::Response &res);


private:
  ros::Publisher wirelessCommunicaitonPublisher_;
  ros::Publisher wirelessCommunicaitonStatusPublisher_;
  ros::ServiceServer wirelessCommunicationService_;

};

} /* namespace m545_wireless_communication */