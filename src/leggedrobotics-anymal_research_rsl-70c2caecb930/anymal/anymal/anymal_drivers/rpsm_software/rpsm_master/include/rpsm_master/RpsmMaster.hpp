#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <serial/serial.h>
#include <signal.h>
#include <sstream>
#include <string>
#include <unistd.h>

#include <boost/thread.hpp>

// MAVLINK Libraries
#include "rpsm_master/mavlink/v2.0/rpsm_firmware_master/mavlink.h"

// ROS Libraries
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>

// any_common
#include <any_node/any_node.hpp>

// Custom ROS message and service calls
#include <rpsm_msgs/ActuationBoardStates.h>
#include <rpsm_msgs/Heartbeat.h>
#include <rpsm_msgs/HumidityState.h>
#include <rpsm_msgs/MainBoardState.h>
#include <rpsm_msgs/RpsmError.h>


namespace rpsm_master {

enum class MasterMavlinkNotification {
  debug = 0,
  info,
  warn,
  error,
  fatal,
  shutdown,
  estop
};

enum class Px4gpioCommand {
  GPIO_LOW = 0,
  GPIO_HIGH = 1,
  GPIO_NA = 2
};

enum class Px4gpio {
  MASTERPWR = 0,
  EX_DEV = 1,
  CANPWR = 2,
  V5 = 3,
  V12 = 4,
  V15 = 5,
  MAINLED = 6
};

class RpsmMaster : public any_node::Node
{
 public:
  RpsmMaster() = delete;
  RpsmMaster(any_node::Node::NodeHandlePtr nh);
  virtual ~RpsmMaster();

  bool init() override;
  void cleanup() override;
  virtual bool update(const any_worker::WorkerEvent& event);

  void initRos();
  void sendWorker();
  bool heartbeatWorker(const any_worker::WorkerEvent& event);
  bool requestWorker(const any_worker::WorkerEvent& event);

  // Handles incoming service calls to send shutdown
  bool shutdownCommandService(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);
  // service that enables AB Power
  bool enableActuationBoardPowerService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response & res);

  // Packs MAVLINK struct and adds onto buffer
  void sendHeartbeat();
  void sendRequests();
  void sendShutdownCommand();
  void sendActuationBoardCommand(uint8_t slave_id, bool enable);

 protected:
  // Serial commands
  bool isSerialCommunicationOpen();
  void openSerialCommunication();
  void createSerialCommunication();
  void destroySerialCommunication();
  void readSerial();
  void stageSerialPackage(uint8_t *buffer, size_t len);

  // Looks at the ID of the message and invokes one of the handle functions
  void handleMavlinkMessage(mavlink_message_t *mavlink_msg);
  virtual void handleMavlinkHeartbeat(mavlink_message_t *mavlink_msg);
  virtual void handleMavlinkMasterCircuitStatus(mavlink_message_t *mavlink_msg);
  virtual void handleMavlinkSlaveCircuitStatus(mavlink_message_t *mavlink_msg);
  virtual void handleMavlinkBmsCircuitStatus(mavlink_message_t *mavlink_msg);
  virtual void handleMavlinkHumidityStatus(mavlink_message_t *mavlink_msg);
  virtual void handleMavlinkMasterMavlinkNotification(mavlink_message_t *mavlink_msg);
  virtual void handleMavlinkFileTransferProtocol(mavlink_message_t *mavlink_msg) {};

  // Serial class variables
  serial::Serial* serial_ = nullptr;
  std::string serialPort_;
  int serialBaudrate_;
  unsigned long serialTimeout_ = 1u;
  ros::Time lastHeartbeatTime_;
  ros::Duration heartbeatTimeout_;

  typedef std::pair<uint16_t, uint8_t[MAVLINK_MAX_PACKET_LEN]> SerialPackage;
  std::queue<SerialPackage> serialQueue_;
  boost::shared_mutex mutexSerialQueue_;
  boost::condition_variable_any cndSerialQueue_;
  boost::atomic<bool> isSendingOverSerial_;
  boost::atomic<bool> isReadingSerial_;

  boost::thread senderThread_;

  const uint8_t mavlinkSystemId_ = ANYMAL_MAVILNK_LOCOMOTION_PC; // According to MAVLINK 2 is the LPC
  const uint8_t mavlinkComponentId_ = 0u;
  mavlink_status_t mavlinkStatus_;

  rpsm_msgs::ActuationBoardStates actuationBoardStates_;
  sensor_msgs::BatteryState batteryState_;
  rpsm_msgs::Heartbeat heartbeat_;
  rpsm_msgs::HumidityState humidityState_;
  rpsm_msgs::MainBoardState mainBoardState_;

  ros::Publisher actuationBoardStatesPublisher_;
  ros::Publisher batteryStatePublisher_;
  ros::Publisher errorEventPublisher_;
  ros::Publisher heartbeatPublisher_;
  ros::Publisher humidityStatePublisher_;
  ros::Publisher mainBoardStatePublisher_;
  ros::Publisher shutdownEventPublisher_;

  ros::ServiceServer shutdownCommandService_;
  ros::ServiceServer enableActuationBoardPowerService_;
};

} // namespace rpsm_master
