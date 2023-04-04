#include "rpsm_master/RpsmMaster.hpp"



namespace rpsm_master {

RpsmMaster::RpsmMaster(any_node::Node::NodeHandlePtr nh)
: any_node::Node(nh)
{}


RpsmMaster::~RpsmMaster()
{}


bool RpsmMaster::init() {
  getNodeHandle().param<std::string>("port", serialPort_, "/dev/ttyS1");
  getNodeHandle().param<int>("baud", serialBaudrate_, 115200);
  createSerialCommunication();
  initRos();
  ros::Duration(1.0).sleep();

  any_worker::WorkerOptions heartbeatOptions;
  heartbeatOptions.callback_ = std::bind(&RpsmMaster::heartbeatWorker, this, std::placeholders::_1);
  heartbeatOptions.defaultPriority_ = 50; // this has middle priority
  heartbeatOptions.name_ = "heartbeatWorker";
  heartbeatOptions.timeStep_ = 0.1;
  this->addWorker(heartbeatOptions);
  ros::Duration(1.0).sleep();

  while (!serial_->isOpen()) {
    ros::Duration(0.1).sleep();
  }
  senderThread_ = boost::thread(boost::bind(&RpsmMaster::sendWorker, this));
  sched_param sched;
  sched.sched_priority = 70;
  if (pthread_setschedparam(senderThread_.native_handle(), SCHED_FIFO, &sched) != 0) {
    ROS_WARN("Failed to set thread priority for senderWorker: %s", strerror(errno));
  }
  ros::Duration(1.0).sleep();

  any_worker::WorkerOptions requestOptions;
  requestOptions.callback_ = std::bind(&RpsmMaster::requestWorker, this, std::placeholders::_1);
  requestOptions.defaultPriority_ = 9;
  requestOptions.name_ = "requestWorker";
  requestOptions.timeStep_ = 1.0;
  this->addWorker(requestOptions);

  return true;
}


void RpsmMaster::cleanup() {
  ROS_DEBUG("RpsmMaster::cleanup()");
  destroySerialCommunication();

  actuationBoardStatesPublisher_.shutdown();
  batteryStatePublisher_.shutdown();
  errorEventPublisher_.shutdown();
  heartbeatPublisher_.shutdown();
  humidityStatePublisher_.shutdown();
  mainBoardStatePublisher_.shutdown();
  shutdownEventPublisher_.shutdown();

  shutdownCommandService_.shutdown();
  enableActuationBoardPowerService_.shutdown();
}


bool RpsmMaster::update(const any_worker::WorkerEvent& event) {
  try {
    readSerial();
  } catch (const std::exception& ex) {
    ROS_ERROR_STREAM("RpsmMaster update caught an exception: " << ex.what());
  } catch (...) {
    ROS_ERROR_STREAM("RpsmMaster update caught an exception.");
  }
  return true;
}


void RpsmMaster::initRos() {
  actuationBoardStatesPublisher_ = getNodeHandle().advertise<rpsm_msgs::ActuationBoardStates>("actuationboard_states", 10);
  batteryStatePublisher_ = getNodeHandle().advertise<sensor_msgs::BatteryState>("battery_state", 10);
  errorEventPublisher_ = getNodeHandle().advertise<rpsm_msgs::RpsmError>("error", 10);
  heartbeatPublisher_ = getNodeHandle().advertise<rpsm_msgs::Heartbeat>("heartbeat", 10);
  humidityStatePublisher_ = getNodeHandle().advertise<rpsm_msgs::HumidityState>("humidity_state", 10);
  mainBoardStatePublisher_ = getNodeHandle().advertise<rpsm_msgs::MainBoardState>("mainboard_state", 10);
  shutdownEventPublisher_ = getNodeHandle().advertise<std_msgs::Empty>("shutdown", 10);

  shutdownCommandService_ = getNodeHandle().advertiseService("shutdown_command", &RpsmMaster::shutdownCommandService, this);
  enableActuationBoardPowerService_  = getNodeHandle().advertiseService("enable_actuationboard_power", &RpsmMaster::enableActuationBoardPowerService, this);

  mainBoardState_.converter_states.resize(5);
  mainBoardState_.converter_states[0].name = "ADS1015 5V";
  mainBoardState_.converter_states[1].name = "ADS1015 12V";
  mainBoardState_.converter_states[2].name = "ADS1015 15V";
  mainBoardState_.converter_states[3].name = "ADS1015 extern";
  mainBoardState_.converter_states[4].name = "LTS4151";
  actuationBoardStates_.states.resize(2);
  batteryState_.cell_voltage.resize(12);

  lastHeartbeatTime_ = ros::Time::now();
  double heartbeatTimeout;
  getNodeHandle().param<double>("heartbeat_timeout", heartbeatTimeout, 1.0);
  heartbeatTimeout_ = ros::Duration(heartbeatTimeout);

}


void RpsmMaster::sendWorker() {
  ROS_DEBUG("RpsmMaster::sendWorker()");
  while (isSendingOverSerial_) {
    if (serial_ != nullptr) {
      boost::unique_lock<boost::shared_mutex> lock(mutexSerialQueue_);
      cndSerialQueue_.wait(lock, [this]() { return (!serialQueue_.empty() || !isSendingOverSerial_); });
      ROS_DEBUG("Send over serial");
      if (!isSendingOverSerial_) {
        return;
      }
      while (!serialQueue_.empty()) {
        auto& package = serialQueue_.front();
        try {
          serial_->write(package.second, package.first);
        } catch (const serial::PortNotOpenedException& e) {
          ROS_ERROR_STREAM("RpsmMaster::sendWorker(): PortNotOpenedException: " << e.what());
        } catch (const serial::SerialException& e) {
          ROS_ERROR_STREAM("RpsmMaster::sendWorker(): SerialException: " << e.what());
        } catch (const serial::IOException& e) {
          ROS_ERROR_STREAM("RpsmMaster::sendWorker(): IOException: " << e.what());
        }
        serialQueue_.pop();
        // Do not blast at full speed
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
  }
  return;
}


bool RpsmMaster::heartbeatWorker(const any_worker::WorkerEvent& event) {
  sendHeartbeat();
  // Check if received Heartbeat
  if (ros::Time::now() - lastHeartbeatTime_ > heartbeatTimeout_) {
    ROS_WARN_STREAM_ONCE("RpsmMaster::heartBeatWorker(): Have not received heartbeat in last "
                      << ros::Time::now() - lastHeartbeatTime_ << " seconds! Reopening serial.");

    isReadingSerial_ = false;
    isSendingOverSerial_ = false;

    cndSerialQueue_.notify_all();
    if (senderThread_.joinable()) {
      senderThread_.join();
    }
    boost::unique_lock<boost::shared_mutex> lock(mutexSerialQueue_);
    if (serial_ != nullptr) {
      serial_->close();
      serial_->open();
      serial_->flushInput();
      serial_->flushOutput();
    }
    isSendingOverSerial_ = true;
    isReadingSerial_ = true;
    ros::Time serialStart = ros::Time::now();
    while (!serial_->isOpen() && ros::Time::now() - serialStart < ros::Duration(2.0)) {
      ros::Duration(0.1).sleep();
    }
    senderThread_ = boost::thread(boost::bind(&RpsmMaster::sendWorker, this));
    sched_param sched;
    sched.sched_priority = 70;
    if (pthread_setschedparam(senderThread_.native_handle(), SCHED_FIFO, &sched) != 0) {
      ROS_WARN_ONCE("Failed to set thread priority for senderWorker: %s", strerror(errno));
    }
  }
  return true;
}


bool RpsmMaster::requestWorker(const any_worker::WorkerEvent& event) {
  sendRequests();
  return true;
}


bool RpsmMaster::shutdownCommandService(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res) {
  ROS_INFO("RpsmMaster::shutdownCommandService()");
  sendShutdownCommand();
  return true;
}


bool RpsmMaster::enableActuationBoardPowerService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response & res) {
  ROS_INFO("RpsmMaster::enableActuationBoardPowerService()");
  sendActuationBoardCommand(255u, req.data);
  res.success = true;
  return true;
}


void RpsmMaster::sendHeartbeat() {
  uint8_t sysid = mavlinkSystemId_;
  uint8_t compid = mavlinkComponentId_;
  mavlink_heartbeat_t heartbeat_to_send;
  heartbeat_to_send.type = ANYMAL_MAVILNK_LOCOMOTION_PC;

  // Initialize the required buffers
  mavlink_message_t msg_to_send;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_heartbeat_pack(sysid, compid, &msg_to_send, heartbeat_to_send.type,
                             heartbeat_to_send.master_status,
                             heartbeat_to_send.subsystem_status,
                             heartbeat_to_send.errors,
                             heartbeat_to_send.bms_working,
                             heartbeat_to_send.bms_voltage_mv,
                             heartbeat_to_send.bms_current_ma,
                             heartbeat_to_send.bms_stateofcharge);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg_to_send);

  this->stageSerialPackage(buf, len);
}


void RpsmMaster::sendRequests() {
  // Initialize command structure and add relevant info to the structure
  mavlink_get_command_t command_to_send;
  command_to_send.slave_id = 0; // Mainboard

  for (int i = 1; i < 5; i++) {
    command_to_send.get_type = i; // Types: master_circuit = 1
                                  //        slave_circuit = 2
                                  //        bms_status = 3
                                  //        humidity = 4

    // Pack the message for MAVLINK
    mavlink_message_t msg_to_send;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_get_command_pack(mavlinkSystemId_, mavlinkComponentId_, &msg_to_send, command_to_send.slave_id, command_to_send.get_type);

    // Add the bits to buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg_to_send);

    // Send buffer via serial
    this->stageSerialPackage(buf, len);
  }
}


void RpsmMaster::sendShutdownCommand(){
  // send soft shutdown command
  mavlink_master_mavlink_notification_t notification;
  notification.code = 1;
  notification.level = 5;

  mavlink_message_t msg_to_send;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_master_mavlink_notification_pack(mavlinkSystemId_, mavlinkComponentId_,
                                               &msg_to_send, notification.code,
                                               notification.level,
                                               nullptr,
                                               nullptr);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg_to_send);
  this->stageSerialPackage(buf, len);
}


void RpsmMaster::sendActuationBoardCommand(uint8_t slave_id, bool enable){
  ROS_DEBUG_STREAM("RpsmMaster::sendActuationBoardCommand(can_node_id: "
                       << slave_id << ", enable: " << (enable ? "true" : "false") << ")");
  // Initialize command structure and add relevant info to the structure
  mavlink_slave_enable_cmd_t config_to_send;
  config_to_send.slave_id = slave_id;
  config_to_send.set_cmd = enable ? 1u : 0u;

  // Pack the message for MAVLINK
  mavlink_message_t msg_to_send;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_slave_enable_cmd_pack(mavlinkSystemId_, mavlinkComponentId_, &msg_to_send, config_to_send.slave_id, config_to_send.set_cmd);

  // Add the bits to buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg_to_send);

  // Send buffer via serial
  this->stageSerialPackage(buf, len);
}


bool RpsmMaster::isSerialCommunicationOpen() {
  if (serial_ != nullptr) {
    return serial_->isOpen();
  }
  return false;

}

void RpsmMaster::openSerialCommunication() {
  if (serial_ != nullptr) {
    try {
      if (!serial_->isOpen()) {
        serial_->open();
      }
    } catch (const serial::IOException& e) {
      ROS_ERROR_STREAM("RpsmMaster::openSerialCommunication(): IOException: " << e.what());
    } catch (const serial::SerialException& e) {
      ROS_ERROR_STREAM("RpsmMaster::openSerialCommunication(): SerialException: " << e.what());
    }
  }
}


void RpsmMaster::createSerialCommunication() {
  // Opening the port and clearing input, output buffers
  ROS_INFO_STREAM("RpsmMaster::createSerialCommunication(): Opening Port " << serialPort_ << " (baud: "<< serialBaudrate_ <<")");
  try {
    serial_ = new serial::Serial(serialPort_, serialBaudrate_, serial::Timeout::simpleTimeout(1000*serialTimeout_)); // 1 Second timeout
  }
  catch (...) {
    ROS_FATAL("RpsmMaster::createSerialCommunication():  Could not open serial port!");
    delete serial_;
    ;
  }
  openSerialCommunication();
  if (serial_ != nullptr) {
    serial_->flushInput();
    serial_->flushOutput();
  }
  isSendingOverSerial_ = true;
  isReadingSerial_ = true;
}


void RpsmMaster::destroySerialCommunication() {
  isReadingSerial_ = false;
  isSendingOverSerial_ = false;
  ROS_DEBUG("RpsmMaster::destroySerialCommunication(): Notify");
  cndSerialQueue_.notify_all();
  if (senderThread_.joinable()) {
    senderThread_.join();
  }
  boost::unique_lock<boost::shared_mutex> lock(mutexSerialQueue_);

  if (serial_ != nullptr) {
    ROS_DEBUG("RpsmMaster::destroySerialCommunication(): Closing serial");
    serial_->close();
  }
  delete serial_;
}


// Looks at a given message and parses it for the MAVLINK ID
void RpsmMaster::readSerial(){
  mavlink_message_t mavlink_msg;
  uint8_t buffer[1];
  try {
    if (serial_ != nullptr) {
      while (isReadingSerial_ && serial_->available() > 0) {
        serial_->read(buffer, 1);
        if (mavlink_parse_char(MAVLINK_COMM_0, buffer[0], &mavlink_msg, &mavlinkStatus_)) {
          handleMavlinkMessage(&mavlink_msg);
        }
      }
    }
  } catch (const serial::IOException& e) {
    ROS_ERROR_STREAM("RpsmMaster::readSerial(): IOException: " << e.what());
  } catch (const serial::PortNotOpenedException& e) {
    ROS_ERROR_STREAM("RpsmMaster::readSerial(): PortNotOpenedException: " << e.what());
  }
}


// Writes any given buffer to the serial bus
void RpsmMaster::stageSerialPackage(uint8_t *buffer, size_t len){
  SerialPackage package;
  package.first = len;
  for (size_t i=0; i<len; i++) {
    package.second[i] = buffer[i];
  }
  {
  boost::unique_lock<boost::shared_mutex>(mutexSerialQueue_);
  serialQueue_.push(package);
  }
  cndSerialQueue_.notify_all();
}


// Looks at the MAVLINK message ID and sends the message to the appropriate function
void RpsmMaster::handleMavlinkMessage(mavlink_message_t* mavlink_msg){
  switch(mavlink_msg->msgid){
  case MAVLINK_MSG_ID_HEARTBEAT:
  {
      ROS_DEBUG("MAVLINK_MSG_ID_HEARTBEAT received!");
      this->handleMavlinkHeartbeat(mavlink_msg);
      break;
  } case MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS:
  {
      ROS_DEBUG("MAVLINK_MSG_ID_MASTER_CIRCUIT_STATUS received!");
      this->handleMavlinkMasterCircuitStatus(mavlink_msg);
      break;
  } case MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS:
  {
      ROS_DEBUG("MAVLINK_MSG_ID_SLAVE_CIRCUIT_STATUS received!");
      this->handleMavlinkSlaveCircuitStatus(mavlink_msg);
      break;
  } case MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS:
  {
      ROS_DEBUG("MAVLINK_MSG_ID_BMS_CIRCUIT_STATUS received!");
      this->handleMavlinkBmsCircuitStatus(mavlink_msg);
      break;
  } case MAVLINK_MSG_ID_HUMIDITY_STATUS:
  {
      ROS_DEBUG("MAVLINK_MSG_ID_HUMIDITY_STATUS received!");
      this->handleMavlinkHumidityStatus(mavlink_msg);
      break;
  } case MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION:
  {
      ROS_DEBUG("MAVLINK_MSG_ID_MASTER_MAVLINK_NOTIFICATION received!");
      this->handleMavlinkMasterMavlinkNotification(mavlink_msg);
      break;
  } case MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL:
  {
      ROS_DEBUG("MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL received!");
      this->handleMavlinkFileTransferProtocol(mavlink_msg);
      break;
  } case MAVLINK_MSG_ID_ACK:
  {
      ROS_DEBUG("MAVLINK_MSG_ID_ACK received!");
      break;
  } default:
      ROS_DEBUG("Problem handling message: MAVLINK_MSG_ID is not recognized!");
      break;
  }
}


// Decodes heartbeat and uploads information to the topic
void RpsmMaster::handleMavlinkHeartbeat(mavlink_message_t *mavlink_msg){
  mavlink_heartbeat_t dec_msg;
  mavlink_msg_heartbeat_decode(mavlink_msg, &dec_msg);

  // TODO ErrorEvent

  lastHeartbeatTime_ = ros::Time::now();

  heartbeat_.master_health = dec_msg.master_status;
  heartbeat_.stamp = ros::Time::now();

  if (heartbeatPublisher_.getNumSubscribers() > 0u) {
    heartbeatPublisher_.publish(heartbeat_);
  }
}


// Decodes master circuit status and uploads information to the topic
void RpsmMaster::handleMavlinkMasterCircuitStatus(mavlink_message_t *mavlink_msg) {
  mavlink_master_circuit_status_t dec_msg;
  mavlink_msg_master_circuit_status_decode(mavlink_msg, &dec_msg);

  mainBoardState_.converter_states[0].voltage = dec_msg.voltage_mv_5V;
  mainBoardState_.converter_states[0].current = dec_msg.current_ma_5V;
  mainBoardState_.converter_states[0].temperature = dec_msg.temperature_5V;
  mainBoardState_.converter_states[0].operational = dec_msg.operational_5V;
  mainBoardState_.converter_states[1].voltage = dec_msg.voltage_mv_12V;
  mainBoardState_.converter_states[1].current = dec_msg.current_ma_12V;
  mainBoardState_.converter_states[1].temperature = dec_msg.temperature_12V;
  mainBoardState_.converter_states[1].operational = dec_msg.operational_12V;
  mainBoardState_.converter_states[2].voltage = dec_msg.voltage_mv_15V;
  mainBoardState_.converter_states[2].current = dec_msg.current_ma_15V;
  mainBoardState_.converter_states[2].temperature = dec_msg.temperature_15V;
  mainBoardState_.converter_states[2].operational = dec_msg.operational_15V;

  if (mainBoardStatePublisher_.getNumSubscribers() > 0u) {
    mainBoardStatePublisher_.publish(mainBoardState_);
  }
}


void RpsmMaster::handleMavlinkSlaveCircuitStatus(mavlink_message_t *mavlink_msg) {
  mavlink_slave_circuit_status_t dec_msg;
  mavlink_msg_slave_circuit_status_decode(mavlink_msg, &dec_msg);

  rpsm_msgs::ActuationBoardState board;
  actuationBoardStates_.states.clear();

  for (unsigned int i = 0; i < sizeof(dec_msg.slave_id); ++i) {
    if (dec_msg.slave_id[i] == 0u || dec_msg.temperature_48V[i] == 0u) {
      continue;
    }
    board.can_node_id = dec_msg.slave_id[i];
    board.state.voltage = dec_msg.voltage_mv_48V[i];
    board.state.current = dec_msg.current_ma_48V[i];
    board.state.temperature = dec_msg.temperature_48V[i];
    board.state.operational = true;
    board.state.enabled = (dec_msg.estop[i] == 0u);
    board.estop_engaged = (dec_msg.estop[i] != 0u);
    actuationBoardStates_.states.push_back(board);
  }

  if (actuationBoardStatesPublisher_.getNumSubscribers() > 0u) {
    actuationBoardStatesPublisher_.publish(actuationBoardStates_);
  }
}


void RpsmMaster::handleMavlinkBmsCircuitStatus(mavlink_message_t *mavlink_msg) {
  mavlink_bms_circuit_status_t dec_msg;
  mavlink_msg_bms_circuit_status_decode(mavlink_msg, &dec_msg);

  batteryState_.header.stamp = ros::Time::now();
  batteryState_.voltage = (float) (dec_msg.voltage / 1000.0);
  batteryState_.current = (float) (dec_msg.current / 1000.0);
  batteryState_.design_capacity = 17.25;
  batteryState_.percentage = (float) (dec_msg.stateofcharge / 100.0);
  batteryState_.cell_voltage[0] = (float) (dec_msg.cellvoltage1 / 1000.0);
  batteryState_.cell_voltage[1] = (float) (dec_msg.cellvoltage2 / 1000.0);
  batteryState_.cell_voltage[2] = (float) (dec_msg.cellvoltage3 / 1000.0);
  batteryState_.cell_voltage[3] = (float) (dec_msg.cellvoltage4 / 1000.0);
  batteryState_.cell_voltage[4] = (float) (dec_msg.cellvoltage5 / 1000.0);
  batteryState_.cell_voltage[5] = (float) (dec_msg.cellvoltage6 / 1000.0);
  batteryState_.cell_voltage[6] = (float) (dec_msg.cellvoltage7 / 1000.0);
  batteryState_.cell_voltage[7] = (float) (dec_msg.cellvoltage8 / 1000.0);
  batteryState_.cell_voltage[8] = (float) (dec_msg.cellvoltage9 / 1000.0);
  batteryState_.cell_voltage[9] = (float) (dec_msg.cellvoltage10 / 1000.0);
  batteryState_.cell_voltage[10] = (float) (dec_msg.cellvoltage11 / 1000.0);
  batteryState_.cell_voltage[11] = (float) (dec_msg.cellvoltage12 / 1000.0);
  batteryState_.serial_number = std::to_string(dec_msg.serialnumber);
  if (batteryStatePublisher_.getNumSubscribers() > 0u) {
    batteryStatePublisher_.publish(batteryState_);
  }

  // Battery Status (commented part copied and adapted but not tested from old BatteryState)
//  if (dec_msg.batterystatus & (1 << 6)) {  // charge fet test is discharging?
//    // is discharging or at rest
//     if (dec_msg.batterystatus & (1 << 5)) { // fully charged?
//       batteryState_.power_supply_status = batteryState_.POWER_SUPPLY_STATUS_FULL;
//     }
//     else {
//       batteryState_.power_supply_status = batteryState_.POWER_SUPPLY_STATUS_DISCHARGING;
//     }
//  }
//  else {
//    batteryState_.power_supply_status = batteryState_.POWER_SUPPLY_STATUS_CHARGING;
//  }
  if (batteryState_.voltage > 50.0) {
    batteryState_.power_supply_status = batteryState_.POWER_SUPPLY_STATUS_FULL;
  } else if (batteryState_.current  > 0.0) {
    batteryState_.power_supply_status = batteryState_.POWER_SUPPLY_STATUS_CHARGING;
  } else {
    batteryState_.power_supply_status = batteryState_.POWER_SUPPLY_STATUS_DISCHARGING;
  }

  // Battery Health
  if (dec_msg.safetystatus == 0u) {
    batteryState_.power_supply_health = batteryState_.POWER_SUPPLY_HEALTH_GOOD;
  } else if (dec_msg.safetystatus & (0b11 << 8)) {
    batteryState_.power_supply_health = batteryState_.POWER_SUPPLY_HEALTH_OVERHEAT;
  } else if (dec_msg.safetystatus & (0b1 << 1)) {
    batteryState_.power_supply_health = batteryState_.POWER_SUPPLY_HEALTH_OVERVOLTAGE;
  } else {
    batteryState_.power_supply_health = batteryState_.POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
  }
}


void RpsmMaster::handleMavlinkHumidityStatus(mavlink_message_t *mavlink_msg) {
  mavlink_humidity_status_t dec_msg;
  mavlink_msg_humidity_status_decode(mavlink_msg, &dec_msg);

  humidityState_.ambient_temperature = dec_msg.ambient_temperature;
  humidityState_.ambient_humidity = dec_msg.ambient_humidity;
  humidityState_.operational = dec_msg.humidity_working;

  if (humidityStatePublisher_.getNumSubscribers() > 0u) {
    humidityStatePublisher_.publish(humidityState_);
  }
}


void RpsmMaster::handleMavlinkMasterMavlinkNotification(mavlink_message_t *mavlink_msg) {
  mavlink_master_mavlink_notification_t dec_msg;
  mavlink_msg_master_mavlink_notification_decode(mavlink_msg, &dec_msg);
  ROS_DEBUG_STREAM("RpsmMaster:: << [Code: " << dec_msg.code
                  << ", Level: " << (int) dec_msg.level
                  << ", Name: " << dec_msg.name
                  << ": " << dec_msg.description << "]");
  switch (dec_msg.level) {
    case 5:
    {
      shutdownEventPublisher_.publish(std_msgs::Empty());
      break;
    }
  }
}

} // namespace rpsm_master
