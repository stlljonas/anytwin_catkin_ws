#include "rpsm_master/RpsmMainboard.hpp"


namespace rpsm_master {

RpsmMainboard::RpsmMainboard(any_node::Node::NodeHandlePtr nh)
: RpsmMaster(nh)
, fileManager_(nh.get(), this)
{
}


RpsmMainboard::~RpsmMainboard()
{
}


bool RpsmMainboard::init() {
  RpsmMaster::init();

  // Publishers
  bmsStatePublisher_ = getNodeHandle().advertise<rpsm_msgs::BmsState>("bms_state", 10);
  powerButtonEventPublisher_ = getNodeHandle().advertise<std_msgs::Empty>("power_button", 10);

  actuationBoardCommandService_ = getNodeHandle().advertiseService("actuation_board_command", &RpsmMainboard::actuationBoardCommandService, this);
  enableCanPowerService_  = getNodeHandle().advertiseService("enable_can_power", &RpsmMainboard::enableCanPowerService, this);
  enablePowersupply5VService_  = getNodeHandle().advertiseService("enable_powersupply_5V", &RpsmMainboard::enablePowersupply5VService, this);
  enablePowersupply12VService_  = getNodeHandle().advertiseService("enable_powersupply_12V", &RpsmMainboard::enablePowersupply12VService, this);
  enablePowersupply15VService_  = getNodeHandle().advertiseService("enable_powersupply_15V", &RpsmMainboard::enablePowersupply15VService, this);

  getNodeHandle().param<std::string>("download_directory", currentDownloadToFilePath_, "/home/integration/");

  // Service Servers
  ftpListDirectoryServer_  = getNodeHandle().advertiseService("ftp_list_directory", &RpsmMainboard::listDirectoryService, this);

  // Action Servers
  ftpDownloadFileServer_.reset(
      new DownloadFileActionServer(
          getNodeHandle(), param<std::string>("servers/download_file/action", "ftp_download_file"),
          false));
  ftpDownloadFileServer_->registerGoalCallback(
      boost::bind(&RpsmMainboard::downloadFileActionGoalCB, this));
  ftpDownloadFileServer_->registerPreemptCallback(
      boost::bind(&RpsmMainboard::downloadFileActionPreemptCB, this));
  ftpDownloadFileServer_->start();

  // Initialize vector
  bmsState_.cellvoltage.resize(12);

  return  addWorker(ros::this_node::getName() + "::updateWorker", param<double>("time_step", 1.0),
      static_cast<bool (RpsmMainboard::*)(const any_worker::WorkerEvent& event)>(&RpsmMainboard::update),
      this, 59);
}


void RpsmMainboard::cleanup() {
  ROS_DEBUG("RpsmMainboard::cleanup()");
  RpsmMaster::cleanup();

  bmsStatePublisher_.shutdown();
  powerButtonEventPublisher_.shutdown();

  actuationBoardCommandService_.shutdown();
  enableCanPowerService_.shutdown();
  enablePowersupply5VService_.shutdown();
  enablePowersupply12VService_.shutdown();
  enablePowersupply15VService_.shutdown();

  ftpListDirectoryServer_.shutdown();
  ftpDownloadFileServer_->shutdown();

}


bool RpsmMainboard::actuationBoardCommandService(rpsm_msgs::ActuationBoardCommand::Request  &req, rpsm_msgs::ActuationBoardCommand::Response &res){
  // Sends a given set command via MAVLINK
  ROS_INFO("RpsmMainboard::ABcommandService(slave id:%d, enable: %s)", req.can_node_id, req.enable ? "true" : "false");
  sendActuationBoardCommand(req.can_node_id, req.enable);
  return true;
}


bool RpsmMainboard::enableCanPowerService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response & res) {
  sendPx4GpioSetCommand(Px4gpio::CANPWR, req.data);
  res.success = true;
  return true;
}


bool RpsmMainboard::enablePowersupply5VService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response & res) {
  sendPx4GpioSetCommand(Px4gpio::V5, req.data);
  res.success = true;
  return true;
}


bool RpsmMainboard::enablePowersupply12VService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response & res) {
  sendPx4GpioSetCommand(Px4gpio::V12, req.data);
  res.success = true;
  return true;
}


bool RpsmMainboard::enablePowersupply15VService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response & res) {
  sendPx4GpioSetCommand(Px4gpio::V15, req.data);
  res.success = true;
  return true;
}


bool RpsmMainboard::listDirectoryService(rpsm_msgs::ListDirectory::Request &req, rpsm_msgs::ListDirectory::Response &res) {
  if (fileManager_.listDirectory(req.absolute_file_path)) {
    std::unique_lock<std::mutex> lock(listDirectoryMutex_);
    cndListDirectory_.wait(lock);
    std::vector<std::string> names(directoryList_.size());
    std::vector<uint8_t> isDirectory(directoryList_.size());
    std::vector<uint32_t> sizes(directoryList_.size());
    int i = 0;
    for (std::string entry : directoryList_) {
      if (entry.empty()) {
        continue;
      }
      std::size_t found = entry.find('\t');
      names[i] = entry.substr(1, found);
      isDirectory[i] = (entry.at(0) == 'D') ? '1' : '0';
      if (isDirectory[i] != '0') {
        sizes[i] = 0;
      } else {
        sizes[i] = (uint32_t) std::stoi(entry.substr(found+1));
      }
      i++;
    }
    res.name = names;
    res.size = sizes;
    res.is_directory = isDirectory;
    return true;
  }
  return false;
}


void RpsmMainboard::sendPx4GpioSetCommand(Px4gpio gpio, bool enable) {
  ROS_DEBUG_STREAM("RpsmMainboard::sendPx4GpioSetCommand(GPIO: "
                       << static_cast<int>(gpio) << ", enable: "
                       << (enable ? "true" : "false") << ")");
  mavlink_px4gpio_set_cmd_t gpio_to_send;
  gpio_to_send.main_enable = static_cast<int>(Px4gpioCommand::GPIO_NA);
  gpio_to_send.enable_can_pwr =  static_cast<int>(Px4gpioCommand::GPIO_NA);
  gpio_to_send.enable_exdev =  static_cast<int>(Px4gpioCommand::GPIO_NA);
  gpio_to_send.enable_5v =  static_cast<int>(Px4gpioCommand::GPIO_NA);
  gpio_to_send.enable_12v =  static_cast<int>(Px4gpioCommand::GPIO_NA);
  gpio_to_send.enable_15v =  static_cast<int>(Px4gpioCommand::GPIO_NA);
  gpio_to_send.mainled_enable =  static_cast<int>(Px4gpioCommand::GPIO_NA);

  switch(gpio){
    case Px4gpio::MASTERPWR:
    {
      gpio_to_send.main_enable = static_cast<int8_t>(enable ?
                                                     Px4gpioCommand::GPIO_HIGH : Px4gpioCommand::GPIO_LOW);
      break;
    } case Px4gpio::CANPWR:
    {
      gpio_to_send.enable_can_pwr  = static_cast<int8_t>(enable ?
                                                         Px4gpioCommand::GPIO_HIGH : Px4gpioCommand::GPIO_LOW);
      break;
    } case Px4gpio::EX_DEV:
    {
      gpio_to_send.enable_exdev  = static_cast<int8_t>(enable ?
                                                       Px4gpioCommand::GPIO_HIGH : Px4gpioCommand::GPIO_LOW);
      break;
    } case Px4gpio::V5:
    {
      gpio_to_send.enable_5v  = static_cast<int8_t>(enable ?
                                                    Px4gpioCommand::GPIO_HIGH : Px4gpioCommand::GPIO_LOW);
      break;
    } case Px4gpio::V12:
    {
      gpio_to_send.enable_12v  = static_cast<int8_t>(enable ?
                                                     Px4gpioCommand::GPIO_HIGH : Px4gpioCommand::GPIO_LOW);
      break;
    } case Px4gpio::V15:
    {
      gpio_to_send.enable_15v  = static_cast<int8_t>(enable ?
                                                     Px4gpioCommand::GPIO_HIGH : Px4gpioCommand::GPIO_LOW);
      break;
    } case Px4gpio::MAINLED:
    {
      gpio_to_send.mainled_enable  = static_cast<int8_t>(enable ?
                                                         Px4gpioCommand::GPIO_HIGH : Px4gpioCommand::GPIO_LOW);
      break;
    }
    default:
      ROS_ERROR("Problem handling srv: gpio is not recognized! \n");
      break;
  }

  // Pack the message for MAVLINK
  mavlink_message_t msg_to_send;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_px4gpio_set_cmd_pack(mavlinkSystemId_, mavlinkComponentId_, &msg_to_send, gpio_to_send.main_enable,
                                   gpio_to_send.enable_can_pwr,
                                   gpio_to_send.enable_exdev,
                                   gpio_to_send.enable_5v,
                                   gpio_to_send.enable_12v,
                                   gpio_to_send.enable_15v,
                                   gpio_to_send.mainled_enable);
  // Add the bits to buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg_to_send);

  // Send buffer via serial
  this->stageSerialPackage(buf, len);
}


void RpsmMainboard::sendFtpMessage(const uint8_t* request) {
  mavlink_message_t msg_to_send;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_file_transfer_protocol_pack(mavlinkSystemId_,     // System ID
                                          mavlinkComponentId_,  // Component ID
                                          &msg_to_send,         // Mavlink Message to pack into
                                          0,                    // Target network
                                          0,                    // Target system TODO what id?
                                          0,                    // Target component
                                          (uint8_t *) request); // Payload
  // Add the bits to buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg_to_send);

  // Send buffer via serial
  this->stageSerialPackage(buf, len);
}


void RpsmMainboard::reactToFtpList(const std::vector<std::string> &list) {

  std::unique_lock<std::mutex> lock(listDirectoryMutex_);
  directoryList_ = list;
  cndListDirectory_.notify_all();
  std::stringstream listStringStream;
  for (auto entry : list) {
    listStringStream << entry << std::endl;
  }  ROS_INFO_STREAM("[RpsmMainboard] New List received: " << std::endl << listStringStream.str());
}


void RpsmMainboard::reactToFtpDownload(const float progress, const std::string& error) {
  ROS_INFO("[RpsmMainboard] File downloading %f %% %s", progress * 100.0, error.c_str());
  if (error.empty() && progress == 1.0) {
    rpsm_msgs::DownloadFileResult result;
    result.success = true;
    ftpDownloadFileServer_->setSucceeded(result);
  }
  else if (progress == 0.0f) {
    rpsm_msgs::DownloadFileResult result;
    result.success = false;
    ftpDownloadFileServer_->setAborted(result, error);
  }
  else {
    rpsm_msgs::DownloadFileFeedback feedback;
    feedback.progress = progress;
    feedback.error_message = error;
    ftpDownloadFileServer_->publishFeedback(feedback);
  }
}


void RpsmMainboard::reactToFtpUpload() {
  ROS_INFO("[RpsmMainboard] File uploaded");
}


void RpsmMainboard::handleMavlinkBmsCircuitStatus(mavlink_message_t *mavlink_msg) {
  mavlink_bms_circuit_status_t dec_msg;
  mavlink_msg_bms_circuit_status_decode(mavlink_msg, &dec_msg);

  bmsState_.serialnumber = dec_msg.serialnumber;
  bmsState_.temperature = dec_msg.temperature;
  bmsState_.voltage = dec_msg.voltage;
  bmsState_.current = dec_msg.current;
  bmsState_.stateofcharge = dec_msg.stateofcharge;
  bmsState_.safetystatus = dec_msg.safetystatus;
  bmsState_.operationstatus = dec_msg.operationstatus;
  bmsState_.cellvoltage[0] = dec_msg.cellvoltage1;
  bmsState_.cellvoltage[1] = dec_msg.cellvoltage2;
  bmsState_.cellvoltage[2] = dec_msg.cellvoltage3;
  bmsState_.cellvoltage[3] = dec_msg.cellvoltage4;
  bmsState_.cellvoltage[4] = dec_msg.cellvoltage5;
  bmsState_.cellvoltage[5] = dec_msg.cellvoltage6;
  bmsState_.cellvoltage[6] = dec_msg.cellvoltage7;
  bmsState_.cellvoltage[7] = dec_msg.cellvoltage8;
  bmsState_.cellvoltage[8] = dec_msg.cellvoltage9;
  bmsState_.cellvoltage[9] = dec_msg.cellvoltage10;
  bmsState_.cellvoltage[10] = dec_msg.cellvoltage11;
  bmsState_.cellvoltage[11] = dec_msg.cellvoltage12;
  if (bmsStatePublisher_.getNumSubscribers() > 0u) {
    bmsStatePublisher_.publish(bmsState_);
  }
}



void RpsmMainboard::handleMavlinkMasterMavlinkNotification(mavlink_message_t *mavlink_msg) {
  mavlink_master_mavlink_notification_t dec_msg;
  mavlink_msg_master_mavlink_notification_decode(mavlink_msg, &dec_msg);
  switch (dec_msg.level) {
    case 5:
    {
      powerButtonEventPublisher_.publish(std_msgs::Empty());
      break;
    }
  }
}


void RpsmMainboard::handleMavlinkFileTransferProtocol(mavlink_message_t *mavlink_msg) {
  fileManager_.receiveMessage(mavlink_msg);
}


void RpsmMainboard::downloadFileActionGoalCB() {
  ROS_INFO("[RpsmMainboard] Start ftp download file action.");
  rpsm_msgs::DownloadFileGoalConstPtr goal = ftpDownloadFileServer_->acceptNewGoal();
  if (goal->from_filepath.empty()) {
    ROS_ERROR("[RpsmMainboard] Received empty filepath!");
    ftpDownloadFileServer_->setAborted(rpsm_msgs::DownloadFileResult(), "Filepath is empty");
    return;
  }
  currentDownloadFromFilePath_ = goal->from_filepath;
  if (goal->to_filepath.empty()) {
    ROS_WARN_STREAM("[RpsmMainboard] No goal filepath specified, set to " << currentDownloadToFilePath_);
  } else {
    currentDownloadToFilePath_ = goal->to_filepath;
  }
  fileManager_.streamPath(currentDownloadFromFilePath_, currentDownloadToFilePath_);
  return;
}


void RpsmMainboard::downloadFileActionPreemptCB() {
  ROS_INFO("[RpsmMainboard] Preempt running downloadFile action.");
  fileManager_.preemptRunningSession();
}



} // namespace rpsm_master
