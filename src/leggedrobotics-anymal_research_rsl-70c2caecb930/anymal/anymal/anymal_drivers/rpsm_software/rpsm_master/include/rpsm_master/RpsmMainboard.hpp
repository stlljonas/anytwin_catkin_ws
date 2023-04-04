#pragma once

#include <condition_variable>

// Rpsm master
#include "rpsm_master/RpsmMaster.hpp"

// ROS Libraries
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/Empty.h>
#include <std_srvs/SetBool.h>

// Custom ROS message and service calls
#include <rpsm_msgs/BmsState.h>
#include <rpsm_msgs/ActuationBoardCommand.h>
#include <rpsm_msgs/DownloadFileAction.h>
#include <rpsm_msgs/ListDirectory.h>

// package
#include "rpsm_master/FileManager.hpp"


namespace rpsm_master {

class RpsmMainboard : public RpsmMaster
{
 public:  
  RpsmMainboard() = delete;
  RpsmMainboard(any_node::Node::NodeHandlePtr nh);
  virtual ~RpsmMainboard();

  bool init() override;
  void cleanup() override;

  bool actuationBoardCommandService(rpsm_msgs::ActuationBoardCommand::Request  &req, rpsm_msgs::ActuationBoardCommand::Response &res);
  bool enableCanPowerService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response & res);
  bool enablePowersupply5VService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response & res);
  bool enablePowersupply12VService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response & res);
  bool enablePowersupply15VService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response & res);

  void sendPx4GpioSetCommand(Px4gpio gpio, bool enable);

  bool listDirectoryService(rpsm_msgs::ListDirectory::Request &req, rpsm_msgs::ListDirectory::Response &res);

  void sendFtpMessage(const uint8_t* request);
  void reactToFtpList(const std::vector<std::string>& list);
  void reactToFtpDownload(const float progress, const std::string& error);
  void reactToFtpUpload();


protected:
  typedef actionlib::SimpleActionServer<rpsm_msgs::DownloadFileAction> DownloadFileActionServer;
  typedef std::shared_ptr<DownloadFileActionServer> DownloadFileActionServerPtr;

  void handleMavlinkBmsCircuitStatus(mavlink_message_t *mavlink_msg);
  void handleMavlinkMasterMavlinkNotification(mavlink_message_t *mavlink_msg);
  void handleMavlinkFileTransferProtocol(mavlink_message_t *mavlink_msg);

  void downloadFileActionPreemptCB();
  void downloadFileActionGoalCB();

  FileManager fileManager_;
  std::string currentDownloadFromFilePath_, currentDownloadToFilePath_;

  rpsm_msgs::BmsState bmsState_;
  ros::Publisher bmsStatePublisher_;
  ros::Publisher powerButtonEventPublisher_;

  ros::ServiceServer actuationBoardCommandService_;
  ros::ServiceServer enableCanPowerService_;
  ros::ServiceServer enablePowersupply5VService_;
  ros::ServiceServer enablePowersupply12VService_;
  ros::ServiceServer enablePowersupply15VService_;

  DownloadFileActionServerPtr ftpDownloadFileServer_;
  ros::ServiceServer ftpListDirectoryServer_;
  std::mutex listDirectoryMutex_;
  std::condition_variable_any cndListDirectory_;
  std::vector<std::string> directoryList_;
};

} // namespace rpsm_master
