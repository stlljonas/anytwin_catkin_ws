/******************************************************************************
 * Copyright (C) RSL ETH Zurich - All Rights Reserved                         *
 * Unauthorized copying of this file, via any medium is strictly prohibited   *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * Author: Linus Isler <lisler@anybotics.com>                                 *
 ******************************************************************************/

#pragma once

#include <QWidget>
#include <QObject>
#include <QTimer>
#include <QMessageBox>

#include <ros/ros.h>

#include <rqt_gui_cpp/plugin.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/BatteryState.h>

#include <rpsm_msgs/ActuationBoardStates.h>
#include <rpsm_msgs/BmsState.h>
#include <rpsm_msgs/DownloadFileAction.h>
#include <rpsm_msgs/Heartbeat.h>
#include <rpsm_msgs/HumidityState.h>
#include <rpsm_msgs/ListDirectory.h>
#include <rpsm_msgs/MainBoardState.h>

#include <rqt_rpsm/ui_RpsmPlugin.h>
#include <rqt_rpsm/WorkerThreadShutdown.h>
#include <rqt_rpsm/WorkerThreadAB.h>
#include <rqt_rpsm/WorkerThreadListDirectory.h>


namespace rqt_rpsm {


class FileTreeItem : public QTreeWidgetItem {
public:
  explicit FileTreeItem(FileTreeItem* parent, bool isFile, std::string fileName)
    : QTreeWidgetItem(static_cast<QTreeWidgetItem*>(parent))
    , isFile_(isFile)
  {
    if (!parent->getFilePath().empty() && parent->getFilePath().back() != '/') {
      filePath_ = parent->getFilePath() + "/" + fileName;
    }
    else {
      filePath_ = parent->getFilePath() + fileName;
    }
  }
  explicit FileTreeItem(QTreeWidget* view, bool isFile, std::string filePath)
    : QTreeWidgetItem(view)
    , isFile_(isFile)
    , filePath_(filePath)
  {}

  bool isFile() { return isFile_; }
  const std::string& getFilePath() { return filePath_; }

private:
  bool isFile_;
  std::string filePath_;
};



class RpsmPlugin
    : public rqt_gui_cpp::Plugin {

Q_OBJECT

public:

  /***************************************************************************/
  /** Constructor/Destructor                                                **/
  /***************************************************************************/

  RpsmPlugin();

  /***************************************************************************/
  /** Initialize/Shutdown                                                   **/
  /***************************************************************************/

  virtual void initPlugin(qt_gui_cpp::PluginContext& context);

  virtual void shutdownPlugin();

  /***************************************************************************/
  /** Settings                                                              **/
  /***************************************************************************/

  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings,
                            qt_gui_cpp::Settings& instance_settings) const;

  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                               const qt_gui_cpp::Settings& instance_settings);

Q_SIGNALS:

  /***************************************************************************/
  /** Signals                                                               **/
  /***************************************************************************/

  void receiveHeartbeat();
  void receiveBatteryState();
  void receiveBmsState();
  void receiveMainboardState();
  void receiveActuationBoardStates();
  void receiveHumidityState();
  void receiveFirstMainboardHeartbeat();
  void receiveFirstLPCHeartbeat();
  void receiveDownloadFileFeedback();

protected slots:

  /***************************************************************************/
  /** Slots                                                                 **/
  /***************************************************************************/

  void checkForHeartbeat();
  void colorHeartbeat();
  void fillBatteryData();
  void fillBmsData();
  void fillMainboardData();
  void fillActuationBoardData();
  void fillHumidityData();
  void changeToMainboard();
  void changeToLpc();
  void updateDownloadProgress();

  void onPushButtonShutdownClicked();
  void onPushButtonABClicked();
  void onPowerWarningFinished(int rc);
  void onABWarningFinished(int rc);
  void onPowerShutdownResult(bool isOk);
  void onABShutdownResult(bool isOk);
  void onTreeWidgetActivated(QTreeWidgetItem* item, int column);
  void onTreeWidgetExpanded();
  void onListDirectoryResult(bool isOk,
                             rpsm_msgs::ListDirectoryResponse response);
  void onTreePushButtonClicked();
  void onDownloadCancelPushButtonClicked();
  void onDumpBackPushButtonClicked();
  void onSaveAsCsvPushButtonClicked();

protected:

  enum class LogToolboxState {
    treeView,
    downloading,
    dump
  };

  /***************************************************************************/
  /** Methods                                                               **/
  /***************************************************************************/

  void heartbeatLpcCallback(const rpsm_msgs::Heartbeat& msg);
  void heartbeatMainboardCallback(const rpsm_msgs::Heartbeat& msg);
  void batteryStateCallback(const sensor_msgs::BatteryState& msg);
  void bmsStateCallback(const rpsm_msgs::BmsState& msg);
  void mainBoardStateCallback(const rpsm_msgs::MainBoardState& msg);
  void actuationBoardStatesCallback(const rpsm_msgs::ActuationBoardStates& msg);
  void humidityStateCallback(const rpsm_msgs::HumidityState& msg);
  void downloadFileActiveCallback();
  void downloadFileFeedbackCallback(const rpsm_msgs::DownloadFileFeedbackConstPtr& feedback);
  void downloadFileResultCallback(const actionlib::SimpleClientGoalState& state,
      const rpsm_msgs::DownloadFileResultConstPtr &result);
  bool specifyDownloadFilePath(std::string* filePath);
  void setLogToolboxState(LogToolboxState state);


  /***************************************************************************/
  /** Constants                                                             **/
  /***************************************************************************/

  const std::string TAG = "RpsmPlugin";

  /***************************************************************************/
  /** Variables                                                             **/
  /***************************************************************************/

  Ui::RpsmPluginWidget ui_;
  QWidget* widget_;
  ros::NodeHandle nh_;

  QMessageBox* confirmPowerShutdownMsgBox_;
  QMessageBox* confirmABShutdownMsgBox_;

  ros::Subscriber heartbeatLpcSubscriber_;
  ros::Subscriber heartbeatMainboardSubscriber_;
  ros::Subscriber batteryStateSubscriber_;
  ros::Subscriber bmsStateSubscriber_;
  ros::Subscriber mainBoardStateSubscriber_;
  ros::Subscriber actuationBoardStatesSubscriber_;
  ros::Subscriber humidityStateSubscriber_;

  ros::ServiceClient powerShutdownClient_;
  ros::ServiceClient abShutdownClient_;
  ros::ServiceClient listDirectoryClient_;

  WorkerThreadShutdown* workerThreadShutdown_;
  WorkerThreadAB* workerThreadAB_;
  WorkerThreadListDirectory* workerThreadListDirectory_;

  typedef actionlib::SimpleActionClient
        <rpsm_msgs::DownloadFileAction> DownloadFileActionClient;
  typedef std::shared_ptr<DownloadFileActionClient> DownloadFileActionClientPtr;
  DownloadFileActionClientPtr downloadFileActionClient_;
  rpsm_msgs::DownloadFileFeedback downloadFileFeedback_;

  rpsm_msgs::Heartbeat lastHeartbeat_;
  sensor_msgs::BatteryState lastBatteryState_;
  rpsm_msgs::BmsState lastBmsState_;
  bool receivedMainboardInsteadOfLpc_;
  rpsm_msgs::MainBoardState lastMainBoardState_;
  rpsm_msgs::ActuationBoardStates lastActuationBoardStates_;
  rpsm_msgs::HumidityState lastHumidityState_;
  bool shownTemperatureWarning_;

  QTimer* heartbeatTimer_;

//  std::string pw_;

};

} // namespace rqt_rpsm
