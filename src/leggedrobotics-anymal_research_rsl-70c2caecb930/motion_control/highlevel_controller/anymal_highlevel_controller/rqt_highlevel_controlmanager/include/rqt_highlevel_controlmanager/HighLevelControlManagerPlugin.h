/******************************************************************************
 * Copyright (C) RSL ETH Zurich - All Rights Reserved                         *
 * Unauthorized copying of this file, via any medium is strictly prohibited   *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * Author: Dario Bellicoso                                                    *
 * Author: Samuel Bachmann <sbachmann@anybotics.com>                          *
 ******************************************************************************/

#pragma once

#include <rqt_highlevel_controlmanager/ui_HighLevelControlManagerPlugin.h>
#include <rqt_highlevel_controlmanager/WorkerThreadCalibrateFeet.h>
#include <rqt_highlevel_controlmanager/WorkerThreadSetController.h>
#include <rqt_highlevel_controlmanager/WorkerThreadSetMode.h>

// std
#include <mutex>

// qt
#include <QWidget>
#include <QModelIndex>
#include <rqt_gui_cpp/plugin.h>

// ros
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>

// anymal msgs
#include <anymal_msgs/GetAvailableControllers.h>
#include <anymal_msgs/ResetStateEstimator.h>

// rocoma msgs
#include <rocoma_msgs/ControllerManagerState.h>
#include <rocoma_msgs/SwitchController.h>
#include <rocoma_msgs/GetAvailableControllers.h>

// param io
#include <param_io/get_param.hpp>

namespace rqt_highlevel_controlmanager {

class HighLevelControlManagerPlugin : public rqt_gui_cpp::Plugin {
Q_OBJECT
public:

  /***************************************************************************/
  /** Constructor/Destructor                                                **/
  /***************************************************************************/

  HighLevelControlManagerPlugin();

  /***************************************************************************/
  /** Init/Shutdown Plugin                                                  **/
  /***************************************************************************/

  virtual void initPlugin(qt_gui_cpp::PluginContext &context);

  virtual void shutdownPlugin();

  /***************************************************************************/
  /** Plugin Settings                                                       **/
  /***************************************************************************/

  virtual void saveSettings(qt_gui_cpp::Settings &plugin_settings,
                            qt_gui_cpp::Settings &instance_settings) const;

  virtual void restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                               const qt_gui_cpp::Settings &instance_settings);

  // Helpers
  void init(ros::NodeHandle& nodeHandle, QWidget* widget);

private:

  /***************************************************************************/
  /** Constants                                                             **/
  /***************************************************************************/

  const std::string TAG = "rqt_highlevel_controlmanager";

  /***************************************************************************/
  /** Variables                                                             **/
  /***************************************************************************/

  Ui::HighLevelControlManagerPlugin ui_;
  QWidget *widget_;
  ros::NodeHandle nodeHandle_;

  // pixmaps
  QPixmap pixmapOk_;
  QPixmap pixmapError_;

  bool simulation_ = false;

  // service clients
  ros::ServiceClient switchLowLevelControllerClient_;
  ros::ServiceClient switchControllerClient_;
  ros::ServiceClient getAvailableControllersClient_;
  ros::ServiceClient emergencyStopClient_;
  ros::ServiceClient clearEmergencyStopClient_;
  ros::ServiceClient resetStateEstimatorClient_;
  ros::ServiceClient resetStateEstimatorHereClient_;
  ros::ServiceClient switchControllerModeClient_;
  ros::ServiceClient getAvailableControllerModesClient_;
  ros::ServiceClient calibrateFeetClient_;

  // subscribers
  ros::Subscriber activeControllerSubscriber_;
  ros::Subscriber controllerManagerStateSubscriber_;

  // services from parameter server
  std::string serviceSwitchLowlevelController_;
  std::string serviceSwitchController_;
  std::string serviceGetAvailableControllers_;
  std::string serviceEmergencyStop_;
  std::string serviceClearEmergencyStop_;
  std::string serviceStateEstimatorReset_;
  std::string serviceStateEstimatorResetHere_;
  std::string serviceCalibrateFeet_;

  // topics from parameter server
  std::string topicActiveController_;
  std::string topicControllerManagerState_;

  std::mutex mutex_;

  /***************************************************************************/
  /** Methods                                                               **/
  /***************************************************************************/

  virtual std::string getStatusName(
      const anymal_msgs::SwitchController::Response &res);

  std::string getStatusName(
      const rocoma_msgs::SwitchController::Response &res);

  virtual bool updateAvailableControllers();

  virtual bool updateAvailableModes();

  virtual void deactivateUi();

  virtual void activateUi();

  virtual void initRefreshModesClient(std::string activeController);

  /***************************************************************************/
  /** Callbacks                                                             **/
  /***************************************************************************/

  void activeControllerCallback(const std_msgs::StringConstPtr& msg);

  void controllerManagerStateCallback(const rocoma_msgs::ControllerManagerStateConstPtr& msg);

protected slots:

  /***************************************************************************/
  /** Slots                                                                 **/
  /***************************************************************************/

  virtual void buttonEmergencyStopClicked();
  virtual void buttonClearEmergencyStopClicked();

  virtual void buttonResetStateEstimatorClicked();

  virtual void buttonResetStateEstimatorHereClicked();

  virtual void buttonRefreshControllerClicked();

  virtual void buttonSetControllerClicked();

  virtual void buttonRefreshModeClicked();

  virtual void buttonSetModeClicked();

  virtual void buttonCalibrateFeetClicked();

  virtual void calibrateFeetResult(bool isOk,
                                   any_msgs::SetUInt32Response response);

  virtual void setModeResult(
      bool isOk, anymal_msgs::SwitchControllerResponse response);

  virtual void setLowLevelControllerResult(
      bool isOk, anymal_msgs::AnymalLowLevelControllerGoToStateResponse responseLowLevel);

  virtual void setControllerResult(
      bool isOk, rocoma_msgs::SwitchControllerResponse response);

  virtual void activeControllerUpdated(std::string activeController);

  virtual void emergencyStateUpdated(const bool isOk);

  virtual void managerStateUpdated(const int state);

signals:

  /***************************************************************************/
  /** Signals                                                               **/
  /***************************************************************************/

  void stateChanged();

  void updateActiveController(std::string activeController);

  void updateEmergencyState(bool isOk);

  void updateManagerState(int state);

};

} // namespace
