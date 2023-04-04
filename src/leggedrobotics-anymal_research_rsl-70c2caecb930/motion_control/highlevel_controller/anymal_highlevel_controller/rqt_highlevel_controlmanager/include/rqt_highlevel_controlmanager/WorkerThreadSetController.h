/******************************************************************************
 * Copyright (C) RSL ETH Zurich - All Rights Reserved                         *
 * Unauthorized copying of this file, via any medium is strictly prohibited   *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * Author: Samuel Bachmann <sbachmann@anybotics.com>                          *
 ******************************************************************************/

#pragma once
// qt
#include <QThread>
// ros
#include <ros/ros.h>
// msgs
#include <rocoma_msgs/SwitchController.h>
#include <anymal_msgs/AnymalLowLevelControllerGoToState.h>

namespace rqt_highlevel_controlmanager {

class WorkerThreadSetController : public QThread {
Q_OBJECT

  /***************************************************************************/
  /** Methods                                                               **/
  /***************************************************************************/

  void run();

public:

  /***************************************************************************/
  /** Accessors                                                             **/
  /***************************************************************************/

  void setSimulation(bool simulation);

  void setLowLevelClient(ros::ServiceClient &client);

  void setClient(ros::ServiceClient &client);

  void setLowLevelRequest(
      anymal_msgs::AnymalLowLevelControllerGoToStateRequest request);

  void setRequest(rocoma_msgs::SwitchControllerRequest request);

private:

  /***************************************************************************/
  /** Variables                                                             **/
  /***************************************************************************/

  bool simulation_ = false;
  anymal_msgs::AnymalLowLevelControllerGoToStateRequest requestLowLevel_;
  anymal_msgs::AnymalLowLevelControllerGoToStateResponse responseLowLevel_;
  rocoma_msgs::SwitchControllerRequest request_;
  rocoma_msgs::SwitchControllerResponse response_;
  ros::ServiceClient switchLowLevelControllerModeClient_;
  ros::ServiceClient switchControllerModeClient_;

signals:

  /***************************************************************************/
  /** Signals                                                               **/
  /***************************************************************************/

  void resultLowLevelReady(
      bool isOk,
      anymal_msgs::AnymalLowLevelControllerGoToStateResponse response);

  void resultReady(
      bool isOk,
      rocoma_msgs::SwitchControllerResponse response);
};

} // namespace
