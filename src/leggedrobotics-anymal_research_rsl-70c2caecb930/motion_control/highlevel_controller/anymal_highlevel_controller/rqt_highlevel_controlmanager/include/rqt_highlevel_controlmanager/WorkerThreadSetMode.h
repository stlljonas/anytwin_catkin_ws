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
#include <anymal_msgs/SwitchController.h>

namespace rqt_highlevel_controlmanager {

class WorkerThreadSetMode : public QThread {
Q_OBJECT

  /***************************************************************************/
  /** Methods                                                               **/
  /***************************************************************************/

  void run();

public:

  /***************************************************************************/
  /** Accessors                                                             **/
  /***************************************************************************/

  void setClient(ros::ServiceClient &client);

  void setRequest(anymal_msgs::SwitchControllerRequest request);

private:

  /***************************************************************************/
  /** Variables                                                             **/
  /***************************************************************************/

  anymal_msgs::SwitchControllerRequest request_;
  anymal_msgs::SwitchControllerResponse response_;
  ros::ServiceClient switchControllerModeClient_;

signals:

  /***************************************************************************/
  /** Signals                                                               **/
  /***************************************************************************/

  void resultReady(bool isOk,
                   anymal_msgs::SwitchControllerResponse response);
};

} // namespace
