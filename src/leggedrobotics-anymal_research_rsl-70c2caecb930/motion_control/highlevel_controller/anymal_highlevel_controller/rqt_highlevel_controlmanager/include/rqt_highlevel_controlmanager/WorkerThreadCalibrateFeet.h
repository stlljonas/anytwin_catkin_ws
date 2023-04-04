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
#include <any_msgs/SetUInt32.h>

namespace rqt_highlevel_controlmanager {

class WorkerThreadCalibrateFeet : public QThread {
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

  void setRequest(any_msgs::SetUInt32Request request);

private:

  /***************************************************************************/
  /** Variables                                                             **/
  /***************************************************************************/

  any_msgs::SetUInt32Request request_;
  any_msgs::SetUInt32Response response_;
  ros::ServiceClient calibrateFeetClient_;

signals:

  /***************************************************************************/
  /** Signals                                                               **/
  /***************************************************************************/

  void resultReady(bool isOk, any_msgs::SetUInt32Response response);
};

} // namespace
