/******************************************************************************
 * Copyright (C) RSL ETH Zurich - All Rights Reserved                         *
 * Unauthorized copying of this file, via any medium is strictly prohibited   *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * Author: Samuel Bachmann <sbachmann@anybotics.com>                          *
 ******************************************************************************/

#pragma once

#include <QThread>

#include <ros/ros.h>

#include <rpsm_msgs/ListDirectory.h>

namespace rqt_rpsm {

class WorkerThreadListDirectory : public QThread {
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

  void setRequest(rpsm_msgs::ListDirectoryRequest request);

private:

  /***************************************************************************/
  /** Variables                                                             **/
  /***************************************************************************/

  rpsm_msgs::ListDirectoryRequest request_;
  rpsm_msgs::ListDirectoryResponse response_;
  ros::ServiceClient client_;

signals:

  /***************************************************************************/
  /** Signals                                                               **/
  /***************************************************************************/

  void result(bool isOk, rpsm_msgs::ListDirectoryResponse response);
};

} // namespace rqt_rpsm
