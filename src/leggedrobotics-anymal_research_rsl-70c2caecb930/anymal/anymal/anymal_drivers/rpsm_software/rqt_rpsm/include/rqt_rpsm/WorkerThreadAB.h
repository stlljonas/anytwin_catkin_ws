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

#include <std_srvs/SetBool.h>

namespace rqt_rpsm {

class WorkerThreadAB : public QThread {
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

  void setRequest(std_srvs::SetBoolRequest request);

private:

  /***************************************************************************/
  /** Variables                                                             **/
  /***************************************************************************/

  std_srvs::SetBoolRequest request_;
  std_srvs::SetBoolResponse response_;
  ros::ServiceClient client_;

signals:

  /***************************************************************************/
  /** Signals                                                               **/
  /***************************************************************************/

  void result(bool isOk);
};

} // namespace
