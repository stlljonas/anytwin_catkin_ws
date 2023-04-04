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

#include <std_srvs/Empty.h>

namespace rqt_rpsm {

class WorkerThreadShutdown : public QThread {
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

  void setRequest(std_srvs::EmptyRequest request);

private:

  /***************************************************************************/
  /** Variables                                                             **/
  /***************************************************************************/

  std_srvs::EmptyRequest request_;
  std_srvs::EmptyResponse response_;
  ros::ServiceClient client_;

signals:

  /***************************************************************************/
  /** Signals                                                               **/
  /***************************************************************************/

  void result(bool isOk);
};

} // namespace
