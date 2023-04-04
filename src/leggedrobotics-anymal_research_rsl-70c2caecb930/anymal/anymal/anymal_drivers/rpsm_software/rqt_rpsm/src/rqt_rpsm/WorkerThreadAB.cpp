/******************************************************************************
 * Copyright (C) RSL ETH Zurich - All Rights Reserved                         *
 * Unauthorized copying of this file, via any medium is strictly prohibited   *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * Author: Samuel Bachmann <sbachmann@anybotics.com>                          *
 ******************************************************************************/

#include "rqt_rpsm/WorkerThreadAB.h"

namespace rqt_rpsm {

/*****************************************************************************/
/** Methods                                                                 **/
/*****************************************************************************/

void WorkerThreadAB::run() {
  ROS_INFO("WorkerThreadAB: run!");
  bool isOk = client_.call(request_, response_);
  emit result(isOk);
}

/*****************************************************************************/
/** Accessors                                                               **/
/*****************************************************************************/

void WorkerThreadAB::setClient(ros::ServiceClient &client) {
  client_ = client;
}

void WorkerThreadAB::setRequest(std_srvs::SetBoolRequest request) {
  request_ = request;
}

} // namespace
