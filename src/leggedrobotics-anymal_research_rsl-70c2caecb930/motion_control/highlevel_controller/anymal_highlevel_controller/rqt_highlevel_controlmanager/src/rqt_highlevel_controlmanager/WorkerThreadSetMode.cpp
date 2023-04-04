/******************************************************************************
 * Copyright (C) RSL ETH Zurich - All Rights Reserved                         *
 * Unauthorized copying of this file, via any medium is strictly prohibited   *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * Author: Samuel Bachmann <sbachmann@anybotics.com>                          *
 ******************************************************************************/

#include "rqt_highlevel_controlmanager/WorkerThreadSetMode.h"

namespace rqt_highlevel_controlmanager {

/*****************************************************************************/
/** Methods                                                                 **/
/*****************************************************************************/

void WorkerThreadSetMode::run() {
  bool isOk = switchControllerModeClient_.call(request_, response_);
  emit resultReady(isOk, response_);
}

/*****************************************************************************/
/** Accessors                                                               **/
/*****************************************************************************/

void WorkerThreadSetMode::setRequest(
    anymal_msgs::SwitchControllerRequest request) {
  request_ = request;
}

void WorkerThreadSetMode::setClient(ros::ServiceClient &client) {
  switchControllerModeClient_ = client;
}

} // namespace
