/******************************************************************************
 * Copyright (C) RSL ETH Zurich - All Rights Reserved                         *
 * Unauthorized copying of this file, via any medium is strictly prohibited   *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * Author: Samuel Bachmann <sbachmann@anybotics.com>                          *
 ******************************************************************************/

#include "rqt_highlevel_controlmanager/WorkerThreadCalibrateFeet.h"

namespace rqt_highlevel_controlmanager {

/*****************************************************************************/
/** Methods                                                                 **/
/*****************************************************************************/

void WorkerThreadCalibrateFeet::run() {
  bool isOk = calibrateFeetClient_.call(request_, response_);
  emit resultReady(isOk, response_);
}

/*****************************************************************************/
/** Accessors                                                               **/
/*****************************************************************************/

void WorkerThreadCalibrateFeet::setRequest(
    any_msgs::SetUInt32Request request) {
  request_ = request;
}

void WorkerThreadCalibrateFeet::setClient(ros::ServiceClient &client) {
  calibrateFeetClient_ = client;
}

} // namespace
