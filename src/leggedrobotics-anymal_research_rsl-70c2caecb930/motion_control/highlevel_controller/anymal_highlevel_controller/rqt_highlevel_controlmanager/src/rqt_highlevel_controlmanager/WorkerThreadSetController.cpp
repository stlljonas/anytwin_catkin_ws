/******************************************************************************
 * Copyright (C) RSL ETH Zurich - All Rights Reserved                         *
 * Unauthorized copying of this file, via any medium is strictly prohibited   *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * Author: Samuel Bachmann <sbachmann@anybotics.com>                          *
 ******************************************************************************/

#include "rqt_highlevel_controlmanager/WorkerThreadSetController.h"

namespace rqt_highlevel_controlmanager {

/*****************************************************************************/
/** Methods                                                                 **/
/*****************************************************************************/

void WorkerThreadSetController::run() {
  // set low-level controller
  if (!simulation_) {
    if (!switchLowLevelControllerModeClient_.exists() ||
        !switchLowLevelControllerModeClient_.call(requestLowLevel_,
                                                  responseLowLevel_)) {
      emit resultLowLevelReady(false, responseLowLevel_);
      return;
    }
    ros::Duration(0.1).sleep();
  }
  // set high-level controller
  bool isOk = switchControllerModeClient_.call(request_, response_);
  emit resultReady(isOk, response_);
}

/*****************************************************************************/
/** Accessors                                                               **/
/*****************************************************************************/

void WorkerThreadSetController::setLowLevelRequest(
    anymal_msgs::AnymalLowLevelControllerGoToStateRequest request) {
  requestLowLevel_ = request;
}

void WorkerThreadSetController::setRequest(
    rocoma_msgs::SwitchControllerRequest request) {
  request_ = request;
}

void WorkerThreadSetController::setSimulation(bool simulation) {
  simulation_ = simulation;
}

void WorkerThreadSetController::setLowLevelClient(ros::ServiceClient &client) {
  switchLowLevelControllerModeClient_ = client;
}

void WorkerThreadSetController::setClient(ros::ServiceClient &client) {
  switchControllerModeClient_ = client;
}

} // namespace
