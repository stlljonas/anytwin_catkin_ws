/******************************************************************************
 * Copyright (C) RSL ETH Zurich - All Rights Reserved                         *
 * Unauthorized copying of this file, via any medium is strictly prohibited   *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * Author: Samuel Bachmann <sbachmann@anybotics.com>                          *
 ******************************************************************************/

#include "rqt_rpsm/WorkerThreadShutdown.h"

namespace rqt_rpsm {

/*****************************************************************************/
/** Methods                                                                 **/
/*****************************************************************************/

void WorkerThreadShutdown::run() {
  bool isOk = client_.call(request_, response_);
  emit result(isOk);
}

/*****************************************************************************/
/** Accessors                                                               **/
/*****************************************************************************/

void WorkerThreadShutdown::setClient(ros::ServiceClient &client) {
  client_ = client;
}

void WorkerThreadShutdown::setRequest(std_srvs::EmptyRequest request) {
  request_ = request;
}

} // namespace
