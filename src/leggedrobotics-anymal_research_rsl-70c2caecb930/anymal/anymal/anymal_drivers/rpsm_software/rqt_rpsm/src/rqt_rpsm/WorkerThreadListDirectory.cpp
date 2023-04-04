/******************************************************************************
 * Copyright (C) RSL ETH Zurich - All Rights Reserved                         *
 * Unauthorized copying of this file, via any medium is strictly prohibited   *
 *                                                                            *
 *                                                                            *
 *                                                                            *
 * Author: Samuel Bachmann <sbachmann@anybotics.com>                          *
 ******************************************************************************/

#include "rqt_rpsm/WorkerThreadListDirectory.h"

namespace rqt_rpsm {

/*****************************************************************************/
/** Methods                                                                 **/
/*****************************************************************************/

void WorkerThreadListDirectory::run() {
  bool isOk = client_.call(request_, response_);
  emit result(isOk, response_);
}

/*****************************************************************************/
/** Accessors                                                               **/
/*****************************************************************************/

void WorkerThreadListDirectory::setClient(ros::ServiceClient &client) {
  client_ = client;
}

void WorkerThreadListDirectory::setRequest(rpsm_msgs::ListDirectoryRequest request) {
  request_ = request;
}

} // namespace rqt_rpsm
