/******************************************************************************
 * Copyright 2017 Samuel Bachmann, Peter Fankhauser                           *
 *                                                                            *
 * Redistribution and use in source and binary forms, with or without         *
 * modification, are permitted provided that the following conditions are met:*
 *                                                                            *
 * 1. Redistributions of source code must retain the above copyright notice,  *
 * this list of conditions and the following disclaimer.                      *
 *                                                                            *
 * 2. Redistributions in binary form must reproduce the above copyright       *
 * notice, this list of conditions and the following disclaimer in the        *
 * documentation and/or other materials provided with the distribution.       *
 *                                                                            *
 * 3. Neither the name of the copyright holder nor the names of its           *
 * contributors may be used to endorse or promote products derived from this  *
 * software without specific prior written permission.                        *
 *                                                                            *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"*
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE  *
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE *
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE  *
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR        *
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF       *
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS   *
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN    *
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)    *
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF     *
 * THE POSSIBILITY OF SUCH DAMAGE.                                            *
 *                                                                            *
 * Authors: Samuel Bachmann <sbachmann@anybotics.com>  ,                      *
 *          Peter Fankhauser <pfankhauser@anybotics.com>                            *
 ******************************************************************************/

#include "rqt_free_gait_action/WorkerThreadGetCollections.h"

namespace rqt_free_gait {

/*****************************************************************************/
/** Methods                                                                 **/
/*****************************************************************************/

void WorkerThreadGetCollections::run() {
  CollectionModel *collectionModel = new CollectionModel();
  free_gait_msgs::GetCollectionsRequest collectionsRequest;
  free_gait_msgs::GetCollectionsResponse collectionsResponse;

  if (!ros::service::waitForService(collectionsClient_.getService(),
                                    ros::Duration(600.0))) {
    ROS_WARN_STREAM_NAMED(TAG, TAG
        << " Service: " << collectionsClient_.getService()
        << " is not available. (Timeout 600.0 seconds)");
    emit result(false, collectionModel);
    return;
  }
  if (!collectionsClient_.call(collectionsRequest, collectionsResponse)) {
    emit result(false, collectionModel);
    return;
  }

  // Add pseudo empty collection to get all available actions.
  free_gait_msgs::CollectionDescription collectionDescription;
  collectionDescription.id = "";
  collectionsResponse.collections.insert(collectionsResponse.collections.begin(),
                                         collectionDescription);

  // Loop over the collections, get their actions and add them finally to the
  // collection model.
  for (auto collectionItem : collectionsResponse.collections) {
    // Get the actions of the collection.
    free_gait_msgs::GetActionsRequest actionsRequest;
    free_gait_msgs::GetActionsResponse actionsResponse;
    actionsRequest.collection_id = collectionItem.id;
    if (!actionsClient_.call(actionsRequest, actionsResponse)) {
      ROS_WARN_STREAM_NAMED(TAG, TAG
          << " Could not get action descriptions for collection id: "
          << collectionItem.id);
      continue;
    }

    // Loop over the actions and add them to the action model.
    ActionModel *actionModel = new ActionModel();
    for (auto actionItem : actionsResponse.actions) {
      Action action(QString::fromStdString(actionItem.id),
                    QString::fromStdString(actionItem.name),
                    QString::fromStdString(actionItem.description),
                    QString::fromStdString(actionItem.type));
      actionModel->addAction(action);
    }


    // Add the action model to the collection.
    QString collectionId = "all";
    QString collectionName = "All";
    bool collectionIsSequence = false;
    if (collectionItem.id.length() != 0) {
      collectionId = QString::fromStdString(collectionItem.id);
      collectionName = QString::fromStdString(collectionItem.name);
      collectionIsSequence = collectionItem.is_sequence;
    }
    Collection collection(collectionId, collectionName, actionModel, collectionIsSequence);

    // Add the collection to the collection model.
    collectionModel->addCollection(collection);
  }

  // Return the collection model.
  emit result(true, collectionModel);
}

/*****************************************************************************/
/** Accessors                                                               **/
/*****************************************************************************/

void WorkerThreadGetCollections::setActionsClient(ros::ServiceClient &client) {
  actionsClient_ = client;
}

void WorkerThreadGetCollections::setCollectionsClient(
    ros::ServiceClient &client) {
  collectionsClient_ = client;
}

} // namespace
