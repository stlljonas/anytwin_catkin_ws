
#include "point_cloud_processor/PointCloudProcessor.hpp"

// C++ standard library
#include <chrono>
#include <unordered_map>

//! tf
#include <tf2_eigen/tf2_eigen.h>

//! Parameter IO
#include <param_io/get_param.hpp>

// pointmatcher_ros
#include <pointmatcher_ros/PmTf.h>

// std_utils
#include <std_utils/make_unique.hpp>

namespace point_cloud_processor {

PointCloudProcessor::PointCloudProcessor(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle), tfListener_(tfBuffer_), externalSubscribersCheckPeriod_(1.0) {
  MELO_DEBUG_STREAM("Initializing node...");
  initialize();
}

void PointCloudProcessor::initialize() {
  if (!getParameters()) {
    MELO_ERROR_STREAM("Couldn't read parameters. Exiting.");
    ros::requestShutdown();
  }

  for (auto& cloudStream : cloudStreams_) {
    if (cloudStream->applyFilters_) {
      cloudStream->pmFiltersIface_.readPipelineFile(configFileFolder_ + "/" + cloudStream->filtersConfigFilename_);
      if (cloudStream->applySelfFilter_) {
        cloudStream->selfFilter_->processConfigFile(configFileFolder_ + "/" + cloudStream->selfFilterConfigFilename_,
                                                    cloudStream->robotDescriptionContents_);
      }
    }
    cloudStream->publisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(cloudStream->outputTopic_, 1, cloudStream->latch_);
    cloudStream->subscriber_ = nodeHandle_.subscribe<sensor_msgs::PointCloud2>(
        cloudStream->inputTopic_, cloudStream->queueSize_,
        std::bind(&PointCloudProcessor::cloudTopicCallback, this, std::placeholders::_1, cloudStream->id_), ros::VoidPtr(),
        ros::TransportHints().tcpNoDelay());
  }

  for (auto& mergedStream : mergedCloudStreams_) {
    if (mergedStream->applyFilters_) {
      mergedStream->pmFiltersIface_.readPipelineFile(configFileFolder_ + "/" + mergedStream->filtersConfigFilename_);
    }
    mergedStream->publisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(mergedStream->outputTopic_, 1, mergedStream->latch_);
  }

  resetService_ = nodeHandle_.advertiseService("reload_pipeline", &PointCloudProcessor::reloadPipelineServiceCallback, this);
  externalSubscribersCheckTimer_ =
      nodeHandle_.createTimer(ros::Duration(externalSubscribersCheckPeriod_), &PointCloudProcessor::externalSubscribersCheckCallback, this);
}

bool PointCloudProcessor::getParameters() {
  bool success(true);

  success &= param_io::getParam(nodeHandle_, "config_files_folder", configFileFolder_);
  success &= param_io::getParam(nodeHandle_, "subscribers_check_period", externalSubscribersCheckPeriod_);

  XmlRpc::XmlRpcValue cloudStreamsParams;
  param_io::getParam(nodeHandle_, "stream_sources", cloudStreamsParams);
  if (cloudStreamsParams.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    for (int i = 0; i < cloudStreamsParams.size(); i++) {  // NOLINT
      CloudStream::Ptr newCloudStream(new CloudStream());
      newCloudStream->id_ = static_cast<int>(cloudStreamsParams[i]["id"]);
      newCloudStream->sourceName_ = static_cast<std::string>(cloudStreamsParams[i]["source_name"]);
      newCloudStream->sourceType_ = static_cast<std::string>(cloudStreamsParams[i]["source_type"]);
      newCloudStream->queueSize_ = static_cast<uint32_t>(static_cast<int>(cloudStreamsParams[i]["queue_size"]));
      newCloudStream->latch_ = static_cast<bool>(cloudStreamsParams[i]["latch"]);
      newCloudStream->inputTopic_ = static_cast<std::string>(cloudStreamsParams[i]["input_topic"]);
      newCloudStream->outputTopic_ = static_cast<std::string>(cloudStreamsParams[i]["output_topic"]);
      newCloudStream->applyFilters_ = static_cast<bool>(cloudStreamsParams[i]["filter"]);
      newCloudStream->filtersConfigFilename_ = static_cast<std::string>(cloudStreamsParams[i]["filters_config"]);

      // Setup self filtering if configured
      if (cloudStreamsParams[i].hasMember("self_filter_config")) {
        if (cloudStreamsParams[i].hasMember("robot_description")) {
          newCloudStream->selfFilterConfigFilename_ = static_cast<std::string>(cloudStreamsParams[i]["self_filter_config"]);
          std::string robotDescription = static_cast<std::string>(cloudStreamsParams[i]["robot_description"]);
          bool descriptionSuccess = param_io::getParam(nodeHandle_, robotDescription, newCloudStream->robotDescriptionContents_);
          if (descriptionSuccess) {
            newCloudStream->applySelfFilter_ = descriptionSuccess;
            success &= descriptionSuccess;
            // TODO (jburklund) use std_utils or C++14?
            newCloudStream->selfFilter_ = std_utils::make_unique<point_cloud_self_filter::SelfFilter>(tfBuffer_);
          } else {
            MELO_ERROR_STREAM("Could not load robot description: " << robotDescription);
          }
        } else {
          MELO_WARN_STREAM("Stream " << newCloudStream->id_ << ": Missing robot description");
        }
      }

      MELO_INFO_STREAM("Stream " << newCloudStream->id_ << " - '" << newCloudStream->sourceName_ << "' of type "
                                 << newCloudStream->sourceType_);

      cloudStreams_.push_back(newCloudStream);
    }
  }

  XmlRpc::XmlRpcValue mergedStreamsParams;
  param_io::getParam(nodeHandle_, "stream_combinations", mergedStreamsParams);
  if (mergedStreamsParams.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    for (int16_t i = 0; i < mergedStreamsParams.size(); i++) {  // NOLINT
      MergedCloudStream::Ptr newMergedStream(new MergedCloudStream());
      newMergedStream->id_ = i;
      newMergedStream->streamName_ = static_cast<std::string>(mergedStreamsParams[i]["name"]);
      newMergedStream->targetFrame_ = static_cast<std::string>(mergedStreamsParams[i]["target_frame"]);
      newMergedStream->keepIntensityDescriptor_ = static_cast<bool>(mergedStreamsParams[i]["keep_intensity"]);
      newMergedStream->masterStreamId_ = static_cast<int>(mergedStreamsParams[i]["master"]);
      newMergedStream->cloudDecimation_ = static_cast<int>(mergedStreamsParams[i]["decimation"]);
      newMergedStream->maxTimeOffset_ = static_cast<double>(mergedStreamsParams[i]["max_time_offset"]);
      newMergedStream->waitTimeTf_ = static_cast<double>(mergedStreamsParams[i]["wait_time_tf"]);
      newMergedStream->useFixedFrameTf_ = static_cast<bool>(mergedStreamsParams[i]["use_fixed_frame_tf"]);
      newMergedStream->fixedFrameTf_ = static_cast<std::string>(mergedStreamsParams[i]["fixed_frame_tf"]);
      newMergedStream->latch_ = static_cast<bool>(mergedStreamsParams[i]["latch"]);
      newMergedStream->outputTopic_ = static_cast<std::string>(mergedStreamsParams[i]["output_topic"]);
      newMergedStream->applyFilters_ = static_cast<bool>(mergedStreamsParams[i]["filter"]);
      newMergedStream->filtersConfigFilename_ = static_cast<std::string>(mergedStreamsParams[i]["filters_config"]);

      MELO_INFO_STREAM("Merged stream " << newMergedStream->id_ << " - '" << newMergedStream->streamName_ << "' with master <"
                                        << cloudStreams_[newMergedStream->masterStreamId_]->sourceName_ << ">, max time offset "
                                        << newMergedStream->maxTimeOffset_ << "s and decimation factor of "
                                        << newMergedStream->cloudDecimation_);

      for (int j = 0; j < mergedStreamsParams[i]["sources"].size(); j++) {
        auto streamId = static_cast<int>(mergedStreamsParams[i]["sources"][j]);
        if (streamId == newMergedStream->masterStreamId_) {
          MELO_INFO_STREAM("  [MASTER] Source " << j << " - '" << cloudStreams_[streamId]->sourceName_ << "'");
        } else {
          MELO_INFO_STREAM("           Source " << j << " - '" << cloudStreams_[streamId]->sourceName_ << "'");
        }
        newMergedStream->cloudStreams_.push_back(cloudStreams_[streamId]);
      }

      mergedCloudStreams_.push_back(newMergedStream);
    }
  }

  return success;
}

bool PointCloudProcessor::transformCloudToMergedFrame(const MergedCloudStream* mergedStream, const ros::Time& targetStamp,
                                                      PmStampedPointCloud& pmStampedCloud) {
  const std::string& sourceFrame = pmStampedCloud.header_.frame_id;
  const ros::Time& sourceStamp = pmStampedCloud.header_.stamp;
  if (!mergedStream->targetFrame_.empty() && !pmStampedCloud.isEmpty()) {
    geometry_msgs::TransformStamped tfSourceToTargetRos;
    if (mergedStream->useFixedFrameTf_) {
      if (!tfBuffer_.canTransform(mergedStream->targetFrame_, targetStamp, sourceFrame, sourceStamp, mergedStream->fixedFrameTf_,
                                  ros::Duration(mergedStream->waitTimeTf_))) {
        MELO_WARN_THROTTLE_STREAM(
            5, "Requested transform from frames \'" + sourceFrame + "' to '" + mergedStream->targetFrame_ + "', with target timestamp "
                   << targetStamp << " cannot be found. (Warning is throttled: 5s)");
        return false;
      }

      // Lookup tf from source frame to target frame, using an intermediate fixed frame
      tfSourceToTargetRos = tfBuffer_.lookupTransform(mergedStream->targetFrame_, targetStamp, sourceFrame, sourceStamp,
                                                      mergedStream->fixedFrameTf_, ros::Duration(mergedStream->waitTimeTf_));

    } else {
      if (!tfBuffer_.canTransform(mergedStream->targetFrame_, sourceFrame, targetStamp, ros::Duration(mergedStream->waitTimeTf_))) {
        MELO_WARN_THROTTLE_STREAM(
            5, "Requested transform from frames \'" + sourceFrame + "' to '" + mergedStream->targetFrame_ + "', with target timestamp "
                   << targetStamp << " cannot be found.  (Warning is throttled: 5s)");
        return false;
      }

      // Lookup tf from source frame to target frame
      tfSourceToTargetRos =
          tfBuffer_.lookupTransform(mergedStream->targetFrame_, sourceFrame, targetStamp, ros::Duration(mergedStream->waitTimeTf_));
    }
    // Transform from source frame to target frame
    PointMatcher_ros::PmTf tfSourceToTarget = PointMatcher_ros::PmTf::FromRosTfMsg(tfSourceToTargetRos);
    pmStampedCloud.transform(tfSourceToTarget);

    pmStampedCloud.header_.stamp = targetStamp;
    return true;
  }
  return false;
}

void PointCloudProcessor::generateAndPublishMergedCloud(const int& mergedStreamId) {
  // Get start time.
  const auto tStart = std::chrono::high_resolution_clock::now();

  // Placeholders for master and merged streams data.
  MergedCloudStream* mergedStream = mergedCloudStreams_[mergedStreamId].get();
  const int& masterStreamId = mergedStream->masterStreamId_;
  CloudStream* masterStream = cloudStreams_[masterStreamId].get();
  const ros::Time& masterStreamStamp = masterStream->latestCloud_.header_.stamp;

  // Add master stream points to merged cloud
  PmStampedPointCloud mergedCloud;
  {
    std::lock_guard<std::mutex> lock(masterStream->latestCloudMutex_);
    mergedCloud = masterStream->getProcessedCloud();
  }

  // Transform master cloud from source frame to target
  if (!transformCloudToMergedFrame(mergedStream, masterStreamStamp, mergedCloud)) {
    return;
  }

  // It is assumed that this function is run after a master cloud becomes available. Hence, counter starts at 1
  size_t numStreamsMerged = 1;
  for (auto& slaveStream : mergedStream->cloudStreams_) {
    if (slaveStream->id_ == masterStreamId) {
      // Ignore master, as it has already been processed
      continue;
    }

    std::lock_guard<std::mutex> lock(slaveStream->latestCloudMutex_);
    const auto& slaveStreamStamp = slaveStream->latestCloud_.header_.stamp;
    const double timeOffset = std::abs((masterStreamStamp - slaveStreamStamp).toSec());

    // If the cloud is not too old, proceed with filtering & merging
    if (timeOffset <= mergedStream->maxTimeOffset_) {
      PmStampedPointCloud& slaveCloud = slaveStream->getProcessedCloud();

      // Transform slave stream cloud to merged stream target frame
      if (transformCloudToMergedFrame(mergedStream, masterStreamStamp, slaveCloud)) {
        // Optionally, keep intensity descriptor if this is desired by the user.
        if (mergedStream->keepIntensityDescriptor_ && mergedCloud.descriptorExists("intensity")) {
          if (!slaveCloud.descriptorExists("intensity")) {
            slaveCloud.addOneDimensionalDescriptor("intensity", -1);
          }
        }

        // Add transformed cloud to merged cloud
        if (mergedCloud.add(slaveCloud)) {
          numStreamsMerged++;
        }
      }
    }
  }

  // Filter merged cloud
  if (mergedStream->applyFilters_) {
    mergedStream->pmFiltersIface_.processInPlace(mergedCloud.dataPoints_);
  }

  // Convert from PointMatcher format to ROS msg type and publish
  sensor_msgs::PointCloud2 msgOutputCloud =
      PointMatcher_ros::pointMatcherCloudToRosMsg<float>(mergedCloud.dataPoints_, mergedStream->targetFrame_, masterStreamStamp);
  mergedStream->publisher_.publish(msgOutputCloud);

  // Notify the user if the expected number of clouds were not received or were too far apart in time.
  if (numStreamsMerged < mergedStream->cloudStreams_.size()) {
    MELO_WARN_THROTTLE_STREAM(5, "Stream " << mergedStream->streamName_ << " merged " << numStreamsMerged << "/"
                                           << mergedStream->cloudStreams_.size()
                                           << " expected clouds. Possible reasons: the input cloud topics may be "
                                              "incorrect or the max allowed time offset may be too low. (Warning is throttled: 5s)");
  }

  // Print timing statistics.
  const auto tEnd = std::chrono::high_resolution_clock::now();
  const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(tEnd - tStart).count();
  MELO_DEBUG_STREAM("Generation of cloud for merged stream '" << mergedStream->streamName_ << "' took " << duration << " milliseconds");
}

void PointCloudProcessor::cloudTopicCallback(const sensor_msgs::PointCloud2::ConstPtr& msgInputCloud, const size_t idStream) {
  CloudStream* cloudStream = cloudStreams_[idStream].get();

  if (!cloudStream->latestCloudMutex_.try_lock()) {
    MELO_DEBUG_THROTTLE_STREAM(
        2, "New data from stream " << cloudStream->sourceName_
                                   << " was skipped. Possible cause: filtering time >= sensor sampling period. (Warning is throttled: 2s)");
    return;
  }

  cloudStream->setPointCloud(PmStampedPointCloud::FromRosMsg(*msgInputCloud));

  // Filter input cloud.
  if (cloudStream->publisher_.getNumSubscribers() > 0 || cloudStream->publisher_.isLatched()) {
    const auto& processedCloud = cloudStream->getProcessedCloud();

    // Publish point cloud only if there are subscribers.
    const sensor_msgs::PointCloud2 msgOutputCloud = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(
        processedCloud.dataPoints_, processedCloud.header_.frame_id, processedCloud.header_.stamp);
    cloudStream->publisher_.publish(msgOutputCloud);
  }
  cloudStream->latestCloudMutex_.unlock();

  // Check if this cloud stream is the master of a merged-cloud stream.
  // If true, generate merged clouds.
  for (auto& mergedStream : mergedCloudStreams_) {
    // Count the input point clouds. Skip callback if decimation is not reached.
    if (mergedStream->masterStreamId_ == cloudStream->id_) {
      mergedStream->cloudsSkippedCounter_++;
      if (mergedStream->cloudsSkippedCounter_ < mergedStream->cloudDecimation_) {
        continue;
      }
      mergedStream->cloudsSkippedCounter_ = 0;

      if (mergedStream->publisher_.getNumSubscribers() > 0 || mergedStream->publisher_.isLatched()) {
        generateAndPublishMergedCloud(mergedStream->id_);
      }
    }
  }
}

bool PointCloudProcessor::reloadPipelineServiceCallback(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
  MELO_INFO_STREAM("Reloading pipeline description");

  // Disable ROS subscribers.
  for (auto& cloudStream : cloudStreams_) {
    cloudStream->subscriber_.shutdown();
  }

  // Update the filter parameters of each source stream.
  for (auto& cloudStream : cloudStreams_) {
    // Pointmatcher filters.
    if (cloudStream->applyFilters_) {
      response.success &=
          static_cast<uint>(cloudStream->pmFiltersIface_.readPipelineFile(configFileFolder_ + "/" + cloudStream->filtersConfigFilename_));
    }

    // Self-filter.
    if (cloudStream->applySelfFilter_) {
      cloudStream->selfFilter_->processConfigFile(configFileFolder_ + "/" + cloudStream->selfFilterConfigFilename_,
                                                  cloudStream->robotDescriptionContents_);
    }
  }

  // Update the filter parameters of each merged stream.
  for (auto& mergedStream : mergedCloudStreams_) {
    if (mergedStream->applyFilters_) {
      response.success &=
          static_cast<uint>(mergedStream->pmFiltersIface_.readPipelineFile(configFileFolder_ + "/" + mergedStream->filtersConfigFilename_));
    }
  }

  // Re-enable ROS subscribers.
  for (auto& cloudStream : cloudStreams_) {
    cloudStream->subscriber_ = nodeHandle_.subscribe<sensor_msgs::PointCloud2>(
        cloudStream->inputTopic_, cloudStream->queueSize_,
        std::bind(&PointCloudProcessor::cloudTopicCallback, this, std::placeholders::_1, cloudStream->id_), ros::VoidPtr(),
        ros::TransportHints().tcpNoDelay());
  }

  // Mark the service call as successful if it reaches this point.
  response.success = 1u;

  return true;
}

void PointCloudProcessor::externalSubscribersCheckCallback(const ros::TimerEvent& /*timerEvent*/) {
  std::unordered_map<int, bool> cloudStreamSubscriberEnableList;

  // Check if source streams have subscribers.
  for (auto& cloudStream : cloudStreams_) {
    cloudStreamSubscriberEnableList[cloudStream->id_] =
        (cloudStream->publisher_.getNumSubscribers() > 0 || cloudStream->publisher_.isLatched());
  }

  // Check if merged streams have subscribers.
  for (auto& mergedStream : mergedCloudStreams_) {
    const bool enableSubscribers = (mergedStream->publisher_.getNumSubscribers() > 0 || mergedStream->publisher_.isLatched());

    for (auto& cloudStream : mergedStream->cloudStreams_) {
      cloudStreamSubscriberEnableList[cloudStream->id_] |= enableSubscribers;
    }
  }

  // Disable unnecessary subscribers.
  for (auto& cloudStream : cloudStreams_) {
    if (cloudStreamSubscriberEnableList[cloudStream->id_]) {
      cloudStream->subscriber_ = nodeHandle_.subscribe<sensor_msgs::PointCloud2>(
          cloudStream->inputTopic_, cloudStream->queueSize_,
          std::bind(&PointCloudProcessor::cloudTopicCallback, this, std::placeholders::_1, cloudStream->id_), ros::VoidPtr(),
          ros::TransportHints().tcpNoDelay());
    } else {
      cloudStream->subscriber_.shutdown();
    }
  }
}

}  // namespace point_cloud_processor
