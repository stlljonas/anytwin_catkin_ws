/*
 * CloudStream.cpp
 *
 *  Author: Yoshua Nava
 *  Organization: ANYbotics AG
 *  License: Proprietary
 */

#include "point_cloud_processor/CloudStream.hpp"

// message logger
#include <message_logger/message_logger.hpp>

namespace point_cloud_processor {

CloudStream::CloudStream()
    : id_(-1),
      sourceName_("source"),
      sourceType_("range"),
      queueSize_(1),
      latch_(false),
      inputTopic_("/range_sensor/points"),
      outputTopic_("/range_sensor/points_filtered"),
      applyFilters_(false),
      filtersAppliedToLatestData_(false),
      filtersConfigFilename_(""),
      applySelfFilter_(false),
      selfFilterConfigFilename_(""),
      robotDescriptionContents_("") {}

void CloudStream::setPointCloud(const PmStampedPointCloud& pointCloud) {
  latestCloud_ = pointCloud;
  filtersAppliedToLatestData_.exchange(false);
}

PmStampedPointCloud& CloudStream::getProcessedCloud() {
  processCloud();
  return latestCloud_;
}

void CloudStream::processCloud() {
  // If filters are to be applied, they'll be applied only once.
  if (applyFilters_ && !filtersAppliedToLatestData_.load()) {
    const auto tStart = std::chrono::high_resolution_clock::now();
    pmFiltersIface_.processInPlace(latestCloud_.dataPoints_);

    const auto sfStart = std::chrono::high_resolution_clock::now();
    // TODO (jburklund) applySelfFilter_ assumes pointer is valid, and controls use of pointer
    if (applySelfFilter_) {
      selfFilter_->processInPlace(latestCloud_);
    }
    const auto tEnd = std::chrono::high_resolution_clock::now();

    // TODO (jburklund) move this to benchmarking only, and remove when building for production. See ethz-asl laser_slam benchmarker
    MELO_DEBUG_STREAM("Filtering of cloud from stream '" << sourceName_ << "' took "
                                                         << std::chrono::duration_cast<std::chrono::milliseconds>(tEnd - tStart).count()
                                                         << " milliseconds");
    MELO_DEBUG_STREAM(" Prefilter time: " << std::chrono::duration_cast<std::chrono::microseconds>(sfStart - tStart).count()
                                          << " microseconds");
    MELO_DEBUG_STREAM(" Self filter time: " << std::chrono::duration_cast<std::chrono::microseconds>(tEnd - sfStart).count()
                                            << " microseconds");
    filtersAppliedToLatestData_.exchange(true);
  }
}

}  // namespace point_cloud_processor
