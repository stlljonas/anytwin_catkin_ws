/**
 * @authors     Gabriel Hottiger
 * @affiliation ANYbotics
 * @brief       TF2 implementation of the transform listener abstraction.
 */

#include <tf/tf.h>

#include <any_measurements_ros/any_measurements_ros.hpp>

#include "geometry_utils_ros/TransformListenerRos.hpp"
#include "geometry_utils_ros/conversions.hpp"

namespace geometry_utils_ros {

bool TransformListenerRos::canTransform(const std::string& targetFrame, const std::string& sourceFrame, const Time& time,
                                        double timeout) const {
  return buffer_.canTransform(targetFrame, sourceFrame, any_measurements_ros::toRos(time), ros::Duration(timeout));
}

bool TransformListenerRos::getTransformation(geometry_utils::TransformStamped* transformStamped, const std::string& targetFrame,
                                             const std::string& sourceFrame, const Time& time, double timeout) const {
  // Check frames for emptiness
  if (targetFrame.empty() || sourceFrame.empty()) {
    MELO_ERROR("[TransformListenerRos]: No transform available. Source frame: '%s' Target frame: '%s'.", sourceFrame.c_str(),
               targetFrame.c_str());
    return false;
  }

  try {
    geometry_msgs::TransformStamped transformStampedMsg =
        buffer_.lookupTransform(targetFrame, sourceFrame, any_measurements_ros::toRos(time), ros::Duration(timeout));
    *transformStamped = fromRos(transformStampedMsg);
    return true;
  } catch (tf2::LookupException& e) {
    MELO_ERROR("[TransformListenerRos]: Lookup exception: %s", e.what());
  } catch (tf2::ConnectivityException& e) {
    MELO_ERROR("[TransformListenerRos]: Connectivity exception: %s", e.what());
  } catch (tf2::ExtrapolationException& e) {
    MELO_ERROR("[TransformListenerRos]: Extrapolation exception: %s", e.what());
  } catch (tf2::InvalidArgumentException& e) {
    MELO_ERROR("[TransformListenerRos]: InvalidArgument exception: %s", e.what());
  }
  return false;
}

}  // namespace geometry_utils_ros
