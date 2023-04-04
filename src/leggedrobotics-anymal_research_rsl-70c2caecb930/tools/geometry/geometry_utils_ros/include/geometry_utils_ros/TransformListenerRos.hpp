/**
 * @authors     Gabriel Hottiger
 * @affiliation ANYbotics
 * @brief       TF2 implementation of the transform listener abstraction.
 */

#pragma once

#include <tf2_ros/transform_listener.h>

#include <any_measurements_ros/any_measurements_ros.hpp>
#include <geometry_utils/TransformListener.hpp>

#include "geometry_utils_ros/typedefs.hpp"

namespace geometry_utils_ros {

class TransformListenerRos : public geometry_utils::TransformListener {
 public:
  //! Constructor
  TransformListenerRos() : geometry_utils::TransformListener(), buffer_{}, listener_{buffer_} {}

  //! Default destructor
  ~TransformListenerRos() override = default;

  //! @copydoc geometry_utils::TransformListener::getCurrentTime
  Time getCurrentTime() const override { return any_measurements_ros::fromRos(ros::Time::now()); }

  /**
   * Checks if transformation to transform from sourceFrame to targetFrame is available.
   * @param targetFrame       Target frame.
   * @param sourceFrame       Source frame.
   * @param time              Time at which transform shall be queried (0.0 will provide latest transform).
   * @param timeout           Wait for timeout seconds to see if transform is possible.
   * @return                  True, if transform is available.
   */
  bool canTransform(const std::string& targetFrame, const std::string& sourceFrame, const Time& time, double timeout) const override;

  /**
   * Get transformation to transform entity from sourceFrame(childFrame_ in Transformstamped) to targetFrame(frame_ in Transformstamped).
   * @param transformStamped  Transform from sourceFrame to targetFrame.
   * @param targetFrame       Target frame (transform is expressed in this frame).
   * @param sourceFrame       Source frame.
   * @param time              Time at which transform shall be queried (0.0 will provide latest transform).
   * @param timeout           Wait for timeout seconds to get transform.
   * @return                  True, if valid transform was set.
   */
  bool getTransformation(TransformStamped* transformStamped, const std::string& targetFrame, const std::string& sourceFrame,
                         const Time& time, double timeout) const override;

 private:
  //! TF Buffer
  tf2_ros::Buffer buffer_;
  //! TF Transform listener
  tf2_ros::TransformListener listener_;
};

}  // namespace geometry_utils_ros
