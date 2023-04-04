/*!
 * @file     AnymalContainersRos.hpp
 * @author   Markus Staeuble
 * @date     Mar, 2018
 */

#pragma once

// anymal model ros
#include "anymal_model_ros/conversions.hpp"

// anymal model
#include <anymal_model/AnymalContainers.hpp>

// any measurements ros
#include <any_measurements_ros/any_measurements_ros.hpp>

// romo
#include <romo/common/RobotContainersRos.hpp>

// anymal msgs
#include <anymal_msgs/AnymalState.h>

// series elastic actuator msgs
#include <series_elastic_actuator_msgs/SeActuatorCommands.h>
#include <series_elastic_actuator_msgs/SeActuatorReadings.h>

// sensor msgs
#include <sensor_msgs/Imu.h>

namespace anymal_model_ros {
/**
 * @brief Static base class for all robot descriptions:
 */
struct AnymalContainersRosImpl {
  //! Delete constructor, class consists only of static members
  AnymalContainersRosImpl() = delete;

  //! Expose Msg Types
  struct RobotStateRos {
    using msgType = anymal_msgs::AnymalState;
    template <typename RobotState_, typename RobotStateRos_>
    using ConversionTrait = conversion_traits::ConversionTraits<RobotState_, RobotStateRos_>;
  };

  struct ActuatorReadingsRos {
    using msgType = series_elastic_actuator_msgs::SeActuatorReadings;
    template <typename ActuatorReadings_, typename ActuatorReadingsRos_>
    using ConversionTrait = conversion_traits::ConversionTraits<ActuatorReadings_, ActuatorReadingsRos_>;
  };

  struct ActuatorCommandsRos {
    using msgType = series_elastic_actuator_msgs::SeActuatorCommands;
    template <typename ActuatorCommands_, typename ActuatorCommandsRos_>
    using ConversionTrait = conversion_traits::ConversionTraits<ActuatorCommands_, ActuatorCommandsRos_>;
  };

  struct ImuRos {
    using msgType = sensor_msgs::Imu;
    template <typename Imu_, typename ImuRos_>
    using ConversionTrait = any_measurements_ros::ConversionTraits<Imu_, ImuRos_>;
  };
};

using AnymalContainersRos = romo::RobotContainersRos<anymal_model::AnymalContainers, AnymalContainersRosImpl>;

}  // namespace anymal_model_ros
