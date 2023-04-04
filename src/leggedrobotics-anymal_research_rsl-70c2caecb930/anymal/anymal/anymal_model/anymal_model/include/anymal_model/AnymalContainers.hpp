#pragma once

// anymal model
#include "anymal_model/ExtendedAnymalState.hpp"
#include "anymal_model/actuator_containers.hpp"

// romo
#include <romo/common/RobotContainers.hpp>

// any measurements
#include <any_measurements/Imu.hpp>

namespace anymal_model {

struct AnymalContainersImpl {
  struct RobotState {
    using type = ExtendedAnymalState;
  };

  struct ActuatorReadings {
    using type = ActuatorReadingRobotContainer;
  };

  struct ActuatorCommands {
    using type = ActuatorCommandRobotContainer;
  };

  struct Imu {
    using type = any_measurements::Imu;
  };
};

using AnymalContainers = romo::RobotContainers<AnymalContainersImpl>;

}  // namespace anymal_model
