#pragma once

// anymal model
#include "anymal_model/AnymalState.hpp"
#include "anymal_model/StateStatus.hpp"
#include "anymal_model/contact_force_calibrator_containers.hpp"

// anymal description
#include <anymal_description/AnymalDescription.hpp>

// any measurements
#include <any_measurements/PointContact.hpp>
#include <any_measurements/Time.hpp>

// std utils
#include <std_utils/containers/EnumArray.hpp>

namespace anymal_model {

struct ExtendedAnymalState {
  any_measurements::Time time_;
  StateStatus status_;
  AnymalState anymalState_;
  std_utils::EnumArray<typename AD::ContactEnum, any_measurements::PointContact> contacts_;
  ContactForceCalibratorStatsContainer forceCalibratorStats_;
};

}  // namespace anymal_model
