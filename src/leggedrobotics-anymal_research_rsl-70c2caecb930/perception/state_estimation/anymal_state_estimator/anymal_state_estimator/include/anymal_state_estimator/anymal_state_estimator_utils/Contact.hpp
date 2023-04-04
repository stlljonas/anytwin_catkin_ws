/*!
 * @file    Contact.hpp
 * @author  Fabian Tresoldi
 * @date    January, 2018
 */

#pragma once

#include <basic_contact_estimation/ContactDetectorBase.hpp>

#include <any_measurements/Time.hpp>

#include <kindr/Core>

namespace anymal_state_estimator {

struct Contact {

  using ContactState = typename basic_contact_estimation::ContactDetectorBase::ContactState;

  bool flag_{false};
  bool previousFlag_{false};
  ContactState state_{ContactState::OPEN};

  any_measurements::Time stamp_;
  kindr::Position3D contactPointInOdom_;

  // time of last detected contact,
  any_measurements::Time lastContactTime_;
  // position of last detected contact point in odom
  kindr::Position3D lastContactPointInOdom_;

};

}  // namespace anymal_state_estimator
