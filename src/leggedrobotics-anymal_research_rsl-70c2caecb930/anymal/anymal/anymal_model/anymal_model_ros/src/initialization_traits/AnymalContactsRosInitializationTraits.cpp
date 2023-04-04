/*!
 * @file     AnymalContactsRosInitializationTraits.cpp
 * @author   Remo Diethelm
 * @date     Jul, 2019
 */

// anymal model ros
#include "anymal_model_ros/initialization_traits/AnymalContactsRosInitializationTraits.hpp"
#include "anymal_model_ros/typedefs.hpp"

namespace anymal_model_ros {
namespace initialization_traits {

void AnymalContactsRosInitializationTraits::initialize(RosType& ros) {
  ros.resize(AD::getContactKeys().size());
  for (const auto& key : AD::getContactKeys()) {
    const auto contactId = key.getId();
    ros[contactId].header.frame_id = "odom";
    ros[contactId].frictionCoefficient = 0.8;
    ros[contactId].restitutionCoefficient = 0.0;
    ros[contactId].state = anymal_msgs::Contact::STATE_OPEN;
    ros[contactId].name = key.getName();
  }
}

}  // namespace initialization_traits
}  // namespace anymal_model_ros
