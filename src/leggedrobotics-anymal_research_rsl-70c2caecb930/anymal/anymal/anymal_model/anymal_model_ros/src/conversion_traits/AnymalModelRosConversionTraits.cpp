/*!
 * @file     AnymalModelRosConversionTraits.cpp
 * @author   Peter Fankhauser, Dario Bellicoso
 * @date     Dec, 2016
 */

// anymal model ros
#include "anymal_model_ros/conversion_traits/AnymalModelRosConversionTraits.hpp"
#include "anymal_model_ros/conversions.hpp"
#include "anymal_model_ros/typedefs.hpp"

// kindr ros
#include <kindr_ros/kindr_ros.hpp>

namespace anymal_model_ros {
namespace conversion_traits {

void AnymalModelRosConversionTraits::convert(const RosType& ros, ObjectType& object) {
  anymal_model::AnymalState state;
  fromRos(ros, state);

  // Joints.
  object.setJointTorques(
      anymal_model::JointTorques(Eigen::Map<const anymal_model::JointTorques::Implementation>(ros.joints.effort.data())));

  // Contacts.
  for (const auto contactKey : AD::getContactKeys()) {
    const auto contactEnum = contactKey.getEnum();
    const auto contactId = contactKey.getId();

    anymal_model::Vector normal;
    kindr_ros::convertFromRosGeometryMsg(ros.contacts[contactId].normal, normal);
    object.getContactContainer()[contactEnum]->setNormal(normal, ObjectType::CoordinateFrameEnum::WORLD);

    anymal_model::Force force;
    kindr_ros::convertFromRosGeometryMsg(ros.contacts[contactId].wrench.force, force);
    object.getContactContainer()[contactEnum]->setForce(force, ObjectType::CoordinateFrameEnum::WORLD);

    if (ros.contacts[contactId].state == ros.contacts[contactId].STATE_CLOSED) {
      object.getContactContainer()[contactEnum]->setState(ObjectType::ContactState::CLOSED);
    } else if (ros.contacts[contactId].state == ros.contacts[contactId].STATE_SLIPPING) {
      object.getContactContainer()[contactEnum]->setState(ObjectType::ContactState::SLIPPING);
    } else if (ros.contacts[contactId].state == ros.contacts[contactId].STATE_OPEN) {
      object.getContactContainer()[contactEnum]->setState(ObjectType::ContactState::OPEN);
    } else {
      throw std::range_error("Contact state is not supported!");
    }
  }

  // Set state in object.
  object.setState(state, true, true, false);
}

}  // namespace conversion_traits
}  // namespace anymal_model_ros
