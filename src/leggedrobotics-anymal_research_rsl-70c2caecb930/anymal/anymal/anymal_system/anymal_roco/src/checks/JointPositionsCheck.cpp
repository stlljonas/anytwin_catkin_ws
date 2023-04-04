/**
 * @authors      Alexander Reske
 * @affiliation  ANYbotics AG
 * @brief        Class for checking for desired joint positions with some error tolerance.
 */

#include "anymal_roco/checks/JointPositionsCheck.hpp"

namespace anymal_roco {

bool JointPositionsCheck::check(const RocoState& state, boost::shared_mutex& stateMutex) const {
  boost::unique_lock<boost::shared_mutex> lock(stateMutex);
  const auto& currentJointPositions = state.getAnymalModelPtr()->getState().getJointPositions().toImplementation();
  JointVector jointPositionErrors = (currentJointPositions - desiredJointPositions_);
  lock.unlock();
  const auto result = jointPositionErrors.cwiseAbs().array() > allowedAbsoluteJointPositionErrors_.cwiseAbs().array();
  bool success = result.sum() == 0;
  if (!success) {
    for (const auto& key : anymal_description::AnymalDescription::getJointKeys()) {
      if (result[key.getId()]) {
        MELO_WARN("'%s' is not in the desired joint position, the error is %f and its absolute value exceeds the tolerance given as %f.", anymal_description::AnymalDescription::mapKeyEnumToKeyName(key.getEnum()), jointPositionErrors[key.getId()], allowedAbsoluteJointPositionErrors_.cwiseAbs()[key.getId()]);
      }
    }
  }
  return success;
}

} /* namespace anymal_roco */