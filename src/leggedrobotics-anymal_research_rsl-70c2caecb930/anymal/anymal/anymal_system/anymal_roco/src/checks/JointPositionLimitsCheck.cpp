/**
 * @authors      Alexander Reske
 * @affiliation  ANYbotics AG
 * @brief        Class for checking if joint position limits are respected.
 */

#include "anymal_roco/checks/JointPositionLimitsCheck.hpp"

namespace anymal_roco {

bool JointPositionLimitsCheck::check(const RocoState& state, boost::shared_mutex& stateMutex) const {
  boost::unique_lock<boost::shared_mutex> lock(stateMutex);
  const auto currentJointPositions = state.getAnymalModelPtr()->getState().getJointPositions().toImplementation();
  lock.unlock();
  bool success = true;
  for (unsigned int i = 0; i < AD::getJointsDimension(); i++) {
    if (!jointPositionLimits_[i]->checkLimit(currentJointPositions[i])) {
      MELO_ERROR_STREAM(AD::mapKeyIdToKeyName<AD::JointEnum>(i) << " is in joint position " << currentJointPositions[i] << ", which is " << jointPositionLimits_[i]->warnMessage() << ".");
      success = false;
    }
  }
  return success;
}

} /* namespace anymal_roco */