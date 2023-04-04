/**
 * @authors      Alexander Reske
 * @affiliation  ANYbotics AG
 * @brief        Class for checking the status of the state.
 */

#include "anymal_roco/checks/StateStatusCheck.hpp"

namespace anymal_roco {

bool StateStatusCheck::check(const RocoState& state, boost::shared_mutex& stateMutex) const {
  boost::unique_lock<boost::shared_mutex> lock(stateMutex);
  const auto status = state.getStatus();
  lock.unlock();
  switch (status) {
    case RocoState::StateStatus::STATUS_OK:
      return true;
      break;
    case RocoState::StateStatus::STATUS_ERROR_SENSOR:
      MELO_WARN("State includes a sensor error!");
      return false;
      break;
    case RocoState::StateStatus::STATUS_ERROR_ESTIMATOR:
      MELO_WARN("State includes an estimator error!");
      return false;
      break;
    case RocoState::StateStatus::STATUS_ERROR_UNKNOWN:
      MELO_WARN("State is unknown!");
      return false;
      break;
    default:
      MELO_WARN("Status of state is undefined (%d)!", static_cast<int>(status));
      return false;
      break;
  }
}

} /* namespace anymal_roco */