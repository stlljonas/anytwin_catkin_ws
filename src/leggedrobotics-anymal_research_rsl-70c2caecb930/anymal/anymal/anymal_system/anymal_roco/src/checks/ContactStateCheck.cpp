/**
 * @authors      Alexander Reske
 * @affiliation  ANYbotics AG
 * @brief        Class for checking for desired contact states (e.g. OPEN, CLOSED, SLIPPING).
 */

#include "anymal_roco/checks/ContactStateCheck.hpp"

namespace anymal_roco {

bool ContactStateCheck::check(const RocoState& state, boost::shared_mutex& stateMutex) const {
  bool success;
  std::unordered_map<std::string, ContactState> mapFootNameToContactState;
  boost::unique_lock<boost::shared_mutex> lock(stateMutex);
  for (const auto& key : anymal_description::AnymalDescription::getContactKeys()) {
    mapFootNameToContactState.insert(std::make_pair(key.getName(), state.getAnymalModelPtr()->getContactContainer()[key.getEnum()]->getState()));
  }
  lock.unlock();
  unsigned int count = std::count_if(mapFootNameToContactState.begin(), mapFootNameToContactState.end(), [&](auto map) { return map.second == desiredContactState_; });
  switch (checkMode_) {
    case CheckMode::AT_LEAST:
      success = count >= threshold_;
      break;
    case CheckMode::AT_MOST:
      success = count <= threshold_;
      break;
    case CheckMode::EQUAL:
      success = count == threshold_;
      break;
    default:
      success = false;
      break;
  }
  if (!success) {
    const std::string desiredContactStateName = anymal_description::AnymalDescription::mapKeyEnumToKeyName(desiredContactState_);
    for (const auto& map : mapFootNameToContactState) {
      if (map.second != desiredContactState_) {
        MELO_WARN("'%s' is not in desired contact state '%s', but in contact state '%s'.", map.first.c_str(), desiredContactStateName.c_str(), anymal_description::AnymalDescription::mapKeyEnumToKeyName(map.second));
      }
    }
  }
  return success;
}

} /* namespace anymal_roco */