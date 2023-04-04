/**
 * @authors      Alexander Reske
 * @affiliation  ANYbotics AG
 * @brief        Class for handling (i.e. adding, performing, erasing) state checks.
 */

#include "anymal_roco/StateChecker.hpp"

namespace anymal_roco {

bool StateChecker::performChecks(const RocoState& state, boost::shared_mutex& stateMutex) const {
  bool isOk = true;
  for (const auto& check : checks_) {
    if (!check.second->check(state, stateMutex)) {
      MELO_WARN("Check '%s' did not pass.", check.first.c_str());
      isOk = false;
    }
  }
  return isOk;
}

bool StateChecker::addCheck(const std::string& name, const StateCheckBaseConstPtr& check) {
  std::pair<std::unordered_map<std::string, StateCheckBaseConstPtr>::iterator, bool> result = checks_.insert(std::make_pair(name, check));
  bool success = result.second;
  if (!success) {
    MELO_WARN("Could not insert check with name '%s', since a check with the same name already exists!", name.c_str());
    return false;
  }
}

bool StateChecker::removeCheck(const std::string& name) {
  std::unordered_map<std::string, StateCheckBaseConstPtr>::size_type result = checks_.erase(name);
  bool success = static_cast<bool>(result);
  if (!success) {
    MELO_WARN("Could not remove check with name '%s', since a check with that name does not exist!", name.c_str());
    return false;
  }
  return true;
}

const std::vector<std::string> StateChecker::getChecks() const {
  std::vector<std::string> names;
  names.reserve(checks_.size());
  for (const auto& check : checks_) {
    names.push_back(check.first);
  }
  return names;
}

} /* namespace anymal_roco */
