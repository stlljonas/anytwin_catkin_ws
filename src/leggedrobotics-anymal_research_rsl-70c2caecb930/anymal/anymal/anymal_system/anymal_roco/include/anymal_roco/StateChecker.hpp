/**
 * @authors      Alexander Reske
 * @affiliation  ANYbotics AG
 * @brief        Class for handling (i.e. adding, performing, erasing) state checks.
 */

#pragma once

// stl
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// anymal_roco
#include "anymal_roco/checks/StateCheckBase.hpp"

namespace anymal_roco {

 class StateChecker {

 public:
   using StateCheckBaseConstPtr = std::shared_ptr<const anymal_roco::StateCheckBase>;

   //! Default Constructor
   StateChecker() = default;

   //! Default Destructor
   ~StateChecker() = default;

   //! Perform all checks for the state
   bool performChecks(const RocoState& state, boost::shared_mutex& stateMutex) const;

   //! Add a check
   bool addCheck(const std::string& name, const StateCheckBaseConstPtr& check);

   //! Erase a check
   bool removeCheck(const std::string& name);

   //! Get the names of all the checks that are currently added
   const std::vector<std::string> getChecks() const;

 private:
   std::unordered_map<std::string, StateCheckBaseConstPtr> checks_;

};

} /* namespace anymal_roco */