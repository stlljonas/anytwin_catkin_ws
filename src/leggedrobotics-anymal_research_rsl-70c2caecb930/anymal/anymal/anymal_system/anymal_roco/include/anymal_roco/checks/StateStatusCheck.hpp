/**
 * @authors      Alexander Reske
 * @affiliation  ANYbotics AG
 * @brief        Class for checking the status of the state.
 */

#pragma once

// anymal_roco
#include "anymal_roco/checks/StateCheckBase.hpp"

namespace anymal_roco {

class StateStatusCheck : public StateCheckBase {

 public:
  //! Default Constructor
  StateStatusCheck() = default;

  //! Default Destructor
  ~StateStatusCheck() override = default;

  //! Check state
  bool check(const RocoState& state, boost::shared_mutex& stateMutex) const override;

};

} /* namespace anymal_roco */
