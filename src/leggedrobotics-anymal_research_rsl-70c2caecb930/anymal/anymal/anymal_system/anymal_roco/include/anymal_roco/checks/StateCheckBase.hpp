/**
 * @authors      Alexander Reske
 * @affiliation  ANYbotics AG
 * @brief        Base class for all state checks. Derive from this class to implement a check.
 */

#pragma once

// boost
#include <boost/thread/pthread/shared_mutex.hpp>

// anymal_roco
#include "anymal_roco/RocoState.hpp"

namespace anymal_roco {

class StateCheckBase {

 public:
  //! Default Constructor
  StateCheckBase() = default;

  //! Default Destructor
  virtual ~StateCheckBase() = default;

  //! Check state
  virtual bool check(const RocoState& /*state*/, boost::shared_mutex& /*stateMutex*/) const = 0;

};

} /* namespace anymal_roco */
