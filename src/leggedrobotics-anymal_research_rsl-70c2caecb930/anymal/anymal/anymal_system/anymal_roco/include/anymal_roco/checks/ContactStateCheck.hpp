/**
 * @authors      Alexander Reske
 * @affiliation  ANYbotics AG
 * @brief        Class for checking for desired contact states (e.g. OPEN, CLOSED, SLIPPING).
 */

#pragma once

// stl
#include <string>
#include <unordered_map>

// anymal description
#include <anymal_description/AnymalDescription.hpp>

// anymal_roco
#include "anymal_roco/checks/StateCheckBase.hpp"

namespace anymal_roco {

class ContactStateCheck : public StateCheckBase {

 public:
  using ContactState = anymal_description::AnymalDescription::ContactStateEnum;

  // Data types
  enum class CheckMode { AT_LEAST, AT_MOST, EQUAL };

  //! Constructor
  ContactStateCheck(ContactState desiredContactState, CheckMode checkMode, unsigned int threshold) : desiredContactState_(desiredContactState), checkMode_(checkMode), threshold_(threshold) {};

  //! Default Destructor
  ~ContactStateCheck() override = default;

  //! Check state
  bool check(const RocoState& state, boost::shared_mutex& stateMutex) const override;

 private:
  ContactState desiredContactState_;
  CheckMode checkMode_;
  unsigned int threshold_;

};

} /* namespace anymal_roco */
