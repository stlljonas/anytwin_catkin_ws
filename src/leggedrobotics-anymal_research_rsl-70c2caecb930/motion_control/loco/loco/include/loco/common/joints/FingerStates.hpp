/*
 * FingerStateBase.hpp
 *
 *  Created on: Jun, 2018
 *      Author: Markus Staeuble
 */

#pragma once

#include "loco/common/typedefs.hpp"

namespace loco {

class FingerStates {
 public:
  FingerStates() = delete;
  explicit FingerStates(unsigned int numFingers);
  ~FingerStates() = default;

  const JointPositions& getFingerPositions() const;
  void setFingerPositions(const JointPositions& fingerPositions);
  void setFingerPosition(const unsigned int index, const JointPosition& fingerPosition);

  const JointPositions& getFingerVelocities() const;
  void setFingerVelocities(const JointPositions& fingerVelocities);
  void setFingerVelocity(const unsigned int index, const JointPosition& fingerVelocity);

 protected:
  const unsigned int numFingers_;
  //! The vector of finger positions (between 0 and 1).
  JointPositions fingerPositions_;
  //! The vector of finger velocities (between -1 and 1).
  JointVelocities fingerVelocities_;
};

} /* namespace loco */
