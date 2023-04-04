/*
 * HandBase.hpp
 *
 *  Created on: Jun 26, 2018
 *      Author: Markus Staeuble
 */

#pragma once

// loco
#include <loco/common/end_effectors/EndEffectorBase.hpp>
#include <loco/common/end_effectors/HandStateDesired.hpp>
#include <loco/common/end_effectors/HandStateMeasured.hpp>

namespace loco {

constexpr unsigned int handNumberOfContactConstraints = 6;

class Hand : public EndEffectorBase {
 public:
  Hand(EndEffectorPropertiesPtr&& handProperties, const unsigned int numFingers, bool contactIsAtOrigin = true);
  ~Hand() override = default;

  const HandStateDesired& getStateDesired(TimeInstant atTime = TimePoint::Now, EndEffectorFrame atFrame = EndEffectorEnum::Origin) const;
  const HandStateMeasured& getStateMeasured(TimeInstant atTime = TimePoint::Now, EndEffectorFrame atFrame = EndEffectorEnum::Origin) const;
  HandStateDesired* getStateDesiredPtr(TimeInstant atTime = TimePoint::Now, EndEffectorFrame atFrame = EndEffectorEnum::Origin);
  HandStateMeasured* getStateMeasuredPtr(TimeInstant atTime = TimePoint::Now, EndEffectorFrame atFrame = EndEffectorEnum::Origin);
  unsigned int getNumFingers() const { return numFingers_; }

 protected:
  const unsigned int numFingers_;
};

using HandPtr = std::unique_ptr<Hand>;

} /* namespace loco */
