/*
 * Wheel.hpp
 *
 *  Created on: Dec 21, 2016
 *      Author: Gabriel Hottiger
 */

#pragma once

// loco
#include "loco/common/end_effectors/FootBase.hpp"
#include "loco/common/end_effectors/WheelProperties.hpp"
#include "loco/common/end_effectors/WheelStateDesired.hpp"
#include "loco/common/end_effectors/WheelStateMeasured.hpp"

namespace loco {

class Wheel : public FootBase {
 public:
  Wheel(WheelPropertiesPtr&& wheelProperties, const unsigned int indexInLimbJoints, bool contactIsAtOrigin = true);
  ~Wheel() override = default;

  const WheelStateDesired& getStateDesired(TimeInstant atTime = TimePoint::Now, EndEffectorFrame atFrame = EndEffectorEnum::Origin) const;
  const WheelStateMeasured& getStateMeasured(TimeInstant atTime = TimePoint::Now, EndEffectorFrame atFrame = EndEffectorEnum::Origin) const;
  const WheelProperties& getProperties() const;

  WheelStateDesired* getStateDesiredPtr(TimeInstant atTime = TimePoint::Now, EndEffectorFrame atFrame = EndEffectorEnum::Origin);
  WheelStateMeasured* getStateMeasuredPtr(TimeInstant atTime = TimePoint::Now, EndEffectorFrame atFrame = EndEffectorEnum::Origin);
  WheelProperties* getPropertiesPtr();

 protected:
  const unsigned int indexInLimbJoints_;
};

using WheelPtr = std::unique_ptr<Wheel>;

} /* namespace loco */
