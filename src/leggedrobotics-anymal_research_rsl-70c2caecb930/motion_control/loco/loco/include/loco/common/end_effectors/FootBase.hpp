/*
 * FootBase.hpp
 *
 *  Created on: Dec 5, 2016
 *      Author: Dario Bellicoso
 */

#pragma once

// loco
#include <loco/common/end_effectors/TimePoint.hpp>
#include "loco/common/end_effectors/EndEffectorBase.hpp"
#include "loco/common/end_effectors/FootBaseStateDesired.hpp"
#include "loco/common/end_effectors/FootBaseStateMeasured.hpp"

// STL
#include <memory>

namespace loco {

class FootBase : public EndEffectorBase {
 public:
  explicit FootBase(EndEffectorPropertiesPtr&& endeffectorProperties, bool contactIsAtOrigin = true);
  ~FootBase() override = default;

  const FootBaseStateDesired& getStateDesired(TimeInstant atTime = TimePoint::Now,
                                              EndEffectorFrame atFrame = EndEffectorContactEnum::Contact) const;
  const FootBaseStateMeasured& getStateMeasured(TimeInstant atTime = TimePoint::Now,
                                                EndEffectorFrame atFrame = EndEffectorContactEnum::Contact) const;
  const EndEffectorProperties& getProperties() const;

  FootBaseStateDesired* getStateDesiredPtr(TimeInstant atTime = TimePoint::Now, EndEffectorFrame atFrame = EndEffectorContactEnum::Contact);
  FootBaseStateMeasured* getStateMeasuredPtr(TimeInstant atTime = TimePoint::Now,
                                             EndEffectorFrame atFrame = EndEffectorContactEnum::Contact);
  EndEffectorProperties* getPropertiesPtr();
};

using FootBasePtr = std::unique_ptr<EndEffectorBase>;

} /* namespace loco */
