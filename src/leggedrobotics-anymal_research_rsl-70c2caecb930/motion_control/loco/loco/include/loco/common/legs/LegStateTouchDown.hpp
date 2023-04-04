/*
 * LegStateTouchDown.hpp
 *
 *  Created on: Feb 26, 2014
 *      Author: Christian Gehring, Dario Bellicoso
 */

#pragma once

// loco
#include "loco/common/legs/LegStateBase.hpp"
#include "loco/common/typedefs.hpp"

namespace loco {

//!  State of the leg at the event of touch-down
/*!
 */
class LegStateTouchDown : public loco::LegStateBase {
 public:
  LegStateTouchDown();
  ~LegStateTouchDown() override = default;

  /*! Save foot position in world frame on touchdown event.
   * @param[in] positionWorldToFootInWorldFrame Measured foot position in world frame
   */
  void setPositionWorldToFootInWorldFrame(const loco::Position& positionWorldToFootInWorldFrame);

  /*! Get the last touchdown foot position in world frame.
   */
  const Position& getPositionWorldToFootInWorldFrame() const;

 protected:
  loco::Position positionWorldToFootInWorldFrame_;
};

} /* namespace loco */