/*
 * FootBaseStateBase.hpp
 *
 *  Created on: Nov, 2017
 *      Author: Gabriel Hottiger
 */

#pragma once

// loco
#include <loco/common/typedefs.hpp>

namespace loco {

class FootBaseStateBase {
 public:
  FootBaseStateBase() = default;
  virtual ~FootBaseStateBase() = default;

  const Position& getPositionWorldToFootholdInWorldFrame() const { return positionWorldToFootholdInWorldFrame_; }

  void setPositionWorldToFootholdInWorldFrame(const Position& positionWorldToDesiredFootholdInWorldFrame) {
    positionWorldToFootholdInWorldFrame_ = positionWorldToDesiredFootholdInWorldFrame;
  }

 protected:
  Position positionWorldToFootholdInWorldFrame_;
};

} /* namespace loco */
