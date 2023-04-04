/*
 * LimbStrategyBase.hpp
 *
 *  Created on: Jun 2, 2016
 *      Author: Dario Bellicoso
 */

#pragma once

namespace loco {

enum class LimbStrategyEnum : int { Support = 1, ContactInvariant = 2, Motion = 3, ContactRecovery = 4, Undefined = 0 };

class LimbStrategyBase {
 public:
  LimbStrategyBase();
  virtual ~LimbStrategyBase();

  virtual bool initialize() = 0;

  virtual void setLimbStrategyEnum(LimbStrategyEnum limbStrategyEnum) = 0;
  virtual LimbStrategyEnum getLimbStrategyEnum() const = 0;
};

} /* namespace loco */
