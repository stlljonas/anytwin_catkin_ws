/*
 * LimbStrategy.hpp
 *
 *  Created on: Jun 2, 2016
 *      Author: Dario Bellicoso
 */

#pragma once

// loco
#include "loco/common/limbs/LimbStrategyBase.hpp"

// signal logger
#include "signal_logger/signal_logger.hpp"

// stl
#include <map>
#include <memory>

namespace loco {

class LimbStrategy : public LimbStrategyBase {
 public:
  LimbStrategy();
  ~LimbStrategy() override = default;

  bool initialize() override;

  void setLimbStrategyEnum(LimbStrategyEnum limbStrategyEnum) override;
  LimbStrategyEnum getLimbStrategyEnum() const override;

  virtual const std::string& getLimbStrategyNameFromEnum(LimbStrategyEnum limbStrategyEnum) const;
  virtual const std::string& getLimbStrategyName() const;

  virtual bool addVariablesToLog(const std::string& ns) const;

 private:
  int limbStrategyId_;
  std::map<LimbStrategyEnum, std::string> limbStrategyEnumNameMap_;
};

using LimbStrategyPtr = std::unique_ptr<LimbStrategy>;

} /* namespace loco */
