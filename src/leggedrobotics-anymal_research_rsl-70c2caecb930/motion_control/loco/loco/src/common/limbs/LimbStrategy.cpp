/*
 * LimbStrategy.cpp
 *
 *  Created on: Jun 2, 2016
 *      Author: Dario Bellicoso
 */

// loco
#include "loco/common/limbs/LimbStrategy.hpp"

namespace loco {

LimbStrategy::LimbStrategy() : limbStrategyId_(static_cast<int>(LimbStrategyEnum::Undefined)) {
  limbStrategyEnumNameMap_.insert(std::pair<LimbStrategyEnum, std::string>(LimbStrategyEnum::ContactInvariant, "contact invariant"));
  limbStrategyEnumNameMap_.insert(std::pair<LimbStrategyEnum, std::string>(LimbStrategyEnum::ContactRecovery, "contact recovery"));
  limbStrategyEnumNameMap_.insert(std::pair<LimbStrategyEnum, std::string>(LimbStrategyEnum::Motion, "motion"));
  limbStrategyEnumNameMap_.insert(std::pair<LimbStrategyEnum, std::string>(LimbStrategyEnum::Support, "support"));
  limbStrategyEnumNameMap_.insert(std::pair<LimbStrategyEnum, std::string>(LimbStrategyEnum::Undefined, "undefined"));
}

bool LimbStrategy::initialize() {
  limbStrategyId_ = static_cast<int>(LimbStrategyEnum::Undefined);
  return true;
}

void LimbStrategy::setLimbStrategyEnum(LimbStrategyEnum limbStrategyEnum) {
  limbStrategyId_ = static_cast<int>(limbStrategyEnum);
}

LimbStrategyEnum LimbStrategy::getLimbStrategyEnum() const {
  return static_cast<LimbStrategyEnum>(limbStrategyId_);
}

const std::string& LimbStrategy::getLimbStrategyNameFromEnum(LimbStrategyEnum limbStrategyEnum) const {
  return limbStrategyEnumNameMap_.at(limbStrategyEnum);
}

const std::string& LimbStrategy::getLimbStrategyName() const {
  return limbStrategyEnumNameMap_.at(static_cast<LimbStrategyEnum>(limbStrategyId_));
}

bool LimbStrategy::addVariablesToLog(const std::string& ns) const {
  signal_logger::add(limbStrategyId_, "limbStrategy", ns);
  return true;
}

} /* namespace loco */
