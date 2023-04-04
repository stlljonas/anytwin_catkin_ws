/*
 * foot_placement_strategy.hpp
 *
 *  Created on: Nov 19, 2018
 *      Author: Fabian Jenelten
 */

// std_utils
#include <std_utils/std_utils.hpp>


#pragma once

namespace loco {

namespace fps {
CONSECUTIVE_ENUM(SlippageRecoveryStrategy,
                 ApproachToCurrentFoothold,
                 ApproachToPreviousFoothold,
                 ApproachToPreviousFootholdIteratively,
                 Undefined)
static std::map<SlippageRecoveryStrategy, std::string> SlippageRecoveryStrategyMap =
{{SlippageRecoveryStrategy::ApproachToCurrentFoothold,                "ApproachToCurrentFoothold"},
 {SlippageRecoveryStrategy::ApproachToPreviousFoothold,               "ApproachToPreviousFoothold"},
 {SlippageRecoveryStrategy::ApproachToPreviousFootholdIteratively,    "ApproachToPreviousFootholdIteratively"},
 {SlippageRecoveryStrategy::Undefined,                                "Undefined"},
 {SlippageRecoveryStrategy::SIZE,                                     "SIZE"}};

} /* namespace fpc */
} /* namespace loco */
