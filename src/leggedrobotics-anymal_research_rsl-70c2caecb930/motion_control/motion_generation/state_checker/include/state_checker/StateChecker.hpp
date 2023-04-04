/*
 * StateChecker.hpp
 *
 *  Created on: Aug 08, 2019
 *      Author: Fabian Jenelten
 */

#pragma once

// motion_generation_utils.
#include "motion_generation_utils/StateCheckerBase.hpp"

// loco.
#include "loco/common/TerrainModelBase.hpp"
#include "loco/common/WholeBody.hpp"

// state checker.
#include "state_checker/indicators/IndicatorContainer.hpp"

// std utils.
#include "std_utils/std_utils.hpp"

class TiXmlHandle;

namespace loco {
namespace state_checker {

class StateChecker : public StateCheckerBase {
 public:
  explicit StateChecker(WholeBody& wholeBody, TerrainModelBase& terrain);
  ~StateChecker() override = default;

  bool initialize(double dt) override;
  bool loadParameters(const TiXmlHandle& handle) override;
  bool advance(double dt) override;

 protected:
  //! A reference to the whole body.
  WholeBody& wholeBody_;

  //! A reference to the terrain.
  TerrainModelBase& terrain_;

  //! Indicators. Can return an indicator value between 0 (stable) to 1 (something went wrong).
  IndicatorContainer indicatorContainer_;

  //! Timer used to transform indicator value into boolean (representing safe or unsafe).
  std_utils::HighResolutionClockTimer timer_;

  //! Helper variable used to trigger timer.
  bool didPinTimer_;

  //! If indicator value is larger than this value, unsafe mode is triggered.
  double indicatorThreshold_;

  //! Unsafe mode will be released after this amount of time has elapsed.
  double timeThreshold_;
};

} /* namespace state_checker */
} /* namespace loco */
