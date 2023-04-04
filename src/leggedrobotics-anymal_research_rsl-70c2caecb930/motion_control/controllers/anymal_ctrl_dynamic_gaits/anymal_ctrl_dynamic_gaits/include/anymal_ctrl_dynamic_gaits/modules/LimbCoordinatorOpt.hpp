/*!
* @file     LimbCoordinatorOpt.hpp
* @author   Dario Bellicoso
* @date     Feb, 2014
* @version  1.0
* @ingroup
* @brief
*/

#pragma once

// loco.
#include "loco/limb_coordinator/LimbCoordinatorBase.hpp"
#include "loco/common/WholeBody.hpp"
#include "loco/gait_pattern/contact_schedule_anymal.hpp"

namespace loco {

class LimbCoordinatorOpt: public LimbCoordinatorBase {
 public:
  explicit LimbCoordinatorOpt(WholeBody& wholeBody);
  ~LimbCoordinatorOpt() override = default;

  bool addVariablesToLog(const std::string & ns = "") const override;
  bool initialize(double dt) override;
  bool advance(double dt) override;
  bool loadParameters(const TiXmlHandle& handle) override;

  GaitPatternBase* getGaitPatternPtr() override;

 private:
  std::ostringstream getContactScheduleInfo();

  //! A reference to the legs.
  Legs& legs_;

  //! If true, information is displayed to the terminal
  bool verbose_;

  //! Rate, the information is displayed to the terminal.
  double logRate_;

  //! Previous stance foot-location.
  std_utils::EnumArray<contact_schedule::LegEnumAnymal, Position> positionWorldToLastOrCurrentContactInWorldFrame_;
};

} /* namespace loco */
