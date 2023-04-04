/*
 * MissionControlParameterHandler.hpp
 *
 *  Created on: Mar 15, 2018
 *      Author: Fabian Jenelten
 */
#pragma once

// motion generation
#include "motion_generation_utils/ParameterHandler.hpp"


namespace loco {
namespace mission_control_zmp {

struct MissionControlParams {

  explicit MissionControlParams() :
    maxHeadinVel_(0.0),
    maxLateralVel_(0.0),
    maxTurningVel_(0.0)
  {
  }

  //! Max heading velocity.
  double maxHeadinVel_;

  //! Max lateral velocity.
  double maxLateralVel_;

  //! max turning velocity.
  double maxTurningVel_;
};


class MissionControlParameterHandler : public motion_generation::ParameterHandler<MissionControlParams> {
public:
  MissionControlParameterHandler() : ParameterHandler() { }
  ~MissionControlParameterHandler() override = default;

  bool loadParameters(
      const TiXmlHandle& gaitHandle,
      unsigned int gaitIndex,
      const std::string& gaitPatternTypeStr) override {
    emplace_params(gaitIndex, gaitPatternTypeStr);

    TiXmlHandle maxHandle = gaitHandle;
    if(!tinyxml_tools::getChildHandle(maxHandle, gaitHandle, "Maximum")) { return false; }
    if(!tinyxml_tools::loadParameter(params_.back().maxHeadinVel_,  maxHandle,  "headingSpeed")) { return false; }
    if(!tinyxml_tools::loadParameter(params_.back().maxLateralVel_, maxHandle,  "lateralSpeed")) { return false; }
    if(!tinyxml_tools::loadParameter(params_.back().maxTurningVel_, maxHandle,  "turningSpeed")) { return false; }

    return true;
  }

};



} /* foothold_generator */
} /* namespace loco */
