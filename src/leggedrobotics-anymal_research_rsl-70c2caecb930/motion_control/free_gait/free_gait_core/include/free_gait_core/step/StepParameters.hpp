/*
 * StepParameters.hpp
 *
 *  Created on: Feb 18, 2016
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// free_gait_core
#include "free_gait_core/TypeDefs.hpp"
#include "free_gait_core/step/StepCompleter.hpp"

// tinyxml tools
#include "tinyxml_tools/tinyxml_tools.hpp"

namespace free_gait {

class StepParameters
{
 public:
  StepParameters() = default;
  virtual ~StepParameters() = default;

  virtual bool loadParameters(const TiXmlHandle& handle) {

    using tinyxml_tools::getChildHandle;
    using tinyxml_tools::loadParameter;

    TiXmlHandle hFootstep(handle);
    if(!getChildHandle(hFootstep, handle, "Footstep")) { return false; }
    if(!loadParameter(footstepParameters.profileType, hFootstep, "profileType")) { return false; }
    if(!loadParameter(footstepParameters.profileHeight, hFootstep, "profileHeight")) { return false; }
    if(!loadParameter(footstepParameters.averageVelocity, hFootstep, "averageVelocity")) { return false; }
    if(!loadParameter(footstepParameters.liftOffSpeed, hFootstep, "liftOffSpeed")) { return false; }
    if(!loadParameter(footstepParameters.touchdownSpeed, hFootstep, "touchdownSpeed")) { return false; }
    if(!loadParameter(footstepParameters.minimumDuration, hFootstep, "minimumDuration")) { return false; }

    TiXmlHandle hEndEffectorTarget(handle);
    if(!getChildHandle(hEndEffectorTarget, handle, "EndEffectorTarget")) { return false; }
    if(!loadParameter(endEffectorTargetParameters.averageVelocity, hEndEffectorTarget, "averageVelocity")) { return false; }
    if(!loadParameter(endEffectorTargetParameters.minimumDuration, hEndEffectorTarget, "minimumDuration")) { return false; }


    TiXmlHandle hLegMode(handle);
    if(!getChildHandle(hLegMode, handle, "LegMode")) { return false; }
    if(!loadParameter(legModeParameters.duration, hLegMode, "duration")) { return false; }
    if(!loadParameter(legModeParameters.frameId, hLegMode, "frameId")) { return false; }

    TiXmlHandle hBaseAuto(handle);
    Eigen::VectorXd planarStance(2);
    if(!getChildHandle(hBaseAuto, handle, "BaseAuto")) { return false; }
    if(!loadParameter(baseAutoParameters.averageLinearVelocity, hBaseAuto, "averageLinearVelocity")) { return false; }
    if(!loadParameter(baseAutoParameters.averageAngularVelocity, hBaseAuto, "averageAngularVelocity")) { return false; }
    if(!loadParameter(baseAutoParameters.supportMargin, hBaseAuto, "supportMargin")) { return false; }
    if(!loadParameter(baseAutoParameters.minimumDuration, hBaseAuto, "minimumDuration")) { return false; }
    if(!loadParameter(baseAutoParameters.centerOfMassTolerance, hBaseAuto, "centerOfMassTolerance")) { return false; }
    if(!loadParameter(baseAutoParameters.legLengthTolerance, hBaseAuto, "legLengthTolerance")) { return false; }
    if(!loadParameter(baseAutoParameters.minLimbLengthScale, hBaseAuto, "minLimbLengthScale")) { return false; }
    if(!loadParameter(baseAutoParameters.maxLimbLengthAtClosingContactScale, hBaseAuto, "maxLimbLengthAtClosingContactScale")) { return false; }
    if(!loadParameter(baseAutoParameters.maxLimbLengthAtOpeningContactScale, hBaseAuto, "maxLimbLengthAtOpeningContactScale")) { return false; }
    if(!loadParameter("nominalPlanarStanceInBaseFrame", planarStance, hBaseAuto, {"x", "y"})) { return false; }

    baseAutoParameters.nominalPlanarStanceInBaseFrame.clear();
    baseAutoParameters.nominalPlanarStanceInBaseFrame.emplace(LimbEnum::LF_LEG, planarStance);
    baseAutoParameters.nominalPlanarStanceInBaseFrame.emplace(LimbEnum::RF_LEG, Position2(Eigen::Vector2d(planarStance(0), -planarStance(1))));
    baseAutoParameters.nominalPlanarStanceInBaseFrame.emplace(LimbEnum::LH_LEG, Position2(Eigen::Vector2d(-planarStance(0), planarStance(1))));
    baseAutoParameters.nominalPlanarStanceInBaseFrame.emplace(LimbEnum::RH_LEG, Position2(Eigen::Vector2d(-planarStance(0), -planarStance(1))));

    TiXmlHandle hBaseTarget(handle);
    if(!getChildHandle(hBaseTarget, handle, "BaseTarget")) { return false; }
    if(!loadParameter(baseTargetParameters.averageLinearVelocity, hBaseTarget, "averageLinearVelocity")) { return false; }
    if(!loadParameter(baseTargetParameters.averageAngularVelocity, hBaseTarget, "averageAngularVelocity")) { return false; }
    if(!loadParameter(baseTargetParameters.minimumDuration, hBaseTarget, "minimumDuration")) { return false; }

    return true;
  }

  friend class StepCompleter;

  struct FootstepParameters
  {
    std::string profileType;
    double profileHeight;
    double averageVelocity;
    double liftOffSpeed;
    double touchdownSpeed;
    double minimumDuration;

    FootstepParameters() {
      profileType = "triangle";
      profileHeight = 0.08;
      averageVelocity = 0.65;
      liftOffSpeed = 0.1;
      touchdownSpeed = 0.2;
      minimumDuration = 0.45;
    }
  } footstepParameters;

  struct EndEffectorTargetParameters
  {
    double averageVelocity;
    double minimumDuration;

    EndEffectorTargetParameters() {
      averageVelocity = 0.3;
      minimumDuration = 0.05;
    }
  } endEffectorTargetParameters;

  struct LegModeParameters
  {
    double duration;
    std::string frameId;

    LegModeParameters() {
      duration = 0.5;
      frameId = "base";
    }
  } legModeParameters;

  struct BaseAutoParameters
  {
    double averageLinearVelocity;
    double averageAngularVelocity;
    double supportMargin;
    double minimumDuration;
    double centerOfMassTolerance;
    double legLengthTolerance;
    double minLimbLengthScale;
    double maxLimbLengthAtClosingContactScale;
    double maxLimbLengthAtOpeningContactScale;
    PlanarStance nominalPlanarStanceInBaseFrame;

    BaseAutoParameters() {
      averageLinearVelocity = 0.2;
      averageAngularVelocity = 0.28;
      supportMargin = 0.04;
      minimumDuration = 0.1;
      centerOfMassTolerance = 0.0;
      legLengthTolerance = 0.0;
      minLimbLengthScale = 0.325;
      maxLimbLengthAtOpeningContactScale = 0.887;
      maxLimbLengthAtClosingContactScale = 0.919;

      Position2 planarStance;
      planarStance << 0.33, 0.22;
      nominalPlanarStanceInBaseFrame.emplace(LimbEnum::LF_LEG, planarStance);
      nominalPlanarStanceInBaseFrame.emplace(LimbEnum::RF_LEG, Position2(Eigen::Vector2d(planarStance(0), -planarStance(1))));
      nominalPlanarStanceInBaseFrame.emplace(LimbEnum::LH_LEG, Position2(Eigen::Vector2d(-planarStance(0), planarStance(1))));
      nominalPlanarStanceInBaseFrame.emplace(LimbEnum::RH_LEG, Position2(Eigen::Vector2d(-planarStance(0), -planarStance(1))));

    }
  } baseAutoParameters;

  struct BaseTargetParameters
  {
    double averageLinearVelocity;
    double averageAngularVelocity;
    double minimumDuration;

    BaseTargetParameters() {
      averageLinearVelocity = 0.05;
      averageAngularVelocity = 0.1;
      minimumDuration = 0.7;
    }
  } baseTargetParameters;
};

} /* namespace */
