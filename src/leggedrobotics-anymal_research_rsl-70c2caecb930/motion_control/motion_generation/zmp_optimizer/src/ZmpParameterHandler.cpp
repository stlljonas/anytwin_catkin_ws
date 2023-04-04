/*
 * ZmpParameterHandler.cpp
 *
 *  Created on: Aug 15, 2017
 *      Author: Fabian Jenelten
 */

// zmp optimizer
#include "zmp_optimizer/ZmpParameterHandler.hpp"

// messagge logger
#include "message_logger/message_logger.hpp"

namespace loco {

ZmpParameterHandler::ZmpParameterHandler()
    : ParameterHandler(),
      activeGaitName_("unknown gait"),
      desiredGaitName_("unknown gait"),
      polygonIdAtSwitch_(-1),
      splineIdAtSwitch_(-1),
      addEpsilonStateMinDeviationActiveGait_(false),
      addEpsilonStateMinDeviationDesiredGait_(false) {}

bool ZmpParameterHandler::initialize(const std::string& activeGaitName, const std::string& desiredGaitName, int polygonIdAtSwitch) {
  return setGaitInfo(activeGaitName, desiredGaitName, polygonIdAtSwitch);
}

void ZmpParameterHandler::clear() {
  ParameterHandler::clear();
  resetGaitInfo();
}

void ZmpParameterHandler::resetGaitInfo() {
  activeGaitName_ = "unknown gait";
  desiredGaitName_ = "unknown gait";
  polygonIdAtSwitch_ = -1;
  splineIdAtSwitch_ = -1;
  std::fill(addEpsilonStateMinDeviationActiveGait_.begin(), addEpsilonStateMinDeviationActiveGait_.end(), false);
  std::fill(addEpsilonStateMinDeviationDesiredGait_.begin(), addEpsilonStateMinDeviationDesiredGait_.end(), false);
}

bool ZmpParameterHandler::loadParameters(const TiXmlHandle& zmpHandle, unsigned int gaitIndex, const std::string& gaitPatternTypeStr) {
  emplace_params(gaitIndex, gaitPatternTypeStr);

  /*************************
   * Dof depending Weights *
   *************************/
  TiXmlHandle weightHandle = zmpHandle;
  if (!tinyxml_tools::getChildHandle(weightHandle, zmpHandle, "Weights")) {
    return false;
  }

  TiXmlHandle dofWeightHandle = zmpHandle;
  params_.back().optimizationDofs_.clear();
  std::cout << " (";
  for (const auto& dim : zmp::optimizationTranslationalRotationalDofs) {
    if (tinyxml_tools::getChildHandle(dofWeightHandle, weightHandle, zmp::cogDimMap[dim], false)) {
      // Add optimization dof
      params_.back().optimizationDofs_.push_back(dim);
      std::cout << zmp::cogDimMap[dim] << " ";

      // Weights for min acceleration.
      TiXmlHandle accelHandle = zmpHandle;
      if (!tinyxml_tools::getChildHandle(accelHandle, dofWeightHandle, "MinAcceleration")) {
        return false;
      }
      if (!tinyxml_tools::loadParameter(params_.back().weightsPathRegularizer_[dim][zmp::Derivative::Second], accelHandle, "der2")) {
        return false;
      }

      // Weights for Path regularizer.
      TiXmlHandle pathHandle = zmpHandle;
      if (!tinyxml_tools::getChildHandle(pathHandle, dofWeightHandle, "PathRegularizer")) {
        return false;
      }
      if (!tinyxml_tools::loadParameter(params_.back().weightsPathRegularizer_[dim][zmp::Derivative::Zero], pathHandle, "der0")) {
        return false;
      }
      if (!tinyxml_tools::loadParameter(params_.back().weightsPathRegularizer_[dim][zmp::Derivative::First], pathHandle, "der1")) {
        return false;
      }

      // Weights for minimizing max deviation w.r.t. path regularizer.
      TiXmlHandle deviationHandle = zmpHandle;
      if (!tinyxml_tools::getChildHandle(deviationHandle, dofWeightHandle, "MinDeviation")) {
        return false;
      }
      if (!tinyxml_tools::loadParameter(params_.back().weightsMinDeviation_[dim][zmp::Objective::Lin], deviationHandle, "lin")) {
        return false;
      }
      if (!tinyxml_tools::loadParameter(params_.back().weightsMinDeviation_[dim][zmp::Objective::Quad], deviationHandle, "quad")) {
        return false;
      }

      if (zmp::isTranslation(dim)) {
        //  Weights for enforcing default leg configuration.
        TiXmlHandle defaultLegConfigHandle = zmpHandle;
        if (!tinyxml_tools::getChildHandle(defaultLegConfigHandle, dofWeightHandle, "DefaultLegConfig")) {
          return false;
        }
        if (!tinyxml_tools::loadParameter(params_.back().weightsDefaultLegConfig_[dim], defaultLegConfigHandle, "der0")) {
          return false;
        }

        //  Weights for minimizing leg over extension.
        TiXmlHandle legExtensionHandle = zmpHandle;
        if (!tinyxml_tools::getChildHandle(legExtensionHandle, dofWeightHandle, "MinLegExtension")) {
          return false;
        }
        if (!tinyxml_tools::loadParameter(params_.back().weightsMinLegExtension_[dim], legExtensionHandle, "der0")) {
          return false;
        }

        // Weights to minimize tip-over.
        TiXmlHandle tipOverHandle = zmpHandle;
        if (!tinyxml_tools::getChildHandle(tipOverHandle, dofWeightHandle, "TipOverAvoidance")) {
          return false;
        }
        if (!tinyxml_tools::loadParameter(params_.back().weightsTipOverAvoidance_[dim], tipOverHandle, "der0")) {
          return false;
        }
      }
    }
  }
  std::cout << ")";
  /*************************/

  /***********
   * Weights *
   ***********/
  // Weights for previous solution.
  TiXmlHandle previousSolutionHandle = zmpHandle;
  if (!tinyxml_tools::getChildHandle(previousSolutionHandle, weightHandle, "PreviousSolution")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(params_.back().weightsPreviousSolution_[zmp::Derivative::Zero], previousSolutionHandle, "der0")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(params_.back().weightsPreviousSolution_[zmp::Derivative::First], previousSolutionHandle, "der1")) {
    return false;
  }

  // Weights for initial state.
  TiXmlHandle initialStateHandle = zmpHandle;
  if (!tinyxml_tools::getChildHandle(initialStateHandle, weightHandle, "SoftInitialState")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(params_.back().weightsInitialState_[zmp::Derivative::Second], initialStateHandle, "der2")) {
    return false;
  }

  // Weights for final state.
  TiXmlHandle finalStateHandle = zmpHandle;
  if (!tinyxml_tools::getChildHandle(finalStateHandle, weightHandle, "SoftFinalState")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(params_.back().weightsFinalState_[zmp::Derivative::Zero], finalStateHandle, "der0")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(params_.back().weightsFinalState_[zmp::Derivative::First], finalStateHandle, "der1")) {
    return false;
  }

  // Weights for zmp inequality relaxation boundaries.
  TiXmlHandle relaxHandle = zmpHandle;
  if (!tinyxml_tools::getChildHandle(relaxHandle, weightHandle, "ZmpInequalityRelaxation")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(params_.back().zmpRelativeRelaxationHorizon_, relaxHandle, "rel_relax_horizon")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(params_.back().weightZmpRelaxation_[zmp::Objective::Lin], relaxHandle, "weight_lin")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(params_.back().weightZmpRelaxation_[zmp::Objective::Quad], relaxHandle, "weight_quad")) {
    return false;
  }

  //  Weights for flight phase trajectory.
  TiXmlHandle flightTrajectoryHandle = zmpHandle;
  if (tinyxml_tools::getChildHandle(flightTrajectoryHandle, weightHandle, "FlightTrajectory", false)) {
    for (const auto& dim : params_.back().optimizationDofs_) {
      if (!tinyxml_tools::loadParameter(params_.back().weightsTouchDownFlightTrajectory_[dim][zmp::Derivative::Zero],
                                        flightTrajectoryHandle, zmp::cogDimMap[dim])) {
        return false;
      }
    }
  }
  /***********/

  // Spline information.
  TiXmlHandle splinesHandle = zmpHandle;
  if (!tinyxml_tools::getChildHandle(splinesHandle, zmpHandle, "SplinesPerPhase")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(params_.back().splinesPerPolygon_[zmp::PolygonType::Rectangle], splinesHandle, "stance")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(params_.back().splinesPerPolygon_[zmp::PolygonType::Triangle], splinesHandle, "single_swing")) {
    return false;
  }
  if (!tinyxml_tools::loadParameter(params_.back().splinesPerPolygon_[zmp::PolygonType::Line], splinesHandle, "double_swing")) {
    return false;
  }
  return tinyxml_tools::loadParameter(params_.back().splinesPerPolygon_[zmp::PolygonType::Point], splinesHandle, "triple_swing");
}

const ZmpParams& ZmpParameterHandler::getParamsBySpline(int splineIndex) const {
  if (splineIdAtSwitch_ == -1) {  // default (-1) is associated to active gait.
    return getParams(activeGaitName_);
  } else if (splineIndex < splineIdAtSwitch_) {  // before switch
    return getParams(activeGaitName_);
  } else {  // after switch
    return getParams(desiredGaitName_);
  }
}

const ZmpParams& ZmpParameterHandler::getActiveParams() const {
  return getParams(activeGaitName_);
}

ZmpParams* ZmpParameterHandler::getActiveParamsPtr() {
  return getParamsPtr(activeGaitName_);
}

const ZmpParams& ZmpParameterHandler::getDesiredParams() const {
  return getParams(desiredGaitName_);
}

void ZmpParameterHandler::setSplineIdAtSwitch(int splineIdAtSwitch) {
  splineIdAtSwitch_ = splineIdAtSwitch;
}

const std::string& ZmpParameterHandler::getActiveGaitName() const {
  return activeGaitName_;
}

const std::string& ZmpParameterHandler::getDesiredGaitName() const {
  return desiredGaitName_;
}

unsigned int ZmpParameterHandler::getActiveGaitId() const {
  return mapGaitNameToId_.at(activeGaitName_);
}

unsigned int ZmpParameterHandler::getDesiredGaitId() const {
  return mapGaitNameToId_.at(desiredGaitName_);
}

int ZmpParameterHandler::getPolygonIdAtSwitch() const {
  return polygonIdAtSwitch_;
}

int ZmpParameterHandler::getSplineIdAtSwitch() const {
  return splineIdAtSwitch_;
}

bool ZmpParameterHandler::setGaitInfo(const std::string& activeGaitName, const std::string& desiredGaitName, int polygonIdAtSwitch) {
  if (mapGaitNameToId_.find(activeGaitName) == mapGaitNameToId_.end() || mapGaitNameToId_.find(desiredGaitName) == mapGaitNameToId_.end()) {
    MELO_WARN_STREAM("[ZmpParameterHandler::setGaitInfo] Invalid gait names: active = " << activeGaitName
                                                                                        << ", desired = " << desiredGaitName << ".")
    return false;
  }

  // Check if the switch has already happened.
  if (polygonIdAtSwitch_ == 0) {
    activeGaitName_ = desiredGaitName;
    desiredGaitName_ = desiredGaitName;
    polygonIdAtSwitch_ = -1;
  } else {
    activeGaitName_ = activeGaitName;
    desiredGaitName_ = desiredGaitName;
    polygonIdAtSwitch_ = polygonIdAtSwitch;
  }

  return true;
}

void ZmpParameterHandler::addEpsilonStateMinDeviationActiveGait(zmp::CogDim dim, bool add) {
  addEpsilonStateMinDeviationActiveGait_[dim] = add;
}

void ZmpParameterHandler::addEpsilonStateMinDeviationDesiredGait(zmp::CogDim dim, bool add) {
  addEpsilonStateMinDeviationDesiredGait_[dim] = add;
}

bool ZmpParameterHandler::getAddEpsilonStateMinDeviationActiveGait(zmp::CogDim dim) const {
  return addEpsilonStateMinDeviationActiveGait_[dim];
}

bool ZmpParameterHandler::getAddEpsilonStateMinDeviationDesiredGait(zmp::CogDim dim) const {
  return addEpsilonStateMinDeviationDesiredGait_[dim];
}

void ZmpParameterHandler::resetAddEpsilonStateMinDeviation() {
  std::fill(addEpsilonStateMinDeviationActiveGait_.begin(), addEpsilonStateMinDeviationActiveGait_.end(), false);
  std::fill(addEpsilonStateMinDeviationDesiredGait_.begin(), addEpsilonStateMinDeviationDesiredGait_.end(), false);
}

unsigned int ZmpParameterHandler::getNumOfEpsilonStateMinDeviationActiveGait() const {
  return std::count(addEpsilonStateMinDeviationActiveGait_.begin(), addEpsilonStateMinDeviationActiveGait_.end(), true);
}

unsigned int ZmpParameterHandler::getNumOfEpsilonStateMinDeviationDesiredGait() const {
  return std::count(addEpsilonStateMinDeviationDesiredGait_.begin(), addEpsilonStateMinDeviationDesiredGait_.end(), true);
}

bool ZmpParameterHandler::copyActiveAndDesiredGaitParams(const ZmpParameterHandler& fullZmpParameter) {
  clear();

  // Copy parameters for active and desired gait
  constexpr unsigned int activeGaitId = 0;
  constexpr unsigned int desiredGaitId = 1;

  params_.reserve(2);
  params_.emplace_back(ZmpParams());
  params_.emplace_back(ZmpParams());
  params_[activeGaitId] = fullZmpParameter.getActiveParams();
  params_[desiredGaitId] = fullZmpParameter.getDesiredParams();

  // Copy gait names and indexes.
  activeGaitName_ = fullZmpParameter.getActiveGaitName();
  desiredGaitName_ = fullZmpParameter.getDesiredGaitName();
  polygonIdAtSwitch_ = fullZmpParameter.getPolygonIdAtSwitch();
  splineIdAtSwitch_ = fullZmpParameter.getSplineIdAtSwitch();

  // Copy map between active and desired gait.
  mapGaitNameToId_.insert(std::make_pair(activeGaitName_, activeGaitId));
  mapGaitNameToId_.insert(std::make_pair(desiredGaitName_, desiredGaitId));

  // Copy other informations.
  for (const auto& dim : std_utils::enum_iterator<zmp::CogDim>()) {
    addEpsilonStateMinDeviationActiveGait_[dim] = fullZmpParameter.getAddEpsilonStateMinDeviationActiveGait(dim);
    addEpsilonStateMinDeviationDesiredGait_[dim] = fullZmpParameter.getAddEpsilonStateMinDeviationDesiredGait(dim);
  }

  /* Equalize zmp parameters.
   * During a gait switch, the union set of the active and desired optimization dofs will be used.
   * To make sure we have proper parameters, we copy these from a proper parmeter set.
   */
  if (activeGaitName_ != desiredGaitName_) {
    // active -> desired.
    for (const auto& activeDim : params_[activeGaitId].optimizationDofs_) {
      if (!std_utils::containsEnum(params_[desiredGaitId].optimizationDofs_, activeDim)) {
        if (!copyGaitParamsForDim(params_[desiredGaitId], params_[activeGaitId], activeDim)) {
          return false;
        }
      }
    }

    // desired -> active.
    for (const auto& desiredDim : params_[desiredGaitId].optimizationDofs_) {
      if (!std_utils::containsEnum(params_[activeGaitId].optimizationDofs_, desiredDim)) {
        if (!copyGaitParamsForDim(params_[activeGaitId], params_[desiredGaitId], desiredDim)) {
          return false;
        }
      }
    }
  }

  return true;
}

bool ZmpParameterHandler::copyGaitParamsForDim(ZmpParams& params1, const ZmpParams& params2, const zmp::CogDim& dim) const {
  params1.weightsPathRegularizer_[dim] = params2.weightsPathRegularizer_[dim];
  params1.weightsTouchDownFlightTrajectory_[dim] = params2.weightsTouchDownFlightTrajectory_[dim];
  params1.weightsMinDeviation_[dim] = params2.weightsMinDeviation_[dim];
  return true;
}

bool ZmpParameterHandler::isSplineIdInActiveGait(unsigned int splineId) const {
  return (splineIdAtSwitch_ == -1 || static_cast<int>(splineId) < splineIdAtSwitch_);
}

bool ZmpParameterHandler::isPolygonIdInActiveGait(unsigned int polygonId) const {
  return (polygonIdAtSwitch_ == -1 || static_cast<int>(polygonId) < polygonIdAtSwitch_);
}

} /* namespace loco */
