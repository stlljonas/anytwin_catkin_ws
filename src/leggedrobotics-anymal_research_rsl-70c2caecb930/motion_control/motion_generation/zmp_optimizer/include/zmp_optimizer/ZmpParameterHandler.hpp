/*
 * ZmpParameterHandler.hpp
 *
 *  Created on: Aug 15, 2017
 *      Author: Fabian Jenelten
 */
#pragma once

// zmp optimizer
#include "zmp_optimizer/zmp_optimizer.hpp"

// motion generation
#include "motion_generation_utils/ParameterHandler.hpp"

// std utils
#include "std_utils/std_utils.hpp"

namespace loco {

struct ZmpParams {
  explicit ZmpParams()
      : weightsFinalState_(-1.0),
        weightsPathRegularizer_(std_utils::EnumArray<zmp::Derivative, double>(-1.0)),
        weightsPreviousSolution_(-1.0),
        weightsTouchDownFlightTrajectory_(std_utils::EnumArray<zmp::Derivative, double>(-1.0)),
        weightsMinDeviation_(std_utils::EnumArray<zmp::Objective, double>(-1.0)),
        weightsInitialState_(-1.0),
        weightZmpRelaxation_(-1.0),
        splinesPerPolygon_(1u),
        zmpRelativeRelaxationHorizon_(0.1),
        optimizationDofs_(),
        weightsDefaultLegConfig_(-1.0),
        weightsMinLegExtension_(-1.0),
        weightsTipOverAvoidance_(-1.0) {}

  //! Weights for minimizing squared 2norm between final spline state and desired final conditions.
  std_utils::EnumArray<zmp::Derivative, double> weightsFinalState_;

  //! Weights for minimizing squared 2norm between sampled spline points (pos, vel, accel)
  // and sampled path regularizer points
  std_utils::EnumArray<zmp::CogDim, std_utils::EnumArray<zmp::Derivative, double>> weightsPathRegularizer_;

  //! Weights on minimizing squared 2norm between sampled spline points (pos, vel, accel)
  // and sampled spline points of previous solution
  std_utils::EnumArray<zmp::Derivative, double> weightsPreviousSolution_;

  //! Weights to track path regularizer at touch-down after full flight phase
  std_utils::EnumArray<zmp::CogDim, std_utils::EnumArray<zmp::Derivative, double>> weightsTouchDownFlightTrajectory_;

  //! Weight for minimizing max peak in z-direction of CoG trajectory
  std_utils::EnumArray<zmp::CogDim, std_utils::EnumArray<zmp::Objective, double>> weightsMinDeviation_;

  //! Weights for approaching first point of solution to initial conditions
  std_utils::EnumArray<zmp::Derivative, double> weightsInitialState_;

  //! The first few zmp constraints c(x)<=0 are approximated as soft constraints c(x)<=epsilon. The weight
  // is used to minimize epsilon.
  std_utils::EnumArray<zmp::Objective, double> weightZmpRelaxation_;

  //! Number of splines to approximate a polygon phase
  std_utils::EnumArray<zmp::PolygonType, unsigned int> splinesPerPolygon_;

  //! The first few zmp constraints c(x)<=0 are approximated as soft constraints c(x)<=epsilon.
  double zmpRelativeRelaxationHorizon_;

  //! Degrees of freedom which are optimized.
  std::vector<zmp::CogDim> optimizationDofs_;

  //! Weights for enforcing default leg configuration using torso adjustment.
  std_utils::EnumArray<zmp::CogDim, double> weightsDefaultLegConfig_;

  //! Weights for minimizing leg over extension.
  std_utils::EnumArray<zmp::CogDim, double> weightsMinLegExtension_;

  //! Weights for avoiding torso tip-over (weight for approaching towards foothold with bad tracking).
  std_utils::EnumArray<zmp::CogDim, double> weightsTipOverAvoidance_;
};

class ZmpParameterHandler : public motion_generation::ParameterHandler<ZmpParams> {
 public:
  ZmpParameterHandler();
  ~ZmpParameterHandler() override = default;

  virtual bool initialize(const std::string& activeGaitName, const std::string& desiredGaitName, int polygonIdAtSwitch);

  void clear() override;

  virtual void resetGaitInfo();

  bool loadParameters(const TiXmlHandle& handle, unsigned int gaitIndex, const std::string& gaitPatternTypeStr) override;

  //! Get zmp parameters by spline index.
  virtual const ZmpParams& getParamsBySpline(int splineIndex) const;

  //! Get zmp parameters for the active gait.
  virtual const ZmpParams& getActiveParams() const;
  virtual ZmpParams* getActiveParamsPtr();

  //! Get zmp parameters for the desired gait.
  virtual const ZmpParams& getDesiredParams() const;

  //! Get active gait name.
  virtual const std::string& getActiveGaitName() const;

  //! Get desired gait name.
  virtual const std::string& getDesiredGaitName() const;

  //! Set spline index at gait transition.
  virtual void setSplineIdAtSwitch(int splineIdAtSwitch);

  //! Get active gait index.
  virtual unsigned int getActiveGaitId() const;

  //! Get desired gait index.
  virtual unsigned int getDesiredGaitId() const;

  //! get polygon index at gait transition.
  virtual int getPolygonIdAtSwitch() const;

  //! Get spline index at gait transition.
  virtual int getSplineIdAtSwitch() const;

  //! Set gait names and polygon index at which switch happens.
  virtual bool setGaitInfo(const std::string& activeGaitName, const std::string& desiredGaitName, int polygonIdAtSwitch);

  virtual void addEpsilonStateMinDeviationActiveGait(zmp::CogDim dim, bool add);
  virtual void addEpsilonStateMinDeviationDesiredGait(zmp::CogDim dim, bool add);

  virtual bool getAddEpsilonStateMinDeviationActiveGait(zmp::CogDim dim) const;
  virtual bool getAddEpsilonStateMinDeviationDesiredGait(zmp::CogDim dim) const;

  virtual void resetAddEpsilonStateMinDeviation();

  virtual unsigned int getNumOfEpsilonStateMinDeviationActiveGait() const;
  virtual unsigned int getNumOfEpsilonStateMinDeviationDesiredGait() const;

  // Copy only the content from the active and the desired gait.
  virtual bool copyActiveAndDesiredGaitParams(const ZmpParameterHandler& fullZmpParameter);

  //! True if spline index corresponds to active gait.
  virtual bool isSplineIdInActiveGait(unsigned int splineId) const;

  //! True if polygon index corresponds to acitve gait.
  virtual bool isPolygonIdInActiveGait(unsigned int polygonId) const;

 protected:
  //! Copy dim-depending parameters from params2 into params1.
  bool copyGaitParamsForDim(ZmpParams& params1, const ZmpParams& params2, const zmp::CogDim& dim) const;

  //! Name of the active gait.
  std::string activeGaitName_;

  //! Name of the desired gait.
  std::string desiredGaitName_;

  //! Polygon  index at which we change from active to desired gait. If there is no switch, the index is -1.
  int polygonIdAtSwitch_;

  //! Spline index at which we change from active to desired gait. If there is no switch, the index is -1.
  int splineIdAtSwitch_;

  //! True if min deviation constraints are active at dimension dim for active gait.
  std_utils::EnumArray<zmp::CogDim, bool> addEpsilonStateMinDeviationActiveGait_;

  //! True if min deviation constraints are active at dimension dim for desired gait.
  std_utils::EnumArray<zmp::CogDim, bool> addEpsilonStateMinDeviationDesiredGait_;
};

} /* namespace loco */
