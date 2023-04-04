/*
 * InvPendParameterHandler.hpp
 *
 *  Created on: Mar 15, 2018
 *      Author: Fabian Jenelten
 */
#pragma once

// motion generation
#include "motion_generation_utils/ParameterHandler.hpp"


namespace loco {
namespace foothold_generator {

struct DynamicLateralFootholdAdaptation {
  DynamicLateralFootholdAdaptation()
  : maxOffset_(0.0),
    nominalOperationVelocity(0.4),
    lateralWidthOffsetAtNominalOperationVelocity(0.08) {
  }

  //! Max allowed offset for the default foothold along lateral axis.
  double maxOffset_;
  ///! Reference lateral velocity of walking for computing the lateral offset.
  double nominalOperationVelocity;
  //! Desired lateral width at the nominal lateral velocity.
  double lateralWidthOffsetAtNominalOperationVelocity;
};

struct InvPendParams {
  using Weight = Eigen::DiagonalMatrix<double, 2, 2>;

  explicit InvPendParams() :
    feedbackScale_(0.0),
    feedforwardScale_(1.0),
    weightsDesiredFoothold_(Weight(1.0, 1.0)),
    weightsPreviousFoothold_(Weight(0.01, 0.01)),
    distanceBaseToDefaultFootholdHeading_(0.0),
    distanceBaseToDefaultFootholdLateralHind_(0.0),
    distanceBaseToDefaultFootholdLateralFront_(0.0),
    relativeOffsetHeading_(0.0),
    relativeOffsetLateral_() {
  }

  //! Velocity feedback gain.
  double feedbackScale_;

  //! Velocity projection feedforward gain.
  double feedforwardScale_;

  //! Weights for tracking desired foothold.
  Weight weightsDesiredFoothold_;

  //! Weight for tracking previous foothold.
  Weight weightsPreviousFoothold_;

  //! Default foothold w.r.t to torso base.
  double distanceBaseToDefaultFootholdHeading_;
  double distanceBaseToDefaultFootholdLateralHind_;
  double distanceBaseToDefaultFootholdLateralFront_;

  //! Offset for shifting the default foothold along heading and lateral axis.
  double relativeOffsetHeading_;
  //! Offset for shifting the default foothold along lateral axis based on lateral velocity of operation.
  DynamicLateralFootholdAdaptation relativeOffsetLateral_;
};


class InvPendParameterHandler : public motion_generation::ParameterHandler<InvPendParams> {
public:
  using Weight = Eigen::DiagonalMatrix<double, 2, 2>;

  InvPendParameterHandler() : ParameterHandler() { }

  ~InvPendParameterHandler() override = default;

  bool loadParameters(
      const TiXmlHandle& gaitHandle,
      unsigned int gaitIndex,
      const std::string& gaitPatternTypeStr) override {
    emplace_params(gaitIndex, gaitPatternTypeStr);

    // Optimization weights.
    const TiXmlHandle weightsHandle = tinyxml_tools::getChildHandle(gaitHandle, "Weights");
    double velProjection, previousSolution;
    if(!tinyxml_tools::loadParameter(velProjection,    weightsHandle, "invertedPendulum", 0.0)) { return false; }
    if(!tinyxml_tools::loadParameter(previousSolution, weightsHandle, "previousSolution", 0.0)) { return false; }
    params_.back().weightsDesiredFoothold_ = Weight(velProjection,    velProjection);
    params_.back().weightsPreviousFoothold_   = Weight(previousSolution, previousSolution);

    const TiXmlHandle baseOffsetHandle = tinyxml_tools::getChildHandle(gaitHandle, "BaseToDefaultFoothold");
    if(!tinyxml_tools::loadParameter(params_.back().distanceBaseToDefaultFootholdHeading_, baseOffsetHandle, "x",  0.0)) { return false; }
    if(!tinyxml_tools::loadParameter(params_.back().distanceBaseToDefaultFootholdLateralHind_, baseOffsetHandle, "y_h",  0.0)) { return false; }
    if(!tinyxml_tools::loadParameter(params_.back().distanceBaseToDefaultFootholdLateralFront_, baseOffsetHandle, "y_f",  0.0)) { return false; }
    if(!tinyxml_tools::loadParameter(params_.back().relativeOffsetHeading_, baseOffsetHandle, "rel_vel_offset",  0.0)) { return false; }

    TiXmlHandle lateralOffsetHandle = gaitHandle;
    if(tinyxml_tools::getChildHandle(lateralOffsetHandle, gaitHandle, "DynamicLateralFootholdAdaptation", false)) {
      tinyxml_tools::loadParameter(params_.back().relativeOffsetLateral_.maxOffset_, lateralOffsetHandle, "max_offset", 0.0);
      tinyxml_tools::loadParameter(params_.back().relativeOffsetLateral_.nominalOperationVelocity, lateralOffsetHandle,
                                   "nominal_operation_velocity", 0.4);
      tinyxml_tools::loadParameter(params_.back().relativeOffsetLateral_.lateralWidthOffsetAtNominalOperationVelocity, lateralOffsetHandle,
                                   "offset_at_nominal_velocity", 0.08);
    }

    //! Gains (inverted pendulum foothold generation).
    const TiXmlHandle gainsHandle = tinyxml_tools::getChildHandle(gaitHandle, "Gains");
    if(!tinyxml_tools::loadParameter(params_.back().feedbackScale_,    gainsHandle,  "feedbackScale",   0.0)) { return false; }
    if(!tinyxml_tools::loadParameter(params_.back().feedforwardScale_, gainsHandle,  "feedforwardScale",0.0)) { return false; }

    return true;
  }

};



} /* foothold_generator */
} /* namespace loco */
