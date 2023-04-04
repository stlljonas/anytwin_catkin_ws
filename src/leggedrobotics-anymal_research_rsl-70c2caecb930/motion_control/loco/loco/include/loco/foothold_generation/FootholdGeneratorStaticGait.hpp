/*
 * FootholdGeneratorStaticGait.hpp
 *
 *  Created on: Feb 1, 2015
 *      Author: C. Dario Bellicoso
 */

#pragma once

// eigen
#include <Eigen/Core>

// loco
#include "loco/common/TerrainModelBase.hpp"
#include "loco/common/WholeBody.hpp"
#include "loco/common/legs/LegBase.hpp"
#include "loco/foothold_generation/FootholdGeneratorBase.hpp"

// robot_utils
#include "robot_utils/surfaces/Ellipsoid.hpp"

// parameter library
#include "parameter_handler/parameter_handler.hpp"

class TiXmlHandle;

namespace loco {

class FootholdGeneratorStaticGait : public FootholdGeneratorBase {
  using Footprint = Eigen::Matrix<double, 3, 4>;

 public:
  FootholdGeneratorStaticGait(WholeBody& wholeBody, TerrainModelBase& terrain);
  ~FootholdGeneratorStaticGait() override = default;

  virtual Position generateFootHold(const int legId);
  virtual Position generateFootHold(const int legId, const Footprint& footprintInWorldFrame, bool compareWithPlannedFootholds);
  virtual void generateFootprint();
  bool loadParameters(const TiXmlHandle& handle) override;
  virtual bool initialize(double dt);

  const Legs& getLegs() const;
  const double getMaxStepLengthHeading() const;
  const double getMaxStepLengthLateral() const;
  const double getMaxStepAngular() const;

  bool addVariablesToLog() override;

  friend std::ostream& operator<<(std::ostream& out, const FootholdGeneratorStaticGait& fhGen);

 protected:
  WholeBody& wholeBody_;
  TerrainModelBase& terrain_;

  TorsoBase& torso_;
  Legs& legs_;

  // Parameters
  parameter_handler::Parameter<double> paramFootholdOffsetX_;
  parameter_handler::Parameter<double> paramFootholdOffsetY_;
  parameter_handler::Parameter<double> paramMaxStepLengthHeading_;
  parameter_handler::Parameter<double> paramMaxStepLengthLateral_;
  parameter_handler::Parameter<double> paramMaxStepAngular_;
  parameter_handler::Parameter<double> paramMaxDistanceForeHind_;
  parameter_handler::Parameter<double> paramMaxDistanceLeftRight_;
  parameter_handler::Parameter<double> paramMaxDistanceDiagonal_;
  parameter_handler::Parameter<double> paramMinDistanceForeHind_;
  parameter_handler::Parameter<double> paramMinDistanceLeftRight_;
  parameter_handler::Parameter<double> paramMinDistanceDiagonal_;

  std::vector<Position> positionFootprintCenterToDefaultFootholdInControlFrame_;
  robot_utils::Ellipsoid ellipsoid_;

  virtual Position getPositionFootprintCenterToDesiredFootHoldInControlFrame(const LegBase& leg);
  void constrainFootholdInFootprintInWorldFrame(Position& positionWorldToFootHoldInWorldFrame, const LegBase& leg,
                                                bool compareWithPlannedFootholds = false);

  // Util methods
  const LegBase& getComplementLegLateral(const LegBase& leg);
  const LegBase& getComplementLegLongitude(const LegBase& leg);
  const LegBase& getComplementLegDiagonal(const LegBase& leg);
};

} /* namespace loco */
