/*!
 * @file     ComSupportControlLeverConfiguration.hpp
 * @author   C. Dario Bellicoso
 * @date     Oct 7, 2014
 * @brief
 */

#pragma once

#include "loco/common/WholeBody.hpp"
#include "loco/torso_control/ComSupportControlBase.hpp"

namespace loco {

class ComSupportControlLeverConfiguration : public ComSupportControlBase {
 public:
  ComSupportControlLeverConfiguration(WholeBody& wholeBody, TerrainModelBase& terrainModel);
  ~ComSupportControlLeverConfiguration() override = default;

  bool setToInterpolated(const ComSupportControlBase& supportPolygon1, const ComSupportControlBase& supportPolygon2, double t) override;

  bool advance(double dt) override;

  bool initialize(double dt) override;

 protected:
  TorsoBase& torso_;
  TerrainModelBase& terrainModel_;

  Position positionCenterToForeHindSupportFeetInControlFrame_[2];
  Position positionWorldToCenterInWorldFrame_;
};

} /* namespace loco */
