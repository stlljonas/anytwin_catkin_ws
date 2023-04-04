/*
 * TorsoControlDynamicGaitFreePlane.hpp
 *
 *  Created on: Sep 11, 2014
 *      Author: Dario Bellicoso
 */

#pragma once

// loco
#include "loco/common/WholeBody.hpp"
#include "loco/torso_control/TorsoControlGaitContainer.hpp"

// basic filters
#include "basic_filters/FirstOrderFilter.hpp"

namespace loco {

class TorsoControlDynamicGaitFreePlane : public TorsoControlGaitContainer {
 private:
  using Base = TorsoControlGaitContainer;

 public:
  enum class AdaptToTerrain : unsigned int { CompleteAdaption = 0, SaturatedLinearAdaption };

  TorsoControlDynamicGaitFreePlane(WholeBody& wholeBody, TerrainModelBase& terrain, ComSupportControlBase& comSupportControl);
  ~TorsoControlDynamicGaitFreePlane() override = default;

  bool initialize(double dt) override;
  bool advance(double dt) override;

  bool loadParameters(const TiXmlHandle& handle) override;

  bool setToInterpolated(const TorsoControlBase& torsoController1, const TorsoControlBase& torsoController2, double t) override;

  const WholeBody& getWholeBody() const;
  WholeBody* getWholeBodyPtr() const;

 protected:
  WholeBody& wholeBody_;

  // Desired center of mass adaptation parameters.
  double lateralOffsetInControlFrameGain_;
  double angularOffsetInControlFrameGain_;

  // Terrain adaptation parameters.
  double maxDesiredPitchRadians_;
  double desiredPitchSlope_;
  double maxDesiredRollRadians_;
  double desiredRollSlope_;
  AdaptToTerrain adaptToTerrain_;
};

} /* namespace loco */
