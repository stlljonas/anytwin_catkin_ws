/*
 * TorsoControlStaticGait.hpp
 *
 *  Created on: Oct 9, 2014
 *      Author: C. Dario Bellicoso
 */

#pragma once

// loco
#include "loco/common/WholeBody.hpp"
#include "loco/gait_pattern/GaitPatternBase.hpp"
#include "loco/torso_control/ComSupportControlBase.hpp"
#include "loco/torso_control/TorsoControlGaitContainer.hpp"

namespace loco {

class TorsoControlStaticGait : public TorsoControlGaitContainer {
 private:
  using Base = TorsoControlGaitContainer;

 public:
  TorsoControlStaticGait(WholeBody& wholeBody, TerrainModelBase& terrain, GaitPatternBase& gaitPattern, ComSupportControlBase& comControl);
  ~TorsoControlStaticGait() override = default;

  bool initialize(double dt) override;
  bool advance(double dt) override;

  bool loadParameters(const TiXmlHandle& handle) override;

  virtual void setIsInStandConfiguration(bool isInStandConfiguration);
  bool setToInterpolated(const TorsoControlBase& torsoController1, const TorsoControlBase& torsoController2, double t) override;

  virtual bool addVariablesToLog(bool updateLogger);

  void setMainBodyDesiredHeightFromTerrain(double height);

 protected:
  GaitPatternBase& gaitPattern_;
};

} /* namespace loco */
