/*
 * TorsoControlZmp.hpp
 *
 *  Created on: Oct 9, 2014
 *      Author: C. Dario Bellicoso
 */

#pragma once

// loco
#include "loco/torso_control/TorsoControlGaitContainer.hpp"
#include "loco/common/WholeBody.hpp"

// motion_generation_loco
#include "loco/torso_control/ComSupportControlZmp.hpp"

class TiXmlHandle;

namespace loco {

class TorsoControlZmp: public TorsoControlGaitContainer {
 private:
  using Base = TorsoControlGaitContainer;

 public:
  TorsoControlZmp(WholeBody& wholeBody,
                  TerrainModelBase& terrain,
                  ComSupportControlZmp& comControl);
  ~TorsoControlZmp() override = default;

  bool loadParameters(const TiXmlHandle& handle) override;
  bool advance(double dt) override;

  virtual bool setToInterpolated(const TorsoControlBase& torsoController1, const TorsoControlBase& torsoController2, double t);

  bool addParametersToHandler(const std::string& ns) override;
  bool removeParametersFromHandler() override;

 protected:

  // Get a reference to the com control module.
  inline ComSupportControlZmp& getComControlRef();
};

} /* namespace loco */
