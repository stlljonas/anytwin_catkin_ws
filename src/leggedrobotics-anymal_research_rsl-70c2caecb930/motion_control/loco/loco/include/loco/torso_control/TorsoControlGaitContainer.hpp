/*
 * TorsoControlGaitContainer.hpp
 *
 *  Created on: Oct 29, 2014
 *      Author: Dario Bellicoso
 */

#pragma once

// loco
#include "loco/common/TerrainModelBase.hpp"
#include "loco/common/WholeBody.hpp"
#include "loco/common/loco_common.hpp"
#include "loco/common/torso/TorsoBase.hpp"
#include "loco/torso_control/ComSupportControlBase.hpp"
#include "loco/torso_control/TorsoControlBase.hpp"

// parameter handler
#include <parameter_handler/parameter_handler.hpp>

namespace loco {

class TorsoControlGaitContainer : public TorsoControlBase {
 public:
  TorsoControlGaitContainer(loco::WholeBody& wholeBody, loco::TerrainModelBase& terrain, loco::ComSupportControlBase& comControl);
  ~TorsoControlGaitContainer() override = default;

  bool initialize(double dt) override;

  /*! Load torso parameters from parameter file
   * @param hTorsoConfiguration A handle to the 'TorsoConfiguration' section in the parameter file
   * @returns true if successful
   */
  bool loadParameters(const TiXmlHandle& handle) override;

  /*! Return default Center of Mass height as defined in parameter file.
   * @returns the default height
   */
  virtual double getDesiredTorsoCoMHeightAboveGroundInControlFrameOffset() const;

  const ComSupportControlBase& getComSupportControl() const override;
  ComSupportControlBase* getComSupportControlPtr() override;

  /*! Log signals.
   *
   * @param ns Namespace prefix of all child signals.
   */
  void addVariablesToLog(const std::string& ns) override;

  bool addParametersToHandler(const std::string& ns) override;

  bool removeParametersFromHandler() override;

 protected:
  RotationQuaternion getOrientationBaseToFootprint() const;
  RotationQuaternion getOrientationWorldToHeadingOnTerrainSurface(const RotationQuaternion& orientationWorldToHeading) const;

  WholeBody& wholeBody_;
  Legs& legs_;
  TorsoBase& torso_;
  TerrainModelBase& terrain_;
  ComSupportControlBase& comControl_;

  double desiredTorsoCoMHeightAboveGroundInControlFrameOffset_;

  parameter_handler::Parameter<double> paramTorsoHeightAboveGround_;

  // The target position and velocity.
  loco::Vector positionControlToTargetBaseInWorldFrame_;
  loco::Vector linearVelocityTargetBaseInBaseFrame_;
};

} /* namespace loco */
