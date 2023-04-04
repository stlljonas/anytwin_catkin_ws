/*
 * StateBase.hpp
 *
 *  Created on: Feb 25, 2014
 *      Author: Christian Gehring, Dario Bellicoso
 */

#pragma once

// loco
#include "loco/common/ModuleBase.hpp"
#include "loco/common/torso/TorsoProperties.hpp"
#include "loco/common/torso/TorsoStateDesired.hpp"
#include "loco/common/torso/TorsoStateMeasured.hpp"
#include "loco/common/typedefs.hpp"

namespace loco {

//! Base class for a torso
/*! This should be used only as a data container
 *
 */
class TorsoBase : public ModuleBase {
 public:
  TorsoBase(const std::string& name, TorsoPropertiesPtr&& properties);
  ~TorsoBase() override = default;

  virtual const TorsoProperties& getProperties() const { return *torsoProperties_; }
  virtual TorsoProperties* getPropertiesPtr() { return torsoProperties_.get(); }

  virtual double getStridePhase() const { return stridePhase_; }
  virtual void setStridePhase(double stridePhase) { stridePhase_ = stridePhase; }

  virtual double getStrideDuration() const { return strideDuration_; }
  virtual void setStrideDuration(double strideDuration) { strideDuration_ = strideDuration; }

  virtual const TorsoStateDesired& getDesiredState() const { return *torsoStateDesired_; }
  virtual TorsoStateDesired* getDesiredStatePtr() { return torsoStateDesired_.get(); }

  virtual const TorsoStateMeasured& getMeasuredState() const { return *torsoStateMeasured_; }
  virtual TorsoStateMeasured* getMeasuredStatePtr() { return torsoStateMeasured_.get(); }

  bool addVariablesToLog(const std::string& ns) const override;
  void print(std::ostream& out) const override;

 protected:
  //! Properties of the torso
  TorsoPropertiesPtr torsoProperties_;
  //! Stride phase
  double stridePhase_;
  //! Stride duration
  double strideDuration_;
  //! Desired state of the torso
  TorsoStateDesiredPtr torsoStateDesired_;
  //! Measured state of the torso
  TorsoStateMeasuredPtr torsoStateMeasured_;
};

} /* namespace loco */
