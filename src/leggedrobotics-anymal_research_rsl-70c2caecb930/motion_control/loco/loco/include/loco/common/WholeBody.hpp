/*
 * WholeBody.hpp
 *
 *  Created on: Feb 8, 2016
 *      Author: Dario Bellicoso, Gabriel Hottiger
 */

#pragma once

// loco
#include "loco/common/ModuleBase.hpp"
#include "loco/common/typedefs.hpp"

#include "loco/common/arms/Arms.hpp"
#include "loco/common/legs/Legs.hpp"
#include "loco/common/limbs/Limbs.hpp"
#include "loco/common/torso/TorsoBase.hpp"

#include "loco/common/WholeBodyProperties.hpp"
#include "loco/common/WholeBodyStateDesired.hpp"
#include "loco/common/WholeBodyStateMeasured.hpp"

// stl
#include <type_traits>

namespace loco {

class WholeBody : public ModuleBase {
 public:
  template <typename LegGroup_ = Legs, typename ArmGroup_ = Arms>
  WholeBody(TorsoBase& torso, WholeBodyPropertiesPtr&& wholeBodyProperties, const LegGroup_& legs = LegGroup_(),
            const ArmGroup_& arms = ArmGroup_(), bool updateDynamics = false)
      : torso_(torso),
        legs_(legs.template make_observer_container_of_type<LegBase>()),
        arms_(arms.template make_observer_container_of_type<ArmBase>()),
        limbs_(),
        wholeBodyStateDesired_(),
        wholeBodyStateMeasured_(),
        wholeBodyProperties_(std::move(wholeBodyProperties)),
        updateDynamics_(updateDynamics),
        wholeBodyMassMatrix_(),
        wholeBodyNonlinearEffects_(),
        wholeBodyGravityTerms_() {
    limbs_.clear();
    limbs_.insert(limbs_.s_end(), legs_.s_begin(), legs_.s_end());
    limbs_.insert(limbs_.s_end(), arms_.s_begin(), arms_.s_end());
  }

  ~WholeBody() override = default;

  virtual const TorsoBase& getTorso() const { return torso_; }
  virtual const Legs& getLegs() const { return legs_; }
  virtual const Arms& getArms() const { return arms_; }
  virtual const Limbs& getLimbs() const { return limbs_; }

  virtual TorsoBase* getTorsoPtr() { return &torso_; }
  virtual Legs* getLegsPtr() { return &legs_; }
  virtual Arms* getArmsPtr() { return &arms_; }
  virtual Limbs* getLimbsPtr() { return &limbs_; }

  virtual const WholeBodyStateDesired& getWholeBodyStateDesired() const { return wholeBodyStateDesired_; }
  virtual const WholeBodyStateMeasured& getWholeBodyStateMeasured() const { return wholeBodyStateMeasured_; }

  virtual WholeBodyStateDesired* getWholeBodyStateDesiredPtr() { return &wholeBodyStateDesired_; }
  virtual WholeBodyStateMeasured* getWholeBodyStateMeasuredPtr() { return &wholeBodyStateMeasured_; }

  WholeBodyProperties* getWholeBodyPropertiesPtr() { return wholeBodyProperties_.get(); }
  const WholeBodyProperties& getWholeBodyProperties() const { return *wholeBodyProperties_; }

  /**
   * @return true, iff all legs in ground contact
   */
  bool allLegsGrounded();

  bool isUpdatingDynamics() const { return updateDynamics_; }
  const Eigen::MatrixXd& getWholeBodyMassMatrix() const { return wholeBodyMassMatrix_; }
  const Eigen::VectorXd& getWholeBodyNonlinearEffects() const { return wholeBodyNonlinearEffects_; }
  const Eigen::VectorXd& getWholeBodyGravityTerms() const { return wholeBodyGravityTerms_; }
  void setWholeBodyMassMatrix(const Eigen::MatrixXd& wholeBodyMassMatrix) { wholeBodyMassMatrix_ = wholeBodyMassMatrix; }
  void setWholeBodyNonlinearEffects(const Eigen::VectorXd& wholeBodyNonlinearEffects) {
    wholeBodyNonlinearEffects_ = wholeBodyNonlinearEffects;
  }
  void setWholeBodyGravityTerms(const Eigen::VectorXd& wholeBodyGravityTerms) { wholeBodyGravityTerms_ = wholeBodyGravityTerms; }

  bool addVariablesToLog(const std::string& ns) const override;

 protected:
  //! Reference to torso.
  TorsoBase& torso_;

  /** The following pointer containers are local copies of the one owned by the controller,
   *  however the pointed to data (legs, arms, limbs) is the same.
   */
  //! Legs group container.
  Legs legs_;

  //! Arms group container.
  Arms arms_;

  //! Limbs group container (combining legs_ and arms_).
  Limbs limbs_;

  //! The whole body desired state.
  WholeBodyStateDesired wholeBodyStateDesired_;

  //! The whole body measured state.
  WholeBodyStateMeasured wholeBodyStateMeasured_;

  //! Properties of the whole body.
  WholeBodyPropertiesPtr wholeBodyProperties_;

  //! Flag deciding if dynamics should be updated
  bool updateDynamics_;

  //! Whole Body Dynamics
  Eigen::MatrixXd wholeBodyMassMatrix_;
  Eigen::VectorXd wholeBodyNonlinearEffects_;
  Eigen::VectorXd wholeBodyGravityTerms_;
};

} /* namespace loco */
