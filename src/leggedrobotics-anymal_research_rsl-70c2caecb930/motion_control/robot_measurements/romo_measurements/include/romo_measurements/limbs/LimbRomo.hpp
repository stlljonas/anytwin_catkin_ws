/*!
 * @file    LimbRomo.hpp
 * @author  Gabriel Hottiger
 * @date	  Nov, 2017
 */

#pragma once

// loco
#include "loco/common/limbs/LimbBase.hpp"
#include "loco/common/limbs/LimbProperties.hpp"
#include "loco/common/legs/LegBase.hpp"
#include "loco/common/legs/LegProperties.hpp"
#include "loco/common/end_effectors/EndEffectorBase.hpp"
#include "loco/common/end_effectors/FootBase.hpp"

// romo
#include "romo/RobotModel.hpp"
#include "romo/ExtendedRobotState.hpp"

namespace romo_measurements {

template <typename ConcreteDescription_, typename RobotState_, typename LimbBase_ = loco::LimbBase>
class LimbRomo: public LimbBase_ {

  static_assert(std::is_base_of<loco::LimbBase, LimbBase_>::value,
                "[LimbRomo]: LimbBase_ must derive from loco::LimbBase");

 protected:
  using RobotModel          = romo::RobotModel<ConcreteDescription_, RobotState_>;
  using RD                  = typename RobotModel::RD;
  using BranchEnum          = typename RD::BranchEnum;
  using LimbEnum            = typename RD::LimbEnum;
  using BodyEnum            = typename RD::BodyEnum;
  using JointEnum           = typename RD::JointEnum;
  using CoordinateFrameEnum = typename RD::CoordinateFrameEnum;

 public:
  template <typename _LimbBase = LimbBase_>
  LimbRomo(LimbEnum limb,
           const std::string& name,
           const RobotModel& model,
           loco::LimbPropertiesPtr&& properties,
           loco::EndEffectorBasePtr&& endEffector,
           bool advanceDynamics = false,
           typename std::enable_if<!std::is_same<loco::LegBase, _LimbBase>::value>::type* = 0);

  template <typename _LimbBase = LimbBase_>
  LimbRomo(LimbEnum limb,
           const std::string& name,
           const RobotModel& model,
           loco::LimbPropertiesPtr&& properties,
           loco::EndEffectorBasePtr&& endEffector,
           loco::LimbStateMeasuredPtr&& stateMeasured,
           loco::LimbStateDesiredPtr&& stateDesired,
           bool advanceDynamics = false,
           typename std::enable_if<!std::is_same<loco::LegBase, _LimbBase>::value>::type* = 0);

  template <typename _LimbBase = LimbBase_>
  LimbRomo(LimbEnum limb,
           const std::string& name,
           const RobotModel& model,
           loco::LegPropertiesPtr&& properties,
           loco::FootBasePtr&& endEffector,
           bool advanceDynamics = false,
           typename std::enable_if<std::is_same<loco::LegBase, _LimbBase>::value>::type* = 0);

  template <typename _LimbBase = LimbBase_>
  LimbRomo(LimbEnum limb,
           const std::string& name,
           const RobotModel& model,
           loco::LegPropertiesPtr&& properties,
           loco::FootBasePtr&& endEffector,
           loco::LimbStateMeasuredPtr&& stateMeasured,
           loco::LimbStateDesiredPtr&& stateDesired,
           bool advanceDynamics = false,
           typename std::enable_if<std::is_same<loco::LegBase, _LimbBase>::value>::type* = 0);

  ~LimbRomo() override = default;

  // Getters for limb definitions
  unsigned int getNumDofLimb() const override { return RD::getNumDofLimb(limbEnum_); }
  unsigned int getNumDofU() const override    { return RD::getGeneralizedVelocitiesDimension(); }
  unsigned int getBranchUInt() const override { return RD::mapKeyEnumToKeyId(branchEnum_); }
  unsigned int getLimbUInt() const override   { return RD::mapKeyEnumToKeyId(limbEnum_); }
  unsigned int getId() const override         { return getLimbUInt(); }

  void create();
  bool initialize(double dt) override;
  bool advance(double dt) override;

  bool addVariablesToLog(const std::string& ns) const override;

 private:
  template <typename _ConcreteDescription = ConcreteDescription_, typename _RobotState = RobotState_>
  void setExtendedRobotState(typename std::enable_if<std::is_base_of<romo::ExtendedRobotState<_ConcreteDescription>, _RobotState>::value>::type* = 0)
  {
    this->getLimbStateMeasuredPtr()->setJointTorques(model_.getState().getJointTorques().getSegment(
      limbStartIndexInJ_, this->getNumDofLimb()).toImplementation());

    this->getLimbStateMeasuredPtr()->setJointAccelerations(model_.getState().getJointAccelerations().getSegment(
            limbStartIndexInJ_, this->getNumDofLimb()).toImplementation());
  }

  template <typename _ConcreteDescription = ConcreteDescription_, typename _RobotState = RobotState_>
  void setExtendedRobotState(typename std::enable_if<!std::is_base_of<romo::ExtendedRobotState<_ConcreteDescription>, _RobotState>::value>::type* = 0)
  {

  }

 protected:
  const RobotModel & model_;
  const LimbEnum limbEnum_;
  const BranchEnum branchEnum_;
  const BodyEnum branchStartBody_;
  const unsigned int limbStartIndexInJ_;
  const unsigned int branchStartIndexInU_;
  const bool advanceDynamics_;

};

}  // namespace romo_measurements

#include "romo_measurements/limbs/LimbRomo.tpp"