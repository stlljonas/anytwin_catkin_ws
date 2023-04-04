/*!
 * @file	 EndEffectorRomo.hpp
 * @author Gabriel Hottiger
 * @date	 Nov, 2017
 */

#pragma once

// loco
#include "loco/common/end_effectors/EndEffectorBase.hpp"
#include "loco/common/end_effectors/FootBase.hpp"
#include "loco/common/end_effectors/Wheel.hpp"
#include "loco/common/end_effectors/Hand.hpp"
#include "loco/common/typedefs.hpp"

// romo_measurements
#include "romo_measurements/end_effectors/EndEffectorPropertiesRomo.hpp"
#include "romo_measurements/end_effectors/WheelPropertiesRomo.hpp"

// romo
#include "romo/RobotModel.hpp"

// STL
#include <unordered_map>

namespace romo_measurements {

template <typename ConcreteDescription_, typename RobotState_, typename EndEffectorBase_ = loco::EndEffectorBase >
class EndEffectorRomo : public EndEffectorBase_ {

  static_assert(std::is_base_of<loco::EndEffectorBase, EndEffectorBase_>::value,
                "[EndEffectorRomo]: EndEffectorBase_ must derive from loco::EndEffectorBase");

 protected:
  using RobotModel                = romo::RobotModel<ConcreteDescription_, RobotState_>;
  using RD                        = typename RobotModel::RD;
  using ContactEnum               = typename RD::ContactEnum;
  using BranchEnum                = typename RD::BranchEnum;
  using LimbEnum                  = typename RD::LimbEnum;
  using BodyEnum                  = typename RD::BodyEnum;
  using BodyNodeEnum              = typename RD::BodyNodeEnum;
  using CoordinateFrameEnum       = typename RD::CoordinateFrameEnum ;
  using TimeInstant               = loco::EndEffectorBase::TimeInstant;
  using EndEffectorFrame          = loco::EndEffectorBase::EndEffectorFrame;
  using ContactsMap               = std::unordered_map<EndEffectorFrame, ContactEnum>;
  using EndEffectorPropertiesRomo = romo_measurements::EndEffectorPropertiesRomo<ConcreteDescription_, RobotState_>;
  using WheelPropertiesRomo       = romo_measurements::WheelPropertiesRomo<ConcreteDescription_, RobotState_>;

 public:
  /**
   * @brief Constructor for an implementation of loco::EndeffectorBase
   * @tparam _EndEffectorBase Necessary for std::enable_if
   * @param bodyEnum  Endeffector body
   * @param name  Name of the endeffector
   * @param model Romo model
   * @param endeffectorProperties Endeffector properties ptr
   * @param contactPointsMap You can provide a arbitrary number of contacts using this map
   * @param autoAdvanceOfContactPoints Set true to advance contact points during advance(dt)
   *
   */
  template <typename _EndEffectorBase = EndEffectorBase_>
  EndEffectorRomo(BodyEnum bodyEnum,
                  const std::string& name,
                  const RobotModel& model,
                  loco::EndEffectorPropertiesPtr&& endeffectorProperties,
                  const ContactsMap & contactPointsMap,
                  const std::vector<TimeInstant>& timeInstants = {loco::TimePoint::Now},
                  bool autoAdvanceOfContactPoints = true,
                  typename std::enable_if<std::is_same<loco::EndEffectorBase, _EndEffectorBase>::value>::type* = 0);

  template <typename _EndeffectorBase = EndEffectorBase_>
  EndEffectorRomo(BodyEnum bodyEnum,
                  const std::string& name,
                  const RobotModel& model,
                  const ContactsMap & contactPointsMap,
                  const std::vector<TimeInstant>& timeInstants = {loco::TimePoint::Now},
                  bool autoAdvanceOfContactPoints = true,
                  typename std::enable_if<std::is_same<loco::EndEffectorBase, _EndeffectorBase>::value>::type* = 0);

  /**
   * @brief Constructor for an implementation of loco::FootBase
   * @tparam _EndeffectorBase Necessary for std::enable_if
   * @param bodyEnum  Endeffector body
   * @param name  Name of the endeffector
   * @param model Romo model
   * @param endeffectorProperties Endeffector properties ptr
   * @param contactPointsMap You can provide a arbitrary number of contacts using this map
   * @param autoAdvanceOfContactPoints Set true to advance contact points during advance(dt)
   */
  template <typename _EndeffectorBase = EndEffectorBase_>
  EndEffectorRomo(BodyEnum bodyEnum,
                  const std::string& name,
                  const RobotModel& model,
                  loco::EndEffectorPropertiesPtr&& endeffectorProperties,
                  const ContactsMap & contactPointsMap,
                  const std::vector<TimeInstant>& timeInstants = {loco::TimePoint::Now, loco::TimePointGait::LiftOff, loco::TimePointGait::TouchDown},
                  bool autoAdvanceOfContactPoints = true,
                  typename std::enable_if<std::is_same<loco::FootBase, _EndeffectorBase>::value>::type* = 0);

  template <typename _EndeffectorBase = EndEffectorBase_>
  EndEffectorRomo(BodyEnum bodyEnum,
                  const std::string& name,
                  const RobotModel& model,
                  const ContactsMap & contactPointsMap,
                  const std::vector<TimeInstant>& timeInstants = {loco::TimePoint::Now, loco::TimePointGait::LiftOff, loco::TimePointGait::TouchDown},
                  bool autoAdvanceOfContactPoints = true,
                  typename std::enable_if<std::is_same<loco::FootBase, _EndeffectorBase>::value>::type* = 0);
  /**
   * @brief Constructor for an implementation of loco::Wheel
   * @tparam _EndeffectorBase Necessary for std::enable_if
   * @param bodyEnum  Endeffector body
   * @param name  Name of the endeffector
   * @param model Romo model
   * @param endeffectorProperties Endeffector properties ptr
   * @param contactPointsMap You can provide a arbitrary number of contacts using this map
   * @param autoAdvanceOfContactPoints Set true to advance contact points during advance(dt)
   */
  template <typename _EndeffectorBase = EndEffectorBase_>
  EndEffectorRomo(BodyEnum bodyEnum,
                  const std::string& name,
                  const RobotModel& model,
                  loco::WheelPropertiesPtr && wheelProperties,
                  const ContactsMap & contactPointsMap,
                  const std::vector<TimeInstant>& timeInstants = {loco::TimePoint::Now, loco::TimePointGait::LiftOff, loco::TimePointGait::TouchDown},
                  bool autoAdvanceOfContactPoints = true,
                  typename std::enable_if<std::is_same<loco::Wheel, _EndeffectorBase>::value>::type* = 0);

  template <typename _EndeffectorBase = EndEffectorBase_>
  EndEffectorRomo(BodyEnum bodyEnum,
                  const std::string& name,
                  const RobotModel& model,
                  const ContactsMap & contactPointsMap,
                  const std::vector<TimeInstant>& timeInstants = {loco::TimePoint::Now, loco::TimePointGait::LiftOff, loco::TimePointGait::TouchDown},
                  bool autoAdvanceOfContactPoints = true,
                  typename std::enable_if<std::is_same<loco::Wheel, _EndeffectorBase>::value>::type* = 0);

  /**
   * @brief Constructor for an implementation of loco::Hand
   * @tparam _EndeffectorBase Necessary for std::enable_if
   * @param bodyEnum  Endeffector body
   * @param name  Name of the endeffector
   * @param model Romo model
   * @param endeffectorProperties Endeffector properties ptr
   * @param contactPointsMap You can provide a arbitrary number of contacts using this map
   * @param autoAdvanceOfContactPoints Set true to advance contact points during advance(dt)
   */
  template <typename _EndeffectorBase = EndEffectorBase_>
  EndEffectorRomo(BodyEnum bodyEnum,
                  unsigned int numFingers,
                  const std::string& name,
                  const RobotModel& model,
                  loco::EndEffectorPropertiesPtr && endEffectorProperties,
                  const ContactsMap & contactPointsMap,
                  const std::vector<TimeInstant>& timeInstants = {loco::TimePoint::Now, loco::TimePointGait::LiftOff, loco::TimePointGait::TouchDown},
                  bool autoAdvanceOfContactPoints = true,
                  typename std::enable_if<std::is_same<loco::Hand, _EndeffectorBase>::value>::type* = 0);

  template <typename _EndeffectorBase = EndEffectorBase_>
  EndEffectorRomo(BodyEnum bodyEnum,
                  unsigned int numFingers,
                  const std::string& name,
                  const RobotModel& model,
                  const ContactsMap & contactPointsMap,
                  const std::vector<TimeInstant>& timeInstants = {loco::TimePoint::Now, loco::TimePointGait::LiftOff, loco::TimePointGait::TouchDown},
                  bool autoAdvanceOfContactPoints = true,
                  typename std::enable_if<std::is_same<loco::Hand, _EndeffectorBase>::value>::type* = 0);

  /**
   * @brief Default Destructor
   */
  ~EndEffectorRomo() override = default;

  /**
   * @brief Creates the measured an desired endeffector states for every contact enum in additionalContactsMap_
   * @tparam EndeffectorStateDesired_ Desired state type
   * @tparam EndeffectorStateMeasured_ Measured state type
   * @param defaultEndeffectorStateDesired Default state desired
   * @param defaultEndeffectorStateMeasured Default state measured
   * @return true iff successful
   */
  template<typename EndeffectorStateDesired_, typename EndeffectorStateMeasured_>
  bool create(const EndeffectorStateDesired_& defaultEndeffectorStateDesired,
              const EndeffectorStateMeasured_& defaultEndeffectorStateMeasured,
              const std::vector<TimeInstant>& timeInstants);


  // Implementation of EndeffectorBase interface
  bool initialize(double dt) override;
  bool advance(double dt) override;
  bool advanceContactPoints(double dt) override;

  loco::JointPositions getJointPositionsFromPositionBaseToEndEffectorInBaseFrame(
    const loco::Position& positionBaseToEndEffectorInBaseFrame) override;
  loco::JointVelocities getJointVelocitiesFromLinearVelocityBaseToEndEffectorInBaseFrame(
    const loco::LinearVelocity& velocity) override;
  loco::JointPositions getJointPositionsFromPositionBaseToEndEffectorInBaseFrameIteratively(
    const loco::Position& positionBaseToEndEffectorInBaseFrame) override;

  loco::Position getPositionWorldToEndEffectorInWorldFrame(const loco::JointPositions& jointPositions) override;
  loco::Position getPositionWorldToEndEffectorInBaseFrame(const loco::JointPositions& jointPositions) override;
  loco::Position getPositionBaseToEndEffectorInBaseFrame(const loco::JointPositions& jointPositions) override;

  bool isInContact() const override;
  bool isSlipping() const override;

 protected:
  static bool isContactAtOrigin(const ContactsMap & additionalContactPointsMap) {
    return (additionalContactPointsMap.size() == 1) &&
        (additionalContactPointsMap.begin()->first == loco::EndEffectorEnum::Origin);
  }

 protected:
  const BranchEnum branchEnum_;
  const LimbEnum limbEnum_;
  const BodyEnum bodyEnum_;
  const BodyNodeEnum bodyNodeEnum_;
  const RobotModel& model_;
  const ContactsMap contactPointsMap_;

  bool autoAdvanceOfContactPoints_;
};

}  // namespace romo_measurements

#include "romo_measurements/end_effectors/EndEffectorRomo.tpp"
