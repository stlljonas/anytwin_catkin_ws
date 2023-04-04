/*!
 * @file	 HandRomo.hpp
 * @author Markus Staeuble
 * @date	 Jun, 2018
 */

#pragma once

// romo_measurements
#include "romo_measurements/end_effectors/EndEffectorRomo.hpp"

// loco
#include "loco/common/end_effectors/Hand.hpp"

namespace romo_measurements {

template <typename ConcreteDescription_, typename RobotState_, typename EndeffectorBase_ = loco::Hand>
class HandRomo : public romo_measurements::EndEffectorRomo<ConcreteDescription_, RobotState_, EndeffectorBase_> {
 protected:
  using RobotModel                = romo::RobotModel<ConcreteDescription_, RobotState_>;
  using RD                        = typename RobotModel::RD;
  using ContactEnum               = typename RD::ContactEnum;
  using BodyEnum                  = typename RD::BodyEnum;
  using TimeInstant               = loco::EndEffectorBase::TimeInstant;
  using EndEffectorFrame          = loco::EndEffectorBase::EndEffectorFrame;
  using ContactsMap               = std::unordered_map<EndEffectorFrame, ContactEnum>;

 public:
  HandRomo(BodyEnum bodyEnum, unsigned int numFingers, const std::string& name,
           const RobotModel& model, loco::EndEffectorPropertiesPtr&& endeffectorProperties,
           const ContactsMap & contactPointsMap,
           const std::vector<TimeInstant>& timeInstants =
               {loco::TimePoint::Now, loco::TimePointGait::LiftOff, loco::TimePointGait::TouchDown},
           bool autoAdvanceOfContactPoints = true);

  HandRomo(BodyEnum bodyEnum, unsigned int numFingers, const std::string& name, const RobotModel& model,
           const ContactsMap & contactPointsMap,
           const std::vector<TimeInstant>& timeInstants =
               {loco::TimePoint::Now, loco::TimePointGait::LiftOff, loco::TimePointGait::TouchDown},
           bool autoAdvanceOfContactPoints = true);

  virtual ~HandRomo() override = default;

  bool initialize(double dt) override;

  bool advance(double dt) override;

  virtual bool advanceFingers() = 0;

  const ContactsMap& getContactPointsMap() const {
    return this->contactPointsMap_;
  }

 protected:
  using EndEffectorRomo = romo_measurements::EndEffectorRomo<ConcreteDescription_, RobotState_, EndeffectorBase_>;
};

}  // namespace romo_measurements

#include "romo_measurements/end_effectors/HandRomo.tpp"
