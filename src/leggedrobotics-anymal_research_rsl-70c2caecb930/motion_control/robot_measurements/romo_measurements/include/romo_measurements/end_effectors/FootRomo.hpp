/*!
 * @file	 FootRomo.hpp
 * @author Gabriel Hottiger, Dario Bellicoso
 * @date	 Nov, 2017
 */

#pragma once

// romo_measurements
#include "romo_measurements/end_effectors/EndEffectorRomo.hpp"

// loco
#include "loco/common/end_effectors/FootBase.hpp"

namespace romo_measurements {

template <typename ConcreteDescription_, typename RobotState_, typename EndeffectorBase_ = loco::FootBase>
class FootRomo : public romo_measurements::EndEffectorRomo<ConcreteDescription_, RobotState_, EndeffectorBase_> {
 protected:
  using RobotModel                = romo::RobotModel<ConcreteDescription_, RobotState_>;
  using RD                        = typename RobotModel::RD;
  using ContactEnum               = typename RD::ContactEnum;
  using BodyEnum                  = typename RD::BodyEnum;
  using TimeInstant               = loco::EndEffectorBase::TimeInstant;
  using EndEffectorFrame          = loco::EndEffectorBase::EndEffectorFrame;
  using ContactsMap               = std::unordered_map<EndEffectorFrame, ContactEnum>;

 public:
  FootRomo(BodyEnum bodyEnum, const std::string& name,
           const RobotModel& model, loco::EndEffectorPropertiesPtr&& endeffectorProperties,
           const ContactsMap & contactPointsMap,
           const std::vector<TimeInstant>& timeInstants =
               {loco::TimePoint::Now, loco::TimePointGait::LiftOff, loco::TimePointGait::TouchDown},
           bool autoAdvanceOfContactPoints = true);

  FootRomo(BodyEnum bodyEnum, const std::string& name, const RobotModel& model,
           const ContactsMap & contactPointsMap,
           const std::vector<TimeInstant>& timeInstants =
               {loco::TimePoint::Now, loco::TimePointGait::LiftOff, loco::TimePointGait::TouchDown},
           bool autoAdvanceOfContactPoints = true);

  bool initialize(double dt) override;

  const ContactsMap& getContactPointsMap() const {
    return this->contactPointsMap_;
  }

 protected:
  using EndEffectorRomo = romo_measurements::EndEffectorRomo<ConcreteDescription_, RobotState_, EndeffectorBase_>;
};

}  // namespace romo_measurements

#include "romo_measurements/end_effectors/FootRomo.tpp"
