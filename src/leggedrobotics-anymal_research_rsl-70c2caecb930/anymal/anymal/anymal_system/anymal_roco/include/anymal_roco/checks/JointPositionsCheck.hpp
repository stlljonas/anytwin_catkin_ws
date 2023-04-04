/**
 * @authors      Alexander Reske
 * @affiliation  ANYbotics AG
 * @brief        Class for checking for desired joint positions with some error tolerance.
 */

#pragma once

// eigen
#include <Eigen/Core>

// anymal description
#include <anymal_description/AnymalDescription.hpp>

// anymal_roco
#include "anymal_roco/checks/StateCheckBase.hpp"

namespace anymal_roco {

class JointPositionsCheck : public StateCheckBase {

 public:
  using JointVector = Eigen::Matrix<double, anymal_description::AnymalDescription::getJointsDimension(), 1>;

  //! Constructor
  JointPositionsCheck(JointVector desiredJointPositions, JointVector allowedAbsoluteJointPositionErrors) : desiredJointPositions_(desiredJointPositions), allowedAbsoluteJointPositionErrors_(allowedAbsoluteJointPositionErrors) {};

  //! Default Destructor
  ~JointPositionsCheck() override = default;

  //! Check state
  bool check(const RocoState& state, boost::shared_mutex& stateMutex) const override;

 private:
  JointVector desiredJointPositions_;
  JointVector allowedAbsoluteJointPositionErrors_;

};

} /* namespace anymal_roco */
