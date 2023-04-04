/*
 * AnymalFramesGenerator.hpp
 *
 *  Created on: Feb 27, 2018
 *      Author: Dario Bellicoso
 */

#pragma once

// anymal model
#include "anymal_model/AnymalFramesGeneratorBase.hpp"
#include "anymal_model/typedefs.hpp"

// anymal description
#include <anymal_description/AnymalDescription.hpp>
#include <anymal_description/AnymalTopology.hpp>

namespace anymal_model {

//! Generates coordinate frame based on the state of the anymal.
template <typename ConcreteAnymalDescription_, typename AnymalState_>
class AnymalFramesGenerator : public AnymalFramesGeneratorBase<ConcreteAnymalDescription_, AnymalState_> {
 public:
  using Base = AnymalFramesGeneratorBase<ConcreteAnymalDescription_, AnymalState_>;
  using typename Base::RobotModel;

  using AD = romo::RobotDescription<ConcreteAnymalDescription_>;
  using AT = typename AD::ConcreteTopology;

  using BranchEnum = typename AT::BranchEnum;
  using ContactEnum = typename AT::ContactEnum;
  using FootEnum = typename AT::FootEnum;

  AnymalFramesGenerator() = default;
  ~AnymalFramesGenerator() override = default;

  void resetDerived() override;

  /*! Updates the frames
   * @param model with the state of the robot and the contact states
   */
  void update(const RobotModel& model) override;

 protected:
  void getHeadingDirectionFootprint(kindr::Position3D& headingInWorldFrame, const RobotModel& model);
  void getHeadingDirectionFeetcenter(kindr::Position3D& headingInWorldFrame, const RobotModel& model);

  //! Position of the feet with respect to the world frame last time they were in contact.
  std_utils::EnumArray<FootEnum, kindr::Position3D> positionsWorldToFootInWorldFrame_;
  //! Boolean storing whether all foot positions have been initialized (= have been in contact at least once).
  bool allPositionsWorldToFootInWorldFrameInitialized_ = false;
};

}  // namespace anymal_model

#include <anymal_model/AnymalFramesGenerator.tpp>
