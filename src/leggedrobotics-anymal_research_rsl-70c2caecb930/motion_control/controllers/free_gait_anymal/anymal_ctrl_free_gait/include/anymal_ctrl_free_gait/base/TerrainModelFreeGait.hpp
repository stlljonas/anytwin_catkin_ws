/*
 * TerrainModelFreeGait.hpp
 *
 *  Created on: Feb 17, 2016
 *      Author: P. Fankhauser
 */

#pragma once

// Loco
#include <loco/common/TerrainModelFreePlane.hpp>

// FreeGait
#include <free_gait_core/executor/Executor.hpp>

namespace loco {

class TerrainModelFreeGait : public TerrainModelFreePlane {
 private:
  using AD = anymal_description::AnymalDescription;

 public:
  explicit TerrainModelFreeGait(free_gait::Executor& executor) : TerrainModelFreePlane(), executor_(executor) {}
  ~TerrainModelFreeGait() override = default;

  /*!
   * Gets the surface normal of the terrain for a certain contact of the robot.
   * @param[in] limb defines the contact point.
   * @param[out] normalInWorldFrame the surface normal (in the world frame).
   * @return true if successful, false otherwise.
   */
  bool getNormal(const AD::LimbEnum limb, loco::Vector& normalInWorldFrame) const;

  /*!
   * Gets the surface normal of the terrain at a certain position.
   * @param[in] positionInWorldFrame the place to get the surface normal from (in the world frame).
   * @param[out] normalInWorldFrame the surface normal (in the world frame).
   * @return true if successful, false otherwise.
   */
  bool getNormal(const loco::Position& positionInWorldFrame, loco::Vector& normalInWorldFrame) const override;

  /*!
   * Return friction coefficient for a foot at a certain position.
   * @param[in] positionInWorldFrame position from origin of world frame to the requested location expressed in world frame.
   * @param[out] frictionCoefficient friction coefficient evaluated at the given position.
   * @return true if successful, false otherwise.
   */
  bool getFrictionCoefficientForFoot(const loco::Position& positionInWorldFrame, double& frictionCoefficient) const override;

 private:
  free_gait::Executor& executor_;
  const Vector defaultFootNormalInWorldFrame_ = Vector::UnitZ();
};

} /* namespace loco */
