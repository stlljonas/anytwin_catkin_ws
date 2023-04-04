/*
 * TerrainModelHorizontalPlane.hpp
 *
 *  Created on: Apr 4, 2014
 *      Author: Christian Gehring, Dario Bellicoso
 */

#pragma once

// loco
#include "loco/common/TerrainModelBase.hpp"

namespace loco {

class TerrainModelHorizontalPlane : public TerrainModelBase {
 public:
  TerrainModelHorizontalPlane();
  ~TerrainModelHorizontalPlane() override = default;

  /*! Initializes the plane at zero height
   * @param dt  time step
   * @returns true
   */
  bool initialize(double dt) override;

  /*!
   * Load parameters.
   * @return true if successful
   */
  bool loadParameters(const TiXmlHandle& handle) override;

  /*! Gets the surface normal of the terrain at a certain position.
   * @param[in] positionWorldToLocationInWorldFrame the place to get the surface normal from (in the world frame)
   * @param[out] normalInWorldFrame the surface normal (in the world frame)
   * @return true if successful, false otherwise
   */
  bool getNormal(const loco::Position& positionWorldToLocationInWorldFrame, loco::Vector& normalInWorldFrame) const override;

  /*! Gets the height of the terrain at the coordinate (positionWorldToLocationInWorldFrame(), positionWorldToLocationInWorldFrame())
   * (in world frame) and sets the position.z() as the height (in world frame).
   * @param[in/out] positionWorldToLocationInWorldFrame   position from origin of world frame to the requested location expressed in world
   * frame
   * @return true if successful, false otherwise
   */
  bool getHeight(loco::Position& positionWorldToLocationInWorldFrame) const override;

  /*!Gets the height of the terrain at the coordinate
   * (in world frame) and sets the position.z() as the height (in world frame).
   * @param[in] positionWorldToLocationInWorldFrame   position from origin of world frame to the requested location expressed in world frame
   * @param[out]  height of the terrain in world frame
   * @return true if successful, false otherwise
   */
  bool getHeight(const loco::Position& positionWorldToLocationInWorldFrame, double& heightInWorldFrame) const override;

  /*! Gets the maximal height of the terrain on a line between two points
   * (in world frame) from the map and sets the height
   * @param[in/out] point1, point2 as the points for the line, height to output the maximal height of the terrain
   * @return true if successful, false otherwise
   */
  bool getMaxHeightBetweenTwoPoints(const loco::Position& point1, const loco::Position& point2, double& height) const override;

  /*! Gets the friction coefficient between foot and terrain at a certain location.
   * @param positionWorldToLocationInWorldFrame   position of the requested location expressed in world frame
   * @param frictionCoefficient   friction coefficient
   * @returns true if successful, false otherwise
   */
  bool getFrictionCoefficientForFoot(const loco::Position& positionWorldToLocationInWorldFrame, double& frictionCoefficient) const override;

  /*! Project the given position along the surface normal onto the terrain plane
   * @param positionWorldToLocationInWorldFrame   position to be projected in world frame
   * @returns the projection onto the terrain plane along the surface normal
   */
  Position getPositionProjectedOnPlaneAlongSurfaceNormalInWorldFrame(const Position& positionWorldToLocationInWorldFrame) const override;

  /*! Gets the distance to the terrain plane (as a strict positive value) of a point in world frame along the surface normal
   * @param positionWorldToLocationInWorldFrame   position for which distance shall be calculated
   * @returns the distance of the point to the terrain plane along the surface normal
   */
  double getDistanceFromSurfaceAlongSurfaceNormalToPositionInWorldFrame(const Position& positionWorldToLocationInWorldFrame) const override;

  /*! Gets the height above the terrain (can also be negative), signed version of
   *  getDistanceFromSurfaceAlongSurfaceNormalToPositionInWorldFrame()
   * @param positionWorldToLocationInWorldFrame   position for which height shall be calculated
   * @returns the height of the point above the terrain plane along the surface normal
   */
  double getHeightAboveTerrainAlongSurfaceNormal(const Position& positionWorldToLocationInWorldFrame) const override;

  /*! Sets the height of the plane expressed in world frame
   *
   * @param heightInWorldFrame
   */
  void setHeight(double heightInWorldFrame);

 protected:
  //! Height of the horizontal plane expressed in world frame
  double heightInWorldFrame_;
  //! Normal vector of the horizontal plane expressed in world frame
  loco::Vector normalInWorldFrame_;
  //! Friction coefficient between terrain and foot
  double frictionCoefficientBetweenTerrainAndFoot_;
};

} /* namespace loco */
