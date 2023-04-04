/*
 * TerrainModelFreePlane.hpp
 *
 *  Created on: Aug 28, 2014
 *      Author: C. Dario Bellicoso
 */

#pragma once

// loco
#include "loco/common/TerrainModelPlane.hpp"

class TiXmlHandle;

namespace loco {

class TerrainModelFreePlane : public TerrainModelPlane {
 public:
  TerrainModelFreePlane();
  ~TerrainModelFreePlane() override = default;

  /*! Initializes the plane at zero height and normal parallel to z-axis in world frame
   * @param dt  time step
   * @returns true
   */
  bool initialize(double dt) override;

  /*!
   * Load parameters.
   * @return true if successful
   */
  bool loadParameters(const TiXmlHandle& handle) override;

  /*!
   * Add variables to signal logger.
   * @return true if successful
   */
  bool addVariablesToLog(const std::string& ns) const override;

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

  /*! Gets the maximal height of the terrain on a line between two points
   * (in world frame) from the map and sets the height
   * @param[in/out] point1, point2 as the points for the line, height to output the maximal height of the terrain
   * @return true if successful, false otherwise
   */
  bool getMaxHeightBetweenTwoPoints(const loco::Position& point1, const loco::Position& point2, double& height) const override;

  /*! Gets the height of the terrain at the coordinate (positionWorldToLocationInWorldFrame(), positionWorldToLocationInWorldFrame())
   * (in world frame) and sets heightInWorldFrame as the height (in world frame).
   * @param[in] positionWorldToLocationInWorldFrame   position from origin of world frame to the requested location expressed in world frame
   * @param[out] heightInWorldFrame   height in world frame evaluated at position positionWorldToLocationInWorldFrame
   * @return true if successful, false otherwise
   */
  bool getHeight(const loco::Position& positionWorldToLocationInWorldFrame, double& heightInWorldFrame) const override;

  /*! Return friction coefficient for a foot at a certain position.
   * @param[in] positionWorldToLocationInWorldFrame position from origin of world frame to the requested location expressed in world frame
   * @param[out] frictionCoefficient friction coefficient evaluated at the given position
   * @return true if successful, false otherwise
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

  /*! Set free plane parameters.
   * @param normal Normal vector to plane in world frame
   * @param position Point on plane in world frame
   */
  void setNormalandPositionInWorldFrame(const loco::Vector& normal, const loco::Position& position) override;

 protected:
  double frictionCoefficientBetweenTerrainAndFoot_;

  // Plane properties
  loco::Vector normalInWorldFrame_;      // normal to the plane
  loco::Position positionInWorldFrame_;  // position of a point on the plane
  loco::EulerAnglesZyx orientationPlaneToWorld_;

};  // class

} /* namespace loco */
