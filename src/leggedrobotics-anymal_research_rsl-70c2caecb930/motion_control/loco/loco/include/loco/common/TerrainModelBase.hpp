/*
 * TerrainModelBase.hpp
 *
 *  Created on: Apr 4, 2014
 *      Author: Christian Gehring, Dario Bellicoso
 */

#pragma once

// tinyxml
#include <tinyxml.h>

// loco
#include "loco/common/typedefs.hpp"

namespace loco {

/*! Base class for model of the terrain used for control
 */
class TerrainModelBase {
 public:
  //! Constructor
  TerrainModelBase() = default;

  //! Destructor
  virtual ~TerrainModelBase() = default;

  /*! Initializes the model
   * @param dt time step
   * @returns true if initialization was successful
   */
  virtual bool initialize(double dt) = 0;

  /*!
   * Load parameters.
   * @return true if successful
   */
  virtual bool loadParameters(const TiXmlHandle& handle) = 0;

  /*! Gets the surface normal of the terrain at a certain position.
   * @param[in] positionWorldToLocationInWorldFrame the place to get the surface normal from (in the world frame)
   * @param[out] normalInWorldFrame the surface normal (in the world frame)
   * @return true if successful, false otherwise
   */
  virtual bool getNormal(const loco::Position& positionWorldToLocationInWorldFrame, loco::Vector& normalInWorldFrame) const = 0;

  /*! Gets the height of the terrain at the coordinate (positionWorldToLocationInWorldFrame(), positionWorldToLocationInWorldFrame())
   * (in world frame) and sets the position.z() as the height (in world frame).
   * @param[in/out] positionWorldToLocationInWorldFrame   position from origin of world frame to the requested location expressed in world
   * frame
   * @return true if successful, false otherwise
   */
  virtual bool getHeight(loco::Position& positionWorldToLocationInWorldFrame) const = 0;

  /*! Gets the height of the terrain at a certain position.
   * (in world frame) and sets the position.z() as the height (in world frame).
   * @param[in] positionWorldToLocationInWorldFrame   position of the requested location expressed in world frame
   * @param[out]  height of the terrain in world frame
   * @returns true if successful, false otherwise
   */
  virtual bool getHeight(const loco::Position& positionWorldToLocationInWorldFrame, double& heightInWorldFrame) const = 0;

  /*! Gets the maximal height of the terrain on a line between two points
   * (in world frame) from the map and sets the height
   * @param[in/out] point1, point2 as the points for the line, height to output the maximal height of the terrain
   * @return true if successful, false otherwise
   */
  virtual bool getMaxHeightBetweenTwoPoints(const loco::Position& point1, const loco::Position& point2, double& height) const = 0;

  /*! Gets the friction coefficient between foot and terrain at a certain location.
   * @param positionWorldToLocationInWorldFrame   position of the requested location expressed in world frame
   * @param frictionCoefficient   friction coefficient
   * @returns true if successful, false otherwise
   */
  virtual bool getFrictionCoefficientForFoot(const loco::Position& positionWorldToLocationInWorldFrame,
                                             double& frictionCoefficient) const = 0;

  /*! Project the given position along the surface normal onto the terrain plane
   * @param positionWorldToLocationInWorldFrame   position to be projected in world frame
   * @returns the projection onto the terrain plane along the surface normal
   */
  virtual Position getPositionProjectedOnPlaneAlongSurfaceNormalInWorldFrame(const Position& positionWorldToLocationInWorldFrame) const = 0;

  /*! Gets the distance to the terrain plane (as a strict positive value) of a point in world frame along the surface normal
   * @param positionWorldToLocationInWorldFrame   position for which distance shall be calculated
   * @returns the distance of the point to the terrain plane along the surface normal
   */
  virtual double getDistanceFromSurfaceAlongSurfaceNormalToPositionInWorldFrame(
      const Position& positionWorldToLocationInWorldFrame) const = 0;

  /*! Gets the height above the terrain (can also be negative), signed version of
   *  getDistanceFromSurfaceAlongSurfaceNormalToPositionInWorldFrame()
   * @param positionWorldToLocationInWorldFrame   position for which height shall be calculated
   * @returns the height of the point above the terrain plane along the surface normal
   */
  virtual double getHeightAboveTerrainAlongSurfaceNormal(const Position& positionWorldToLocationInWorldFrame) const = 0;

  /*! Gets terrain roll and pitch at location, projected along plane normal onto surface plane,
   * w.r.t the orientation given by vectorHeadingDirectionInWorldFrame.
   * @param positionWorldToLocationInWorldFrame position at which we compute the terrain orientation (not necessarily on surface)
   */
  virtual bool getTerrainOrientation(const Position& positionWorldToLocationInWorldFrame, const Vector& vectorHeadingDirectionInWorldFrame,
                                     double& terrainPitch, double& terrainRoll) const;

  /** Adds variables to the signal logger
   * @param ns            namespace of the variables
   * @return              true, iff successful
   */
  virtual bool addVariablesToLog(const std::string& /* ns */) const { return true; }
};

} /* namespace loco */
