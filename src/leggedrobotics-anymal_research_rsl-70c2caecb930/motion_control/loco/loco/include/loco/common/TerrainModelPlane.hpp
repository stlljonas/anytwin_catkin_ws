/*
 * TerrainModelPlane.hpp
 *
 *  Created on: Aug 28, 2014
 *      Author: C. Dario Bellicoso
 */

#pragma once

// loco
#include "loco/common/TerrainModelBase.hpp"

namespace loco {

class TerrainModelPlane : public TerrainModelBase {
 public:
  TerrainModelPlane() = default;
  ~TerrainModelPlane() override = default;

  /*! Set free plane parameters.
   * @param normal Normal vector to plane in world frame
   * @param position Point on plane in world frame
   */
  virtual void setNormalandPositionInWorldFrame(const loco::Vector& normal, const loco::Position& position) = 0;

};  // class

} /* namespace loco */
