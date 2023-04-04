/*
 * ColliderManagerFcl.hpp
 *
 *  Created on: Jul 18, 2017
 *      Author: Perry Franklin
 */

#pragma once

#include <collisions/ColliderManager.hpp>
#include <collisions/CollisionBody.hpp>

#include <fcl/math/bv/AABB.h>

#include <collisions_fcl/conversions.hpp>

namespace collisions_fcl {

// Implements ColliderManager using FCL
class ColliderManagerFcl: public collisions::ColliderManager {
public:
  using FclConverter = Converter<double, fcl::AABB<double>>;

public:

  // Constructs the ColliderManagerFcl
  explicit ColliderManagerFcl() = default;
  ~ColliderManagerFcl() override = default;


  collisions::DistanceResults checkDistance(const collisions::CollisionGroup& collision_group1,
                                            const collisions::CollisionGroup& collision_group2,
                                            const collisions::DistanceOptions& distance_options = collisions::DistanceOptions()) override;


  collisions::CollisionResults checkCollision(const collisions::CollisionGroup& group1,
                                              const collisions::CollisionGroup& group2,
                                              const collisions::CollisionOptions& collision_options) override;

};

}
