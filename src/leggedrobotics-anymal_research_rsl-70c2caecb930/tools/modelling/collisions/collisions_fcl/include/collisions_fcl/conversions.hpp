/*
 * conversions.hpp
 *
 *  Created on: Jun 1, 2017
 *      Author: Perry Franklin
 */

#pragma once

#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/geometry/collision_geometry.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/capsule.h>
#include <fcl/geometry/shape/cylinder.h>
#include <fcl/geometry/shape/sphere.h>
#include <fcl/narrowphase/collision_object.h>

#include <collisions/CollisionBody.hpp>

#include <collisions_geometry/CollisionGeometry.hpp>
#include <collisions_geometry/CollisionGeometryBox.hpp>
#include <collisions_geometry/CollisionGeometryCapsule.hpp>
#include <collisions_geometry/CollisionGeometryCylinder.hpp>
#include <collisions_geometry/CollisionGeometrySphere.hpp>
#include <collisions_geometry/CollisionGeometryMesh.hpp>
#include <collisions_geometry/CollisionGeometryCombo.hpp> // Technically unused in the hpp, just here for completeness

namespace collisions_fcl {

// This is a convenience class that contains functions for converting romo
// collision structures to FCL collision structures. It is desigend so that one
// can define the template parameters once, instead of every time a function is
// used. The object never needs to be actually instantiated; all functions are
// static.
template <typename scalar_type_, typename bounding_volume_>
class Converter {
 public:

  Converter() = delete;
  ~Converter() = delete;

  using bounding_volume = bounding_volume_;
  using scalar_type = scalar_type_;

  // Creates a new FclObject from a romo::RigidBody. The final transformation of
  // the FCL Object is in the world frame, ie the RigidBody Transformation and
  // the CollisionGeometry Transformation combined.
  static bool FclObjectFromCollisionBody(
      const collisions::CollisionBody& collision_body,
      const double& margin,
      std::vector< fcl::CollisionObject<scalar_type_> >& fcl_object_out);

  // Creates an FCL Object from a collisions_geometry::CollisionGeometry. This
  // FCL object will contain the transform stored in the CollisionGeometry
  static bool FclGeom(
      const collisions_geometry::CollisionGeometry& collision_geometry,
      const double& margin,
      std::vector< fcl::CollisionObject<scalar_type_> >& fcl_object_out);

  // Creates an FCL Box geom from a collisions_geometry::CollisionGeometryBox.
  // Since FCL Geoms has no transformation, the CollisionGeometry's
  // transformation is ignored.
  static fcl::Box<scalar_type_> FclBox(
      const collisions_geometry::CollisionGeometryBox& collisions_geometry_box,
      const double& margin = 0.0);

  // Creates an FCL Cylinder geom from a
  // collisions_geometry::CollisionGeometryCylinder. Since FCL Geoms has no
  // transformation, the CollisionGeometry's transformation is ignored.
  static fcl::Cylinder<scalar_type_> FclCylinder(
      const collisions_geometry::CollisionGeometryCylinder& collisions_geometry_cylinder,
      const double& margin = 0.0);

  // Creates an FCL Capsule geom from a
  // collisions_geometry::CollisionGeometryCapsule. Since FCL Geoms has no
  // transformation, the CollisionGeometry's transformation is ignored.
  static fcl::Capsule<scalar_type_> FclCapsule(
      const collisions_geometry::CollisionGeometryCapsule& collisions_geometry_capsule,
      const double& margin = 0.0);

  // Creates an FCL Sphere geom from a
  // collisions_geometry::CollisionGeometrySphere. Since FCL Geoms has no
  // transformation, the CollisionGeometry's transformation is ignored.
  static fcl::Sphere<scalar_type_> FclSphere(
      const collisions_geometry::CollisionGeometrySphere& collisions_geometry_sphere,
      const double& margin = 0.0);

  // Creates an FCL Mesh geom from a collisions_geometry::CollisionGeometryMesh.
  // Since FCL Geoms has no transformation, the CollisionGeometry's
  // transformation is ignored.
  // TODO static fcl::BVHModel<bounding_volume_> FclMesh(const
  // collisions_geometry::CollisionGeometryMesh collsiion_geometry_mesh,
  // const double& margin);
};

}  // namespace collisions_fcl

#include <collisions_fcl/conversions.tpp>
