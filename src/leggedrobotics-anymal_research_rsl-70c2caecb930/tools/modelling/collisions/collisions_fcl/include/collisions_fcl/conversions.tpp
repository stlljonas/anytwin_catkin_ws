/*
 * conversions.tpp
 *
 *  Created on: Jun 1, 2017
 *      Author: Perry Franklin
 */

#pragma once

#include <collisions_fcl/conversions.hpp>

#include <collisions_geometry/CollisionGeometryCombo.hpp>

namespace collisions_fcl {

template <typename scalar_type_, typename bounding_volume_>
bool Converter<scalar_type_, bounding_volume_ >::
    FclObjectFromCollisionBody(const collisions::CollisionBody& collision_body,
                               const double& margin,
                               std::vector< fcl::CollisionObject<scalar_type_> >& fcl_object_out) {

  if (collision_body.getCollisionGeometry() == 0){
    return false;
  }

  const collisions_geometry::CollisionGeometry& collision_geom_ptr(*collision_body.getCollisionGeometry());

  if (!FclGeom(collision_geom_ptr, margin, fcl_object_out)) {
    return false;
  }
  // We apply all the transforms in the collision geometries and rigid body to get the fcl world transformation.

  collisions_geometry::CollisionGeometry::Transform rigid_body_transform(collision_body.getPositionInWorld(),collision_body.getOrientationInWorld());

  for (size_t i = 0; i < fcl_object_out.size(); ++i){

    fcl::CollisionObject<scalar_type_> & object = fcl_object_out[i];

    collisions_geometry::CollisionGeometry::Transform col_geometry_transform(
        collisions_geometry::CollisionGeometry::Position(object.getTransform().translation()),
        collisions_geometry::CollisionGeometry::Rotation(object.getTransform().rotation()));

    collisions_geometry::CollisionGeometry::Transform world_transform =
        rigid_body_transform * col_geometry_transform;

    object.setTransform(world_transform.getRotation().matrix(), world_transform.getPosition().vector());
  }
  return true;
}

template <typename scalar_type_, typename bounding_volume_>
bool Converter<scalar_type_, bounding_volume_>::FclGeom( const collisions_geometry::CollisionGeometry& collision_geometry,
                                                                            const double& margin,
                                                                            std::vector< fcl::CollisionObject<scalar_type_> >& fcl_object_out) {

  switch (collision_geometry.getType()) {
    case collisions_geometry::CollisionGeometry::GeomType::BOX: {
      const collisions_geometry::CollisionGeometryBox& box = static_cast<const collisions_geometry::CollisionGeometryBox&>(collision_geometry);

      fcl::Transform3d transform;
      transform.translation() = box.getPosition().toImplementation();
      transform.matrix().block<3,3>(0,0) = box.getRotation().toImplementation();


      fcl::CollisionObject<scalar_type_> box_obj(std::shared_ptr<fcl::CollisionGeometry<scalar_type_> >(new fcl::Box<scalar_type>(FclBox(box, margin))), transform);

      fcl_object_out.push_back(box_obj);

    } break;

    case collisions_geometry::CollisionGeometry::GeomType::CYLINDER: {
      const collisions_geometry::CollisionGeometryCylinder& cylinder =
          static_cast<const collisions_geometry::CollisionGeometryCylinder&>(
              collision_geometry);

      fcl::Transform3d transform;
      transform.translation() = cylinder.getPosition().toImplementation();
      transform.matrix().block<3,3>(0,0) = cylinder.getRotation().toImplementation();

      fcl::CollisionObject<scalar_type_>  cyl_obj(std::shared_ptr<fcl::CollisionGeometry<scalar_type_> >(new fcl::Cylinder<scalar_type>(FclCylinder(cylinder, margin))), transform);

      fcl_object_out.push_back(cyl_obj);
    } break;

    case collisions_geometry::CollisionGeometry::GeomType::CAPSULE: {
      const collisions_geometry::CollisionGeometryCapsule& capsule =
          static_cast<const collisions_geometry::CollisionGeometryCapsule&>(
              collision_geometry);

      fcl::Transform3d transform;
           transform.translation() = capsule.getPosition().toImplementation();
           transform.matrix().block<3,3>(0,0) = capsule.getRotation().toImplementation();

      fcl::CollisionObject<scalar_type_> cap_obj(std::shared_ptr<fcl::CollisionGeometry<scalar_type_> >(new fcl::Capsule<scalar_type>(FclCapsule(capsule, margin))), transform);

      fcl_object_out.push_back(cap_obj);
    } break;

    case collisions_geometry::CollisionGeometry::GeomType::SPHERE: {
      const collisions_geometry::CollisionGeometrySphere& sphere =
          static_cast<const collisions_geometry::CollisionGeometrySphere&>(
              collision_geometry);

      fcl::Transform3d transform;
           transform.translation() = sphere.getPosition().toImplementation();
           transform.matrix().block<3,3>(0,0) = sphere.getRotation().toImplementation();

      fcl::CollisionObject<scalar_type_> sph_obj(std::shared_ptr<fcl::CollisionGeometry<scalar_type_> >(new fcl::Sphere<scalar_type>(FclSphere(sphere, margin))), transform);

      fcl_object_out.push_back(sph_obj);
    } break;

    case collisions_geometry::CollisionGeometry::GeomType::COMBO: {
      const collisions_geometry::CollisionGeometryCombo& combo =
          static_cast<const collisions_geometry::CollisionGeometryCombo&>(
              collision_geometry);

      const std::vector< std::unique_ptr<collisions_geometry::CollisionGeometry> >& geoms = combo.getGeometries();
      for (size_t i = 0; i < geoms.size(); ++i){
        FclGeom(*geoms[i], margin, fcl_object_out);
      }

    } break;
    default:
      return false;
  }

  return true;
}

template <typename scalar_type_, typename bounding_volume_>
fcl::Box<scalar_type_>
Converter<scalar_type_, bounding_volume_>::FclBox(
    const collisions_geometry::CollisionGeometryBox& collision_geometry_box,
    const double& margin) {
  return fcl::Box<scalar_type_>(collision_geometry_box.getXSize() + (2.0*margin),
                                collision_geometry_box.getYSize() + (2.0*margin),
                                collision_geometry_box.getZSize() + (2.0*margin));
}

template <typename scalar_type_, typename bounding_volume_>
fcl::Cylinder<scalar_type_>
Converter<scalar_type_, bounding_volume_>::FclCylinder(
    const collisions_geometry::CollisionGeometryCylinder& collision_geometry_cylinder,
    const double& margin) {
  return fcl::Cylinder<scalar_type_>(collision_geometry_cylinder.getRadius() + margin,
                                     collision_geometry_cylinder.getLength() + (2.0*margin));
}

template <typename scalar_type_, typename bounding_volume_>
fcl::Capsule<scalar_type_>
Converter<scalar_type_, bounding_volume_>::FclCapsule(
    const collisions_geometry::CollisionGeometryCapsule& collision_geometry_capsule,
    const double& margin) {
  return fcl::Capsule<scalar_type_>(collision_geometry_capsule.getRadius() + margin,
                                    collision_geometry_capsule.getLength() + (2.0*margin));
}
template <typename scalar_type_, typename bounding_volume_>
fcl::Sphere<scalar_type_>
Converter<scalar_type_, bounding_volume_>::FclSphere(
      const collisions_geometry::CollisionGeometrySphere& collision_geometry_sphere,
      const double& margin){
  return fcl::Sphere<scalar_type_>(collision_geometry_sphere.getRadius()+margin);
}

// static fcl::BVHModel<bounding_volume_> FclMesh(const
// collision_geometry::CollisionGeometryMesh collsiion_geometry_mesh);

}  // namespace collision_detection_fcl
