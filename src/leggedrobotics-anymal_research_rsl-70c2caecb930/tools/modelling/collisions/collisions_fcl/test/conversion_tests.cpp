/*!
 * @file    conversions_test.cpp
 * @author  Perry Franklin
 * @date    Jun 1, 2017
 */

#include <gtest/gtest.h>

#include <collisions_fcl/conversions.hpp>

#include "test_declarations.hpp"

using namespace collisions_fcl_tests;
using namespace collisions_fcl;

namespace collisions_fcl_tests {

using bounding_vol = fcl::OBBRSS<double>;
using scalar_type = bounding_vol::S;

using TestConverter = Converter<scalar_type, bounding_vol, CoordinateFrame>;

template <typename M1, typename M2, typename T>
void expectNear(const M1& A, const M2& B, T tolerance,
                std::string const& message = "") {
  EXPECT_EQ((size_t)A.rows(), (size_t)B.rows()) << message << "\nMatrix A:\n"
                                                << A << "\nand matrix B\n"
                                                << B << "\nare not the same\n";
  EXPECT_EQ((size_t)A.cols(), (size_t)B.cols()) << message << "\nMatrix A:\n"
                                                << A << "\nand matrix B\n"
                                                << B << "\nare not the same\n";

  for (int r = 0; r < A.rows(); r++) {
    for (int c = 0; c < A.cols(); c++) {
      EXPECT_NEAR(A(r, c), B(r, c), tolerance)
          << message << "\nTolerance comparison failed at (" << r << "," << c
          << ")\n"
          << "\nMatrix A:\n"
          << A << "\nand matrix B\n"
          << B;
    }
  }
}

// This tests the conversion of the CollisionGeometryBox to the FCL Box geomtype
TEST(CONVERSIONSTest, Box) {
  double x_size = 0.5;
  double y_size = 1.7;
  double z_size = 2.6;
  collisions_geometry::CollisionGeometryBox box_geom(
      x_size, y_size, z_size,
      collisions_geometry::CollisionGeometryBox::Position(),
      collisions_geometry::CollisionGeometryBox::Rotation());

  fcl::Box<scalar_type> fcl_box = TestConverter::FclBox(box_geom);

  EXPECT_EQ(x_size, box_geom.getXSize());
  EXPECT_EQ(y_size, box_geom.getYSize());
  EXPECT_EQ(z_size, box_geom.getZSize());

  EXPECT_EQ(fcl_box.getNodeType(), fcl::NODE_TYPE::GEOM_BOX);
  EXPECT_EQ(x_size, fcl_box.side[0]);
  EXPECT_EQ(y_size, fcl_box.side[1]);
  EXPECT_EQ(z_size, fcl_box.side[2]);
}

// This tests the coversion of a CollisionGeometry with a Box to the FCL
// CollisionObject
TEST(CONVERSIONSTest, BoxObject) {
  double x_size = 0.5;
  double y_size = 1.7;
  double z_size = 2.6;

  TestRigidBody rigid_body;

  // This sets the transform of the RigidBody
  rigid_body.setTransform(kindr::HomTransformMatrixD(kindr::HomTransformMatrixD::Position(0.1, 0.2, 0.3),
                          kindr::HomTransformMatrixD::Rotation(0, -1, 0, 1, 0, 0, 0, 0, 1)));

  // This sets the internal transform of the Collision Geometry; ie the
  // transform from the pose of the RigidBody to the collision Geometry
  kindr::HomTransformMatrixD col_transform(kindr::HomTransformMatrixD::Position(0.1, 0.2, 0.3),
                                           kindr::HomTransformMatrixD::Rotation(1, 0, 0, 0, 0, 1, 0, -1, 0));
  rigid_body.collision_geometry.reset(
      new collisions_geometry::CollisionGeometryBox(x_size, y_size, z_size,
                                                   col_transform.getPosition(),
                                                   col_transform.getRotation()));

  std::vector< fcl::CollisionObject<scalar_type> > collision_objects;
  ASSERT_TRUE( TestConverter::FclObjectFromRigidBody(rigid_body, 0.0, collision_objects));

  fcl::CollisionObject<scalar_type>& collision_object = collision_objects[0];

  const std::shared_ptr<const fcl::CollisionGeometry<scalar_type> >
      fcl_col_geom = collision_object.collisionGeometry();

  const std::shared_ptr<const fcl::Box<scalar_type> > fcl_col_box =
      std::dynamic_pointer_cast<const fcl::Box<scalar_type>,
                                const fcl::CollisionGeometry<scalar_type> >(
          fcl_col_geom);

  ASSERT_TRUE(fcl_col_box.get() != NULL);

  EXPECT_EQ(x_size, fcl_col_box->side[0]);
  EXPECT_EQ(y_size, fcl_col_box->side[1]);
  EXPECT_EQ(z_size, fcl_col_box->side[2]);

  kindr::HomTransformMatrixD world_transform = rigid_body.getTransform()*col_transform;

  expectNear(Eigen::Affine3d(collision_object.getTransform()).matrix(),
             world_transform.getTransformationMatrix(), 1e-5);
}

// This tests the coversion of a CollisionGeometry with a Box to the FCL
// CollisionObject, with a margin
TEST(CONVERSIONSTest, BoxObjectMargin) {
  double x_size = 0.5;
  double y_size = 1.7;
  double z_size = 2.6;
  double margin = 0.1;

  TestRigidBody rigid_body;

  // This sets the transform of the RigidBody
  rigid_body.setTransform(kindr::HomTransformMatrixD(kindr::HomTransformMatrixD::Position(0.1, 0.2, 0.3),
                          kindr::HomTransformMatrixD::Rotation(0, -1, 0, 1, 0, 0, 0, 0, 1)));

  // This sets the internal transform of the Collision Geometry; ie the
  // transform from the pose of the RigidBody to the collision Geometry
  kindr::HomTransformMatrixD col_transform(kindr::HomTransformMatrixD::Position(0.1, 0.2, 0.3),
                                           kindr::HomTransformMatrixD::Rotation(1, 0, 0, 0, 0, 1, 0, -1, 0));
  rigid_body.collision_geometry.reset(
      new collisions_geometry::CollisionGeometryBox(x_size, y_size, z_size,
                                                   col_transform.getPosition(),
                                                   col_transform.getRotation()));

  std::vector< fcl::CollisionObject<scalar_type> > collision_objects;
  ASSERT_TRUE( TestConverter::FclObjectFromRigidBody(rigid_body, margin, collision_objects));

  fcl::CollisionObject<scalar_type>& collision_object = collision_objects[0];

  const std::shared_ptr<const fcl::CollisionGeometry<scalar_type> >
      fcl_col_geom = collision_object.collisionGeometry();

  const std::shared_ptr<const fcl::Box<scalar_type> > fcl_col_box =
      std::dynamic_pointer_cast<const fcl::Box<scalar_type>,
                                const fcl::CollisionGeometry<scalar_type> >(
          fcl_col_geom);

  ASSERT_TRUE(fcl_col_box.get() != NULL);

  // The margin is added twice, since it is added to one side (positive), then to the other (negative).
  EXPECT_DOUBLE_EQ(x_size+margin+margin, fcl_col_box->side[0]);
  EXPECT_DOUBLE_EQ(y_size+margin+margin, fcl_col_box->side[1]);
  EXPECT_DOUBLE_EQ(z_size+margin+margin, fcl_col_box->side[2]);

  kindr::HomTransformMatrixD world_transform = rigid_body.getTransform()*col_transform;

  expectNear(Eigen::Affine3d(collision_object.getTransform()).matrix(),
             world_transform.getTransformationMatrix(), 1e-5);
}

// This tests the conversion of the CollisionGeometryBox to the FCL Cylinder
// geomtype
TEST(CONVERSIONSTest, Cylinder) {
  double radius = 0.5;
  double length = 1.7;
  collisions_geometry::CollisionGeometryCylinder cylinder_geom(
      radius, length,
      collisions_geometry::CollisionGeometryBox::Position(),
      collisions_geometry::CollisionGeometryBox::Rotation());

  fcl::Cylinder<scalar_type> fcl_cylinder =
      TestConverter::FclCylinder(cylinder_geom);

  EXPECT_EQ(radius, cylinder_geom.getRadius());
  EXPECT_EQ(length, cylinder_geom.getLength());

  EXPECT_EQ(fcl_cylinder.getNodeType(), fcl::NODE_TYPE::GEOM_CYLINDER);
  EXPECT_EQ(radius, fcl_cylinder.radius);
  EXPECT_EQ(length, fcl_cylinder.lz);
}

// This tests the coversion of a CollisionGeometry with a Cylinder to the FCL
// CollisionObject
TEST(CONVERSIONSTest, CylinderObject) {
  double radius = 0.5;
  double length = 1.7;

  TestRigidBody rigid_body;

  // This sets the transform of the RigidBody
  rigid_body.setTransform(kindr::HomTransformMatrixD(kindr::HomTransformMatrixD::Position(0.1, 0.2, 0.3),
                          kindr::HomTransformMatrixD::Rotation(0, -1, 0, 1, 0, 0, 0, 0, 1)));

  // This sets the internal transform of the Collision Geometry; ie the
  // transform from the pose of the RigidBody to the collision Geometry
  kindr::HomTransformMatrixD col_transform(kindr::HomTransformMatrixD::Position(0.1, 0.2, 0.3),
                                           kindr::HomTransformMatrixD::Rotation(1, 0, 0, 0, 0, 1, 0, -1, 0));
  rigid_body.collision_geometry.reset(
      new collisions_geometry::CollisionGeometryCylinder(radius, length,
                                                         col_transform.getPosition(),
                                                         col_transform.getRotation()));

  std::vector< fcl::CollisionObject<scalar_type> > collision_objects;
  ASSERT_TRUE(TestConverter::FclObjectFromRigidBody(rigid_body, 0.0, collision_objects));

  fcl::CollisionObject<scalar_type>& collision_object = collision_objects[0];

  const std::shared_ptr<const fcl::CollisionGeometry<scalar_type> >
      fcl_col_geom = collision_object.collisionGeometry();

  const std::shared_ptr<const fcl::Cylinder<scalar_type> > fcl_col_cylinder =
      std::dynamic_pointer_cast<const fcl::Cylinder<scalar_type>,
                                const fcl::CollisionGeometry<scalar_type> >(
          fcl_col_geom);

  ASSERT_TRUE(fcl_col_cylinder.get() != NULL);

  EXPECT_EQ(radius, fcl_col_cylinder->radius);
  EXPECT_EQ(length, fcl_col_cylinder->lz);

  kindr::HomTransformMatrixD world_transform = rigid_body.getTransform()*col_transform;

  expectNear(Eigen::Affine3d(collision_object.getTransform()).matrix(),
             world_transform.getTransformationMatrix(), 1e-5);
}

// This tests the coversion of a CollisionGeometry with a Cylinder to the FCL
// CollisionObject, with a margin
TEST(CONVERSIONSTest, CylinderObjectMargin) {
  double radius = 0.5;
  double length = 1.7;
  double margin = 0.1;

  TestRigidBody rigid_body;

  // This sets the transform of the RigidBody
  rigid_body.setTransform(kindr::HomTransformMatrixD(kindr::HomTransformMatrixD::Position(0.1, 0.2, 0.3),
                          kindr::HomTransformMatrixD::Rotation(0, -1, 0, 1, 0, 0, 0, 0, 1)));

  // This sets the internal transform of the Collision Geometry; ie the
  // transform from the pose of the RigidBody to the collision Geometry
  kindr::HomTransformMatrixD col_transform(kindr::HomTransformMatrixD::Position(0.1, 0.2, 0.3),
                                           kindr::HomTransformMatrixD::Rotation(1, 0, 0, 0, 0, 1, 0, -1, 0));
  rigid_body.collision_geometry.reset(
      new collisions_geometry::CollisionGeometryCylinder(radius, length,
                                                         col_transform.getPosition(),
                                                         col_transform.getRotation()));

  std::vector< fcl::CollisionObject<scalar_type> > collision_objects;
  ASSERT_TRUE(TestConverter::FclObjectFromRigidBody(rigid_body, margin, collision_objects));

  fcl::CollisionObject<scalar_type>& collision_object = collision_objects[0];

  const std::shared_ptr<const fcl::CollisionGeometry<scalar_type> >
      fcl_col_geom = collision_object.collisionGeometry();

  const std::shared_ptr<const fcl::Cylinder<scalar_type> > fcl_col_cylinder =
      std::dynamic_pointer_cast<const fcl::Cylinder<scalar_type>,
                                const fcl::CollisionGeometry<scalar_type> >(
          fcl_col_geom);

  ASSERT_TRUE(fcl_col_cylinder.get() != NULL);

  // Margin is added once, since the radius is all around.
  EXPECT_DOUBLE_EQ(radius + margin, fcl_col_cylinder->radius);

  // Margin is added twice, once for each side (positive and negative)
  EXPECT_DOUBLE_EQ(length + margin + margin, fcl_col_cylinder->lz);

  kindr::HomTransformMatrixD world_transform = rigid_body.getTransform()*col_transform;

  expectNear(Eigen::Affine3d(collision_object.getTransform()).matrix(),
             world_transform.getTransformationMatrix(), 1e-5);
}

// This tests the coversion of a CollisionGeometry with a Cylinder to the FCL
// CollisionObject
TEST(CONVERSIONSTest, Capsule) {
  double radius = 0.5;
  double length = 1.7;
  collisions_geometry::CollisionGeometryCapsule capsule_geom(
      radius, length,
      collisions_geometry::CollisionGeometryBox::Position(),
      collisions_geometry::CollisionGeometryBox::Rotation());

  fcl::Capsule<scalar_type> fcl_capsule =
      TestConverter::FclCapsule(capsule_geom);

  EXPECT_EQ(radius, capsule_geom.getRadius());
  EXPECT_EQ(length, capsule_geom.getLength());

  EXPECT_EQ(fcl_capsule.getNodeType(), fcl::NODE_TYPE::GEOM_CAPSULE);
  EXPECT_EQ(radius, fcl_capsule.radius);
  EXPECT_EQ(length, fcl_capsule.lz);
}

// This tests the coversion of a CollisionGeometry with a Capsule to the FCL
// CollisionObject
TEST(CONVERSIONSTest, CapsuleObject) {
  double radius = 0.5;
  double length = 1.7;

  TestRigidBody rigid_body;

  // This sets the transform of the RigidBody
 rigid_body.setTransform(kindr::HomTransformMatrixD(kindr::HomTransformMatrixD::Position(0.1, 0.2, 0.3),
                          kindr::HomTransformMatrixD::Rotation(0, -1, 0, 1, 0, 0, 0, 0, 1)));

  // This sets the internal transform of the Collision Geometry; ie the
  // transform from the pose of the RigidBody to the collision Geometry
  kindr::HomTransformMatrixD col_transform(kindr::HomTransformMatrixD::Position(0.1, 0.2, 0.3),
                                           kindr::HomTransformMatrixD::Rotation(1, 0, 0, 0, 0, 1, 0, -1, 0));

  rigid_body.collision_geometry.reset(
      new collisions_geometry::CollisionGeometryCapsule(radius, length,
                                                        col_transform.getPosition(),
                                                        col_transform.getRotation()));

  std::vector< fcl::CollisionObject<scalar_type> > collision_objects;
  ASSERT_TRUE(TestConverter::FclObjectFromRigidBody(rigid_body, 0.0, collision_objects));

  fcl::CollisionObject<scalar_type>& collision_object = collision_objects[0];

  const std::shared_ptr<const fcl::CollisionGeometry<scalar_type> >
      fcl_col_geom = collision_object.collisionGeometry();

  const std::shared_ptr<const fcl::Capsule<scalar_type> > fcl_col_capsule =
      std::dynamic_pointer_cast<const fcl::Capsule<scalar_type>,
                                const fcl::CollisionGeometry<scalar_type> >(
          fcl_col_geom);

  ASSERT_TRUE(fcl_col_capsule.get() != NULL);

  EXPECT_EQ(radius, fcl_col_capsule->radius);
  EXPECT_EQ(length, fcl_col_capsule->lz);

  kindr::HomTransformMatrixD world_transform = rigid_body.getTransform()*col_transform;

  expectNear(Eigen::Affine3d(collision_object.getTransform()).matrix(),
             world_transform.getTransformationMatrix(), 1e-5);
}


// This tests the coversion of a CollisionGeometry with a Capsule to the FCL
// CollisionObject, with a margin
TEST(CONVERSIONSTest, CapsuleObjectMargin) {
  double radius = 0.5;
  double length = 1.7;
  double margin = 0.1;

  TestRigidBody rigid_body;

  // This sets the transform of the RigidBody
 rigid_body.setTransform(kindr::HomTransformMatrixD(kindr::HomTransformMatrixD::Position(0.1, 0.2, 0.3),
                          kindr::HomTransformMatrixD::Rotation(0, -1, 0, 1, 0, 0, 0, 0, 1)));

  // This sets the internal transform of the Collision Geometry; ie the
  // transform from the pose of the RigidBody to the collision Geometry
  kindr::HomTransformMatrixD col_transform(kindr::HomTransformMatrixD::Position(0.1, 0.2, 0.3),
                                           kindr::HomTransformMatrixD::Rotation(1, 0, 0, 0, 0, 1, 0, -1, 0));

  rigid_body.collision_geometry.reset(
      new collisions_geometry::CollisionGeometryCapsule(radius, length,
                                                        col_transform.getPosition(),
                                                        col_transform.getRotation()));

  std::vector< fcl::CollisionObject<scalar_type> > collision_objects;
  ASSERT_TRUE(TestConverter::FclObjectFromRigidBody(rigid_body, margin, collision_objects));

  fcl::CollisionObject<scalar_type>& collision_object = collision_objects[0];

  const std::shared_ptr<const fcl::CollisionGeometry<scalar_type> >
      fcl_col_geom = collision_object.collisionGeometry();

  const std::shared_ptr<const fcl::Capsule<scalar_type> > fcl_col_capsule =
      std::dynamic_pointer_cast<const fcl::Capsule<scalar_type>,
                                const fcl::CollisionGeometry<scalar_type> >(
          fcl_col_geom);

  ASSERT_TRUE(fcl_col_capsule.get() != NULL);

  // Margin is added once, for the a radius
  EXPECT_DOUBLE_EQ(radius + margin, fcl_col_capsule->radius);

  // Margin is added twice, once for each side (positive and negative)
  EXPECT_DOUBLE_EQ(length + margin + margin, fcl_col_capsule->lz);

  kindr::HomTransformMatrixD world_transform = rigid_body.getTransform()*col_transform;

  expectNear(Eigen::Affine3d(collision_object.getTransform()).matrix(),
             world_transform.getTransformationMatrix(), 1e-5);
}


// This tests the conversion of a CollisionGeometry with a Combo type to the FCL collision object collection
TEST(CONVERSIONSTest, ComboObject) {

  TestRigidBody rigid_body;

  // This sets the transform of the RigidBody
  rigid_body.setTransform(kindr::HomTransformMatrixD(kindr::HomTransformMatrixD::Position(0.1, 0.2, 0.3),
                          kindr::HomTransformMatrixD::Rotation(0, -1, 0, 1, 0, 0, 0, 0, 1)));


  // Generate a new (empty) collisions_geometry Combo geometry, and set the rigid_body's pointer to it.
  std::shared_ptr<collisions_geometry::CollisionGeometryCombo> combo_geom( new collisions_geometry::CollisionGeometryCombo() );
  rigid_body.collision_geometry = combo_geom;

  // Add a box geom to the combo.
  double x_size = 0.5;
  double y_size = 1.7;
  double z_size = 2.6;
  kindr::HomTransformMatrixD box_transform(kindr::HomTransformMatrixD::Position(0.1, 0.2, 0.3),
                                           kindr::HomTransformMatrixD::Rotation(1, 0, 0, 0, 0, 1, 0, -1, 0));
  std::unique_ptr<collisions_geometry::CollisionGeometry> box_geom(new collisions_geometry::CollisionGeometryBox(x_size, y_size, z_size,
                                                                   box_transform.getPosition(),
                                                                   box_transform.getRotation()));
  combo_geom->addGeometry(box_geom);

  // Add a cylinder geom to the combo.
  double radius = 0.1;
  double length = 0.5;
  kindr::HomTransformMatrixD cyl_transform(kindr::HomTransformMatrixD::Position(0.4, 0.5, 0.6),
                                           kindr::HomTransformMatrixD::Rotation(0,-1, 0, 1, 0, 0, 0, 0, 1));
  std::unique_ptr<collisions_geometry::CollisionGeometry> cyl_geom(new collisions_geometry::CollisionGeometryCylinder(radius, length,
                                                                    cyl_transform.getPosition(),
                                                                    cyl_transform.getRotation()));
  combo_geom->addGeometry(cyl_geom);


  // Convert the Rigid Body to it representative parts.
  std::vector< fcl::CollisionObject<scalar_type> > collision_objects;
  ASSERT_TRUE( TestConverter::FclObjectFromRigidBody(rigid_body, 0.0, collision_objects));

  {
  // Extract the box and check if it matches
  // NOTE: this conversion to the box takes advantage of the ordering of the combo objects, which is technically not guaranteed.
  const std::shared_ptr<const fcl::CollisionGeometry<scalar_type> >
      fcl_col_geom = collision_objects[0].collisionGeometry();

  const std::shared_ptr<const fcl::Box<scalar_type> > fcl_col_box =
      std::dynamic_pointer_cast<const fcl::Box<scalar_type>,
                                const fcl::CollisionGeometry<scalar_type> >(fcl_col_geom);

  ASSERT_TRUE(fcl_col_box.get() != NULL);

  EXPECT_EQ(x_size, fcl_col_box->side[0]);
  EXPECT_EQ(y_size, fcl_col_box->side[1]);
  EXPECT_EQ(z_size, fcl_col_box->side[2]);

  kindr::HomTransformMatrixD world_transform = rigid_body.getTransform()*box_transform;

  expectNear(Eigen::Affine3d(collision_objects[0].getTransform()).matrix(),
             world_transform.getTransformationMatrix(), 1e-5);
  }

  {
  // Extract the cylinder and check if it matches
  // NOTE: this conversion to the cylinder takes advantage of the ordering of the combo objects, which is technically not guaranteed.
  const std::shared_ptr<const fcl::CollisionGeometry<scalar_type> >
      fcl_col_geom = collision_objects[1].collisionGeometry();

  const std::shared_ptr<const fcl::Cylinder<scalar_type> > fcl_col_cylinder =
      std::dynamic_pointer_cast<const fcl::Cylinder<scalar_type>,
                                const fcl::CollisionGeometry<scalar_type> >(
          fcl_col_geom);

  ASSERT_TRUE(fcl_col_cylinder.get() != NULL);

  EXPECT_EQ(radius, fcl_col_cylinder->radius);
  EXPECT_EQ(length, fcl_col_cylinder->lz);

  kindr::HomTransformMatrixD world_transform = rigid_body.getTransform()*cyl_transform;

  expectNear(Eigen::Affine3d(collision_objects[1].getTransform()).matrix(),
             world_transform.getTransformationMatrix(), 1e-5);
  }
}


// This tests the conversion of a CollisionGeometry with a Combo type to the FCL collision object collection, with a margin
TEST(CONVERSIONSTest, ComboObjectMargin) {

  TestRigidBody rigid_body;

  // This sets the transform of the RigidBody
  rigid_body.setTransform(kindr::HomTransformMatrixD(kindr::HomTransformMatrixD::Position(0.1, 0.2, 0.3),
                          kindr::HomTransformMatrixD::Rotation(0, -1, 0, 1, 0, 0, 0, 0, 1)));


  // Generate a new (empty) collisions_geometry Combo geometry, and set the rigid_body's pointer to it.
  std::shared_ptr<collisions_geometry::CollisionGeometryCombo> combo_geom( new collisions_geometry::CollisionGeometryCombo() );
  rigid_body.collision_geometry = combo_geom;

  double margin = 0.1;

  // Add a box geom to the combo.
  double x_size = 0.5;
  double y_size = 1.7;
  double z_size = 2.6;
  kindr::HomTransformMatrixD box_transform(kindr::HomTransformMatrixD::Position(0.1, 0.2, 0.3),
                                           kindr::HomTransformMatrixD::Rotation(1, 0, 0, 0, 0, 1, 0, -1, 0));
  std::unique_ptr<collisions_geometry::CollisionGeometry> box_geom(new collisions_geometry::CollisionGeometryBox(x_size, y_size, z_size,
                                                                   box_transform.getPosition(),
                                                                   box_transform.getRotation()));
  combo_geom->addGeometry(box_geom);

  // Add a cylinder geom to the combo.
  double radius = 0.1;
  double length = 0.5;
  kindr::HomTransformMatrixD cyl_transform(kindr::HomTransformMatrixD::Position(0.4, 0.5, 0.6),
                                           kindr::HomTransformMatrixD::Rotation(0,-1, 0, 1, 0, 0, 0, 0, 1));
  std::unique_ptr<collisions_geometry::CollisionGeometry> cyl_geom(new collisions_geometry::CollisionGeometryCylinder(radius, length,
                                                                    cyl_transform.getPosition(),
                                                                    cyl_transform.getRotation()));
  combo_geom->addGeometry(cyl_geom);


  // Convert the Rigid Body to it representative parts.
  std::vector< fcl::CollisionObject<scalar_type> > collision_objects;
  ASSERT_TRUE( TestConverter::FclObjectFromRigidBody(rigid_body, margin, collision_objects));

  {
  // Extract the box and check if it matches
  // NOTE: this conversion to the box takes advantage of the ordering of the combo objects, which is technically not guaranteed.
  const std::shared_ptr<const fcl::CollisionGeometry<scalar_type> >
      fcl_col_geom = collision_objects[0].collisionGeometry();

  const std::shared_ptr<const fcl::Box<scalar_type> > fcl_col_box =
      std::dynamic_pointer_cast<const fcl::Box<scalar_type>,
                                const fcl::CollisionGeometry<scalar_type> >(fcl_col_geom);

  ASSERT_TRUE(fcl_col_box.get() != NULL);

  EXPECT_DOUBLE_EQ(x_size + margin + margin, fcl_col_box->side[0]);
  EXPECT_DOUBLE_EQ(y_size + margin + margin, fcl_col_box->side[1]);
  EXPECT_DOUBLE_EQ(z_size + margin + margin, fcl_col_box->side[2]);

  kindr::HomTransformMatrixD world_transform = rigid_body.getTransform()*box_transform;

  expectNear(Eigen::Affine3d(collision_objects[0].getTransform()).matrix(),
             world_transform.getTransformationMatrix(), 1e-5);
  }

  {
  // Extract the cylinder and check if it matches
  // NOTE: this conversion to the cylinder takes advantage of the ordering of the combo objects, which is technically not guaranteed.
  const std::shared_ptr<const fcl::CollisionGeometry<scalar_type> >
      fcl_col_geom = collision_objects[1].collisionGeometry();

  const std::shared_ptr<const fcl::Cylinder<scalar_type> > fcl_col_cylinder =
      std::dynamic_pointer_cast<const fcl::Cylinder<scalar_type>,
                                const fcl::CollisionGeometry<scalar_type> >(
          fcl_col_geom);

  ASSERT_TRUE(fcl_col_cylinder.get() != NULL);

  EXPECT_EQ(radius + margin, fcl_col_cylinder->radius);
  EXPECT_EQ(length + margin + margin, fcl_col_cylinder->lz);

  kindr::HomTransformMatrixD world_transform = rigid_body.getTransform()*cyl_transform;

  expectNear(Eigen::Affine3d(collision_objects[1].getTransform()).matrix(),
             world_transform.getTransformationMatrix(), 1e-5);
  }
}


}  // namespace collision_detection_fcl
