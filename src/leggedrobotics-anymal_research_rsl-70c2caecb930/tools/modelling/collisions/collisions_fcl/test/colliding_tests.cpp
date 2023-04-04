/*
 * colliding_tests.cpp
 *
 *  Created on: Jul 19, 2017
 *      Author: Perry Franklin
 */


#include <memory>

#include "test_declarations.hpp"

#include <collisions_fcl/ColliderManagerFcl.hpp>

#include <gtest/gtest.h>

#include <ros/ros.h>

using namespace collisions_fcl;

namespace collisions_fcl_tests {

using TestColliderManagerFcl = ColliderManagerFcl<CoordinateFrame>;

// Checks that no collision occurs if no geometry is set.
TEST(COLLIDINGTest, No_geometry) {

  std::shared_ptr< collisions::ColliderManager<CoordinateFrame> >
     collider_manager(new TestColliderManagerFcl);

  TestRigidBodyContainer test_rigid_body_container;
  fillTestBodyContainer(test_rigid_body_container);

  std::set<BodyEnum> cg0_set = {BodyEnum::BODY0};
  std::set<BodyEnum> cg1_set = {BodyEnum::BODY1};

  TestCollisionGroup cg0(cg0_set, test_rigid_body_container);
  TestCollisionGroup cg1(cg1_set, test_rigid_body_container);

  auto results = collider_manager->checkCollision(cg0, cg1);
  EXPECT_FALSE(results.colliding());

}

// Checks that no collision occurs between a geometry and no geometry.
TEST(COLLIDINGTest, One_and_none_geometry) {

  std::shared_ptr< collisions::ColliderManager<CoordinateFrame> >
     collider_manager(new TestColliderManagerFcl);

  TestRigidBodyContainer test_rigid_body_container;
  fillTestBodyContainer(test_rigid_body_container);

  std::set<BodyEnum> cg0_set = {BodyEnum::BODY0};
  std::set<BodyEnum> cg1_set = {BodyEnum::BODY1};

  TestCollisionGroup cg0(cg0_set, test_rigid_body_container);
  TestCollisionGroup cg1(cg1_set, test_rigid_body_container);

  kindr::HomTransformMatrixD col_transform(kindr::HomTransformMatrixD::Position(0.2, 0.0, 0.0),
                                            kindr::HomTransformMatrixD::Rotation());
   test_rigid_body_container[BodyEnum::BODY0]->collision_geometry.reset(
       new collisions_geometry::CollisionGeometryBox(1, 1, 1,
                                                    col_transform.getPosition(),
                                                    col_transform.getRotation()));

  auto results = collider_manager->checkCollision(cg0, cg1);
  EXPECT_FALSE(results.colliding());

}

// Check that two boxes will collide.
// The boxes are positioned using the collision_geometry transforms.
// The rigid_bodies are both at the origin.
TEST(COLLIDINGTest, two_boxes_colliding_internal_translation) {

  std::shared_ptr< collisions::ColliderManager<CoordinateFrame> >
     collider_manager(new TestColliderManagerFcl);

  TestRigidBodyContainer test_rigid_body_container;
  fillTestBodyContainer(test_rigid_body_container);

  std::set<BodyEnum> cg0_set = {BodyEnum::BODY0};
  std::set<BodyEnum> cg1_set = {BodyEnum::BODY1};

  TestCollisionGroup cg0(cg0_set, test_rigid_body_container);
  TestCollisionGroup cg1(cg1_set, test_rigid_body_container);

  kindr::HomTransformMatrixD col_transform(kindr::HomTransformMatrixD::Position(0.2, 0.0, 0.0),
                                            kindr::HomTransformMatrixD::Rotation());
   test_rigid_body_container[BodyEnum::BODY0]->collision_geometry.reset(
       new collisions_geometry::CollisionGeometryBox(1, 1, 1,
                                                    col_transform.getPosition(),
                                                    col_transform.getRotation()));

   kindr::HomTransformMatrixD col_transform2(kindr::HomTransformMatrixD::Position(-0.2, 0.1, 0.0),
                                             kindr::HomTransformMatrixD::Rotation());
    test_rigid_body_container[BodyEnum::BODY1]->collision_geometry.reset(
        new collisions_geometry::CollisionGeometryBox(1, 1, 1,
                                                     col_transform2.getPosition(),
                                                     col_transform2.getRotation()));

  auto results = collider_manager->checkCollision(cg0, cg1);
  EXPECT_TRUE(results.colliding());
  expectContains(results.colliding_pairs,std::make_pair(BodyEnum::BODY0, BodyEnum::BODY1));

}

// Check that two boxes don't collide if they are not close enough.
// The boxes are positioned using the collision_geometry transforms.
// The rigid_bodies are both at the origin.
TEST(COLLIDINGTest, two_boxes_not_colliding_internal_translation) {

  std::shared_ptr< collisions::ColliderManager<CoordinateFrame> >
     collider_manager(new TestColliderManagerFcl);

  TestRigidBodyContainer test_rigid_body_container;
  fillTestBodyContainer(test_rigid_body_container);

  std::set<BodyEnum> cg0_set = {BodyEnum::BODY0};
  std::set<BodyEnum> cg1_set = {BodyEnum::BODY1};

  TestCollisionGroup cg0(cg0_set, test_rigid_body_container);
  TestCollisionGroup cg1(cg1_set, test_rigid_body_container);

  kindr::HomTransformMatrixD col_transform(kindr::HomTransformMatrixD::Position(0.6, 0.0, 0.0),
                                            kindr::HomTransformMatrixD::Rotation());
   test_rigid_body_container[BodyEnum::BODY0]->collision_geometry.reset(
       new collisions_geometry::CollisionGeometryBox(1, 1, 1,
                                                    col_transform.getPosition(),
                                                    col_transform.getRotation()));

   kindr::HomTransformMatrixD col_transform2(kindr::HomTransformMatrixD::Position(-0.6, 0.0, 0.0),
                                             kindr::HomTransformMatrixD::Rotation());
    test_rigid_body_container[BodyEnum::BODY1]->collision_geometry.reset(
        new collisions_geometry::CollisionGeometryBox(1, 1, 1,
                                                     col_transform2.getPosition(),
                                                     col_transform2.getRotation()));

  auto results = collider_manager->checkCollision(cg0, cg1);
  EXPECT_FALSE(results.colliding());

}

// Check that two boxes will collide if they are rotated to collide.
// The boxes are positioned using the collision_geometry transforms.
// The rigid_bodies are both at the origin.
TEST(COLLIDINGTest, two_boxes_colliding_internal_rotation) {

  std::shared_ptr< collisions::ColliderManager<CoordinateFrame> >
     collider_manager(new TestColliderManagerFcl);

  TestRigidBodyContainer test_rigid_body_container;
  fillTestBodyContainer(test_rigid_body_container);

  std::set<BodyEnum> cg0_set = {BodyEnum::BODY0};
  std::set<BodyEnum> cg1_set = {BodyEnum::BODY1};

  TestCollisionGroup cg0(cg0_set, test_rigid_body_container);
  TestCollisionGroup cg1(cg1_set, test_rigid_body_container);

  kindr::HomTransformMatrixD col_transform(kindr::HomTransformMatrixD::Position(0.0, 0.2, 0.0),
                                            kindr::HomTransformMatrixD::Rotation());
   test_rigid_body_container[BodyEnum::BODY0]->collision_geometry.reset(
       new collisions_geometry::CollisionGeometryBox(1, 0.1, 0.1,
                                                    col_transform.getPosition(),
                                                    col_transform.getRotation()));

   kindr::HomTransformMatrixD col_transform2(kindr::HomTransformMatrixD::Position(0.0, -0.2, 0.0),
                                             kindr::HomTransformMatrixD::Rotation(0.0,  1.0, 0.0,
                                                                                  -1.0, 0.0, 0.0,
                                                                                  0.0,  0.0, 1.0));
    test_rigid_body_container[BodyEnum::BODY1]->collision_geometry.reset(
        new collisions_geometry::CollisionGeometryBox(1, 0.1, 0.1,
                                                     col_transform2.getPosition(),
                                                     col_transform2.getRotation()));

    auto results = collider_manager->checkCollision(cg0, cg1);
    EXPECT_TRUE(results.colliding());
    expectContains(results.colliding_pairs,std::make_pair(BodyEnum::BODY0, BodyEnum::BODY1));

}

// Check that two boxes will don't collide if they are rotated to not collide.
// The boxes are positioned using the collision_geometry transforms.
// The rigid_bodies are both at the origin.
TEST(COLLIDINGTest, two_boxes_not_colliding_internal_rotation) {

  std::shared_ptr< collisions::ColliderManager<CoordinateFrame> >
     collider_manager(new TestColliderManagerFcl);

  TestRigidBodyContainer test_rigid_body_container;
  fillTestBodyContainer(test_rigid_body_container);

  std::set<BodyEnum> cg0_set = {BodyEnum::BODY0};
  std::set<BodyEnum> cg1_set = {BodyEnum::BODY1};

  TestCollisionGroup cg0(cg0_set, test_rigid_body_container);
  TestCollisionGroup cg1(cg1_set, test_rigid_body_container);

  kindr::HomTransformMatrixD col_transform(kindr::HomTransformMatrixD::Position(0.0, 0.3, 0.0),
                                            kindr::HomTransformMatrixD::Rotation());
   test_rigid_body_container[BodyEnum::BODY0]->collision_geometry.reset(
       new collisions_geometry::CollisionGeometryBox(0.1, 1.0, 0.1,
                                                    col_transform.getPosition(),
                                                    col_transform.getRotation()));

   kindr::HomTransformMatrixD col_transform2(kindr::HomTransformMatrixD::Position(0.0, -0.3, 0.0),
                                             kindr::HomTransformMatrixD::Rotation(0.0,  1.0, 0.0,
                                                                                  -1.0, 0.0, 0.0,
                                                                                  0.0,  0.0, 1.0));
    test_rigid_body_container[BodyEnum::BODY1]->collision_geometry.reset(
        new collisions_geometry::CollisionGeometryBox(0.1, 1.0, 0.1,
                                                     col_transform2.getPosition(),
                                                     col_transform2.getRotation()));

  auto results = collider_manager->checkCollision(cg0, cg1);
  EXPECT_FALSE(results.colliding());

}

// Check that two boxes will collide.
// The boxes are positioned using the rigid_body transforms.
// The collision_bodies are both at the origin of the rigid_bodies.
TEST(COLLIDINGTest, two_boxes_colliding_external_translation) {

  std::shared_ptr< collisions::ColliderManager<CoordinateFrame> >
     collider_manager(new TestColliderManagerFcl);

  TestRigidBodyContainer test_rigid_body_container;
  fillTestBodyContainer(test_rigid_body_container);

  std::set<BodyEnum> cg0_set = {BodyEnum::BODY0};
  std::set<BodyEnum> cg1_set = {BodyEnum::BODY1};

  TestCollisionGroup cg0(cg0_set, test_rigid_body_container);
  TestCollisionGroup cg1(cg1_set, test_rigid_body_container);

  kindr::HomTransformMatrixD col_transform(kindr::HomTransformMatrixD::Position(0.0,0.0,0.0),
                                            kindr::HomTransformMatrixD::Rotation());
   test_rigid_body_container[BodyEnum::BODY0]->collision_geometry.reset(
       new collisions_geometry::CollisionGeometryBox(1, 1, 1,
                                                    col_transform.getPosition(),
                                                    col_transform.getRotation()));
   kindr::HomTransformMatrixD body_transform(kindr::HomTransformMatrixD::Position(0.2,0.0,0.0),
                                             kindr::HomTransformMatrixD::Rotation());
   std::static_pointer_cast<TestRigidBody>(test_rigid_body_container[BodyEnum::BODY0])->setTransform(body_transform);

   kindr::HomTransformMatrixD col_transform2(kindr::HomTransformMatrixD::Position(0.0,0.0,0.0),
                                             kindr::HomTransformMatrixD::Rotation());
    test_rigid_body_container[BodyEnum::BODY1]->collision_geometry.reset(
        new collisions_geometry::CollisionGeometryBox(1, 1, 1,
                                                     col_transform2.getPosition(),
                                                     col_transform2.getRotation()));
    kindr::HomTransformMatrixD body_transform2(kindr::HomTransformMatrixD::Position(-0.2,0.0,0.0),
                                              kindr::HomTransformMatrixD::Rotation());
    std::static_pointer_cast<TestRigidBody>(test_rigid_body_container[BodyEnum::BODY1])->setTransform(body_transform2);

  auto results = collider_manager->checkCollision(cg0, cg1);
  EXPECT_TRUE(results.colliding());
  expectContains(results.colliding_pairs,std::make_pair(BodyEnum::BODY0, BodyEnum::BODY1));

}

// Check that two boxes don't collide if they are too far apart.
// The boxes are positioned using the rigid_body transforms.
// The collision_bodies are both at the origin of the rigid_bodies.
TEST(COLLIDINGTest, two_boxes_not_colliding_external_translation) {

  std::shared_ptr< collisions::ColliderManager<CoordinateFrame> >
     collider_manager(new TestColliderManagerFcl);

  TestRigidBodyContainer test_rigid_body_container;
  fillTestBodyContainer(test_rigid_body_container);

  std::set<BodyEnum> cg0_set = {BodyEnum::BODY0};
  std::set<BodyEnum> cg1_set = {BodyEnum::BODY1};

  TestCollisionGroup cg0(cg0_set, test_rigid_body_container);
  TestCollisionGroup cg1(cg1_set, test_rigid_body_container);

  kindr::HomTransformMatrixD col_transform(kindr::HomTransformMatrixD::Position(0.0,0.0,0.0),
                                            kindr::HomTransformMatrixD::Rotation());
   test_rigid_body_container[BodyEnum::BODY0]->collision_geometry.reset(
       new collisions_geometry::CollisionGeometryBox(1, 1, 1,
                                                    col_transform.getPosition(),
                                                    col_transform.getRotation()));
   kindr::HomTransformMatrixD body_transform(kindr::HomTransformMatrixD::Position(0.6,0.0,0.0),
                                             kindr::HomTransformMatrixD::Rotation());
   std::static_pointer_cast<TestRigidBody>(test_rigid_body_container[BodyEnum::BODY0])->setTransform(body_transform);

   kindr::HomTransformMatrixD col_transform2(kindr::HomTransformMatrixD::Position(0.0,0.0,0.0),
                                             kindr::HomTransformMatrixD::Rotation());
    test_rigid_body_container[BodyEnum::BODY1]->collision_geometry.reset(
        new collisions_geometry::CollisionGeometryBox(1, 1, 1,
                                                     col_transform2.getPosition(),
                                                     col_transform2.getRotation()));
    kindr::HomTransformMatrixD body_transform2(kindr::HomTransformMatrixD::Position(-0.6,0.0,0.0),
                                              kindr::HomTransformMatrixD::Rotation());
    std::static_pointer_cast<TestRigidBody>(test_rigid_body_container[BodyEnum::BODY1])->setTransform(body_transform2);

  auto results = collider_manager->checkCollision(cg0, cg1);
  EXPECT_FALSE(results.colliding());

}

// Check that two boxes will collide if rotated to collide.
// The boxes are positioned using the rigid_body transforms.
// The collision_bodies are both at the origin of the rigid_bodies.
TEST(COLLIDINGTest, two_boxes_colliding_external_rotation) {

  std::shared_ptr< collisions::ColliderManager<CoordinateFrame> >
     collider_manager(new TestColliderManagerFcl);

  TestRigidBodyContainer test_rigid_body_container;
  fillTestBodyContainer(test_rigid_body_container);

  std::set<BodyEnum> cg0_set = {BodyEnum::BODY0};
  std::set<BodyEnum> cg1_set = {BodyEnum::BODY1};

  TestCollisionGroup cg0(cg0_set, test_rigid_body_container);
  TestCollisionGroup cg1(cg1_set, test_rigid_body_container);

  kindr::HomTransformMatrixD col_transform(kindr::HomTransformMatrixD::Position(0.0,0.0,0.0),
                                            kindr::HomTransformMatrixD::Rotation());
   test_rigid_body_container[BodyEnum::BODY0]->collision_geometry.reset(
       new collisions_geometry::CollisionGeometryBox(1, 0.1, 0.1,
                                                    col_transform.getPosition(),
                                                    col_transform.getRotation()));
   kindr::HomTransformMatrixD body_transform(kindr::HomTransformMatrixD::Position(-0.2,0.0,0.0),
                                             kindr::HomTransformMatrixD::Rotation(0.0,  1.0, 0.0,
                                                                                 -1.0, 0.0, 0.0,
                                                                                 0.0,  0.0, 1.0));
   std::static_pointer_cast<TestRigidBody>(test_rigid_body_container[BodyEnum::BODY0])->setTransform(body_transform);

   kindr::HomTransformMatrixD col_transform2(kindr::HomTransformMatrixD::Position(0.0,0.0,0.0),
                                             kindr::HomTransformMatrixD::Rotation());
    test_rigid_body_container[BodyEnum::BODY1]->collision_geometry.reset(
        new collisions_geometry::CollisionGeometryBox(1, 0.1, 0.1,
                                                     col_transform2.getPosition(),
                                                     col_transform2.getRotation()));
    kindr::HomTransformMatrixD body_transform2(kindr::HomTransformMatrixD::Position(-0.2,0.0,0.0),
                                              kindr::HomTransformMatrixD::Rotation());
    std::static_pointer_cast<TestRigidBody>(test_rigid_body_container[BodyEnum::BODY1])->setTransform(body_transform2);

  auto results = collider_manager->checkCollision(cg0, cg1);
  EXPECT_TRUE(results.colliding());
  expectContains(results.colliding_pairs,std::make_pair(BodyEnum::BODY0, BodyEnum::BODY1));

}

// Check that two boxes don't collide if rotated to not collide.
// The boxes are positioned using the rigid_body transforms.
// The collision_bodies are both at the origin of the rigid_bodies.
TEST(COLLIDINGTest, two_boxes_not_colliding_external_rotation) {

  std::shared_ptr< collisions::ColliderManager<CoordinateFrame> >
     collider_manager(new TestColliderManagerFcl);

  TestRigidBodyContainer test_rigid_body_container;
  fillTestBodyContainer(test_rigid_body_container);

  std::set<BodyEnum> cg0_set = {BodyEnum::BODY0};
  std::set<BodyEnum> cg1_set = {BodyEnum::BODY1};

  TestCollisionGroup cg0(cg0_set, test_rigid_body_container);
  TestCollisionGroup cg1(cg1_set, test_rigid_body_container);

  kindr::HomTransformMatrixD col_transform(kindr::HomTransformMatrixD::Position(0.0,0.3,0.0),
                                            kindr::HomTransformMatrixD::Rotation());
   test_rigid_body_container[BodyEnum::BODY0]->collision_geometry.reset(
       new collisions_geometry::CollisionGeometryBox(0.1, 1.0, 0.1,
                                                    col_transform.getPosition(),
                                                    col_transform.getRotation()));
   kindr::HomTransformMatrixD body_transform(kindr::HomTransformMatrixD::Position(0.0,0.0,0.0),
                                             kindr::HomTransformMatrixD::Rotation());
   std::static_pointer_cast<TestRigidBody>(test_rigid_body_container[BodyEnum::BODY0])->setTransform(body_transform);


   kindr::HomTransformMatrixD col_transform2(kindr::HomTransformMatrixD::Position(0.0,0.0,0.0),
                                             kindr::HomTransformMatrixD::Rotation());
    test_rigid_body_container[BodyEnum::BODY1]->collision_geometry.reset(
        new collisions_geometry::CollisionGeometryBox(0.1, 1., 0.1,
                                                     col_transform2.getPosition(),
                                                     col_transform2.getRotation()));
    kindr::HomTransformMatrixD body_transform2(kindr::HomTransformMatrixD::Position(0.0,-0.3,0.0),
                                              kindr::HomTransformMatrixD::Rotation(0.0,  1.0, 0.0,
                                                                                   -1.0, 0.0, 0.0,
                                                                                   0.0,  0.0, 1.0));
    std::static_pointer_cast<TestRigidBody>(test_rigid_body_container[BodyEnum::BODY1])->setTransform(body_transform2);


  auto results = collider_manager->checkCollision(cg0, cg1);
  EXPECT_FALSE(results.colliding());

}

// Tests collisions with both colliding objects and empty objects.
TEST(COLLIDINGTest, two_boxes_colliding_internal_translation_two_empty) {

  std::shared_ptr< collisions::ColliderManager<CoordinateFrame> >
     collider_manager(new TestColliderManagerFcl);

  TestRigidBodyContainer test_rigid_body_container;
  fillTestBodyContainer(test_rigid_body_container);

  std::set<BodyEnum> cg0_set = {BodyEnum::BODY0, BodyEnum::BODY2};
  std::set<BodyEnum> cg1_set = {BodyEnum::BODY1, BodyEnum::BODY3};

  TestCollisionGroup cg0(cg0_set, test_rigid_body_container);
  TestCollisionGroup cg1(cg1_set, test_rigid_body_container);

  kindr::HomTransformMatrixD col_transform(kindr::HomTransformMatrixD::Position(0.2, 0.0, 0.0),
                                            kindr::HomTransformMatrixD::Rotation());
   test_rigid_body_container[BodyEnum::BODY0]->collision_geometry.reset(
       new collisions_geometry::CollisionGeometryBox(1, 1, 1,
                                                    col_transform.getPosition(),
                                                    col_transform.getRotation()));

   kindr::HomTransformMatrixD col_transform2(kindr::HomTransformMatrixD::Position(-0.2, 0.0, 0.0),
                                             kindr::HomTransformMatrixD::Rotation());
    test_rigid_body_container[BodyEnum::BODY1]->collision_geometry.reset(
        new collisions_geometry::CollisionGeometryBox(1, 1, 1,
                                                     col_transform2.getPosition(),
                                                     col_transform2.getRotation()));

  auto results = collider_manager->checkCollision(cg0, cg1);
  EXPECT_TRUE(results.colliding());
  expectContains(results.colliding_pairs,std::make_pair(BodyEnum::BODY0, BodyEnum::BODY1));

}

// Tests collisions of one object with three objects
TEST(COLLIDINGTest, one_box_vs_three_colliding_internal_translation) {

  std::shared_ptr< collisions::ColliderManager<CoordinateFrame> >
     collider_manager(new TestColliderManagerFcl);

  TestRigidBodyContainer test_rigid_body_container;
  fillTestBodyContainer(test_rigid_body_container);

  std::set<BodyEnum> cg0_set = {BodyEnum::BODY0};
  std::set<BodyEnum> cg1_set = {BodyEnum::BODY1, BodyEnum::BODY2, BodyEnum::BODY3};

  TestCollisionGroup cg0(cg0_set, test_rigid_body_container);
  TestCollisionGroup cg1(cg1_set, test_rigid_body_container);

  kindr::HomTransformMatrixD col_transform(kindr::HomTransformMatrixD::Position(0.2, 0.0, 0.0),
                                            kindr::HomTransformMatrixD::Rotation());
   test_rigid_body_container[BodyEnum::BODY0]->collision_geometry.reset(
       new collisions_geometry::CollisionGeometryBox(1, 1, 1,
                                                    col_transform.getPosition(),
                                                    col_transform.getRotation()));

   kindr::HomTransformMatrixD col_transform2(kindr::HomTransformMatrixD::Position(-0.2, 0.0, 0.0),
                                             kindr::HomTransformMatrixD::Rotation());
    test_rigid_body_container[BodyEnum::BODY1]->collision_geometry.reset(
        new collisions_geometry::CollisionGeometryBox(1, 1, 1,
                                                     col_transform2.getPosition(),
                                                     col_transform2.getRotation()));

    kindr::HomTransformMatrixD col_transform3(kindr::HomTransformMatrixD::Position(0.0, 0.2, 0.0),
                                              kindr::HomTransformMatrixD::Rotation());
     test_rigid_body_container[BodyEnum::BODY2]->collision_geometry.reset(
         new collisions_geometry::CollisionGeometryBox(1, 1, 1,
                                                      col_transform3.getPosition(),
                                                      col_transform3.getRotation()));

     kindr::HomTransformMatrixD col_transform4(kindr::HomTransformMatrixD::Position(0.0, -0.2, 0.0),
                                               kindr::HomTransformMatrixD::Rotation());
      test_rigid_body_container[BodyEnum::BODY3]->collision_geometry.reset(
          new collisions_geometry::CollisionGeometryBox(1, 1, 1,
                                                       col_transform4.getPosition(),
                                                       col_transform4.getRotation()));



  auto results = collider_manager->checkCollision(cg0, cg1);
  EXPECT_TRUE(results.colliding());
  expectContains(results.colliding_pairs,std::make_pair(BodyEnum::BODY0, BodyEnum::BODY1));
  expectContains(results.colliding_pairs,std::make_pair(BodyEnum::BODY0, BodyEnum::BODY2));
  expectContains(results.colliding_pairs,std::make_pair(BodyEnum::BODY0, BodyEnum::BODY3));
}

// Tests collision of three objects vs one object
TEST(COLLIDINGTest, three_boxes_vs_one_colliding_internal_translation) {

  std::shared_ptr< collisions::ColliderManager<CoordinateFrame> >
     collider_manager(new TestColliderManagerFcl);

  TestRigidBodyContainer test_rigid_body_container;
  fillTestBodyContainer(test_rigid_body_container);

  std::set<BodyEnum> cg0_set = {BodyEnum::BODY0, BodyEnum::BODY1, BodyEnum::BODY2};
  std::set<BodyEnum> cg1_set = { BodyEnum::BODY3};

  TestCollisionGroup cg0(cg0_set, test_rigid_body_container);
  TestCollisionGroup cg1(cg1_set, test_rigid_body_container);

  kindr::HomTransformMatrixD col_transform(kindr::HomTransformMatrixD::Position(0.2, 0.0, 0.0),
                                            kindr::HomTransformMatrixD::Rotation());
   test_rigid_body_container[BodyEnum::BODY0]->collision_geometry.reset(
       new collisions_geometry::CollisionGeometryBox(1, 1, 1,
                                                    col_transform.getPosition(),
                                                    col_transform.getRotation()));

   kindr::HomTransformMatrixD col_transform2(kindr::HomTransformMatrixD::Position(-0.2, 0.0, 0.0),
                                             kindr::HomTransformMatrixD::Rotation());
    test_rigid_body_container[BodyEnum::BODY1]->collision_geometry.reset(
        new collisions_geometry::CollisionGeometryBox(1, 1, 1,
                                                     col_transform2.getPosition(),
                                                     col_transform2.getRotation()));

    kindr::HomTransformMatrixD col_transform3(kindr::HomTransformMatrixD::Position(0.0, 0.2, 0.0),
                                              kindr::HomTransformMatrixD::Rotation());
     test_rigid_body_container[BodyEnum::BODY2]->collision_geometry.reset(
         new collisions_geometry::CollisionGeometryBox(1, 1, 1,
                                                      col_transform3.getPosition(),
                                                      col_transform3.getRotation()));

     kindr::HomTransformMatrixD col_transform4(kindr::HomTransformMatrixD::Position(0.0, -0.2, 0.0),
                                               kindr::HomTransformMatrixD::Rotation());
      test_rigid_body_container[BodyEnum::BODY3]->collision_geometry.reset(
          new collisions_geometry::CollisionGeometryBox(1, 1, 1,
                                                       col_transform4.getPosition(),
                                                       col_transform4.getRotation()));

  auto results = collider_manager->checkCollision(cg0, cg1);

  EXPECT_TRUE(results.colliding());
  expectContains(results.colliding_pairs,std::make_pair(BodyEnum::BODY0, BodyEnum::BODY3));
  expectContains(results.colliding_pairs,std::make_pair(BodyEnum::BODY1, BodyEnum::BODY3));
  expectContains(results.colliding_pairs,std::make_pair(BodyEnum::BODY2, BodyEnum::BODY3));

}


} // namespace collisions_fcl_tests
