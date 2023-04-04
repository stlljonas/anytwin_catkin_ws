/*
 * multiple_enum_tests.cpp
 *
 *  Created on: Jul 30, 2017
 *      Author: Perry Franklin
 */

#include "test_declarations.hpp"

#include <collisions_fcl/ColliderManagerFcl.hpp>

#include <gtest/gtest.h>

using namespace collisions_fcl;

namespace collisions_fcl_tests {

enum class BodyEnum2 : unsigned int {
  BODY4 = 0,
  BODY5,
  BODY6
};

enum class BodyNodeEnum2 : unsigned int {
  NODE0 = 0,
  NODE1,
  NODE2
};

enum class BranchEnum2 : unsigned int {
  ALL = 0
};

using TestRigidBody2 = TestRigidBodyBase<BodyEnum2, BodyNodeEnum2, BranchEnum2, CoordinateFrame>;

using TestRigidBodyContainer2 = romo::RigidBodyShPtrContainer<BodyEnum2, BodyNodeEnum2, BranchEnum2, CoordinateFrame>;

//===========================================================


using TestColliderManagerFcl = ColliderManagerFcl<CoordinateFrame>;
using TestCollisionGroup = collisions::CollisionGroup<BodyEnum, BodyNodeEnum, BranchEnum, CoordinateFrame>;

using TestCollisionGroup2 = collisions::CollisionGroup<BodyEnum2, BodyNodeEnum2, BranchEnum2, CoordinateFrame>;


void fillTestBodyContainer2(TestRigidBodyContainer2& rigidBodyContainer)
{
    rigidBodyContainer.clear();

      std::tuple<std::string, unsigned int, BodyEnum2> key_tuple0 = std::make_tuple("Body4", 0 ,BodyEnum2::BODY4);
      rigidBodyContainer.insert(key_tuple0, std::make_shared<TestRigidBody2>("Body4",
                                                                            BodyEnum2::BODY4,
                                                                            BodyNodeEnum2::NODE0,
                                                                            BranchEnum2::ALL));

      std::tuple<std::string, unsigned int, BodyEnum2> key_tuple1 = std::make_tuple("Body5", 1 ,BodyEnum2::BODY5);
      rigidBodyContainer.insert(key_tuple1, std::make_shared<TestRigidBody2>("Body5",
                                                                            BodyEnum2::BODY5,
                                                                            BodyNodeEnum2::NODE1,
                                                                            BranchEnum2::ALL));

      std::tuple<std::string, unsigned int, BodyEnum2> key_tuple2 = std::make_tuple("Body6", 2 ,BodyEnum2::BODY6);
      rigidBodyContainer.insert(key_tuple2, std::make_shared<TestRigidBody2>("Body6",
                                                                            BodyEnum2::BODY6,
                                                                            BodyNodeEnum2::NODE2,
                                                                            BranchEnum2::ALL));

}

//================================================================================================================================

// Checks that no collision occurs if no geometry is set.
TEST(COLLIDING_MULTI_ENUMTest, No_geometry) {

  std::shared_ptr< collisions::ColliderManager<CoordinateFrame> >
     collider_manager(new TestColliderManagerFcl);

  TestRigidBodyContainer test_rigid_body_container;
  fillTestBodyContainer(test_rigid_body_container);

  TestRigidBodyContainer2 test_rigid_body_container2;
  fillTestBodyContainer2(test_rigid_body_container2);

  std::set<BodyEnum> cg0_set = {BodyEnum::BODY0};
  std::set<BodyEnum2> cg1_set = {BodyEnum2::BODY5};

  TestCollisionGroup cg0(cg0_set, test_rigid_body_container);
  TestCollisionGroup2 cg1(cg1_set, test_rigid_body_container2);

  auto results = collider_manager->checkCollision(cg0, cg1);
  EXPECT_FALSE(results.colliding());

}

// Checks that no collision occurs between a geometry and no geometry.
TEST(COLLIDING_MULTI_ENUMTest, One_and_none_geometry) {

  std::shared_ptr< collisions::ColliderManager<CoordinateFrame> >
     collider_manager(new TestColliderManagerFcl);

  TestRigidBodyContainer test_rigid_body_container;
  fillTestBodyContainer(test_rigid_body_container);

  TestRigidBodyContainer2 test_rigid_body_container2;
  fillTestBodyContainer2(test_rigid_body_container2);

  std::set<BodyEnum> cg0_set = {BodyEnum::BODY0};
  std::set<BodyEnum2> cg1_set = {BodyEnum2::BODY4};

  TestCollisionGroup cg0(cg0_set, test_rigid_body_container);
  TestCollisionGroup2 cg1(cg1_set, test_rigid_body_container2);

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
TEST(COLLIDING_MULTI_ENUMTest, two_boxes_colliding_internal_translation) {

  std::shared_ptr< collisions::ColliderManager<CoordinateFrame> >
     collider_manager(new TestColliderManagerFcl);

  TestRigidBodyContainer test_rigid_body_container;
  fillTestBodyContainer(test_rigid_body_container);

  TestRigidBodyContainer2 test_rigid_body_container2;
  fillTestBodyContainer2(test_rigid_body_container2);

  std::set<BodyEnum> cg0_set = {BodyEnum::BODY0};
  std::set<BodyEnum2> cg1_set = {BodyEnum2::BODY4};

  TestCollisionGroup cg0(cg0_set, test_rigid_body_container);
  TestCollisionGroup2 cg1(cg1_set, test_rigid_body_container2);

  kindr::HomTransformMatrixD col_transform(kindr::HomTransformMatrixD::Position(0.2, 0.0, 0.0),
                                            kindr::HomTransformMatrixD::Rotation());
   test_rigid_body_container[BodyEnum::BODY0]->collision_geometry.reset(
       new collisions_geometry::CollisionGeometryBox(1, 1, 1,
                                                    col_transform.getPosition(),
                                                    col_transform.getRotation()));

   kindr::HomTransformMatrixD col_transform2(kindr::HomTransformMatrixD::Position(-0.2, 0.0, 0.0),
                                             kindr::HomTransformMatrixD::Rotation());
    test_rigid_body_container2[BodyEnum2::BODY4]->collision_geometry.reset(
        new collisions_geometry::CollisionGeometryBox(1, 1, 1,
                                                     col_transform2.getPosition(),
                                                     col_transform2.getRotation()));

  auto results = collider_manager->checkCollision(cg0, cg1);
  EXPECT_TRUE(results.colliding());
  expectContains(results.colliding_pairs,std::make_pair(BodyEnum::BODY0, BodyEnum2::BODY4));

}

// Check that two boxes don't collide if they are not close enough.
// The boxes are positioned using the collision_geometry transforms.
// The rigid_bodies are both at the origin.
TEST(COLLIDING_MULTI_ENUMTest, two_boxes_not_colliding_internal_translation) {

  std::shared_ptr< collisions::ColliderManager<CoordinateFrame> >
     collider_manager(new TestColliderManagerFcl);

  TestRigidBodyContainer test_rigid_body_container;
  fillTestBodyContainer(test_rigid_body_container);

  TestRigidBodyContainer2 test_rigid_body_container2;
  fillTestBodyContainer2(test_rigid_body_container2);

  std::set<BodyEnum> cg0_set = {BodyEnum::BODY0};
  std::set<BodyEnum2> cg1_set = {BodyEnum2::BODY4};

  TestCollisionGroup cg0(cg0_set, test_rigid_body_container);
  TestCollisionGroup2 cg1(cg1_set, test_rigid_body_container2);

  kindr::HomTransformMatrixD col_transform(kindr::HomTransformMatrixD::Position(0.6, 0.0, 0.0),
                                            kindr::HomTransformMatrixD::Rotation());
   test_rigid_body_container[BodyEnum::BODY0]->collision_geometry.reset(
       new collisions_geometry::CollisionGeometryBox(1, 1, 1,
                                                    col_transform.getPosition(),
                                                    col_transform.getRotation()));

   kindr::HomTransformMatrixD col_transform2(kindr::HomTransformMatrixD::Position(-0.6, 0.0, 0.0),
                                             kindr::HomTransformMatrixD::Rotation());
    test_rigid_body_container2[BodyEnum2::BODY4]->collision_geometry.reset(
        new collisions_geometry::CollisionGeometryBox(1, 1, 1,
                                                     col_transform2.getPosition(),
                                                     col_transform2.getRotation()));

  auto results = collider_manager->checkCollision(cg0, cg1);
  EXPECT_FALSE(results.colliding());

}

// Check that two boxes will collide if they are rotated to collide.
// The boxes are positioned using the collision_geometry transforms.
// The rigid_bodies are both at the origin.
TEST(COLLIDING_MULTI_ENUMTest, two_boxes_colliding_internal_rotation) {

  std::shared_ptr< collisions::ColliderManager<CoordinateFrame> >
     collider_manager(new TestColliderManagerFcl);

  TestRigidBodyContainer test_rigid_body_container;
  fillTestBodyContainer(test_rigid_body_container);

  TestRigidBodyContainer2 test_rigid_body_container2;
  fillTestBodyContainer2(test_rigid_body_container2);

  std::set<BodyEnum> cg0_set = {BodyEnum::BODY0};
  std::set<BodyEnum2> cg1_set = {BodyEnum2::BODY4};

  TestCollisionGroup cg0(cg0_set, test_rigid_body_container);
  TestCollisionGroup2 cg1(cg1_set, test_rigid_body_container2);

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
    test_rigid_body_container2[BodyEnum2::BODY4]->collision_geometry.reset(
        new collisions_geometry::CollisionGeometryBox(1, 0.1, 0.1,
                                                     col_transform2.getPosition(),
                                                     col_transform2.getRotation()));

  auto results = collider_manager->checkCollision(cg0, cg1);
  EXPECT_TRUE(results.colliding());
  expectContains(results.colliding_pairs,std::make_pair(BodyEnum::BODY0, BodyEnum2::BODY4));

}

// Check that two boxes will don't collide if they are rotated to not collide.
// The boxes are positioned using the collision_geometry transforms.
// The rigid_bodies are both at the origin.
TEST(COLLIDING_MULTI_ENUMTest, two_boxes_not_colliding_internal_rotation) {

  std::shared_ptr< collisions::ColliderManager<CoordinateFrame> >
     collider_manager(new TestColliderManagerFcl);

  TestRigidBodyContainer test_rigid_body_container;
  fillTestBodyContainer(test_rigid_body_container);

  TestRigidBodyContainer2 test_rigid_body_container2;
  fillTestBodyContainer2(test_rigid_body_container2);

  std::set<BodyEnum> cg0_set = {BodyEnum::BODY0};
  std::set<BodyEnum2> cg1_set = {BodyEnum2::BODY4};

  TestCollisionGroup cg0(cg0_set, test_rigid_body_container);
  TestCollisionGroup2 cg1(cg1_set, test_rigid_body_container2);

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
    test_rigid_body_container2[BodyEnum2::BODY4]->collision_geometry.reset(
        new collisions_geometry::CollisionGeometryBox(0.1, 1.0, 0.1,
                                                     col_transform2.getPosition(),
                                                     col_transform2.getRotation()));

  auto results = collider_manager->checkCollision(cg0, cg1);
  EXPECT_FALSE(results.colliding());

}

// Check that two boxes will collide.
// The boxes are positioned using the rigid_body transforms.
// The collision_bodies are both at the origin of the rigid_bodies.
TEST(COLLIDING_MULTI_ENUMTest, two_boxes_colliding_external_translation) {

  std::shared_ptr< collisions::ColliderManager<CoordinateFrame> >
     collider_manager(new TestColliderManagerFcl);

  TestRigidBodyContainer test_rigid_body_container;
  fillTestBodyContainer(test_rigid_body_container);

  TestRigidBodyContainer2 test_rigid_body_container2;
  fillTestBodyContainer2(test_rigid_body_container2);

  std::set<BodyEnum> cg0_set = {BodyEnum::BODY0};
  std::set<BodyEnum2> cg1_set = {BodyEnum2::BODY4};

  TestCollisionGroup cg0(cg0_set, test_rigid_body_container);
  TestCollisionGroup2 cg1(cg1_set, test_rigid_body_container2);

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
    test_rigid_body_container2[BodyEnum2::BODY4]->collision_geometry.reset(
        new collisions_geometry::CollisionGeometryBox(1, 1, 1,
                                                     col_transform2.getPosition(),
                                                     col_transform2.getRotation()));
    kindr::HomTransformMatrixD body_transform2(kindr::HomTransformMatrixD::Position(-0.2,0.0,0.0),
                                              kindr::HomTransformMatrixD::Rotation());
    std::static_pointer_cast<TestRigidBody2>(test_rigid_body_container2[BodyEnum2::BODY4])->setTransform(body_transform2);

  auto results = collider_manager->checkCollision(cg0, cg1);
  EXPECT_TRUE(results.colliding());
  expectContains(results.colliding_pairs,std::make_pair(BodyEnum::BODY0, BodyEnum2::BODY4));

}

// Check that two boxes don't collide if they are too far apart.
// The boxes are positioned using the rigid_body transforms.
// The collision_bodies are both at the origin of the rigid_bodies.
TEST(COLLIDING_MULTI_ENUMTest, two_boxes_not_colliding_external_translation) {

  std::shared_ptr< collisions::ColliderManager<CoordinateFrame> >
     collider_manager(new TestColliderManagerFcl);

  TestRigidBodyContainer test_rigid_body_container;
  fillTestBodyContainer(test_rigid_body_container);

  TestRigidBodyContainer2 test_rigid_body_container2;
  fillTestBodyContainer2(test_rigid_body_container2);

  std::set<BodyEnum> cg0_set = {BodyEnum::BODY0};
  std::set<BodyEnum2> cg1_set = {BodyEnum2::BODY4};

  TestCollisionGroup cg0(cg0_set, test_rigid_body_container);
  TestCollisionGroup2 cg1(cg1_set, test_rigid_body_container2);

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
    std::static_pointer_cast<TestRigidBody2>(test_rigid_body_container2[BodyEnum2::BODY4])->setTransform(body_transform2);

  auto results = collider_manager->checkCollision(cg0, cg1);
  EXPECT_FALSE(results.colliding());

}

// Check that two boxes will collide if rotated to collide.
// The boxes are positioned using the rigid_body transforms.
// The collision_bodies are both at the origin of the rigid_bodies.
TEST(COLLIDING_MULTI_ENUMTest, two_boxes_colliding_external_rotation) {

  std::shared_ptr< collisions::ColliderManager<CoordinateFrame> >
     collider_manager(new TestColliderManagerFcl);

  TestRigidBodyContainer test_rigid_body_container;
  fillTestBodyContainer(test_rigid_body_container);

  TestRigidBodyContainer2 test_rigid_body_container2;
  fillTestBodyContainer2(test_rigid_body_container2);

  std::set<BodyEnum> cg0_set = {BodyEnum::BODY0};
  std::set<BodyEnum2> cg1_set = {BodyEnum2::BODY4};

  TestCollisionGroup cg0(cg0_set, test_rigid_body_container);
  TestCollisionGroup2 cg1(cg1_set, test_rigid_body_container2);

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
    test_rigid_body_container2[BodyEnum2::BODY4]->collision_geometry.reset(
        new collisions_geometry::CollisionGeometryBox(1, 0.1, 0.1,
                                                     col_transform2.getPosition(),
                                                     col_transform2.getRotation()));
    kindr::HomTransformMatrixD body_transform2(kindr::HomTransformMatrixD::Position(-0.2,0.0,0.0),
                                              kindr::HomTransformMatrixD::Rotation());
    std::static_pointer_cast<TestRigidBody2>(test_rigid_body_container2[BodyEnum2::BODY4])->setTransform(body_transform2);

  auto results = collider_manager->checkCollision(cg0, cg1);
  EXPECT_TRUE(results.colliding());
  expectContains(results.colliding_pairs,std::make_pair(BodyEnum::BODY0, BodyEnum2::BODY4));

}

// Check that two boxes don't collide if rotated to not collide.
// The boxes are positioned using the rigid_body transforms.
// The collision_bodies are both at the origin of the rigid_bodies.
TEST(COLLIDING_MULTI_ENUMTest, two_boxes_not_colliding_external_rotation) {

  std::shared_ptr< collisions::ColliderManager<CoordinateFrame> >
     collider_manager(new TestColliderManagerFcl);

  TestRigidBodyContainer test_rigid_body_container;
  fillTestBodyContainer(test_rigid_body_container);

  TestRigidBodyContainer2 test_rigid_body_container2;
  fillTestBodyContainer2(test_rigid_body_container2);

  std::set<BodyEnum> cg0_set = {BodyEnum::BODY0};
  std::set<BodyEnum2> cg1_set = {BodyEnum2::BODY4};

  TestCollisionGroup cg0(cg0_set, test_rigid_body_container);
  TestCollisionGroup2 cg1(cg1_set, test_rigid_body_container2);

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
    test_rigid_body_container2[BodyEnum2::BODY4]->collision_geometry.reset(
        new collisions_geometry::CollisionGeometryBox(0.1, 1., 0.1,
                                                     col_transform2.getPosition(),
                                                     col_transform2.getRotation()));
    kindr::HomTransformMatrixD body_transform2(kindr::HomTransformMatrixD::Position(0.0,-0.3,0.0),
                                              kindr::HomTransformMatrixD::Rotation(0.0,  1.0, 0.0,
                                                                                   -1.0, 0.0, 0.0,
                                                                                   0.0,  0.0, 1.0));
    std::static_pointer_cast<TestRigidBody2>(test_rigid_body_container2[BodyEnum2::BODY4])->setTransform(body_transform2);


  auto results = collider_manager->checkCollision(cg0, cg1);
  EXPECT_FALSE(results.colliding());

}

// Tests collisions with both colliding objects and empty objects.
TEST(COLLIDING_MULTI_ENUMTest, two_boxes_colliding_internal_translation_two_empty) {

  std::shared_ptr< collisions::ColliderManager<CoordinateFrame> >
     collider_manager(new TestColliderManagerFcl);

  TestRigidBodyContainer test_rigid_body_container;
  fillTestBodyContainer(test_rigid_body_container);

  TestRigidBodyContainer2 test_rigid_body_container2;
  fillTestBodyContainer2(test_rigid_body_container2);

  std::set<BodyEnum> cg0_set = {BodyEnum::BODY0, BodyEnum::BODY2};
  std::set<BodyEnum2> cg1_set = {BodyEnum2::BODY4, BodyEnum2::BODY5};

  TestCollisionGroup cg0(cg0_set, test_rigid_body_container);
  TestCollisionGroup2 cg1(cg1_set, test_rigid_body_container2);

  kindr::HomTransformMatrixD col_transform(kindr::HomTransformMatrixD::Position(0.2, 0.0, 0.0),
                                            kindr::HomTransformMatrixD::Rotation());
   test_rigid_body_container[BodyEnum::BODY0]->collision_geometry.reset(
       new collisions_geometry::CollisionGeometryBox(1, 1, 1,
                                                    col_transform.getPosition(),
                                                    col_transform.getRotation()));

   kindr::HomTransformMatrixD col_transform2(kindr::HomTransformMatrixD::Position(-0.2, 0.0, 0.0),
                                             kindr::HomTransformMatrixD::Rotation());
    test_rigid_body_container2[BodyEnum2::BODY4]->collision_geometry.reset(
        new collisions_geometry::CollisionGeometryBox(1, 1, 1,
                                                     col_transform2.getPosition(),
                                                     col_transform2.getRotation()));

  auto results = collider_manager->checkCollision(cg0, cg1);
  EXPECT_TRUE(results.colliding());
  expectContains(results.colliding_pairs,std::make_pair(BodyEnum::BODY0, BodyEnum2::BODY4));

}

// Tests collisions of one object with three objects
TEST(COLLIDING_MULTI_ENUMTest, one_box_vs_three_colliding_internal_translation) {

  std::shared_ptr< collisions::ColliderManager<CoordinateFrame> >
     collider_manager(new TestColliderManagerFcl);

  TestRigidBodyContainer test_rigid_body_container;
  fillTestBodyContainer(test_rigid_body_container);

  TestRigidBodyContainer2 test_rigid_body_container2;
  fillTestBodyContainer2(test_rigid_body_container2);

  std::set<BodyEnum> cg0_set = {BodyEnum::BODY0};
  std::set<BodyEnum2> cg1_set = {BodyEnum2::BODY4, BodyEnum2::BODY5, BodyEnum2::BODY6};

  TestCollisionGroup cg0(cg0_set, test_rigid_body_container);
  TestCollisionGroup2 cg1(cg1_set, test_rigid_body_container2);

  kindr::HomTransformMatrixD col_transform(kindr::HomTransformMatrixD::Position(0.2, 0.0, 0.0),
                                            kindr::HomTransformMatrixD::Rotation());
   test_rigid_body_container[BodyEnum::BODY0]->collision_geometry.reset(
       new collisions_geometry::CollisionGeometryBox(1, 1, 1,
                                                    col_transform.getPosition(),
                                                    col_transform.getRotation()));

   kindr::HomTransformMatrixD col_transform2(kindr::HomTransformMatrixD::Position(-0.2, 0.0, 0.0),
                                             kindr::HomTransformMatrixD::Rotation());
    test_rigid_body_container2[BodyEnum2::BODY4]->collision_geometry.reset(
        new collisions_geometry::CollisionGeometryBox(1, 1, 1,
                                                     col_transform2.getPosition(),
                                                     col_transform2.getRotation()));

    kindr::HomTransformMatrixD col_transform3(kindr::HomTransformMatrixD::Position(0.0, 0.2, 0.0),
                                              kindr::HomTransformMatrixD::Rotation());
     test_rigid_body_container2[BodyEnum2::BODY5]->collision_geometry.reset(
         new collisions_geometry::CollisionGeometryBox(1, 1, 1,
                                                      col_transform3.getPosition(),
                                                      col_transform3.getRotation()));

     kindr::HomTransformMatrixD col_transform4(kindr::HomTransformMatrixD::Position(0.0, -0.2, 0.0),
                                               kindr::HomTransformMatrixD::Rotation());
      test_rigid_body_container2[BodyEnum2::BODY6]->collision_geometry.reset(
          new collisions_geometry::CollisionGeometryBox(1, 1, 1,
                                                       col_transform4.getPosition(),
                                                       col_transform4.getRotation()));



  auto results = collider_manager->checkCollision(cg0, cg1);
  EXPECT_TRUE(results.colliding());
  expectContains(results.colliding_pairs,std::make_pair(BodyEnum::BODY0, BodyEnum2::BODY4));
  expectContains(results.colliding_pairs,std::make_pair(BodyEnum::BODY0, BodyEnum2::BODY5));
  expectContains(results.colliding_pairs,std::make_pair(BodyEnum::BODY0, BodyEnum2::BODY6));

}

// Tests collision of three objects vs one object
TEST(COLLIDING_MULTI_ENUMTest, three_boxes_vs_one_colliding_internal_translation) {

  std::shared_ptr< collisions::ColliderManager<CoordinateFrame> >
     collider_manager(new TestColliderManagerFcl);

  TestRigidBodyContainer test_rigid_body_container;
  fillTestBodyContainer(test_rigid_body_container);

  TestRigidBodyContainer2 test_rigid_body_container2;
  fillTestBodyContainer2(test_rigid_body_container2);

  std::set<BodyEnum> cg0_set = {BodyEnum::BODY0, BodyEnum::BODY1, BodyEnum::BODY2};
  std::set<BodyEnum2> cg1_set = { BodyEnum2::BODY4};

  TestCollisionGroup cg0(cg0_set, test_rigid_body_container);
  TestCollisionGroup2 cg1(cg1_set, test_rigid_body_container2);

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
      test_rigid_body_container2[BodyEnum2::BODY4]->collision_geometry.reset(
          new collisions_geometry::CollisionGeometryBox(1, 1, 1,
                                                       col_transform4.getPosition(),
                                                       col_transform4.getRotation()));

  auto results = collider_manager->checkCollision(cg0, cg1);

  EXPECT_TRUE(results.colliding());
  expectContains(results.colliding_pairs,std::make_pair(BodyEnum::BODY0, BodyEnum2::BODY4));
  expectContains(results.colliding_pairs,std::make_pair(BodyEnum::BODY1, BodyEnum2::BODY4));
  expectContains(results.colliding_pairs,std::make_pair(BodyEnum::BODY2, BodyEnum2::BODY4));

}

} // namespace collisions_fcl_tests
