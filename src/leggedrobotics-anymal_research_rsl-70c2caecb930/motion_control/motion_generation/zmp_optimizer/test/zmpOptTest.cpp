/*!
 * @file     zmpOptTest.cpp
 * @author   Fabian Jenelten
 * @date     Feb. 08, 2018
 * @brief
 */

// gtest
#include <gtest/gtest.h>

// zmp_optimizer
#include <zmp_optimizer/QPReader.hpp>
#include <zmp_optimizer/ZmpOptimizerDynamicWalk.hpp>

///*
// * For this test you need measurements for the following gait:
// *  2D: crawling, dynamic_lateral_walk, trot, pace
// *  3D: crawling, dynamic_lateral_walk, trot, pace, flying trot, pronking
// * For reporting the measurements, go to ComSupportControlZmp.cpp and set
// *  writeOptimizationProblemsToFile_ = true
// * and select the desired gait in the ControlParametersSim.xml file.
// */
// TEST(ZMP, DISABLED_feedforward_optimization) {
//  using namespace loco;
//  bool success = true;
//
//  const std::string path = static_cast<std::string>(std::getenv("HOME")) + "/catkin_ws/logs/";
//
//  // QP problems
//  success &= readQPProblem(path, "QP", "crawling");
//  success &= readQPProblem(path, "QP", "dynamic_lateral_walk");
//  success &= readQPProblem(path, "QP", "trot");
//  success &= readQPProblem(path, "QP", "pace");
//
//  // SQP problems
//  success &= readQPProblem(path, "SQP", "crawling");
//  success &= readQPProblem(path, "SQP", "dynamic_lateral_walk");
//  success &= readQPProblem(path, "SQP", "trot");
//  success &= readQPProblem(path, "SQP", "pace");
//  success &= readQPProblem(path, "SQP", "flying_trot");
//  success &= readQPProblem(path, "SQP", "pronking");
//
//  EXPECT_TRUE(success);
//}
//
// bool runZmpOpt(
//    zmp::MotionPlan& motionPlan,
//    loco::ZmpOptimizerDynamicWalk& optimimzer,
//    const std::vector<zmp::CogDim>& optimizationDofs) {
//
//  constexpr double wholeBodyMass = 20.0;
//  constexpr unsigned int numOfLegs = 4u;
//  constexpr double optimzationHorizon = 3.0;
//
//  // Initialize optimizer.
//  if(!optimimzer.initialize(optimizationDofs, wholeBodyMass, Eigen::Matrix3d::Identity())) {
//    MELO_WARN_STREAM("Failed to initialize optimizer.");
//    return false;
//  }
//
//  // Set some motion plan.
//  if(!motionPlan.setIdentity(optimzationHorizon, numOfLegs, optimizationDofs)) {
//    MELO_WARN_STREAM("Failed to set motion plan.");
//    return false;
//  }
//
//  // Run the optimization.
//  if(!optimimzer.computeTrajectory(motionPlan)) {
//    MELO_WARN_STREAM("Optimization failed.");
//    return false;
//  }
//
//  // Check if constraints are satisfied.
//  if(!motionPlan.checkConstraints()) {
//    MELO_WARN_STREAM("Constraints are not satisfied.");
//    return false;
//  }
//
//  std::cout << "Passed!\n";
//  return true;
//}
//
//
// TEST(ZMP, DISABLED_zmp_optimizer_dynamic_walk) {
//  using namespace loco;
//  bool success = true;
//
//  std::vector<zmp::CogDim> optimizationDofs;
//  ZmpOptimizerDynamicWalk optimimzer;
//  zmp::MotionPlan motionPlan;
//
//  // 2d optimizer.
//  optimizationDofs.push_back(zmp::CogDim::x);
//  optimizationDofs.push_back(zmp::CogDim::y);
//  std::cout << "Run optimization with solver as...\n";
//  success &= runZmpOpt(motionPlan, optimimzer, optimizationDofs);
//
//  // 3d optimizer.
//  optimizationDofs.push_back(zmp::CogDim::z);
//  std::cout << "Run optimization with solver SQP...\n";
//  success &= runZmpOpt(motionPlan, optimimzer, optimizationDofs);
//
//  EXPECT_TRUE(success);
//}
