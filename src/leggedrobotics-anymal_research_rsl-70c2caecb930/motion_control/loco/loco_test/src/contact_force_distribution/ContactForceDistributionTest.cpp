/*!
* @file     ContactForceDistributionTest.cpp
* @author   Christian Gehring
* @date     Feb 3, 2015
* @brief
*/


#include <gtest/gtest.h>

#include "loco/common/ParameterSet.hpp"

#include <loco/common/LegBase.hpp>
#include <loco/common/LegGroup.hpp>
#include <loco/common/TorsoBase.hpp>
#include <loco/common/TerrainModelBase.hpp>

#include <loco/common/TerrainModelFreePlane.hpp>

#include <anymal_model/AnymalModel.hpp>
#include <anymal_model/common/Command.hpp>
#include <anymal_model/common/State.hpp>
#include <anymal_model/robots/starleth.hpp>

#include <loco/common/LegStarlETH.hpp>

#include <loco/common/TorsoStarlETH.hpp>


#include <loco/motion_control/ContactForceDistribution.hpp>

#include <kindr/common/gtest_eigen.hpp>

#include <memory>

// robotutils
#include <robot_utils/physical_definitions.hpp>


struct CommonFixture : public ::testing::Test {
  double timeStep_;
  std::string parameterFile_;
  std::shared_ptr<loco::LegBase> leftForeLeg_;
  std::shared_ptr<loco::LegBase> rightForeLeg_;
  std::shared_ptr<loco::LegBase> leftHindLeg_;
  std::shared_ptr<loco::LegBase> rightHindLeg_;
  std::shared_ptr<loco::LegGroup> legs_;
  std::shared_ptr<loco::TorsoBase> torso_;
  std::shared_ptr<loco::TerrainModelBase> terrainModel_;

  std::shared_ptr<loco::ParameterSet> parameterSet_;
  std::shared_ptr<anymal_model::AnymalModel> anymalModel_;
  std::shared_ptr<anymal_model::Command> command_;
  std::shared_ptr<anymal_model::State> state_;
  CommonFixture():
    timeStep_(0.0025),
    parameterFile_("motion_control/data/ContactForceDistributionTest.xml")
  {
  }
  virtual ~CommonFixture() {};
};



struct StarlETHCommonFixture: public CommonFixture {
  StarlETHCommonFixture() {
    // fixme: intialize with urdf
    anymalModel_.reset(new anymal_model::AnymalModel(timeStep_));

    command_.reset(new anymal_model::Command);
    state_.reset(new anymal_model::State(anymalModel_.get()));
    anymal_model::anymals::starleth::initializeCommand(*command_);
    anymal_model::anymals::starleth::initializeState(*state_);

    this->anymalModel_->init();
    // do not update!
    //this->anymalModel_->update();

    leftForeLeg_.reset(new loco::LegStarlETH("leftFore",   0,  this->anymalModel_.get()));
    rightForeLeg_.reset(new loco::LegStarlETH("rightFore", 1,  this->anymalModel_.get()));
    leftHindLeg_.reset(new loco::LegStarlETH("leftHind", 2,  this->anymalModel_.get()));
    rightHindLeg_.reset(new loco::LegStarlETH("rightHind", 3,  this->anymalModel_.get()));
    legs_.reset( new loco::LegGroup(leftForeLeg_.get(), rightForeLeg_.get(), leftHindLeg_.get(), rightHindLeg_.get()));
    torso_.reset(new loco::TorsoAnymal(this->anymalModel_.get()));
    terrainModel_.reset(new loco::TerrainModelFreePlane);


  }
  virtual ~StarlETHCommonFixture() {

  }
};

struct ContactForceDistributionTest: public StarlETHCommonFixture{
  loco::ContactForceDistribution contactForceDistribution_;
  ContactForceDistributionTest(): contactForceDistribution_(this->torso_, this->legs_, this->terrainModel_) {

  }

};




TEST_F(ContactForceDistributionTest, loadParameters) {
  this->parameterSet_.reset(new loco::ParameterSet());
  ASSERT_TRUE(parameterSet_->loadXmlDocument(this->parameterFile_)) << "Could not load parameter file: " << this->parameterFile_;
  ASSERT_TRUE(this->contactForceDistribution_.loadParameters(parameterSet_->getHandle()));
  for (auto& legInfo : this->contactForceDistribution_.legInfos_) {
    ASSERT_EQ(0.7, legInfo.second.frictionCoefficient_);
    //ASSERT_EQ(2.0, legInfo.second.minimalNormalGroundForce_);
  }
}


TEST_F(ContactForceDistributionTest, distribute) {
  this->parameterSet_.reset(new loco::ParameterSet());
  ASSERT_TRUE(parameterSet_->loadXmlDocument(this->parameterFile_)) << "Could not load parameter file: " << this->parameterFile_;
  ASSERT_TRUE(this->contactForceDistribution_.loadParameters(parameterSet_->getHandle()));

  anymal_model::VectorQ generalizedCoordinates = anymal_model::VectorQ::Zero();
  double qLF_HAA = 0.0;
  double qLF_HFE = 0.7;
  double qLF_KFE = -1.4;
  generalizedCoordinates << 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0,
                            qLF_HAA, qLF_HFE, qLF_KFE,
                            -qLF_HAA, qLF_HFE, qLF_KFE,
                            qLF_HAA, -qLF_HFE, -qLF_KFE,
                            -qLF_HAA, -qLF_HFE, -qLF_KFE;

  anymal_model::VectorQ generalizedVelocities  = anymal_model::VectorQ::Zero();
  anymalModel_->setGeneralizedPositions(generalizedCoordinates);
  anymalModel_->setGeneralizedVelocities(generalizedVelocities);

  for (auto leg : *legs_) {
    leg->advance(this->timeStep_);
  }
  torso_->advance(this->timeStep_);


  loco::Position expectedLeftForeFootPositionInWorldFrame(0.275048, 0.185, -0.401206);


  loco::Vector expectedFootContactNormalInWorldFrame(0, 0, 1.0);
  for (auto leg : *legs_) {
//    std::cout << leg->getDesiredJointTorques();
//    std::cout << leg->getPositionWorldToFootInWorldFrame() << std::endl;
    loco::Vector footContactNormalInWorldFrame;
    terrainModel_->getNormal(leg->getPositionWorldToFootInWorldFrame(), footContactNormalInWorldFrame);
//    std::cout << footContactNormalInWorldFrame << std::endl;
    KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL(expectedFootContactNormalInWorldFrame.toImplementation(), footContactNormalInWorldFrame.toImplementation(), 1e-3, 1e-3, "contact normal");


    std::cout << leg->getTranslationJacobianFromBaseToFootInBaseFrame() << std::endl;

  }


  for (int i=0; i<10; i++) {
    std::cout << "======================== Iteration: " << i << std::endl;



    if (i == 8) {
      leftForeLeg_->setIsSupportLimb(true);
      rightHindLeg_->setIsSupportLimb(false);
      rightForeLeg_->setIsSupportLimb(true);
      leftHindLeg_->setIsSupportLimb(true);
    } else if(i == 9 ) {
      leftForeLeg_->setIsSupportLimb(true);
      rightHindLeg_->setIsSupportLimb(true);
      rightForeLeg_->setIsSupportLimb(false);
      leftHindLeg_->setIsSupportLimb(false);
    }
    else {
      leftForeLeg_->setIsSupportLimb(false);
      rightHindLeg_->setIsSupportLimb(false);
      rightForeLeg_->setIsSupportLimb(true);
      leftHindLeg_->setIsSupportLimb(true);

    }

    loco::Force virtualForceInBaseFrame(0, 0, 25.0*robot_utils::physical_definitions::getAbsoluteGravityAcceleration());
    loco::Torque virtualTorqueInBaseFrame;
    loco::Force distVirtualForceInBaseFrame;
    loco::Torque distVirtualTorqueInBaseFrame;

    ASSERT_TRUE(this->contactForceDistribution_.computeForceDistribution(virtualForceInBaseFrame, virtualTorqueInBaseFrame));
    ASSERT_TRUE(this->contactForceDistribution_.getNetForceAndTorqueOnBase(distVirtualForceInBaseFrame, distVirtualTorqueInBaseFrame));

    for (auto leg : *legs_) {
       std::cout << leg->getName() << "Leg: Contact force: " << this->contactForceDistribution_.getLegInfo(leg).desiredContactForce_ << std::endl;
       std::cout << leg->getName() << "Leg: Joint torque : " << leg->getDesiredJointTorques() << std::endl;
     }
  }




//  ASSERT_EQ(3, this->contactForceDistribution_.getNumberOfLegsInForceDistribution());
//
//  loco::Force expectedDistVirtualForceInBaseFrame(0.0, 0.0,  25.0*robot_utils::physical_definitions::getAbsoluteGravityAcceleration());
//
//  KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL(expectedDistVirtualForceInBaseFrame.toImplementation(), distVirtualForceInBaseFrame.toImplementation(), 1e-3, 1e-3, "distributed virtual force");
//  KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL(virtualTorqueInBaseFrame.toImplementation(), distVirtualTorqueInBaseFrame.toImplementation(), 1e-3, 1e-3, "distributed virtual torque");




//  loco::LegBase::JointTorques leftForeLegJointTorques;
//  loco::LegBase::JointTorques rightForeLegJointTorques;
//  loco::LegBase::JointTorques leftHindLegJointTorques;
//  loco::LegBase::JointTorques rightHindLegJointTorques;
//  KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL(leftForeLegJointTorques, leftForeLeg_->getDesiredJointTorques(), 1e-3, 1e-3, "joint torques of left fore leg");
//  KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL(rightForeLegJointTorques, rightForeLeg_->getDesiredJointTorques(), 1e-3, 1e-3, "joint torques of right fore leg");
//  KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL(leftHindLegJointTorques, leftHindLeg_->getDesiredJointTorques(), 1e-3, 1e-3, "joint torques of left hind leg");
//  KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL(rightHindLegJointTorques, rightHindLeg_->getDesiredJointTorques(), 1e-3, 1e-3, "joint torques of right hind leg");
//


}
