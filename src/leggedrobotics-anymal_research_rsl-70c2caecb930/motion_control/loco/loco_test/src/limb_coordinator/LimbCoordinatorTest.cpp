/*!
* @file     LimbCoordinatorTest.cpp
* @author   Christian Gehring
* @date     Jun, 2013
* @version  1.0
* @ingroup
* @brief
*/


#include <gtest/gtest.h>


#include "loco/limb_coordinator/LimbCoordinatorDeprecated.hpp"
#include "loco/gait_pattern/GaitPatternAPS.hpp"

#include "loco/common/TorsoStarlETH.hpp"

TEST(LimbCoordinatorTest, test) {
  loco::APS aps(0.8, 0.8, 0.5, 0.5, 0.5, 0.5, 0.5);
  loco::GaitPatternAPS gaitPatternAPS;
  double dt = 0.0025;

  robotModel::RobotModel robotModel;
  robotModel.init();
  robotModel.update();

  loco::LegGroup legs;
  loco::TorsoAnymal torso(&robotModel);

  gaitPatternAPS.initialize(aps, dt);
  loco::LimbCoordinatorDeprecated limbCoordinator(&legs, &torso, &gaitPatternAPS);
}
