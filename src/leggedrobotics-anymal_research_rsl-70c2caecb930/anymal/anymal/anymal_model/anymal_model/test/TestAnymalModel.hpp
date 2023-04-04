/*!
 * @file    TestAnymalModel.hpp
 * @author  Christian Gehring, Dario Bellicoso
 * @date    Sep, 2015
 */

#pragma once

// anymal model
#include "anymal_model/AnymalModel.hpp"
#include "anymal_model/rbdl_utils.hpp"

// romo test
#include <romo_test/TestRobotModel.hpp>

// eigen
#include <Eigen/Core>

// boost
#include <boost/filesystem.hpp>

class TestAnymalModel : virtual public romo_test::TestRobotModel<anymal_model::AnymalModel> {
 private:
  using BaseTest = romo_test::TestRobotModel<anymal_model::AnymalModel>;

 public:
  TestAnymalModel() : BaseTest() {
    model_.reset(new anymal_model::AnymalModel());
    state_.reset(new anymal_model::AnymalState());
  }

  virtual ~TestAnymalModel() {}

  void init() { initModel("starleth_unit_test"); }

  void initModel(const std::string& urdfName) {
    // Reset the state.
    state_->setZero();

    //-- Load model for testing
    boost::filesystem::path filePath(__FILE__);
    std::string path = filePath.parent_path().parent_path().generic_string() + std::string{"/resources/" + urdfName + ".urdf"};
    ASSERT_TRUE(getModelPtr()->initializeFromUrdf(path));

    // Set the state to the model.
    model_->setState(*state_, true, false, false);
  }

  void getRandomGeneralizedPositionsRbdl(Eigen::VectorXd& q) override {
    anymal_model::AnymalState state;
    state.setRandom();
    anymal_model::setRbdlQFromState(q, state);
  }

  void setRbdlQFromState(Eigen::VectorXd& rbdlQ, const RobotStateType& state) override { anymal_model::setRbdlQFromState(rbdlQ, state); }

  void setStateFromRbdlQ(RobotStateType& state, const Eigen::VectorXd& rbdlQ) override { anymal_model::setStateFromRbdlQ(state, rbdlQ); }
};
