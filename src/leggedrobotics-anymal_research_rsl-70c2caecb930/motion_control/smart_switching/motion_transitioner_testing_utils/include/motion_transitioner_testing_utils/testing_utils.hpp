/**
 * @authors     Francisco Giraldez Gamez
 * @affiliation ANYbotics
 * @brief       Util classes used for unit testing Motion Transitioner
 */
#pragma once

#include <motion_transitioner/MotionExecutionInterface.hpp>
#include <motion_transitioner/MotionParametersInterface.hpp>
#include <motion_transitioner/MotionTransitionerConfigLoader.hpp>

namespace motion_transitioner_testing_utils {

class MotionExecutionInterfaceTest : public motion_transitioner::MotionExecutionInterface {
 public:
  MotionExecutionInterfaceTest() = default;

  ~MotionExecutionInterfaceTest() override = default;

  void preemptMotionExecution() override;

  motion_transitioner::MotionExecutionResult executeMotion(const motion_transitioner::Motion& motion) override;

 private:
  bool preempted_ = false;
};

class MotionTransitionerConfigLoaderTest : public motion_transitioner::MotionTransitionerConfigLoader {
 public:
  MotionTransitionerConfigLoaderTest(const motion_transitioner::MotionStates& testingMotionStates,
                                     const motion_transitioner::Transitions& testingTransitions);

  ~MotionTransitionerConfigLoaderTest() override = default;

  bool loadMotionStates(motion_transitioner::MotionStates& loadedMotionStates) override;

  bool loadTransitions(motion_transitioner::Transitions& loadedTransitions) override;

  std::string loadInitialMotionState() override;

 private:
  const motion_transitioner::MotionStates& testingMotionStates_;
  const motion_transitioner::Transitions& testingTransitions_;
};

class MotionParametersInterfaceTest : public motion_transitioner::MotionParametersInterface {
 public:
  MotionParametersInterfaceTest() = default;

  ~MotionParametersInterfaceTest() override = default;

  bool requestMotionParameterChange(const anymal_motion_control::Motion& motion, const std::string& yamlString) override;
  bool controllerParametersToYamlString(const anymal_motion_control::Motion& motion, std::string& yamlString) const override;
  bool waitForAllDynamicMotionParametersApplied(const anymal_motion_control::Motion& motion, const int timeout) const override;
};

class TimeInterfaceTest : public motion_transitioner::TimeInterface {
 public:
  TimeInterfaceTest() = default;

  ~TimeInterfaceTest() override = default;

 private:
  robot_control::TimePoint now() const override;
  void sleep(double duration) const override;
  double getTimeStep() const override;
};

}  // namespace motion_transitioner_testing_utils
