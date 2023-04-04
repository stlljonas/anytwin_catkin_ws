/**
 * @authors     Francisco Giraldez Gamez
 * @affiliation ANYbotics
 * @brief       Util classes used for unit testing Motion Transitioner
 */

#include <thread>

#include "motion_transitioner_testing_utils/testing_utils.hpp"

namespace motion_transitioner_testing_utils {

motion_transitioner::MotionExecutionResult MotionExecutionInterfaceTest::executeMotion(const motion_transitioner::Motion& motion) {
  // Sleep required to not saturate ROS communication
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  if (motion.operationMode_ == "new_mode") {
    preempted_ = false;
    return motion_transitioner::MotionExecutionResult::EXECUTED;
  } else if (motion.operationMode_ == "preemptable_mode") {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    if (preempted_) {
      return motion_transitioner::MotionExecutionResult::PREEMPTED;
    } else {
      return motion_transitioner::MotionExecutionResult::EXECUTED;
    }
  } else if (motion.operationMode_ == "impossible_mode") {
    return motion_transitioner::MotionExecutionResult::ERROR;
  } else {
    return motion_transitioner::MotionExecutionResult::NOT_FOUND;
  }
}

void MotionExecutionInterfaceTest::preemptMotionExecution() {
  preempted_ = true;
}

MotionTransitionerConfigLoaderTest::MotionTransitionerConfigLoaderTest(const motion_transitioner::MotionStates& testingMotionStates,
                                                                       const motion_transitioner::Transitions& testingTransitions)
    : testingMotionStates_(testingMotionStates), testingTransitions_(testingTransitions) {}

bool MotionTransitionerConfigLoaderTest::loadMotionStates(motion_transitioner::MotionStates& loadedMotionStates) {
  loadedMotionStates = testingMotionStates_;
  return true;
}

bool MotionTransitionerConfigLoaderTest::loadTransitions(motion_transitioner::Transitions& loadedTransitions) {
  loadedTransitions = testingTransitions_;
  return true;
}

std::string MotionTransitionerConfigLoaderTest::loadInitialMotionState() {
  return "motion_state_0";
}

bool MotionParametersInterfaceTest::requestMotionParameterChange(const anymal_motion_control::Motion& /*motion*/,
                                                                 const std::string& yamlString) {
  return yamlString != "impossible_parameters";
}

bool MotionParametersInterfaceTest::controllerParametersToYamlString(const anymal_motion_control::Motion& motion,
                                                                     std::string& yamlString) const {
  if (motion.operationMode_ == "new_mode") {
    yamlString = "good_parameters";
  } else if (motion.operationMode_ == "impossible_mode") {
    yamlString = "impossible_parameters";
  } else {
    yamlString = "";
  }
  return false;
}

bool MotionParametersInterfaceTest::waitForAllDynamicMotionParametersApplied(const anymal_motion_control::Motion& /*motion*/,
                                                                             const int /*timeout*/) const {
  return true;
}

robot_control::TimePoint TimeInterfaceTest::now() const {
  const auto& wallTimeInNanoseconds =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch());
  const auto& secondsPortion = std::chrono::duration_cast<std::chrono::seconds>(wallTimeInNanoseconds).count();
  const auto nanosecondsPortion = (wallTimeInNanoseconds % secondsPortion).count();
  auto timePoint = robot_control::TimePoint();
  timePoint.seconds_ = secondsPortion;
  timePoint.nanoseconds_ = nanosecondsPortion;
  return timePoint;
}

void TimeInterfaceTest::sleep(double duration) const {
  std::this_thread::sleep_for(std::chrono::duration<double>(duration));
}

double TimeInterfaceTest::getTimeStep() const {
  return 0;
}
}  // namespace motion_transitioner_testing_utils