#include "anydrive/JointPositionConfigurationManager.hpp"
#include "anydrive/AnydriveManager.hpp"

namespace anydrive {

JointPositionConfigurationManager::JointPositionConfigurationManager(AnydriveManager& anydriveManager)
    : anydriveManager_(anydriveManager) {}

bool JointPositionConfigurationManager::loadSetup(const setup::JointPositionConfigurationManager& setup) {
  maxJointVelocity_ = setup.maxJointVelocity_;
  return true;
}

std::set<std::string> JointPositionConfigurationManager::getConfigurationNames() const {
  // Dynamically search for all joint position configuration names of all ANYdrives.
  std::set<std::string> configurationNames;
  for (const auto& anydrive : anydriveManager_.getAnydrives()) {
    for (const auto& jointPositionConfiguration : anydrive->getConfiguration().getJointPositionConfigurations()) {
      configurationNames.insert(jointPositionConfiguration.first);
    }
  }
  return configurationNames;
}

void JointPositionConfigurationManager::setMaxJointVelocity(const double maxJointVelocity) {
  maxJointVelocity_ = maxJointVelocity;
}

double JointPositionConfigurationManager::getMaxJointVelocity() const {
  return maxJointVelocity_;
}

bool JointPositionConfigurationManager::generateCommandsQueueForConfiguration(const std::string& configurationName) {
  // Do some initial checks.
  if (maxJointVelocity_ <= 0.0) {
    ANYDRIVE_ERROR("Max joint velocity is smaller or equal to zero.");
    return false;
  }

  const double timeStep = anydriveManager_.getTimeStep();
  if (timeStep <= 0.0) {
    ANYDRIVE_ERROR("Time step is smaller or equal to zero.");
    return false;
  }

  clearCommandsInQueue();
  const unsigned int numberOfAnydrives = anydriveManager_.getNumberOfAnydrives();

  // Get all goals and compute all errors.
  std::vector<double> jointPositionGoals;
  std::vector<double> jointPositionErrors;
  double maxAbsJointPositionError = 0.0;
  for (const auto& anydrive : anydriveManager_.getAnydrives()) {
    const double jointPosition = anydrive->getReading().getState().getJointPosition();
    double jointPositionGoal = jointPosition;
    // Get the joint position goal if it exists, otherwise just use the current joint position.
    const bool jointPositionGoalIsSet =
        anydrive->getConfiguration().getJointPositionConfigurationValue(configurationName, jointPositionGoal);
    const double jointPositionError = jointPositionGoal - jointPosition;
    jointPositionGoals.push_back(jointPositionGoal);
    jointPositionErrors.push_back(jointPositionError);
    if (jointPositionGoalIsSet) {
      const double absJointPositionError = std::abs(jointPositionError);
      if (absJointPositionError > maxAbsJointPositionError) {
        maxAbsJointPositionError = absJointPositionError;
      }
    }
  }

  // If the max error is still 0.0, none of the devices has this configuration.
  if (maxAbsJointPositionError == 0.0) {
    ANYDRIVE_ERROR("Joint position configuration '" << configurationName << "' does not exist.");
    return false;
  }

  // The goal of this algorithm is to reach all desired joint positions at the same instance,
  // so the individual joint velocities depend on the according joint position errors.
  // The biggest error and the maximal joint velocity define the duration of the maneuver.
  const double duration = maxAbsJointPositionError / maxJointVelocity_;
  if (duration <= 0.0) {
    ANYDRIVE_ERROR("Duration is smaller or equal zero.");
    return false;
  }
  const auto steps = static_cast<unsigned int>(duration / timeStep);  // This is rounded up.
  const any_measurements::Time now = any_measurements::Time::NowWallClock();
  Command command;
  command.setModeEnum(mode::ModeEnum::JointPositionVelocity);
  command.setStamp(now);
  std::vector<Command> commands;
  commands.resize(numberOfAnydrives);

  // Create commands for all time steps.
  for (unsigned int i = 0; i <= steps; i++) {
    // Calculate the progress in percentage.
    const double progress = static_cast<double>(i) / static_cast<double>(steps);

    // Check if the progress is valid.
    if (progress < 0.0 || 1.0 < progress) {
      ANYDRIVE_ERROR("Progress is invalid (" << progress << " < 0.0 || 1.0 < " << progress << "), skipping it.");
      continue;
    }

    // Compute commands for all drives at this time step.
    for (unsigned int j = 0; j < numberOfAnydrives; j++) {
      // Compute joint position and velocity.
      const double jointPosition = jointPositionGoals[j] - (1.0 - progress) * jointPositionErrors[j];
      double jointVelocity = jointPositionErrors[j] / duration;

      // Double-check joint velocity to be sure. If this check fails, the algorithm is faulty.
      if (jointVelocity < -maxJointVelocity_) {
        ANYDRIVE_DEBUG("Joint velocity is smaller than allowed (" << jointVelocity << " < " << -maxJointVelocity_ << "), saturating it.");
        jointVelocity = -maxJointVelocity_;
      } else if (jointVelocity > maxJointVelocity_) {
        ANYDRIVE_DEBUG("Joint velocity is bigger than allowed (" << jointVelocity << " > " << maxJointVelocity_ << "), saturating it.");
        jointVelocity = maxJointVelocity_;
      }

      // Set the command.
      command.setJointPosition(jointPosition);
      command.setJointVelocity(jointVelocity);
      commands[j] = command;
    }
    {
      std::lock_guard<std::recursive_mutex> lock(commandsDequeMutex_);
      commandsDeque_.push_back(commands);
    }
  }

  // Add freezes in the end.
  Command freeze;
  freeze.setModeEnum(mode::ModeEnum::Freeze);
  freeze.setStamp(now);
  for (unsigned int i = 0; i < numberOfAnydrives; i++) {
    commands[i] = freeze;
  }

  // Add the commands to the deque.
  {
    std::lock_guard<std::recursive_mutex> lock(commandsDequeMutex_);
    commandsDeque_.push_back(commands);
  }
  return true;
}

bool JointPositionConfigurationManager::getNextCommandsInQueue(std::vector<Command>& commands) {
  if (!isActive()) {
    return false;
  }

  {
    std::lock_guard<std::recursive_mutex> lock(commandsDequeMutex_);
    commands = commandsDeque_.front();
    commandsDeque_.pop_front();
  }
  return true;
}

bool JointPositionConfigurationManager::isActive() const {
  std::lock_guard<std::recursive_mutex> lock(commandsDequeMutex_);
  return !commandsDeque_.empty();
}

void JointPositionConfigurationManager::clearCommandsInQueue() {
  std::lock_guard<std::recursive_mutex> lock(commandsDequeMutex_);
  commandsDeque_.clear();
}

}  // namespace anydrive
