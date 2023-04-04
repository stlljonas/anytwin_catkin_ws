#pragma once

#include <deque>
#include <mutex>
#include <vector>

#include "anydrive/Command.hpp"
#include "anydrive/setup/JointPositionConfigurationManager.hpp"

namespace anydrive {

class AnydriveManager;

class JointPositionConfigurationManager {
 protected:
  AnydriveManager& anydriveManager_;
  double maxJointVelocity_ = 0;

  mutable std::recursive_mutex commandsDequeMutex_;
  std::deque<std::vector<Command>> commandsDeque_;

 public:
  explicit JointPositionConfigurationManager(AnydriveManager& anydriveManager);
  virtual ~JointPositionConfigurationManager() = default;

  bool loadSetup(const setup::JointPositionConfigurationManager& setup);

  std::set<std::string> getConfigurationNames() const;

  void setMaxJointVelocity(const double maxJointVelocity);
  double getMaxJointVelocity() const;

  bool generateCommandsQueueForConfiguration(const std::string& configurationName);
  bool getNextCommandsInQueue(std::vector<Command>& commands);

  bool isActive() const;
  void clearCommandsInQueue();
};

}  // namespace anydrive
