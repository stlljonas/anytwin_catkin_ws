/*!
 * @file	memory_tools.hpp
 * @author	Philipp Leemann
 * @date	Sep, 2017
 */

#pragma once

#include <string>
#include <vector>

namespace cosmo {

struct TopicInfo {
  std::string topicName_{};
  unsigned int numSubscribers_ = 0;
  unsigned int numPublishers_ = 0;
  unsigned long int messageCounter_ = 0;
};

struct MemoryInfo {
  std::string poolName_{};
  unsigned int size_ = 0;
  unsigned int freeMemory_ = 0;
  bool allDeallocated_ = false;
  bool sane_ = true;
  unsigned int numObjects_ = 0;
  unsigned int numUniques_ = 0;
  unsigned int numCommunicators_ = 0;
  unsigned int numTopics_ = 0;
  std::vector<TopicInfo> topicInfos_{};
};

/**
 * @brief      Removes the memory pool
 *
 * @param[in]  poolName  The name of the memory pool
 *
 * @return     Returns true of the memory pool has been removed
 */
bool removeMemoryPool(const std::string& poolName);

/**
 * @brief      Retrieve information about the shared memory management
 *
 * @param[in]  poolName  The name of the memory pool
 * @param[out] info      A string with information
 *
 * @return     Returns true if the memory pool has been opened
 */
bool getMemoryInfo(const std::string& poolName, MemoryInfo* info);

std::string memoryInfoToString(const MemoryInfo& info);

}  // namespace cosmo
