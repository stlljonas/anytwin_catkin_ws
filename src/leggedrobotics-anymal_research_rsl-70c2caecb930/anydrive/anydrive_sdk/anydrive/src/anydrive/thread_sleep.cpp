#include <chrono>
#include <thread>

#include "anydrive/thread_sleep.hpp"

namespace anydrive {

void threadSleep(const double duration) {
  std::this_thread::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(1e9 * duration)));
}

}  // namespace anydrive
