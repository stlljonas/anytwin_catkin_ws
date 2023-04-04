/*
 * PlannerBoostThread.cpp
 *
 *  Created on: Feb 5, 2016
 *      Author: Christian Gehring
 */

// loco
#include "loco/planner/PlannerBoostThread.hpp"

namespace loco {

PlannerBoostThread::~PlannerBoostThread() {
  stopThread();
}

bool PlannerBoostThread::startThread() {
  thread_ = boost::thread{&PlannerBoostThread::runThread, this};
  sched_param sched{};
  sched.sched_priority = threadPriority_;
  pthread_setschedparam(thread_.native_handle(), SCHED_FIFO, &sched);

  return true;
}

bool PlannerBoostThread::stopThread() {
  stopPlanningThread_ = true;
  thread_.interrupt();
  if (thread_.joinable()) {
    thread_.join();
  }
  return true;
}

} /* namespace loco */
