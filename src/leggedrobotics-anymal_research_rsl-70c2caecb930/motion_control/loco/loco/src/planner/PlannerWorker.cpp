/*
 * PlannerWorker.cpp
 *
 *  Created on: Aug 27, 2015
 *      Author: Christian Gehring
 */

// loco
#include "loco/planner/PlannerWorker.hpp"

namespace loco {

PlannerWorker::PlannerWorker(const std::string& workerName) : PlannerBase(false), Worker(workerName) {
  options_.synchronous_ = false;
  options_.frequency_ = 0.0;
  options_.autostart_ = false;
  options_.priority_ = threadPriority_;
}

bool PlannerWorker::stopThread() {
  cancel(true);
  return true;
}

bool PlannerWorker::startThread() {
  return start();
}

bool PlannerWorker::work(const roco::WorkerEvent& event) {
  return runThread();
}

} /* namespace loco */
