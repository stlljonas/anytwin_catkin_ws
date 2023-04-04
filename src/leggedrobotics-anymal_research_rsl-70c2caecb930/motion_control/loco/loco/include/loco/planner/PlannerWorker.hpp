/*
 * PlannerBase.hpp
 *
 *  Created on: Aug 27, 2015
 *      Author: Christian Gehring
 */

#pragma once

// loco
#include "loco/planner/PlannerBase.hpp"

// roco
#include <roco/workers/Worker.hpp>

// stl
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>

namespace loco {

class PlannerWorker : public PlannerBase, public roco::Worker {
 public:
  explicit PlannerWorker(const std::string& workerName);
  ~PlannerWorker() override = default;

 protected:
  bool startThread() override;
  bool stopThread() override;

 private:
  bool work(const roco::WorkerEvent& event) override;
};

} /* namespace loco */
