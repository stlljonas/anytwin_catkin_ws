/*
 * PlannerBoostThread.hpp
 *
 *  Created on: Feb 5, 2016
 *      Author: Christian Gehring
 */

#pragma once

#include "loco/planner/PlannerBase.hpp"

#include <boost/thread.hpp>

namespace loco {

class PlannerBoostThread : public PlannerBase {
 public:
  PlannerBoostThread() = default;
  ~PlannerBoostThread() override;

 protected:
  bool startThread() override;
  bool stopThread() override;

 protected:
  boost::thread thread_;
};

} /* namespace loco */
