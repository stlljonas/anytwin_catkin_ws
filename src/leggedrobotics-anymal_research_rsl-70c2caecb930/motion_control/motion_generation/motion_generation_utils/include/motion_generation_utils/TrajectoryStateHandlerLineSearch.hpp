/*
 * TrajectoryStateHandlerLineSearch.hpp
 *
 *  Created on: 05.08, 2017
 *      Author: Fabian Jenelten
 */

#pragma once

// motion generation utils
#include <motion_generation_utils/TrajectoryStateHandlerBase.hpp>

namespace motion_generation {

class TrajectoryStateHandlerLineSearch: virtual public TrajectoryStateHandlerBase {
public:

  TrajectoryStateHandlerLineSearch();
  ~TrajectoryStateHandlerLineSearch() override = default;

  void copyLineSearchOptions(const TrajectoryStateHandlerLineSearch& trajectoryStateHandler);
  zmp::LineSearchOptions getLineSearchOptions() const;
  void setLineSearchOptions(const zmp::LineSearchOptions& lineSearchOptions);

  virtual void computeGradientAndHessian(double& gradient, double& hessian, double dt) const = 0;
  virtual double computeObjective(double dt) const = 0;

protected:
  double lineSearch(
      double containerTimeGuess,
      double minTime = -1.0,
      double maxTime = -1.0) const;

  // Infomration for line search.
  zmp::LineSearchOptions lineSearchOptions_;
};

} /* namespace motion_generation */
