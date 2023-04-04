/*
 * TrajectoryStateHandlerLineSearch.hpp
 *
 *  Created on: 05.08, 2017
 *      Author: Fabian Jenelten
 */

// motion generation
#include <motion_generation_utils/TrajectoryStateHandlerLineSearch.hpp>

// messagge logger
#include <message_logger/message_logger.hpp>

// robot utils
#include <robot_utils/math/math.hpp>

namespace motion_generation {

TrajectoryStateHandlerLineSearch::TrajectoryStateHandlerLineSearch():
  TrajectoryStateHandlerBase(),
  lineSearchOptions_()
{

}

void TrajectoryStateHandlerLineSearch::setLineSearchOptions(const zmp::LineSearchOptions& lineSearchOptions) {
  lineSearchOptions_ = lineSearchOptions;
}

void TrajectoryStateHandlerLineSearch::copyLineSearchOptions(const TrajectoryStateHandlerLineSearch& trajectoryStateHandler) {
  lineSearchOptions_ = trajectoryStateHandler.getLineSearchOptions();
}

zmp::LineSearchOptions TrajectoryStateHandlerLineSearch::getLineSearchOptions() const {
  return lineSearchOptions_;
}


double TrajectoryStateHandlerLineSearch::lineSearch(
    double containerTimeGuess,
    double minTime,
    double maxTime) const {

  // ToDo: weights as a function of optimization dofs.

  // Adjust boundaries.
  if (minTime < 0.0) { minTime = 0.0; }
  if (maxTime < 0.0 || maxTime>containerDuration_) { maxTime = containerDuration_; }

  // Clip initial guess to constraints.
  robot_utils::boundToRange(&containerTimeGuess, minTime, maxTime);

  if (lineSearchOptions_.maxIter_==0u) { return containerTimeGuess; }

  // Initialization.
  double t_star = containerTimeGuess;

  std::stringstream msg;
  if (lineSearchOptions_.verbose_) {
    msg << std::left << "========Line Search Info======\n";
    msg << std::setw(15) << "Newton"
        << std::setw(15) << "back tracing"
        << std::setw(20) << "objective_bt [1e9]"
        << std::setw(20) << "step length [-]"
        << std::setw(20) << "correction length [mu s]"
        << std::endl;
  }

  // Newton Iteration.
  double objective_bt; double gradient; double hessian;
  for (auto numOfIterations=1u; numOfIterations<=lineSearchOptions_.maxIter_; ++numOfIterations) {

    // Compute hessian and gradient at current Newton step.
    computeGradientAndHessian(gradient, hessian, t_star);

    if (robot_utils::areNear(hessian, 1e-10)) {
      if (lineSearchOptions_.verbose_ ) {
        msg << "Convergence criterion: " << message_logger::color::green << " Hessian is zero.\n";
        msg << message_logger::color::white << "Tot Newton iterations: "    << numOfIterations << "\n";
        msg << "Initial guess [ms]: "       << containerTimeGuess*1000.0  << "\n";
        msg << "Solution [ms]: "            << t_star*1000.0 << "\n";
        msg << "---------------------------------------------------\n";
        MELO_INFO_THROTTLE_STREAM(lineSearchOptions_.rate_, msg.str());
      }
      return t_star;
    }

    // Compute objective at current Newton step.
    const double objective_newton = computeObjective(t_star);

    if (lineSearchOptions_.verbose_  && numOfIterations == 1u) {
      msg << std::setw(15) << "init"
          << std::setw(15) << ""
          << std::setw(20) << std::to_string(objective_newton*1.0e9)
          << std::setw(20) << ""
          << std::setw(20) << ""
          << std::endl;
    }

    // Compute seach direction.
    const double delta_t = -gradient/hessian;

    /***************************
     * Back Tracing Iterations *
     ***************************/
    double h = 1.0;
    double t_bt;
    const double backTrackingLength = lineSearchOptions_.alpha_*gradient*delta_t;
    for (auto numOfBackTraces=1u; numOfBackTraces<=lineSearchOptions_.maxIterBackTraces_; ++numOfBackTraces) {

      // Update time.
      t_bt = t_star+h*delta_t;

      // If we reached boundaries, decrease step length and try again.
      if (t_bt>=maxTime || t_bt<=minTime) {
        h *= lineSearchOptions_.beta_;

        // We found the optimum (we cannot take smaller steps).
        if (numOfBackTraces>=lineSearchOptions_.maxIterBackTraces_) {
          if (lineSearchOptions_.verbose_ ) {
            msg << "Convergence criterion: " << message_logger::color::yellow << " Back tracing reached boundary constraints.\n";
            msg << message_logger::color::white << "Tot Newton iterations: "    << numOfIterations << "\n";
            msg << "Boundaries [ms]: ["         << minTime*1000.0  << ", " << maxTime*1000.0 << "]\n";
            msg << "Search direction [ms]: "    << h*delta_t*1000.0 << "\n";
            msg << "Initial guess [ms]: "       << containerTimeGuess*1000.0  << "\n";
            msg << "Solution [ms]: "            << t_star*1000.0 << "\n";
            msg << "---------------------------------------------------\n";
            MELO_INFO_THROTTLE_STREAM(lineSearchOptions_.rate_, msg.str());
          }
          return t_star;
        }

        continue;
      }

      // Compute objective at backtracing step.
      objective_bt = computeObjective(t_bt);

      // Check if we have found optimal step length.
      if (objective_bt <= objective_newton + h*backTrackingLength) {
        if (lineSearchOptions_.verbose_) {
          msg << std::setw(15) << std::to_string(numOfIterations)
              << std::setw(15) << std::to_string(numOfBackTraces)
              << std::setw(20) << std::to_string(objective_bt*1e9)
              << std::setw(20) << std::to_string(h)
              << std::setw(20) << std::to_string(delta_t*1e6)
              << std::endl;
        }
        break;
      }

      // Check for divergence.
      else if (numOfBackTraces>=lineSearchOptions_.maxIterBackTraces_) {

        // Update time if improvement.
        if (objective_bt < objective_newton) { t_star = t_bt; }

        if (lineSearchOptions_.verbose_ ) {
          msg << "Convergence criterion: " << message_logger::color::green << " Back tracing reached max number of iterations.\n";
          msg << message_logger::color::white << "Tot Newton iterations: "    << numOfIterations << "\n";
          msg << "Initial guess [ms]: "       << containerTimeGuess*1000.0  << "\n";
          msg << "Solution [ms]: "            << t_star*1000.0 << "\n";
          msg << "---------------------------------------------------\n";
          MELO_INFO_THROTTLE_STREAM(lineSearchOptions_.rate_, msg.str());
        }
        return t_star;
      }

      // reduce step length and try again.
      h *= lineSearchOptions_.beta_;
    }
    /***************************/

    // Update time.
    t_star = t_bt;

    // Check if we are done (i.e., if the correction step becomes too small).
    if (std::fabs(h*delta_t)<=lineSearchOptions_.tol_) {
      if (lineSearchOptions_.verbose_) {
        msg << "Convergence criterion: "    << message_logger::color::green << "Backtracing step length is smaller than tolerance\n";
        msg << message_logger::color::white << "Tot Newton iterations: "    << numOfIterations << "\n";
        msg << "Initial guess [ms]: "       << containerTimeGuess*1000.0 << "\n";
        msg << "Solution [ms]: "            << t_star*1000.0 << "\n";
        msg << "---------------------------------------------------\n";
        MELO_INFO_THROTTLE_STREAM(lineSearchOptions_.rate_, msg.str());
      }
      return t_star;
    }
  }

  /*******************************************
   * Max number of Newton iterations reached *
   *******************************************/
  robot_utils::boundToRange(&t_star, minTime, maxTime);

  if (lineSearchOptions_.verbose_) {
    msg << "Convergence criterion: "    << message_logger::color::yellow << "Newton method reached max number of iteration.\n";
    msg << message_logger::color::white <<"Tot Newton iterations: "    << lineSearchOptions_.maxIter_ << "\n";
    msg << "Initial guess [ms]: "       << containerTimeGuess*1000.0 << "\n";
    msg << "Solution [ms]: "            << t_star*1000.0 << "\n";
    msg << "---------------------------------------------------\n";
    MELO_INFO_THROTTLE_STREAM(lineSearchOptions_.rate_, msg.str());
  }
  /*******************************************/

  return t_star;

}


} /* namespace motion_generation */
