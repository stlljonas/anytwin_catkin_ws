/*!
 * @file 	FctLogChirp.hpp
 * @author 	Christian Gehring
 * @date 	Jun 7, 2012
 * @version 	1.0
 * @ingroup 	robot_utils
 * @brief
 */

#pragma once

#include "robot_utils/function_generators/FunctionGeneratorBase.hpp"

namespace robot_utils {

//! Log chirp function
/*!
 * Set all variables param*_ before using a function.
 *
 * Reference of log chirp function:
 * A. Farina, "Simultaneous Measurement of Impulse Response and Distortion with a Swept-Sine Technique", AES, Paris, 2000.
 *
 * System Identification - rules
 * ------------------------------
 * Reference:
 * M. B. Tischler, R. K. Remple, "Aircraft and Rotorcraft System Identification - Engineering Methods with Flight Test Examples",2006.
 *
 * Parameters of your system you need to know:
 * bandwidthFrequencyHz_: the bandwidth of your system
 * outOfPhaseFrequencyHz_: the frequency at which the phase of the response is exactly out of phase (-180°)
 *
 * Calculation of the required frequencies and time interval:
 * paramMinFrequencyHz_ = 0.5* bandwidthFrequencyHz_;
 * paramMaxFrequencyHz_ = 2.5* outOfPhaseFrequencyHz_;
 * maxTimeInteval_ = 1.0/paramMinFrequencyHz_;
 * paramTimeInteval_ = 2*maxTimeInteval_;
 *
 * The experiment should start with two complete long-period inputs at the beginning of the sweep
 * corresponding to paramMinFrequencyHz_
 *
 * The total sweep record length should be about
 * recTimeInterval = (4 to 5)maxTimeInteval_
 *
 *
 * @ingroup robot_utils
 */
class FctLogChirp : public FunctionGeneratorBase {
 public:
  FctLogChirp();
  ~FctLogChirp() override = default;

  /*! Up Sweep Function
   * @param time time [s]
   * @return	value
   */
  double getUpSweepValue(double time) override;
  double getUpSweepFirstDerivativeValue(double time);

  /*! Up and Down Sweep Function
   * @param time time [s]
   * @return	value
   */
  double getUpAndDownSweepValue(double time) override;

  virtual double getParamOffset();
  void setParamOffset(double offset);

 protected:
  double paramOffset_ = 0.0;
};

}  // namespace robot_utils
