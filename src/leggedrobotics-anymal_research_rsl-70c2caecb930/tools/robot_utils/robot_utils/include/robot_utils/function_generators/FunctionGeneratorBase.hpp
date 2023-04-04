/*
 * FunctionGeneratorBase.hpp
 *
 *  Created on: Nov 12, 2014
 *      Author: Dario Bellicoso
 */

#pragma once

namespace robot_utils {

class FunctionGeneratorBase {
 public:
  FunctionGeneratorBase();
  virtual ~FunctionGeneratorBase();

  /*! Up and Down Sweep Function
   * @param time time [s]
   * @return  value
   */
  virtual double getUpAndDownSweepValue(double time) = 0;

  /*! Up Sweep Function
   * @param time time [s]
   * @return  value
   */
  virtual double getUpSweepValue(double time) = 0;

  virtual void setParamAmplitude(double paramAmplitude);
  virtual void setParamMinFrequencyHz(double minFreq);
  virtual void setParamMaxFrequencyHz(double maxFreq);
  virtual void setParamTimeInteval(double timeInterval);
  virtual void setCurrentFrequencyHz(double currentFreq);
  virtual void setComputedTimeInteval(double computedTimeInterval);

  virtual double getParamAmplitude() const;
  virtual double getParamMinFrequencyHz() const;
  virtual double getParamMaxFrequencyHz() const;
  virtual double getParamTimeInteval() const;
  virtual double getCurrentFrequencyHz() const;
  virtual double getComputedTimeInteval() const;

  virtual double& getCurrentFrequencyHz();

 protected:
  //! amplitude of sine
  double paramAmplitude_;

  //! minimum frequency [Hz]
  double paramMinFrequencyHz_;

  //! maximum frequency [Hz]
  double paramMaxFrequencyHz_;

  //! time interval of the sweep (estimated time interval for up and down sweep)
  double paramTimeInteval_;

  //! current frequency for logging
  double currentFrequencyHz_;

  //! exact time interval for up and down sweep
  double computedTimeInteval_;
};

}  // namespace robot_utils
