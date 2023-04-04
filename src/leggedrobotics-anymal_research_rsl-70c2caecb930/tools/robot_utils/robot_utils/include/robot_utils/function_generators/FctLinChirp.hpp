/*!
 * @file 	FctLinChirp.hpp
 * @author 	Christian Gehring
 * @date 	Jun 7, 2012
 * @version 	1.0
 * @ingroup 	robot_utils
 * @brief
 */

#pragma once

#include "robot_utils/function_generators/FunctionGeneratorBase.hpp"

namespace robot_utils {

//! Linear chirp function
/*!
 * @ingroup robot_utils
 */
class FctLinChirp : public FunctionGeneratorBase {
 public:
  FctLinChirp();
  ~FctLinChirp() override;

  /*! Up and Down Sweep Function
   * @param time time [s]
   * @return	value
   */
  double getUpAndDownSweepValue(double time) override;

  /*! Up Sweep Function
   * @param time time [s]
   * @return	value
   */
  double getUpSweepValue(double time) override;
};

}  // namespace robot_utils
