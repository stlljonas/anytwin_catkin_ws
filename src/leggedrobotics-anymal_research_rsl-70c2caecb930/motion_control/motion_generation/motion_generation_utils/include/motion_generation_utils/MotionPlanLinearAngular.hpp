/*
 * MotionPlanLinearAngular.hpp
 *
 *  Created on: 07.08, 2018
 *      Author: Fabian Jenelten
 *
 *                        MotionPlanBase
 *                      /                 \
 *                     /                   \
 *            MotionPlanLinear      MotionPlanAngular
 *                   |                      |
 *                   |                      |
 *                    MotionPlanLinearAngular
 */

#pragma once

// motion generation utils
#include "motion_generation_utils/MotionPlanLinear.hpp"
#include "motion_generation_utils/MotionPlanAngular.hpp"

namespace motion_generation {

class MotionPlanLinearAngular :
    virtual public MotionPlanLinear,
    virtual public MotionPlanAngular {
public:
  MotionPlanLinearAngular();
  ~MotionPlanLinearAngular() override = default;

};

} /* namespace motion_generation */
