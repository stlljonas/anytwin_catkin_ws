/*
 * rbdl_utils.h
 *
 *  Created on: Oct 1, 2015
 *      Author: Dario Bellicoso
 */

#pragma once

// anymal model
#include "anymal_model/AnymalState.hpp"

// std_utils
#include "std_utils/std_utils.hpp"

// eigen
#include <Eigen/Core>

namespace anymal_model {

//! Enum RBDL generalized coordinates vector
CONSECUTIVE_ENUM(GeneralizedCoordinatesRbdlEnum, X, Y, Z, Q_X, Q_Y, Q_Z, LF_HAA, LF_HFE, LF_KFE, RF_HAA, RF_HFE, RF_KFE, LH_HAA, LH_HFE,
                 LH_KFE, RH_HAA, RH_HFE, RH_KFE, Q_W)

void setRbdlQFromState(Eigen::VectorXd& rbdlQ, const AnymalState& state);
void setRbdlQDotFromState(Eigen::VectorXd& rbdlQDot, const AnymalState& state);
void setStateFromRbdlQ(AnymalState& state, const Eigen::VectorXd& rbdlQ);

}  // namespace anymal_model
