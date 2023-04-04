#include "anydrive/fsm/StateConfigure.hpp"
#include "anydrive/Anydrive.hpp"

namespace anydrive {
namespace fsm {

StateConfigure::StateConfigure(Anydrive& anydrive, std::atomic<StateEnum>& goalStateEnum)
    : StateBase(anydrive, goalStateEnum, StateEnum::Configure,
                {{StateEnum::Calibrate, ANYDRIVE_CW_ID_CONFIGURE_TO_CALIBRATE},
                 {StateEnum::Standby, ANYDRIVE_CW_ID_CONFIGURE_TO_STANDBY},
                 {StateEnum::MotorOp, ANYDRIVE_CW_ID_CONFIGURE_TO_STANDBY},
                 {StateEnum::ControlOp, ANYDRIVE_CW_ID_CONFIGURE_TO_STANDBY}}) {}

void StateConfigure::enterDerived() {
  isDone_ = false;
}

void StateConfigure::updateDerived() {
  if (isDone_) {
    return;
  }

  if (step_ == 0) {
    if (anydrive_.getConfiguration().getErrorStateBehavior().isSet()) {
      // Check if already tried to many times to set the error state behavior.
      if (stepErrorStateBehavior_ > stepErrorStateBehaviorThreshold_) {
        MELO_FATAL_STREAM("Could not set the 'error state behavior' after "
                          << stepErrorStateBehaviorThreshold_ << " attempts! This is a critical safety issue -> exiting the software.");
        return;
      }
      if (anydrive_.setErrorStateBehavior(anydrive_.getConfiguration().getErrorStateBehavior())) {
        // Setting the error state behavior was successful, proceed with the next configuration.
        step_++;
      } else {
        // Setting the error state behavior was not successful, try again in the next iteration.
        stepErrorStateBehavior_++;
      }
    } else {
      step_++;
    }
    return;
  }

  if (step_ == 1) {
    if (anydrive_.getConfiguration().getMaxCurrent().isSet()) {
      anydrive_.setMaxCurrent(anydrive_.getConfiguration().getMaxCurrent());
    }
    step_++;
    return;
  }

  if (step_ == 2) {
    if (anydrive_.getConfiguration().getMaxMotorVelocity().isSet()) {
      anydrive_.setMaxMotorVelocity(anydrive_.getConfiguration().getMaxMotorVelocity());
    }
    step_++;
    return;
  }

  if (step_ == 3) {
    if (anydrive_.getConfiguration().getDirection().isSet()) {
      anydrive_.setDirection(anydrive_.getConfiguration().getDirection());
    }
    step_++;
    return;
  }

  if (step_ == 4) {
    if (anydrive_.getConfiguration().getJointPositionLimitsSoft().isSet()) {
      anydrive_.setJointPositionLimitsSoft(anydrive_.getConfiguration().getJointPositionLimitsSoft());
    }
    step_++;
    return;
  }

  if (step_ == 5) {
    if (anydrive_.getConfiguration().getJointPositionLimitsHard().isSet()) {
      anydrive_.setJointPositionLimitsHard(anydrive_.getConfiguration().getJointPositionLimitsHard());
    }
    step_++;
    return;
  }

  if (step_ == 6) {
    if (anydrive_.getConfiguration().getImuEnable().isSet()) {
      anydrive_.setImuEnable(anydrive_.getConfiguration().getImuEnable());
    }
    step_++;
    return;
  }

  if (step_ == 7) {
    if (anydrive_.getConfiguration().getImuAccelerometerRange().isSet()) {
      anydrive_.setImuAccelerometerRange(anydrive_.getConfiguration().getImuAccelerometerRange());
    }
    step_++;
    return;
  }

  if (step_ == 8) {
    if (anydrive_.getConfiguration().getImuGyroscopeRange().isSet()) {
      anydrive_.setImuGyroscopeRange(anydrive_.getConfiguration().getImuGyroscopeRange());
    }
    step_++;
    return;
  }

  if (step_ == 9) {
    if (anydrive_.getConfiguration().getMaxJointTorque().isSet()) {
      anydrive_.setMaxJointTorque(anydrive_.getConfiguration().getMaxJointTorque());
    }
    step_++;
    return;
  }

  if (step_ == 10) {
    if (anydrive_.getConfiguration().getCurrentIntegratorSaturation().isSet()) {
      anydrive_.setCurrentIntegratorSaturation(anydrive_.getConfiguration().getCurrentIntegratorSaturation());
    }
    step_++;
    return;
  }

  if (step_ == 11) {
    if (anydrive_.getConfiguration().getJointTorqueIntegratorSaturation().isSet()) {
      anydrive_.setJointTorqueIntegratorSaturation(anydrive_.getConfiguration().getJointTorqueIntegratorSaturation());
    }
    step_++;
    return;
  }

  if (step_ == 12) {
    if (anydrive_.getConfiguration().getFanMode().isSet()) {
      anydrive_.setFanMode(anydrive_.getConfiguration().getFanMode());
    }
    step_++;
    return;
  }

  if (step_ == 13) {
    if (anydrive_.getConfiguration().getFanIntensity().isSet()) {
      anydrive_.setFanIntensity(anydrive_.getConfiguration().getFanIntensity());
    }
    step_++;
    return;
  }

  if (step_ == 14) {
    if (anydrive_.getConfiguration().getFanLowerTemperature().isSet()) {
      anydrive_.setFanLowerTemperature(anydrive_.getConfiguration().getFanLowerTemperature());
    }
    step_++;
    return;
  }

  if (step_ == 15) {
    if (anydrive_.getConfiguration().getFanUpperTemperature().isSet()) {
      anydrive_.setFanUpperTemperature(anydrive_.getConfiguration().getFanUpperTemperature());
    }
    step_++;
    return;
  }

  if (step_ == 16) {
    if (anydrive_.getConfiguration().getMaxFreezeCurrent().isSet()) {
      anydrive_.setMaxFreezeCurrent(anydrive_.getConfiguration().getMaxFreezeCurrent());
    }
    step_++;
    return;
  }

  if (step_ == 17) {
    const auto& modes = anydrive_.getConfiguration().getModes();
    if (stepMode_ < modes.size()) {
      auto modeIt = modes.begin();
      std::advance(modeIt, stepMode_);
      if (modeIt->second->getPidGains().isSet()) {
        anydrive_.setControlGains(modeIt->first, modeIt->second->getPidGains());
      }
      stepMode_++;
      return;
    } else {
      step_++;
    }
  }

  if (step_ == 18) {
    if (anydrive_.getConfiguration().getGearJointVelocityFilterType().isSet()) {
      anydrive_.setGearJointVelocityFilterType(anydrive_.getConfiguration().getGearJointVelocityFilterType());
    }
    step_++;
    return;
  }

  if (step_ == 19) {
    if (anydrive_.getConfiguration().getGearJointVelocityKfNoiseVariance().isSet()) {
      anydrive_.setGearJointVelocityKfNoiseVariance(anydrive_.getConfiguration().getGearJointVelocityKfNoiseVariance());
    }
    step_++;
    return;
  }

  if (step_ == 20) {
    if (anydrive_.getConfiguration().getGearJointVelocityKfLambda2().isSet()) {
      anydrive_.setGearJointVelocityKfLambda2(anydrive_.getConfiguration().getGearJointVelocityKfLambda2());
    }
    step_++;
    return;
  }

  if (step_ == 21) {
    if (anydrive_.getConfiguration().getGearJointVelocityKfGamma().isSet()) {
      anydrive_.setGearJointVelocityKfGamma(anydrive_.getConfiguration().getGearJointVelocityKfGamma());
    }
    step_++;
    return;
  }

  if (step_ == 22) {
    if (anydrive_.getConfiguration().getGearJointVelocityEmaAlpha().isSet()) {
      anydrive_.setGearJointVelocityEmaAlpha(anydrive_.getConfiguration().getGearJointVelocityEmaAlpha());
    }
    step_++;
    return;
  }

  if (step_ == 23) {
    if (anydrive_.getConfiguration().getJointVelocityForAccelerationFilterType().isSet()) {
      anydrive_.setJointVelocityForAccelerationFilterType(anydrive_.getConfiguration().getJointVelocityForAccelerationFilterType());
    }
    step_++;
    return;
  }

  if (step_ == 24) {
    if (anydrive_.getConfiguration().getJointVelocityForAccelerationKfNoiseVariance().isSet()) {
      anydrive_.setJointVelocityForAccelerationKfNoiseVariance(
          anydrive_.getConfiguration().getJointVelocityForAccelerationKfNoiseVariance());
    }
    step_++;
    return;
  }

  if (step_ == 25) {
    if (anydrive_.getConfiguration().getJointVelocityForAccelerationKfLambda2().isSet()) {
      anydrive_.setJointVelocityForAccelerationKfLambda2(anydrive_.getConfiguration().getJointVelocityForAccelerationKfLambda2());
    }
    step_++;
    return;
  }

  if (step_ == 26) {
    if (anydrive_.getConfiguration().getJointVelocityForAccelerationKfGamma().isSet()) {
      anydrive_.setJointVelocityForAccelerationKfGamma(anydrive_.getConfiguration().getJointVelocityForAccelerationKfGamma());
    }
    step_++;
    return;
  }

  if (step_ == 27) {
    if (anydrive_.getConfiguration().getJointVelocityForAccelerationEmaAlpha().isSet()) {
      anydrive_.setJointVelocityForAccelerationEmaAlpha(anydrive_.getConfiguration().getJointVelocityForAccelerationEmaAlpha());
    }
    step_++;
    return;
  }

  if (step_ == 28) {
    if (anydrive_.getConfiguration().getJointAccelerationFilterType().isSet()) {
      anydrive_.setJointAccelerationFilterType(anydrive_.getConfiguration().getJointAccelerationFilterType());
    }
    step_++;
    return;
  }

  if (step_ == 29) {
    if (anydrive_.getConfiguration().getJointAccelerationKfNoiseVariance().isSet()) {
      anydrive_.setJointAccelerationKfNoiseVariance(anydrive_.getConfiguration().getJointAccelerationKfNoiseVariance());
    }
    step_++;
    return;
  }

  if (step_ == 30) {
    if (anydrive_.getConfiguration().getJointAccelerationKfLambda2().isSet()) {
      anydrive_.setJointAccelerationKfLambda2(anydrive_.getConfiguration().getJointAccelerationKfLambda2());
    }
    step_++;
    return;
  }

  if (step_ == 31) {
    if (anydrive_.getConfiguration().getJointAccelerationKfGamma().isSet()) {
      anydrive_.setJointAccelerationKfGamma(anydrive_.getConfiguration().getJointAccelerationKfGamma());
    }
    step_++;
    return;
  }

  if (step_ == 32) {
    if (anydrive_.getConfiguration().getJointAccelerationEmaAlpha().isSet()) {
      anydrive_.setJointAccelerationEmaAlpha(anydrive_.getConfiguration().getJointAccelerationEmaAlpha());
    }
    step_++;
    return;
  }

  isDone_ = true;
}

}  // namespace fsm
}  // namespace anydrive
