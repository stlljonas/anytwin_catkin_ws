#pragma once

#include <anydrive/AnydriveManager.hpp>
#include <anydrive/setup/Setup.hpp>

#include "anydrive/calibration/parameter/FrictionEstimation.hpp"
#include "anydrive/calibration/parameter/GearAndJointEncoderHoming.hpp"
#include "anydrive/calibration/parameter/GearJointEncoderOffset.hpp"
#include "anydrive/calibration/routine/FrictionEstimation.hpp"
#include "anydrive/calibration/routine/GearJointEncoderOffset.hpp"
#include "anydrive/calibration/routine/GravityCompensation.hpp"
#include "anydrive/calibration/routine/SafeJointVelocity.hpp"

#include "anydrive_ethercat/AnydriveEthercatSlave.hpp"

namespace anydrive_ethercat {

class AnydriveEthercatBusManager;

class AnydriveManagerEthercat : public anydrive::AnydriveManager {
 public:
  /*!
   * Constructor.
   * @param standalone           If true, this class will start its own worker.
   * @param installSignalHandler If true, this class will install its own signal handler.
   * @param timeStep             Time step for standalone mode.
   */
  AnydriveManagerEthercat(const bool standalone, const bool installSignalHandler, const double timeStep);

  /*!
   * Destructor.
   */
  ~AnydriveManagerEthercat() override = default;

  /*!
   * Create a new default setup.
   * @return New default setup.
   */
  anydrive::setup::SetupPtr createSetup() const override;

  bool calibrate(const std::string& deviceName, const anydrive::calibration::CalibrationModeEnum calibrationModeEnum,
                 const bool gearAndJointEncoderHomingAbsolute, const double gearAndJointEncoderHomingNewJointPosition) override;

 private:
  AnydriveEthercatSlavePtr getEthercatCommunicationInterface(anydrive::AnydrivePtr anydrive) {
    return std::dynamic_pointer_cast<AnydriveEthercatSlave>(anydrive->getCommunicationInterface());
  }
};

using AnydriveManagerEthercatPtr = std::shared_ptr<AnydriveManagerEthercat>;

}  // namespace anydrive_ethercat