#include <anydrive/Exception.hpp>
#include <anydrive_ethercat/AnydriveManagerEthercat.hpp>
#include "anydrive/calibration/CalibrationState.hpp"
#include "anydrive_ethercat/AnydriveEthercatBusManager.hpp"
#include "anydrive_ethercat/setup/SetupEthercat.hpp"

namespace anydrive_ethercat {

AnydriveManagerEthercat::AnydriveManagerEthercat(const bool standalone, const bool installSignalHandler, const double timeStep)
    : anydrive::AnydriveManager(standalone, installSignalHandler, timeStep) {
  setCommunicationManager(anydrive::communication::CommunicationManagerBasePtr(new anydrive_ethercat::AnydriveEthercatBusManager()));
}

anydrive::setup::SetupPtr AnydriveManagerEthercat::createSetup() const {
  return anydrive::setup::SetupPtr(new setup::SetupEthercat());
}

bool AnydriveManagerEthercat::calibrate(const std::string& deviceName, const anydrive::calibration::CalibrationModeEnum calibrationModeEnum,
                                        const bool gearAndJointEncoderHomingAbsolute,
                                        const double gearAndJointEncoderHomingNewJointPosition) {
  // Get anydrive.
  anydrive::AnydrivePtr anydrive = getAnydrive(deviceName);
  if (!anydrive) {
    ANYDRIVE_LOGGED_ERROR("Device '" << deviceName << "' does not exist.");
    return false;
  }

  if (anydrive->getActiveStateEnum() != anydrive::fsm::StateEnum::Calibrate) {
    ANYDRIVE_LOGGED_ERROR("Device needs to be in calibrate state for calibration.");
    return false;
  }

  // These calibrations are solely performed on the drive:
  if (calibrationModeEnum == anydrive::calibration::CalibrationModeEnum::MotorEncoderOffset ||
      calibrationModeEnum == anydrive::calibration::CalibrationModeEnum::MotorEncoderParameters ||
      calibrationModeEnum == anydrive::calibration::CalibrationModeEnum::GearAndJointEncoderHoming ||
      calibrationModeEnum == anydrive::calibration::CalibrationModeEnum::ImuGyroscopeDcBias ||
      calibrationModeEnum == anydrive::calibration::CalibrationModeEnum::AksimSelfCalibMode) {
    if (calibrationModeEnum == anydrive::calibration::CalibrationModeEnum::GearAndJointEncoderHoming) {
      /* First send new joint position (it will be wrapped in the firmware if outside of [-pi, pi)).
       * There are two ways of calibrating the position offset:
       * Absolute: The new joint position will be set as the current.
       * Relative: The new joint position will be set as the new zero.
       */
      double newJointPosition = gearAndJointEncoderHomingNewJointPosition;
      if (!gearAndJointEncoderHomingAbsolute) {
        newJointPosition += anydrive->getReading().getState().getJointPosition();
      }
      anydrive->getCommunicationInterface()->sendCalibrationGearAndJointEncoderHomingNewJointPosition(newJointPosition);
    } else if (calibrationModeEnum == anydrive::calibration::CalibrationModeEnum::AksimSelfCalibMode) {
      // Only ANYdrive 3 supports the Aksim-Encoder: Check if an ANYdrive 3 is connected:
      std::string model;
      if (!anydrive->getDriveModel(model)) {
        ANYDRIVE_ERROR("Could not read drive model. Aborting.).");
        return false;
      }
      if (model == "ANYdrive02") {
        ANYDRIVE_ERROR("ANYdrive 2 detected. The gear+joint encoder self-calibration is only available on ANYdrive 3.")
        return false;
      } else if (model == "ANYdrive03") {
        ANYDRIVE_INFO("ANYdrive 3 detected. Performing gear+joint encoder self-calibration.");
      } else {
        ANYDRIVE_INFO("ANYdrive type could not be determined");
        ANYDRIVE_INFO("Obtained drive model name is:");
        ANYDRIVE_INFO(model);
        ANYDRIVE_INFO("Attempting gear+joint encoder self-calibration nonetheless.");
        // Todo: If testing works, this could be omitted by returning false here.
      }
      if (!anydrive->getCommunicationInterface()->startCalibration(calibrationModeEnum)) {
        ANYDRIVE_ERROR("Could not start the calibration.")
        return false;
      }
      if (!anydrive->waitForCalibrationDone(20.0, 10.0)) {
        ANYDRIVE_ERROR("Encoder self-calibration routine failed: It did not terminate within 20s!")
        return false;
      }
      // Read the encoder calibration success/state from the drive to output user feedback.
      anydrive::calibration::CalibrationState calibrationState;
      if (!anydrive->getCalibrationState(anydrive::calibration::CalibrationTypeEnum::Custom, calibrationState)) {
        ANYDRIVE_ERROR("Could not read calibration state from drive.")
        return false;
      }
      bool checkEncCalibStatus = true;
      if (!static_cast<bool>(calibrationState.single_.aksimJoint_)) {  // NOLINT(cppcoreguidelines-pro-type-union-access)
        ANYDRIVE_ERROR("Joint encoder self-calibration failed.")
        checkEncCalibStatus = false;
      }
      if (!static_cast<bool>(calibrationState.single_.aksimGear_)) {  // NOLINT(cppcoreguidelines-pro-type-union-access)
        ANYDRIVE_ERROR("Gear encoder self-calibration failed.")
        checkEncCalibStatus = false;
      }
      if (checkEncCalibStatus) {
        ANYDRIVE_INFO("Gear+Joint encoder self-calibration successful.")
        return true;
      } else {
        return false;
      }
    }

    // Run the other calibrations that do not need special SDK-side actions:
    anydrive->getCommunicationInterface()->startCalibration(calibrationModeEnum);
    if (!anydrive->waitForCalibrationDone(100.0, 10.0)) {
      ANYDRIVE_ERROR("Calibration did not terminate on drive. Aborting.")
      return false;
    }
    // Check the state of the calibration outcomes:
    anydrive::calibration::CalibrationState _calibrationState;
    if (!anydrive->getCalibrationState(anydrive::calibration::CalibrationTypeEnum::Custom, _calibrationState)) {
      ANYDRIVE_ERROR("Could not read calibration state from drive.")
      return false;
    }
    if (calibrationModeEnum == anydrive::calibration::CalibrationModeEnum::MotorEncoderOffset) {
      if (!static_cast<bool>(_calibrationState.single_.motorEncoderOffset_)) {  // NOLINT(cppcoreguidelines-pro-type-union-access)
        ANYDRIVE_ERROR("Motor encoder offset calibration failed.")
        return false;
      }
    } else if (calibrationModeEnum == anydrive::calibration::CalibrationModeEnum::MotorEncoderParameters) {
      if (!static_cast<bool>(_calibrationState.single_.motorEncoderParameters_)) {  // NOLINT(cppcoreguidelines-pro-type-union-access)
        ANYDRIVE_ERROR("Motor encoder parameters calibration failed.")
        return false;
      }
    } else if (calibrationModeEnum == anydrive::calibration::CalibrationModeEnum::ImuGyroscopeDcBias) {
      if (!static_cast<bool>(_calibrationState.single_.imuGyroscopeDcBias_)) {  // NOLINT(cppcoreguidelines-pro-type-union-access)
        ANYDRIVE_ERROR("IMU bias calibration failed.")
        return false;
      }
    }
    return true;
  }
  // The following calibrations are performed with the SDK and the drive together.
  else if (calibrationModeEnum == anydrive::calibration::CalibrationModeEnum::GearJointEncoderOffset) {
    // Check if the correct PDO is being used.
    if (!anydrive->getCommunicationInterface()->receivesGearAndJointEncoderTicks()) {
      ANYDRIVE_ERROR(
          "The gear/joint encoder offset calibration algorithm requires the gear and joint encoder ticks. Please change the PDO type to C "
          "or D.");
      return false;
    }

    // Start the calibration on the drive.
    anydrive->getCommunicationInterface()->startCalibration(calibrationModeEnum);
    anydrive::threadSleep(5.0);  // Sleep to enable FOC.

    // Start the calibration on the host.
    runningCalibration_.reset(new anydrive::calibration::routine::GearJointEncoderOffset(anydrive, 100.0, 3));
    bool success = runningCalibration_->run();
    if (!success) {
      // Stop the calibration on the drive.
      getEthercatCommunicationInterface(anydrive)->sendSdoWrite(OD_CALIB_MODE_ID, 0x00, false, OD_CALIB_MODE_ID_VAL_IDLE);
      ANYDRIVE_ERROR("The gear/joint encoder offset calibration failed.")
      return false;
    }

    // If routine terminated successfully, the parameters are OK. Get and send them to the drive.
    using Vector = anydrive::calibration::routine::GearJointEncoderOffset::Vector;
    const Vector params =
        std::static_pointer_cast<anydrive::calibration::routine::GearJointEncoderOffset>(runningCalibration_)->getParams();

    ANYDRIVE_INFO("Writing the calibration parameters to the drive.");
    success &= getEthercatCommunicationInterface(anydrive)->sendSdoWrite(
        OD_CALIB_GEAR_JOINT_ENCODER_OFFSET_ID, OD_CALIB_GEAR_JOINT_ENCODER_OFFSET_SID_CONSTANT, false, static_cast<int32_t>(params[0]));
    anydrive::threadSleep(0.1);
    success &= getEthercatCommunicationInterface(anydrive)->sendSdoWrite(
        OD_CALIB_GEAR_JOINT_ENCODER_OFFSET_ID, OD_CALIB_GEAR_JOINT_ENCODER_OFFSET_SID_SIN1_AMPLITUDE, false, static_cast<float>(params[1]));
    anydrive::threadSleep(0.1);
    success &= getEthercatCommunicationInterface(anydrive)->sendSdoWrite(OD_CALIB_GEAR_JOINT_ENCODER_OFFSET_ID,
                                                                         OD_CALIB_GEAR_JOINT_ENCODER_OFFSET_SID_SIN1_PHASESHIFT, false,
                                                                         static_cast<float>(params[2]));
    anydrive::threadSleep(0.1);
    success &= getEthercatCommunicationInterface(anydrive)->sendSdoWrite(
        OD_CALIB_GEAR_JOINT_ENCODER_OFFSET_ID, OD_CALIB_GEAR_JOINT_ENCODER_OFFSET_SID_SIN2_AMPLITUDE, false, static_cast<float>(params[3]));
    anydrive::threadSleep(0.1);
    success &= getEthercatCommunicationInterface(anydrive)->sendSdoWrite(OD_CALIB_GEAR_JOINT_ENCODER_OFFSET_ID,
                                                                         OD_CALIB_GEAR_JOINT_ENCODER_OFFSET_SID_SIN2_PHASESHIFT, false,
                                                                         static_cast<float>(params[4]));
    anydrive::threadSleep(0.1);
    ANYDRIVE_INFO("Sent gear/joint encoder offset calibration parameters to the drive.");

    // Stop the calibration on the drive.
    success &= getEthercatCommunicationInterface(anydrive)->sendSdoWrite(OD_CALIB_MODE_ID, 0x00, false,
                                                                         OD_CALIB_MODE_ID_VAL_GEAR_JOINT_ENCODER_OFFSET_END);
    success &= anydrive->waitForCalibrationDone();
    return (success);
  } else if (calibrationModeEnum == anydrive::calibration::CalibrationModeEnum::FrictionEstimation) {
    // Start the calibration on the drive.
    anydrive->getCommunicationInterface()->startCalibration(calibrationModeEnum);
    anydrive::threadSleep(5.0);  // Sleep to enable FOC.

    // Start the calibration on the host.
    runningCalibration_.reset(new anydrive::calibration::routine::FrictionEstimation(anydrive, 2.5));
    bool success = runningCalibration_->run();
    if (!success) {
      // Stop the calibration on the drive.
      getEthercatCommunicationInterface(anydrive)->sendSdoWrite(OD_CALIB_MODE_ID, 0x00, false, OD_CALIB_MODE_ID_VAL_IDLE);
      return false;
    }

    // Get the calibration parameters.
    using Vector = anydrive::calibration::routine::FrictionEstimation::Vector;
    const Vector params = std::static_pointer_cast<anydrive::calibration::routine::FrictionEstimation>(runningCalibration_)->getParams();
    const double breakAwayTorqueBand =
        std::static_pointer_cast<anydrive::calibration::routine::FrictionEstimation>(runningCalibration_)->getBreakAwayTorqueBand();

    // Write the calibration to the drive.
    success &= getEthercatCommunicationInterface(anydrive)->sendSdoWrite(
        OD_CALIB_FRICTION_ESTIMATION_ID, OD_CALIB_FRICTION_ESTIMATION_SID_BREAK_AWAY_FRICTION, false, static_cast<float>(params[0]));
    anydrive::threadSleep(0.1);
    success &= getEthercatCommunicationInterface(anydrive)->sendSdoWrite(OD_CALIB_FRICTION_ESTIMATION_ID,
                                                                         OD_CALIB_FRICTION_ESTIMATION_SID_BREAK_AWAY_FRICTION_BAND, false,
                                                                         static_cast<float>(breakAwayTorqueBand));
    anydrive::threadSleep(0.1);
    success &= getEthercatCommunicationInterface(anydrive)->sendSdoWrite(
        OD_CALIB_FRICTION_ESTIMATION_ID, OD_CALIB_FRICTION_ESTIMATION_SID_VISCOUS_FRICTION_COEFF_NEG, false, static_cast<float>(params[1]));
    anydrive::threadSleep(0.1);
    success &= getEthercatCommunicationInterface(anydrive)->sendSdoWrite(
        OD_CALIB_FRICTION_ESTIMATION_ID, OD_CALIB_FRICTION_ESTIMATION_SID_VISCOUS_FRICTION_COEFF_POS, false, static_cast<float>(params[2]));
    anydrive::threadSleep(0.1);
    ANYDRIVE_INFO("Sent encoder offset calibration to drive.");

    // Stop the calibration on the drive.
    success &= getEthercatCommunicationInterface(anydrive)->sendSdoWrite(OD_CALIB_MODE_ID, 0x00, false,
                                                                         OD_CALIB_MODE_ID_VAL_FRICTION_ESTIMATION_END);

    success &= anydrive->waitForCalibrationDone();

    return success;
  } else if (calibrationModeEnum == anydrive::calibration::CalibrationModeEnum::GravityCompensation) {
    // Check if the correct PDO is being used.
    if (!anydrive->getCommunicationInterface()->receivesGearAndJointEncoderTicks()) {
      ANYDRIVE_ERROR(
          "The gravity compensation calibration algorithm requires the gear and joint encoder ticks. Please change the PDO type to C or "
          "D.");
      return false;
    }
    runningCalibration_.reset(new anydrive::calibration::routine::GravityCompensation(anydrive, 100.0, 3));
    return runningCalibration_->run();
  } else if (calibrationModeEnum == anydrive::calibration::CalibrationModeEnum::SafeJointVelocity) {
    // Check if the correct PDO is being used.
    if (!anydrive->getCommunicationInterface()->receivesGearAndJointEncoderTicks()) {
      ANYDRIVE_ERROR(
          "The safe joint velocity calibration algorithm requires the gear and joint encoder ticks. Please change the PDO type to C or D.");
      return false;
    }
    runningCalibration_.reset(new anydrive::calibration::routine::SafeJointVelocity(anydrive, 5.0, 3));
    return runningCalibration_->run();
  } else {
    ANYDRIVE_LOGGED_ERROR("Calibration is not implemented.");
    return false;
  }
  return false;
}

}  // namespace anydrive_ethercat
