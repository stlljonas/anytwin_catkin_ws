/*
 * ForceSensorCalibrator.hpp
 *
 *  Created on: Jan 4, 2017
 *      Author: Christian Gehring
 */

#pragma once

// loco
#include <loco/common/legs/Legs.hpp>

// anymal_roco
#include <anymal_roco/RocoCommand.hpp>

// anymal model
#include <anymal_model/AnymalModel.hpp>

// tinyxml
#include <tinyxml.h>

namespace loco_anymal {

class ForceSensorsCalibrator {
 public:
  using AD = anymal_description::AnymalDescription;

  explicit ForceSensorsCalibrator(anymal_roco::RocoCommand& command)
    : isLegCalibrated_(AD::getNumLegs(), false),
      firstCycle_(AD::getNumLegs(), true),
      durationIsGrounded_(AD::getNumLegs(), 0.0)
  { }

  virtual ~ForceSensorsCalibrator() = default;

  void initialize(double dt) {
    firstCycle_ = std::vector<bool>(AD::getNumLegs(), true);
  }

  void advance(anymal_roco::RocoCommand& command, const loco::Legs& legs, double dt) {
    for (auto leg : legs) {
      const double swingPhase = leg->getContactSchedule().getSwingPhase();
      const int legId = leg->getId();

      // Calibrate force sensor.
      if (swingPhase > startCalibrationAtSwingPhase_ &&
          swingPhase < endCalibrationAtSwingPhase_ &&
          !isLegCalibrated_.at(legId)) {
        command.getForceCalibratorCommands()[AD::mapKeyIdToKeyEnum<AD::ContactEnum>(leg->getId())].cmdStart_ = true;

        // Check duration of grounded leg.
        if (durationIsGrounded_.at(legId) > thresholdDurationDisableOulierDetection_ || firstCycle_.at(legId)) {
          command.getForceCalibratorCommands()[AD::mapKeyIdToKeyEnum<AD::ContactEnum>(leg->getId())].enableOutlierDetector_ = false;
          firstCycle_.at(legId) = false;
        } else {
          command.getForceCalibratorCommands()[AD::mapKeyIdToKeyEnum<AD::ContactEnum>(leg->getId())].enableOutlierDetector_ = true;
        }
        command.getForceCalibratorCommands()[AD::mapKeyIdToKeyEnum<AD::ContactEnum>(leg->getId())].numSamples_= leg->getContactSchedule().getSwingDuration()*durationPhaseSamples_/dt;
        command.getForceCalibratorCommands()[AD::mapKeyIdToKeyEnum<AD::ContactEnum>(leg->getId())].numGoodSamples_ = numberGoodSamples_;

        isLegCalibrated_.at(legId) = true;
      } else if (leg->getContactSchedule().shouldBeGrounded()) {
        isLegCalibrated_.at(legId) = false;
      }

      // Update duration of grounded leg.
      if (leg->getContactSchedule().isGrounded()) {
        durationIsGrounded_.at(legId) += dt;
      } else if (isLegCalibrated_.at(legId)) {
        durationIsGrounded_.at(legId) = 0.0;
      }
    }
  }

  bool loadParameters(const TiXmlHandle& handle) {
    TiXmlElement* pElem;

    TiXmlHandle CalibrationSwingPhaseTimining(handle.FirstChild("ForceCalibration").FirstChild("SwingPhaseTiming"));
    pElem = CalibrationSwingPhaseTimining.Element();
    if (pElem == nullptr) {
      printf("Could not find ForceCalibration/SwingPhaseTiming\n");
      return false;
    }
    if (pElem->QueryDoubleAttribute("start", &startCalibrationAtSwingPhase_)!=TIXML_SUCCESS) {
      printf("Could not find start phase of calibration\n");
      return false;
    }
    if (pElem->QueryDoubleAttribute("end", &endCalibrationAtSwingPhase_)!=TIXML_SUCCESS) {
      printf("Could not find end phase of calibration\n");
      return false;
    }

    TiXmlHandle handleSampling(handle.FirstChild("ForceCalibration").FirstChild("Sampling"));
    pElem = handleSampling.Element();
    if (pElem == nullptr) {
      printf("Could not find ForceCalibration/Sampling\n");
      return false;
    }
    if (pElem->QueryDoubleAttribute("swingPhaseDuration", &durationPhaseSamples_)!=TIXML_SUCCESS) {
      printf("Could not find duration of calibration sampling\n");
      return false;
    }
    if (pElem->QueryUnsignedAttribute("numGoodSamples", &numberGoodSamples_)!=TIXML_SUCCESS) {
      printf("Could not find number of good samples for calibration\n");
      return false;
    }

    TiXmlHandle handleDisableOutlierDetection(handle.FirstChild("ForceCalibration").FirstChild("DisableOutlierDetection"));
    pElem = handleDisableOutlierDetection.Element();
    if (pElem == nullptr) {
      printf("Could not find ForceCalibration/DisableOutlierDetection\n");
      return false;
    }
    if (pElem->QueryDoubleAttribute("thresholdDuration", &thresholdDurationDisableOulierDetection_)!=TIXML_SUCCESS) {
      printf("Could not find duration for disable outlier detection.\n");
      return false;
    }

    return true;
  }

 protected:
  std::vector<bool> isLegCalibrated_;
  std::vector<bool> firstCycle_;

  double startCalibrationAtSwingPhase_  = 0.0;
  double endCalibrationAtSwingPhase_  = 0.0;

  double durationPhaseSamples_ = 0.0;
  unsigned int numberGoodSamples_ = 0;

  std::vector<double> durationIsGrounded_;
  double thresholdDurationDisableOulierDetection_ = 0.0;
};


} // namespace
