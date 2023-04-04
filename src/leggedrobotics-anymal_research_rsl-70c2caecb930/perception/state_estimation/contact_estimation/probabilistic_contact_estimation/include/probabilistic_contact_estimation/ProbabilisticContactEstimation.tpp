/*
 * ProbabilisticContactEstimation.cpp
 *
 *  Created on: Mar 16, 2016
 *      Author: Jemin Hwangbo, Dario Bellicoso
 *
 * Theory: https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/119957/eth-49681-01.pdf?sequence=1
 *
 * Revision 2019, Fabian Jenelten
 */

// probabilistic contact estimation.
#include "probabilistic_contact_estimation/ProbabilisticContactEstimation.hpp"
#include "probabilistic_contact_estimation/measurement_model/ContactProbabilityDynamics.hpp"
#include "probabilistic_contact_estimation/measurement_model/ContactProbabilityKinematics.hpp"
#include "probabilistic_contact_estimation/measurement_model/ContactProbabilityDiffKinematics.hpp"
#include "probabilistic_contact_estimation/measurement_model/SlippingProbabilityDiffKinematics.hpp"
#include "probabilistic_contact_estimation/transition_model/TransitionProbabilityFromContact.hpp"
#include "probabilistic_contact_estimation/transition_model/TransitionProbabilityFromAir.hpp"
#include "probabilistic_contact_estimation/transition_model/TransitionProbabilityFromNoSlip.hpp"
#include "probabilistic_contact_estimation/transition_model/TransitionProbabilityFromSlip.hpp"

// signal logger.
#include <signal_logger/signal_logger.hpp>


// tinyxml tools
#include "tinyxml_tools/tinyxml_tools.hpp"

namespace contact_estimation {

namespace internal {

template<typename State_>
ProbabilisticContactEstimation<State_>::ProbabilisticContactEstimation(State_* state, anymal_model::AnymalModel* model) :
    basic_contact_estimation::ContactDetectorBase("probabilistic_contact_detector"),
    state_(state),
    contactProbabilities_(),
    slipProbabilities_(),
    transitionProbabilitiesContact_(),
    transitionProbabilitiesSlip_(),
    measurementModelContact_(Probability(1.0, 0.0)),
    measurementModelSlip_(Probability(1.0, 0.0)),
    contactProbability_(Probability(1.0, 0.0)),
    slippingProbability_(Probability(1.0, 0.0)),
    contactState_(ContactState::OPEN),
    slipState_(false),
    isRobotStateValid_(false),
    wasRobotStateValid_(false),
    addContactProbability_(std_utils::EnumArray<ProbabilityEnum, bool>(true)),
    addSlippingProbability_(std_utils::EnumArray<ProbabilityEnum, bool>(true)),
    enableLogging_(false)
{
  selectMeasurementModels();
  addContactProbabilities();
}

template<typename State_>
void ProbabilisticContactEstimation<State_>::selectMeasurementModels() {
  // By default, all available models are added to the overall measurement model.
  for (const auto contactKey : AD::getContactKeys()) {
    const auto contactEnum = contactKey.getEnum();
    addContactProbability_[contactEnum][ProbabilityEnum::Dynamics] = true;
    addContactProbability_[contactEnum][ProbabilityEnum::Kinematics] = true;
    addContactProbability_[contactEnum][ProbabilityEnum::DiffKinematics] = true;

    addSlippingProbability_[contactEnum][ProbabilityEnum::Dynamics] = true;
    addSlippingProbability_[contactEnum][ProbabilityEnum::Kinematics] = true;
    addSlippingProbability_[contactEnum][ProbabilityEnum::DiffKinematics] = true;
  }
}

template<typename State_>
void ProbabilisticContactEstimation<State_>::addContactProbabilities() {
  contactProbabilities_.clear();
  slipProbabilities_.clear();
  transitionProbabilitiesContact_.clear();
  transitionProbabilitiesSlip_.clear();

  for (const auto contactKey : AD::getContactKeys()) {
    const auto contactEnum = contactKey.getEnum();
    const auto limbEnum = AD::mapKeyEnumToKeyEnum<AD::ContactEnum, AD::LimbEnum>(contactEnum);
    const auto numOfDofs = AD::getNumDofLimb(limbEnum);
    const auto keyName = std::string(AD::mapKeyEnumToKeyName<AD::ContactEnum, AD::BranchEnum>(contactEnum));

    // Step 1:  Add contact probabilities.
    // Note: Make sure that for each contact there is at least one contact probability type.
    if (addContactProbability_[contactEnum][ProbabilityEnum::Dynamics]) {
      contactProbabilities_.addProbability(new ContactProbabilityDynamics(contactEnum, numOfDofs), contactEnum);
    } else {
      MELO_INFO_STREAM("[ProbabilisticContactEstimation::addContactProbabilities] Skip " << probabilityEnumMap[ProbabilityEnum::Dynamics] << " contact-model for limb " << keyName <<".");
    }
    if (addContactProbability_[contactEnum][ProbabilityEnum::Kinematics]) {
      contactProbabilities_.addProbability(new ContactProbabilityKinematics(contactEnum, numOfDofs), contactEnum);
    } else {
      MELO_INFO_STREAM("[ProbabilisticContactEstimation::addContactProbabilities]  Skip " << probabilityEnumMap[ProbabilityEnum::Kinematics] << " contact-model for limb " << keyName <<".");
    }
    if (addContactProbability_[contactEnum][ProbabilityEnum::DiffKinematics]) {
      contactProbabilities_.addProbability(new ContactProbabilityDiffKinematics(contactEnum, numOfDofs), contactEnum);
    } else {
      MELO_INFO_STREAM("[ProbabilisticContactEstimation::addContactProbabilities] Skip " << probabilityEnumMap[ProbabilityEnum::DiffKinematics] << " contact-model for limb " << keyName <<".");
    }

    // Step 2: Add slipping probabilities.
    // Note: vector can be empty.
    if (addSlippingProbability_[contactEnum][ProbabilityEnum::DiffKinematics]) {
      slipProbabilities_.addProbability(new SlippingProbabilityDiffKinematics(contactEnum, numOfDofs), contactEnum);
    } else {
      MELO_INFO_STREAM("[ProbabilisticContactEstimation::addContactProbabilities] Skip " << probabilityEnumMap[ProbabilityEnum::DiffKinematics] << " slipping-model for limb " << keyName <<".");
    }

    // Step 3: Add transition probabilities for contact estimation.
    // Note: For each contact enum that has at least one contact probability, we need a complete transition model.
    transitionProbabilitiesContact_.addProbability(
        new TransitionProbabilityFromContact(contactEnum, numOfDofs),
        new TransitionProbabilityFromAir(contactEnum, numOfDofs),
        contactEnum);

    // Step 4: Add transition probabilities for slipping estimation.
    // Note: For each contact enum that has at least one slipping probability, we need a complete transition model.
    transitionProbabilitiesSlip_.addProbability(
        new TransitionProbabilityFromNoSlip(contactEnum, numOfDofs),
        new TransitionProbabilityFromSlip(contactEnum, numOfDofs),
        contactEnum);
  }
}

template<typename State_>
bool ProbabilisticContactEstimation<State_>::loadParameters(const TiXmlHandle& handle) {
  if(!state_->loadParameters(handle)) {
    MELO_WARN_STREAM("[ ProbabilisticContactEstimation::loadParameters] Failed to load state parameters.");
    return false;
  }

  if(!contactProbabilities_.loadParameters(handle)) {
    MELO_WARN_STREAM("[ ProbabilisticContactEstimation::loadParameters] Failed to load contact probability parameters.");
    return false;
  }

  if(!slipProbabilities_.loadParameters(handle)) {
    MELO_WARN_STREAM("[ ProbabilisticContactEstimation::loadParameters] Failed to load slipping probability parameters.");
    return false;
  }

  if(!transitionProbabilitiesContact_.loadParameters(handle)) {
    MELO_WARN_STREAM("[ ProbabilisticContactEstimation::loadParameters] Failed to load contact transition probability parameters.");
    return false;
  }

  if(!transitionProbabilitiesSlip_.loadParameters(handle)) {
    MELO_WARN_STREAM("[ ProbabilisticContactEstimation::loadParameters] Failed to load slip transition probability parameters.");
    return false;
  }

  return true;
}

template<typename State_>
bool ProbabilisticContactEstimation<State_>::initialize(double dt) {
  // Reset.
  for (const auto contactKey : AD::getContactKeys()) {
    const auto contactEnum = contactKey.getEnum();
    contactProbability_[contactEnum] = initProbability;
    measurementModelContact_[contactEnum] = initProbability;
    measurementModelSlip_[contactEnum] = initProbability;
    slippingProbability_[contactEnum] = initProbabilitySlip;
    contactState_[contactEnum] = ContactState::OPEN;
    slipState_[contactEnum] = false;
  }

  if(!state_->initialize(dt)) {
    MELO_WARN_STREAM("[ProbabilisticContactEstimation::initialize] Failed to initialize state.");
    return false;
  }

  if(!contactProbabilities_.initialize(dt)) {
    MELO_WARN_STREAM("[ProbabilisticContactEstimation::initialize] Failed to initialize contact probabilities.");
    return false;
  }

  if(!slipProbabilities_.initialize(dt)) {
    MELO_WARN_STREAM("[ProbabilisticContactEstimation::initialize] Failed to initialize slipping probabilities.");
    return false;
  }

  if(!transitionProbabilitiesContact_.initialize(dt)) {
    MELO_WARN_STREAM("[ProbabilisticContactEstimation::initialize] Failed to initialize contact transition probabilities.");
    return false;
  }

  if(!transitionProbabilitiesSlip_.initialize(dt)) {
    MELO_WARN_STREAM("[ProbabilisticContactEstimation::initialize] Failed to initialize slip transition probabilities.");
    return false;
  }

  return true;
}

template<typename State_>
bool ProbabilisticContactEstimation<State_>::addVariablesToLog(const std::string& ns) const {
  if (enableLogging_) {
    if(!contactProbabilities_.addVariablesToLog(ns)) { return false; }
    if(!slipProbabilities_.addVariablesToLog(ns)) { return false; }
    if(!transitionProbabilitiesContact_.addVariablesToLog(ns)) { return false; }
    if(!transitionProbabilitiesSlip_.addVariablesToLog(ns)) { return false; }

    for (const auto contactKey : AD::getContactKeys()) {
      const auto contactEnum = contactKey.getEnum();
      const auto keyName = std::string(AD::mapKeyEnumToKeyName<AD::ContactEnum, AD::BranchEnum>(contactEnum));

      // Contact State.
      signal_logger::add(contactState_[contactEnum], "/contactState_" + keyName, "/contact_estimation", "-");
      signal_logger::add(contactProbability_[contactEnum](contactId), "/filteredContactProbability_" + keyName, "/contact_estimation", "-");

      // Slipping State.
      signal_logger::add(slipState_[contactEnum], "/isSlipping_" + keyName, "/contact_estimation", "-");
      signal_logger::add(slippingProbability_[contactEnum](airId), "/probSlipping_" + keyName, "/contact_estimation", "-");
    }
  }

  return true;
}

template<typename State_>
bool ProbabilisticContactEstimation<State_>::advance(double dt) {
  // Reinitialize if robot state becomes valid.
  if(!wasRobotStateValid_ && isRobotStateValid_) {
    if(!initialize(dt)) {
      MELO_WARN_STREAM("[ProbabilisticContactEstimation::advance] Failed to initialize contact detector.");
      return false;
    }
  }

  if(!state_->advance(dt, contactState_)) {
    MELO_WARN_STREAM("[ProbabilisticContactEstimation::advance] Failed to advance state.");
    return false;
  }

  if(!contactProbabilities_.advance(dt, state_.get())) {
    MELO_WARN_STREAM("[ProbabilisticContactEstimation::advance] Failed to advance contact probabilities.");
    return false;
  }

  if(!slipProbabilities_.advance(dt, state_.get())) {
    MELO_WARN_STREAM("[ProbabilisticContactEstimation::advance] Failed to advance slipping probabilities.");
    return false;
  }

  if(!transitionProbabilitiesContact_.advance(dt, state_.get())) {
    MELO_WARN_STREAM("[ProbabilisticContactEstimation::advance] Failed to advance contact transition probabilities.");
    return false;
  }

  if(!transitionProbabilitiesSlip_.advance(dt, state_.get())) {
    MELO_WARN_STREAM("[ProbabilisticContactEstimation::advance] Failed to advance slip transition probabilities.");
    return false;
  }

  if (!updateMeasurementModel(dt)) {
    MELO_WARN_STREAM("[ProbabilisticContactEstimation::advance] Failed to update measurement model.");
    return false;
  }

  if (!updateTransitionModel(dt)) {
    MELO_WARN_STREAM("[ProbabilisticContactEstimation::advance] Failed to update transition model.");
    return false;
  }

  if (!updateContactAndSlippingState(dt)) {
    MELO_WARN_STREAM("[ProbabilisticContactEstimation::advance] Failed to update contact/slipping state.");
    return false;
  }
  
  wasRobotStateValid_ = isRobotStateValid_;

  return true;
}

template<typename State_>
bool ProbabilisticContactEstimation<State_>::updateMeasurementModel(double dt) {
  // Reset.
  for (auto& measurementKey : measurementModelContact_) {measurementKey = Probability(1.0, 1.0); }
  for (auto& measurementKey : measurementModelSlip_) {measurementKey = Probability(1.0, 1.0); }

  // Measurement model (Equation 3) for contact probability.
  for (auto const& contactProbability : contactProbabilities_.getProbabilities()) {
    const auto& contactEnum = contactProbability->getContactEnum();

    // If robot state is not valid, we trust only the dynamics.
    if (!isRobotStateValid_ && contactProbability->getProbabilityEnum()!=ProbabilityEnum::Dynamics) {
      continue;
    }

    measurementModelContact_[contactEnum](contactId) *= contactProbability->getContactProbability();
    measurementModelContact_[contactEnum](airId) *=  contactProbability->getAirProbability();
  }

  // Measurement model (Equation 3) for slip probability.
  for (auto const& slipProbability : slipProbabilities_.getProbabilities()) {
    const auto& contactEnum = slipProbability->getContactEnum();

    // If robot state is not valid, we trust only the dynamics.
    if (!isRobotStateValid_ && slipProbability->getProbabilityEnum()!=ProbabilityEnum::Dynamics) {
      continue;
    }

    // Fuse with contact measurement model
    double pMeasurementContact = 1.0;
    double pMeasurementAir = 0.0;
    for (auto const& contactProbability : contactProbabilities_.getProbabilities()) {
      if(contactProbability->getContactEnum()     == slipProbability->getContactEnum() &&
         contactProbability->getProbabilityEnum() == slipProbability->getProbabilityEnum()) {
        pMeasurementContact = contactProbability->getContactProbability();
        pMeasurementAir     = contactProbability->getAirProbability();
      }
    }
    measurementModelSlip_[contactEnum](contactId) *= slipProbability->getContactProbability()*pMeasurementContact + pMeasurementAir;
    measurementModelSlip_[contactEnum](airId)     *= slipProbability->getAirProbability()*pMeasurementContact;
  }

  return true;
}

template<typename State_>
bool ProbabilisticContactEstimation<State_>::updateTransitionModel(double dt) {
  // Transition model for contact probability.
  for (const auto contactKey : AD::getContactKeys()) {
    const auto contactEnum = contactKey.getEnum();

    // Default transition probabilities.
    double pContact2Contact = one_;
    double pContact2Air     = eps_;
    double pAir2Air         = one_;
    double pAir2Contact     = eps_;

    if (isRobotStateValid_ && transitionProbabilitiesContact_.containsTransitionProbability(contactEnum)) {
      const auto& transitionFromContact = transitionProbabilitiesContact_.getTransitionProbabilityFromContactEnum(contactEnum);
      const auto& transitionFromAir     = transitionProbabilitiesContact_.getTransitionProbabilityFromAir(contactEnum);

      // Transition probabilities.
      pContact2Contact = transitionFromContact.getProbabilityOfStateConstant();
      pContact2Air     = transitionFromContact.getProbabilityOfStateTransition();
      pAir2Air         = transitionFromAir.getProbabilityOfStateConstant();
      pAir2Contact     = transitionFromAir.getProbabilityOfStateTransition();
    }

    // Previous alpha values.
    const double alphaContactOld  = contactProbability_[contactEnum](contactId);
    const double alphaAirOld      = contactProbability_[contactEnum](airId);

    // Measurement probabilities.
    const double pContactMeasurement = measurementModelContact_[contactEnum](contactId);
    const double pAirMeasurement     = measurementModelContact_[contactEnum](airId);

    // Update transition model (Equation 1).
    contactProbability_[contactEnum](contactId) = pContactMeasurement*(alphaContactOld*pContact2Contact + alphaAirOld*pAir2Contact);
    contactProbability_[contactEnum](airId)     = pAirMeasurement*    (alphaContactOld*pContact2Air     + alphaAirOld*pAir2Air);

    // Compute contact probability (Equation 2).
    if(!normalize2dProb(contactProbability_[contactEnum])) { return false; }
  }

  // Transition model for slip probability.
  for (const auto contactKey : AD::getContactKeys()) {
    const auto contactEnum = contactKey.getEnum();

    // Default transition probabilities.
    double pSlip2Slip       = one_;
    double pSlip2NoSlip     = eps_;
    double pNoSlip2NoSlip   = one_;
    double pNoSlip2Slip     = eps_;

    if (isRobotStateValid_ && transitionProbabilitiesSlip_.containsTransitionProbability(contactEnum)) {
      const auto& transitionFromNoSlip = transitionProbabilitiesSlip_.getTransitionProbabilityFromContactEnum(contactEnum);
      const auto& transitionFromSlip   = transitionProbabilitiesSlip_.getTransitionProbabilityFromAir(contactEnum);

      // Transition probabilities.
      pSlip2Slip       = transitionFromSlip.getProbabilityOfStateConstant();
      pSlip2NoSlip     = transitionFromSlip.getProbabilityOfStateTransition();
      pNoSlip2NoSlip   = transitionFromNoSlip.getProbabilityOfStateConstant();
      pNoSlip2Slip     = transitionFromNoSlip.getProbabilityOfStateTransition();
    }

    // Previous alpha values.
    const double alphaNoSlipOld   = slippingProbability_[contactEnum](contactId);
    const double alphaSlipOld     = slippingProbability_[contactEnum](airId);

    // Measurement probabilities (conditioned on contact probability).
    const double pNoSlipMeasurement = measurementModelSlip_[contactEnum](contactId)*contactProbability_[contactEnum](contactId) + contactProbability_[contactEnum](airId);
    const double pSlipMeasurement   = measurementModelSlip_[contactEnum](airId)    *contactProbability_[contactEnum](contactId);

    // Update transition model (Equation 1).
    slippingProbability_[contactEnum](contactId) = pNoSlipMeasurement*(alphaNoSlipOld*pNoSlip2NoSlip + alphaSlipOld*pSlip2NoSlip);
    slippingProbability_[contactEnum](airId)     = pSlipMeasurement*  (alphaNoSlipOld*pNoSlip2Slip   + alphaSlipOld*pSlip2Slip);

    // Compute contact probability (Equation 2).
    if(!normalize2dProb(slippingProbability_[contactEnum])) { return false; }
  }

  return true;
}

template<typename State_>
bool ProbabilisticContactEstimation<State_>::updateContactAndSlippingState(double dt) {
  for (const auto contactKey : AD::getContactKeys()) {
    const auto contactEnum = contactKey.getEnum();

    // Case A: Contact.
    if (contactProbability_[contactEnum](contactId) > 0.5) {
      // Case A1: Contact and slipping
      if (slippingProbability_[contactEnum](airId)>0.5){
        slipState_[contactEnum] = true;
        contactState_[contactEnum] = ContactState::SLIPPING;
        MELO_WARN_THROTTLE_STREAM(0.5, "[ ProbabilisticContactEstimation::updateContactAndSlippingState] " << AD::mapKeyEnumToKeyName(contactEnum) << " is slipping.");
      }

      // Case A2: Contact and not slipping.
      else{
        slipState_[contactEnum] = false;
        contactState_[contactEnum] = ContactState::CLOSED;
      }
    }

    // Case B: No contact (and hence not slipping).
    else {
      slipState_[contactEnum] = false;
      contactState_[contactEnum] = ContactState::OPEN;
    }
  }

  return true;
}

template<typename State_>
const ContactState& ProbabilisticContactEstimation<State_>::getContactState(const AD::ContactEnum contactEnum) const {
  return contactState_[contactEnum];
}

template<typename State_>
void ProbabilisticContactEstimation<State_>::setIsRobotStateValid(bool isRobotStateValid) noexcept {
  isRobotStateValid_ = isRobotStateValid;
}

template<typename State_>
void ProbabilisticContactEstimation<State_>::setEnableLogging(bool enableLogging) {
  enableLogging_ = enableLogging;
}

} // namespace internal

} /* namespace contact_estimation */
