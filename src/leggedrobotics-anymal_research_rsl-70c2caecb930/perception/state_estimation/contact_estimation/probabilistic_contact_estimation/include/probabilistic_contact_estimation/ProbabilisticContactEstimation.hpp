/*
 * ProbabilisticContactEstimation.hpp
 *
 *  Created on: Mar 16, 2016
 *      Author: Jemin Hwangbo, Dario Bellicoso
 *
 * Revision 2019, Fabian Jenelten
 */

#pragma once

// basic contact estimation
#include <basic_contact_estimation/ContactDetectorBase.hpp>

// probabilistic contact estimation.
#include "probabilistic_contact_estimation/measurement_model/ContactProbabilityContainer.hpp"
#include "probabilistic_contact_estimation/transition_model/TransitionProbabilityContainer.hpp"

// stl
#include <memory>

namespace contact_estimation {

namespace internal {

template<typename State_>
class ProbabilisticContactEstimation : public basic_contact_estimation::ContactDetectorBase{
    static_assert(std::is_base_of<State, State_>::value,
                  "[ProbabilisticContactEstimation]: State_ must derive from contact_estimation::State");
  public:
    ProbabilisticContactEstimation(State_* state, anymal_model::AnymalModel* model);
    ~ProbabilisticContactEstimation() override = default;

    bool loadParameters(const TiXmlHandle& handle);
    bool advance(double dt) override;
    bool initialize(const double dt) override;
    bool addVariablesToLog(const std::string& ns = "") const;

    //! Returns the contact state at the contact enum.
    const ContactState& getContactState(const AD::ContactEnum contactEnum) const;    //! Set robot state.
    void setIsRobotStateValid(bool isRobotStateValid) noexcept;

    //! Enable or disable logging.
    void setEnableLogging(bool enableLogging);

  protected:
    //! Specifies which contact/slipping probability models should be added to the measurement model.
    void selectMeasurementModels();

    //! Push back a vector with contact probabilities that are taken into account to compute the global contact probability.
    void addContactProbabilities();

    //! Advance measurement probabilities.
    bool updateMeasurementModel(double dt);

    //! Advance transition probabilities.
    bool updateTransitionModel(double dt);

    //! Advance contact/slipping probabilities.
    bool updateContactAndSlippingState(double dt);

  private:

    // Computes control frame based on current and previous stance footholds.
    bool updatePlaneEstimation();

    //! Measured robot state.
    std::unique_ptr<State_> state_;

    //! Container of individual contact probabilities (measurement model for contact estimation).
    ContactProbabilityContainer contactProbabilities_;

    //! Container of individual slipping probabilities (measurement model for slipping estimation).
    ContactProbabilityContainer slipProbabilities_;

    //! Container of individual transition probabilities (transition model for contact estimation).
    TransitionProbabilityContainer transitionProbabilitiesContact_;

    //! Container of individual transition probabilities (transition model for slipping estimation).
    TransitionProbabilityContainer transitionProbabilitiesSlip_;

    //! Measurement model (multiplied contact probabilities).
    std_utils::EnumArray<AD::ContactEnum, Probability> measurementModelContact_;

    //! Measurement model (multiplied slipping probabilities).
    std_utils::EnumArray<AD::ContactEnum, Probability> measurementModelSlip_;

    //! Global contact probability.
    std_utils::EnumArray<AD::ContactEnum, Probability> contactProbability_;

    //! Global slipping probability.
    std_utils::EnumArray<AD::ContactEnum, Probability> slippingProbability_;

    //! Contact state vector.
    std_utils::EnumArray<AD::ContactEnum, ContactState> contactState_;

    //! Slipping state vector.
    std_utils::EnumArray<AD::ContactEnum, bool> slipState_;

    //! True if robot state is OK (valid).
    // If the state is not valid, we trust only the dynamic-model the measurement model.
    // The transition model will be completely ignored.
    bool isRobotStateValid_;

    //! If true, robot state was OK one step before.
    bool wasRobotStateValid_;

    //! Which probabilities should be added to the contact-measurement model.
    std_utils::EnumArray<AD::ContactEnum, std_utils::EnumArray<ProbabilityEnum, bool>> addContactProbability_;

    //! Which probabilities should be added to the slipping-measurement model.
    std_utils::EnumArray<AD::ContactEnum, std_utils::EnumArray<ProbabilityEnum, bool>> addSlippingProbability_;

    //! If true, contact estimation signals are logged.
    bool enableLogging_;
};

} // namespace internal

using ProbabilisticContactEstimation = internal::ProbabilisticContactEstimation<State>;

} /* namespace contact_estimation */

#include <probabilistic_contact_estimation/ProbabilisticContactEstimation.tpp>
