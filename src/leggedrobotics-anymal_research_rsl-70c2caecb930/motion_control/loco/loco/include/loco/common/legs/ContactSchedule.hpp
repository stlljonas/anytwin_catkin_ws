/*
 * ContactSchedule.hpp
 *
 *  Created on: Dec 22, 2016
 *      Author: Dario Bellicoso
 */

#pragma once

#include <memory>
#include <string>

namespace loco {

class ContactSchedule {
 public:
  ContactSchedule();
  virtual ~ContactSchedule() = default;

  //! Reset to a known state.
  bool initialize(double dt);

  /*!Add variables to log.
   * @return true if successful.
   */
  virtual bool addVariablesToLog(const std::string& ns) const;

  /*! @returns the stance phase. The phase is -1 if the leg is in swing mode,
   * otherwise it is between 0 (start) and 1 (end).
   */
  virtual double getStancePhase() const;

  /*! @returns the swing phase. The phase is -1 if the leg is in stance mode,
   * otherwise it is between 0 (start) and 1 (end).
   */
  virtual double getSwingPhase() const;

  /*! @returns the duration of the stance phase in seconds.
   * The leg should be grounded during the stance phase.
   */
  virtual double getStanceDuration() const;

  /*! @returns the duration of the swing phase in seconds.
   * The leg should be in the air during the swing phase.
   */
  virtual double getSwingDuration() const;

  /*! @returns true if the leg is in contact with the ground according to sensor measurements.
   */
  virtual bool isGrounded() const;

  /*! @returns true if the leg was in contact with the ground according to sensor measurements
   * during the previous control update.
   */
  virtual bool wasGrounded() const;

  /*! @returns true if the leg should be in contact with the ground
   *  according to the plan (timing).
   */
  virtual bool shouldBeGrounded() const;

  /*! @returns true if the leg is grounded according to sensor measurements and should be grounded
   *  according to the plan (timing).
   *  @see isGrounded, shouldBeGrounded
   */
  virtual bool isAndShouldBeGrounded() const;

  /*! @returns true if the leg is slipping, i.e it is grounded according to sensor measurements,
   * but the foot still moves.
   */
  virtual bool isSlipping() const;

  /*! @returns true if the leg is supposed to be grounded, but lost contact according to sensor measurements.
   */
  virtual bool isLosingContact() const;

  virtual void setIsLosingContact(bool isLosingContact);

  virtual void setStancePhase(double phase);
  virtual void setSwingPhase(double phase);

  virtual void setStanceDuration(double duration);
  virtual void setSwingDuration(double duration);

  virtual void setIsGrounded(bool isGrounded);
  virtual void setWasGrounded(bool wasGrounded);
  virtual void setShouldBeGrounded(bool shouldBeGrounded);
  virtual void setIsSlipping(bool isSlipping);

  void setPreviousStancePhase(double previousStancePhase);
  double getPreviousStancePhase() const;

  void setPreviousSwingPhase(double previousSwingPhase);
  double getPreviousSwingPhase() const;

  virtual void setLastOrCurrentContactPhase(double stridePhase);
  virtual double getLastOrCurrentContactPhase() const;

  void setIsInStandConfiguration(bool isInStandConfiguration);
  bool isInStandConfiguration() const;

  void setInterpolationParameter(double interpolationParameter);
  double getInterpolationParameter() const;

 private:
  /*! The stance phase is -1 if the leg is in swing mode,
   * otherwise it is between 0 (start) and 1 (end).
   */
  double stancePhase_;

  //! The stance phase during the previous control update.
  double previousStancePhase_;

  /*! The swing phase is -1 if the leg is in stance mode,
   * otherwise it is between 0 (start) and 1 (end).
   */
  double swingPhase_;

  //! The swing phase during the previous control update.
  double previousSwingPhase_;

  /*! The duration of the stance phase in seconds.
   * The leg should be grounded during the stance phase.
   */
  double stanceDuration_;

  /*! The duration of the swing phase in seconds.
   * The leg should be in the air during the swing phase.
   */
  double swingDuration_;

  //! Indicates if the leg is in contact with the ground according to sensor measurements.
  bool isGrounded_;

  /*! Indicates if the leg was in contact with the ground according to sensor measurements
   * during the previous control update.
   */
  bool wasGrounded_;

  /*! Indicates if the leg should be in contact with the ground according to the plan (timing).
   */
  bool shouldBeGrounded_;

  /*! Indicates if the leg is slipping, i.e. it is in contact with the ground according to sensor measurements,
   * but its foot is moving.
   */
  bool isSlipping_;

  //! Indicates if the leg is supposed to be grounded, but lost contact according to sensor measurements.
  bool isLosingContact_;

  //! This flag is used to force a leg to be in support mode.
  bool isInStandConfiguration_;

  //! The last or current contact phase.
  double lastOrCurrentContactPhase_;

  //! This parameter is used to interpolate swing motions.
  double interpolationParameter_;
};

using ContactSchedulePtr = std::unique_ptr<ContactSchedule>;

} /* namespace loco */
