/*!
 * @file    MotionGeneration.hpp
 * @author  Alexander Reske
 * @brief   Motion generator for the controller JointConfigurationsController.
 */

#pragma once

// eigen
#include <Eigen/Core>

// anymal_description
#include <anymal_description/AnymalDescription.hpp>

namespace anymal_ctrl_joint_configurations {

class MotionGeneration {
 public:
  using AD = anymal_description::AnymalDescription;
  using JointVector = Eigen::Matrix<double, AD::getJointsDimension(), 1>;

  //! Construct MotionGeneration.
  MotionGeneration();

  //! Destruct MotionGeneration.
  ~MotionGeneration() = default;

  /*!
   * Create generator MotionGeneration.
   * @returns true if successful.
   */
  bool create();

  /*!
   * Initialize generator MotionGeneration.
   * @returns true if successful.
   */
  bool initialize();

  /*!
   * Advance generator MotionGeneration.
   * @param[in] dt  time step [s].
   * @returns true if successful.
   */
  bool advance(double dt);

  /*!
   * reset generator MotionGeneration.
   * @returns true if successful.
   */
  bool reset();

  /*!
   * Get the current time of the motion plan from generator MotionGeneration.
   * @returns time [s].
   */
  double getTime() const;

  /*!
   * Get the duration of the motion plan from generator MotionGeneration.
   * @returns duration [s].
   */
  double getDuration() const;

  /*!
   * Set the maximum joint velocity for generator MotionGeneration.
   * @param[in] maxJointVelocity maximum joint velocity [rad/s].
   */
  void setMaxJointVelocity(double maxJointVelocity);

  /*!
   * Set the start joint positions for generator MotionGeneration.
   * @param[in] startJointPositions start joint positions [rad].
   */
  void setStartJointPositions(const JointVector& startJointPositions);

  /*!
   * Set the goal joint positions for generator MotionGeneration.
   * @param[in] goalJointPositions start joint positions [rad].
   */
  void setGoalJointPositions(const JointVector& goalJointPositions);

  /*!
   * Get the desired joint positions from generator MotionGeneration.
   * @returns desired joint positions [rad].
   */
  const JointVector& getDesJointPositions() const;

  /*!
   * Get the desired joint velocities from generator MotionGeneration.
   * @returns desired joint velocities [rad/s].
   */
  const JointVector& getDesJointVelocities() const;

  /*!
   * Plan the motion.
   */
  void planMotion();

 private:
  // Member variables
  double time_;
  double duration_;
  double maxJointVelocity_;
  JointVector startJointPositions_;
  JointVector goalJointPositions_;
  JointVector desJointPositions_;
  JointVector desJointVelocities_;
};

} /* namespace anymal_ctrl_joint_configurations */
