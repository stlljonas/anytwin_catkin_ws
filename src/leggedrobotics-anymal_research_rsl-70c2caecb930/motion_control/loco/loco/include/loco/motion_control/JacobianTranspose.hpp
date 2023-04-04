/*!
 * @file     JacobianTranspose.hpp
 * @author   Christian Gehring, Péter Fankhauser
 * @date     Aug, 2015
 * @brief
 */

#pragma once

#include "loco/common/WholeBody.hpp"
#include "loco/motion_control/MotionControllerBase.hpp"

namespace loco {

/*! Motion controller where joint torques \f$\tau\f$ are computed from desired contact forces \f$F\f$ (applied by the robot on the
 * environment) by \f$\tau = J^T F\f$.
 *
 * @note This control law is only valid when the robot is not moving. Use it at most for quasi-static motions (i.e. slow motions where the
 * robot can freeze any time without falling). Don't use it for more dynamic motions.
 *
 * @note Here are more details. Start from the generalized equation of motion:
 * \f[
 *   M(q) \ddot{q} + C(q, \dot{q} \dot{q} + g(q) = S^T \tau - J^T F
 * \f]
 * where S is the actuated-joint selection matrix, and \f$F\f$ are contact forces applied by the robot on the environment. Assume no
 * acceleration nor velocity:
 * \f[
 *   g(q) = S^T \tau - J^T F
 * \f]
 * With a proper choice of coordinates, \f$g(q)\f$ has non-zero values only for floating base coordinates (see e.g. Equation (4) in "Dynamic
 * Walking on Compliant and Uneven Terrain Using DCM and Passivity-Based Whole-Body Control", Mesesan et al. 2019). Then, multiplying both
 * sides of the equality by \f$S\f$ we end with \f$\tau = J^T F\f$.
 */
class JacobianTranspose : public MotionControllerBase {
 public:
  /*!
   * Constructor.
   *
   * @param wholeBody Whole-body state of the robot.
   * @param isSettingJointPositionsFromDesiredBase If true, desired joint angles will be updated to track the measured position of the
   * end-effector in the world frame.
   */
  explicit JacobianTranspose(WholeBody& wholeBody, const bool isSettingJointPositionsFromDesiredBase = true);

  /*!
   * Destructor.
   */
  ~JacobianTranspose() override = default;

  /*!
   * Load parameters.
   * @return true if successful
   */
  bool loadParameters(const TiXmlHandle& handle) override;

  /*!
   * Add data to logger (optional).
   * @return true if successful
   */
  bool addVariablesToLog(const std::string& ns) const override;

  /*!
   * Initializes the motion controller
   * @return true if successful
   */
  bool initialize(double dt) override;

  /*!
   * Computes the joint torques from the desired base pose.
   * @return true if successful
   */
  bool advance(double dt) override;

  void computeJointTorquesFromForceAtEndEffectorInWorldFrame(JointTorques& jointTorques, LimbBase* limb,
                                                             const Force& desiredContactForceAtEndEffectorInWorldFrame) const;

  void setJointPositionsFromDesiredBase();

  void activateIsSettingJointPositionsFromDesiredBase(const bool activate);

 protected:
  void setJointPositionsFromDesiredBase(LimbBase* limb);

 protected:
  bool isSettingJointPositionsFromDesiredBase_;
};

} /* namespace loco */
