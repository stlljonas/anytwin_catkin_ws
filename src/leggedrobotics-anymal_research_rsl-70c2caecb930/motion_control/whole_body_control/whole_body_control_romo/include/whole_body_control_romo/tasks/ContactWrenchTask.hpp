// Created by Koen Krämer on 08.03.19.


#pragma once


// whole_body_control_romo
#include <whole_body_control_romo/WholeBodyStateRomo.hpp>
#include <whole_body_control_romo/tasks/TaskRomo.hpp>
#include <whole_body_control_romo/typedefs.hpp>

namespace whole_body_control_romo {

template <typename ConcreteDescription_, typename RobotState_>
    class ContactWrenchTask : public TaskRomo<ConcreteDescription_, RobotState_> {
    protected:
        using Base = TaskRomo<ConcreteDescription_, RobotState_>;
        using RD = romo::RobotDescription<ConcreteDescription_>;
        using CoordinateFrameEnum = typename RD::CoordinateFrameEnum;
        using LimbEnum = typename RD::LimbEnum;
        using WholeBodyState = WholeBodyStateRomo<ConcreteDescription_, RobotState_>;

    public:
        explicit ContactWrenchTask(WholeBodyState& wholeBodyState) : Base(wholeBodyState) {
            this->inequalityConstraintJacobian_ = Eigen::MatrixXd();
            this->inequalityConstraintMaxValues_ = Eigen::VectorXd();
            this->inequalityConstraintRelativeWeights_ = Eigen::VectorXd();
            this->equalityConstraintRelativeWeights_ = Eigen::VectorXd::Ones(RD::getBaseGeneralizedVelocitiesDimension());
            this->constraintType_ = hopt::ConstraintType::Equality;
        }

        static std::unique_ptr<whole_body_control::Task> createTask(WholeBodyState& wholeBodyState) {
            return std_utils::make_unique<ContactWrenchTask<ConcreteDescription_, RobotState_>>(wholeBodyState);
        }

        static std::string getTaskTypeName() { return "endeffector_contact_wrench"; }

        //! Load parameters
        bool loadParameters(TiXmlHandle taskHandle) override {

          // Get all definitions of wrench properties, later ones overwrite the previous declarations.
          std::vector<TiXmlElement*> wrenchPropertyElements;

          if (!tinyxml_tools::getChildElements(wrenchPropertyElements, taskHandle, "Wrench")) {
            return false;
          }

          for (auto& elem : wrenchPropertyElements) {
            // Get limbs from comma separated list.
            std::string limbList;
            if (!tinyxml_tools::loadParameter(limbList, elem, "limbs")) {
              MELO_ERROR_STREAM("[ContactWrenchTask] Cannot find specified limbs for task in xml file.");
              return false;
            }
            //Fill taskLimbs_
            std::vector<std::string> limbNames = this->getLimbNamesFromCommaSeperatedString(limbList);
            for (const auto &limbName : limbNames) {
              unsigned int limbId;
              // Get limb id
              try {
                limbId = static_cast<unsigned int>(RD::template mapKeyNameToKeyId<LimbEnum>(limbName));
              } catch (std::out_of_range &e) {
                MELO_WARN_STREAM("[ContactWrenchTask]: Cannot map limbName " << limbName << " to LimbId." << e.what());
                return false;
              }
              // Get limb from id
              auto &limbs = this->getWholeBodyState().getWholeBody().getLimbs();
              auto it = std::find_if(limbs.begin(), limbs.end(),
                                     [limbId](loco::LimbBase const *const limb) { return limb->getLimbUInt() == limbId; });
              if (it != limbs.end()) {
                // Check if taskLimbs_ does not yet contain that limb
                if (std::find(taskLimbs_.begin(), taskLimbs_.end(), *it) != taskLimbs_.end()) {
                  MELO_WARN_STREAM("[ContactWrenchTask]: Limb " << limbName
                                                                << " has already been specified as task limb. Will not overwrite. Fix task xml parameters.");
                  continue;
                } else {
                  // If limb exists and is not processed yet, add it to taskLimbs_
                  taskLimbs_.push_back(*it);
                }
              }
              else {
                MELO_ERROR_STREAM("[ContactWrenchTask]: Limb " << limbName << " is not contained by wholeBody.");
                return false;
              }
            }
          }

          return Base::loadParameters(taskHandle);
        }

        bool update(double dt, int solutionSpaceDimension) override {

          // Compute numContacts
          int num6DofContacts = 0;

          for (const auto& limb : taskLimbs_) {
            LimbEnum limbEnum;
            try {
              limbEnum = RD::template mapKeyIdToKeyEnum<LimbEnum>(limb->getLimbUInt());
            } catch (std::out_of_range &e) {
              MELO_WARN_STREAM("[ContactWrenchTask] Cannot map LimbId of limb " << limb->getName() << " to LimbEnum."
                                                                                << e.what());
              return false;
            }
            if (this->getWholeBodyState().getContactFlags()[limbEnum] == ContactStateEnum::ContactClosed6Dof) {
              ++num6DofContacts;
            }
          }

          this->equalityConstraintJacobian_ = Eigen::MatrixXd::Zero(6 * num6DofContacts, solutionSpaceDimension);
          this->equalityConstraintTargetValues_ = Eigen::VectorXd::Zero(6 * num6DofContacts);
          this->equalityConstraintRelativeWeights_ = Eigen::VectorXd::Ones(6 * num6DofContacts);

          RotationQuaternion orientationWorldToForceFrame;

          if (this->getWholeBodyState().getSupportJacobian().getForceFrame() == CoordinateFrameEnum::BASE) {
            orientationWorldToForceFrame = RotationQuaternion(RotationMatrix(
                    this->getWholeBodyState().getRobotModel().getOrientationWorldToBody(RD::BodyEnum::BASE)));
          }
          else {
            orientationWorldToForceFrame.setIdentity();
          }

          unsigned int k = 0;
          for (const auto& limb : taskLimbs_) {
            LimbEnum limbEnum;
            try {
              limbEnum = RD::template mapKeyIdToKeyEnum<LimbEnum>(limb->getLimbUInt());
            } catch (std::out_of_range& e) {
              MELO_WARN_STREAM("[ContactWrenchTask] Cannot map LimbId of limb " << limb->getName() << " to LimbEnum." << e.what());
              return false;
            }

            // Skip this limb if it is not in 6DOF contact.
            if (this->getWholeBodyState().getContactFlags()[limbEnum] != ContactStateEnum::ContactClosed6Dof) {
              continue;
            }

            const int supportJacobianIndex = this->getWholeBodyState().getSupportJacobian().getStartIndexInSupportJacobian(limbEnum);
            this->equalityConstraintJacobian_.template block<6, 6>(6*k, RD::getNumDof() + supportJacobianIndex).setIdentity();
            this->equalityConstraintTargetValues_.template segment<3>(6*k) =
                    orientationWorldToForceFrame.rotate(limb->getEndEffector().getStateDesired().getTorqueAtEndEffectorInWorldFrame().toImplementation());
            this->equalityConstraintTargetValues_.template segment<3>(6*k + 3) =
                    orientationWorldToForceFrame.rotate(limb->getEndEffector().getStateDesired().getForceAtEndEffectorInWorldFrame().toImplementation());
            ++k;
          }
          return true;
        }

    private:
        std::vector<loco::LimbBase const*> taskLimbs_;
  };
}  // namespace whole_body_control_romo
