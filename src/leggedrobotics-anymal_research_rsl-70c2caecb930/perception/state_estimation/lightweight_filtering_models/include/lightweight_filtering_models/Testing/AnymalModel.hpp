/*
* Copyright (c) 2014, Autonomous Systems Lab
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
* names of its contributors may be used to endorse or promote products
* derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#ifndef STARLETHKINEMATICS_H_
#define STARLETHKINEMATICS_H_

#include <Eigen/Dense>
#include "kindr/Core"

#define PI 3.14159265359

namespace rot = kindr;

namespace anymal_model {

class AnymalModel {
public:
  static constexpr double bx = 0.2525;
  static constexpr double by = 0.185;
  static constexpr double lH_0 = -0.0685;
  static constexpr double lT_0 = -0.2;
  static constexpr double lS_0 = -0.235;
  static constexpr double g_ = 9.81;

  AnymalModel(){};

  bool initializeFromUrdf(const std::string& urdf){
    // dummy to have the same interface as the real AnymalModel
    return true;
  }

  double getGravityAcceleration(){
    return g_;
  }

  Eigen::Vector3d forwardKinematicsBaseToFootInBaseFrame(Eigen::Vector3d angles,unsigned int legId){
    double sc0;
    double sc1;
    double sc2;
    double sc3;
    double sc4;
    double sc5;
    sc0 = sin(angles(0));
    sc1 = sin(angles(1));
    sc2 = sin(angles(1)+angles(2));
    sc3 = cos(angles(0));
    sc4 = cos(angles(1));
    sc5 = cos(angles(1)+angles(2));
    Eigen::Vector3d s;
    s(0) = ((legId<2)*2-1)*bx+lT_0*sc1+lS_0*sc2;
    s(1) = -(((int)legId%2)*2-1)*by-sc0*(lH_0+lT_0*sc4+lS_0*sc5);
    s(2) = sc3*(lH_0+lT_0*sc4+lS_0*sc5);
    return s;
  };
  Eigen::Matrix3d getJacobianTranslationBaseToFoot(Eigen::Vector3d angles,unsigned int legId){
    return getJacobianTranslationBaseToSegment(angles,legId,0);
  }
	Eigen::Matrix3d getJacobianTranslationBaseToSegment(Eigen::Vector3d angles,unsigned int legId,unsigned int segmentId){
	  double lH;
	  double lT;
	  double lS;

	  switch (segmentId) {
	    case 0:   // foot (contact point to ground)
	      lH = lH_0;
	      lT = lT_0;
	      lS = lS_0;
	      break;
	    case 1:   // CoM of shank-link
	      lH = lH_0;
	      lT = lT_0;
	      lS = lS_0/2;
	      break;
	    case 2:   // CoM of thigh-link
	      lH = lH_0;
	      lT = lT_0/2;
	      lS = 0;
	      break;
	    case 3:   // CoM of hip-link
	      lH = lH_0/2;
	      lT = 0;
	      lS = 0;
	      break;
	    default:
	      std::cout << "no legKinJac assigned to segmentID " << segmentId << "." << std::endl;
	      break;
	  }

	  double sc0;
	  double sc1;
	  double sc2;
	  double sc3;
	  double sc4;
	  double sc5;
	  sc0 = sin(angles(0));
	  sc1 = sin(angles(1));
	  sc2 = sin(angles(1)+angles(2));
	  sc3 = cos(angles(0));
	  sc4 = cos(angles(1));
	  sc5 = cos(angles(1)+angles(2));
	  Eigen::Matrix<double,3,3> J;
	  J.setZero();
	  J(0,1) = lS*sc5+lT*sc4;
	  J(0,2) = lS*sc5;
	  J(1,0) = -sc3*(lH+lT*sc4+lS*sc5);
	  J(1,1) = sc0*(lT*sc1+lS*sc2);
	  J(1,2) = lS*sc0*sc2;
	  J(2,0) = -sc0*(lH+lT*sc4+lS*sc5);
	  J(2,1) = -sc3*(lT*sc1+lS*sc2);
	  J(2,2) = -lS*sc3*sc2;
	  return J;
	};
	void setGeneralizedPositions(const Eigen::VectorXd& genPos, bool update){};
  void setGeneralizedVelocities(const Eigen::VectorXd& genVel, bool update){};
  bool getMassInertiaMatrix(Eigen::MatrixXd& M){return false;};
  bool getNonlinearEffects(Eigen::VectorXd& h){return false;};
  bool getJacobianFullTranslationBaseToFoot(Eigen::MatrixXd& J, int i){return false;};
  bool getSelectionMatrix(Eigen::MatrixXd& S){return false;};
  const Eigen::Matrix3d getOrientationWorldToFoot(int legId){return Eigen::Matrix3d::Identity();};
};

} // anymal_model

#endif /* STARLETHKINEMATICS_H_ */
