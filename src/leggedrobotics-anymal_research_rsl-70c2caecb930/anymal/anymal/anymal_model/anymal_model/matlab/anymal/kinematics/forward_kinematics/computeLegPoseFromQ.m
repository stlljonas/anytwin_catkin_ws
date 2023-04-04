function [B_r_BF, C_BF] = computeLegPoseFromQ(q, legName)
%COMPUTELEGPOSE Computes the numerical poses of the foot of a leg given a
%vector of joint angles.

%   Author(s): C. Dario Bellicoso
%% Load the kinematic parameters of a named leg.
kin = loadKinematicParametersForLeg(legName);

%% Position
B_r_BH = kin.B_r_BH;
H_r_HT = kin.H_r_HT;
T_r_TS = kin.T_r_TS;
S_r_SF = kin.S_r_SF;

%% Orientation
C_BH = rotx(q(1));
C_HT = roty(q(2));
C_TS = roty(q(3));
C_SF = kin.C_SF;

%% Pose
T_BH = [C_BH B_r_BH;
        0 0 0 1];

T_HT = [C_HT H_r_HT;
        0 0 0 1];

T_TS = [C_TS T_r_TS;
        0 0 0 1];

T_SF = [C_SF S_r_SF;
        0 0 0 1];

T_BF = T_BH*T_HT*T_TS*T_SF;
B_r_BF = T_BF(1:3, 4);
C_BF = T_BF(1:3, 1:3);

end