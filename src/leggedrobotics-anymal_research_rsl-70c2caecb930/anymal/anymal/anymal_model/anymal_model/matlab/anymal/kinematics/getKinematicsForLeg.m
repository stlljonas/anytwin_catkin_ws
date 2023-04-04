function kin_leg = getKinematicsForLeg(q, legName)
%GETKINEMATICSFORLEG Loads the homogeneous transformations for the links of
%a leg.

%   Author(s): C. Dario Bellicoso

kin = loadKinematicParametersForLeg(legName);

kin_leg.T_BH = [kin.C_BH(q) kin.B_r_BH;
                0 0 0 1];

kin_leg.T_HT = [kin.C_HT(q) kin.H_r_HT;
                0 0 0 1];
            
kin_leg.T_TS = [kin.C_TS(q) kin.T_r_TS;
                0 0 0 1];

kin_leg.T_SF = [kin.C_SF    kin.S_r_SF;
                0 0 0 1];

end