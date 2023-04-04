function kin = loadKinematicParametersForLeg(legName)
%LOADKINEMATICPARAMETERSFORLEG Loads the numerical kinematic parameters for
%a leg given a string identifier.

%   Author(s): C. Dario Bellicoso

kin = struct();

%% Check the leg name.
if (legName == 'LF')
    flipX =  1;
    flipY =  1;
    kin.fore = 1;
elseif (legName == 'RF')
    flipX =  1;
    flipY = -1;
    kin.fore = 1;
elseif (legName == 'LH')
    flipX = -1;
    flipY =  1;
    kin.fore = -1;
elseif (legName == 'RH')
    flipX = -1;
    flipY = -1;
    kin.fore = -1;
else
    disp('Unhandled leg name! The name can be any of [LF, RF, LH, RH]');
    return;
end

flip = [flipX; flipY; 1];

%% ANYmal kinematic parameters for the left fore leg.
B_r_BH = [ 0.2535;  0.1150;  0.0000];
H_r_HT = [ 0.0585;  0.0315;  0.0000];
T_r_TS = [ 0.0000;  0.0800; -0.2500];
S_r_SF = [-0.0024; -0.0065; -0.3055];

C_SF_rpy = [0; 0; 0];

%% Position
kin.B_r_BH = flip.*B_r_BH;
kin.H_r_HT = flip.*H_r_HT;
kin.T_r_TS = flip.*T_r_TS;
kin.S_r_SF = flip.*S_r_SF;

kin.C_BH = @(q)rotx(q(1));
kin.C_HT = @(q)roty(q(2));
kin.C_TS = @(q)roty(q(3));

%% Orientation
C_SF_rpy_leg = flip.*C_SF_rpy;
kin.C_SF = rotx(C_SF_rpy_leg(1))*roty(C_SF_rpy_leg(2))*rotz(C_SF_rpy_leg(3));

end