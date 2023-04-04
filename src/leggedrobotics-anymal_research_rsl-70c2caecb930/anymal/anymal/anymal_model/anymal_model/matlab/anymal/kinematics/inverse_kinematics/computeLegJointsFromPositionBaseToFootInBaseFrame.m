 function joints = computeLegJointsFromPositionBaseToFootInBaseFrame(B_r_BF, legName, hipUp, elbowDown)
%COMPUTELEGJOINTSFROMPOSITIONBASETOFOOTINBASEFRAME Computes the inverse
%kinematics for a leg given a desired position of a foot w.r.t. to the base
%expressed in base frame.
%The analytical inverse kinematics can have four distinct solutions. The
%user can select which one should be computed by setting flags hipUp (1 or 
%-1 for true or false) and elbowDown (1 or -1 for true of false).

%   Author(s): C. Dario Bellicoso

joints = zeros(3,1);

kin = loadKinematicParametersForLeg(legName);

B_r_BH = kin.B_r_BH;
H_r_HT = kin.H_r_HT;
T_r_TS = kin.T_r_TS;
S_r_SF = kin.S_r_SF;

% Needed to compute the qHAA joint.
B_r_HF_y_0 = H_r_HT(2)+T_r_TS(2)+S_r_SF(2);
R_BH = @(q)rotx(q);

%% qHAA
B_r_HF = B_r_BF - B_r_BH;
p = B_r_HF;

d = B_r_HF_y_0;

r_delta = p(2)^2+p(3)^2 - d^2;
if (r_delta < 0)
    r_delta = 0;
end
r = sqrt(r_delta);
beta = atan2(r,d);
if (hipUp == 1)
    delta = atan2(p(2),-p(3));
    qHAA = delta + beta - pi/2;
else
    delta = atan2(p(3),p(2));
    qHAA = delta - beta;
end

joints(1) = qHAA;

%% qKFE

% Compute the position vector B_r_TF.
B_r_BT = B_r_BH + R_BH(qHAA)*H_r_HT;
B_r_TF = B_r_BF - B_r_BT;

% Compute the coefficients of the linear trigonometric equation which
% solves for qKFE.
d = norm(B_r_TF)^2;
a = trig_eq_kfe_a(T_r_TS, S_r_SF);
b = trig_eq_kfe_b(T_r_TS, S_r_SF);
c = trig_eq_kfe_c(T_r_TS, S_r_SF);

if (kin.fore == 1)
    if (hipUp)
        if (elbowDown == 1)
            joints(3) = solveTrigonometricEquation(a,b,d-c,1);
        else
            joints(3) = solveTrigonometricEquation(a,b,d-c,2);
        end
    else
        if (elbowDown == 1)
            joints(3) = solveTrigonometricEquation(a,b,d-c,2);
        else
            joints(3) = solveTrigonometricEquation(a,b,d-c,1);
        end
    end
else
    if (hipUp)
        if (elbowDown == 1)
            joints(3) = solveTrigonometricEquation(a,b,d-c,2);
        else
            joints(3) = solveTrigonometricEquation(a,b,d-c,1);
        end   
    else
        if (elbowDown == 1)
            joints(3) = solveTrigonometricEquation(a,b,d-c,1);
        else
            joints(3) = solveTrigonometricEquation(a,b,d-c,2);
        end
    end
end

%% qHFE

% Compute the position vector B_r_HF.
B_r_HF = B_r_BF - B_r_BH;

% Compute the coefficients of the linear trigonometric equation which
% solves for qHFE.
d = norm(B_r_HF)^2;
a = trig_eq_hfe_a(joints(3), H_r_HT, T_r_TS, S_r_SF);
b = trig_eq_hfe_b(joints(3), H_r_HT, T_r_TS, S_r_SF);
c = trig_eq_hfe_c(joints(3), H_r_HT, T_r_TS, S_r_SF);

if (kin.fore == 1)
    if (hipUp)
        joints(2) = solveTrigonometricEquation(a,b,d-c,2);
    else
        joints(2) = solveTrigonometricEquation(a,b,d-c,1);
    end
else
    if (hipUp)
        joints(2) = solveTrigonometricEquation(a,b,d-c,1);
    else
        joints(2) = solveTrigonometricEquation(a,b,d-c,2);
    end
end

end