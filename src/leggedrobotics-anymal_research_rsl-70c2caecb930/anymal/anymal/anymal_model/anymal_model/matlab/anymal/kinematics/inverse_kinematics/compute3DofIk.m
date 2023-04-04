function [] = compute3DofIk()
%COMPUTE3DOFIK Finds the analytical and symbolical terms needed to compute
%the inverse kinematics of a 3DOF kinematic chain.

%This script will generate or update script files which compute the
%analytical terms needed to compute the inverse kinematics. See the
%comments for more info on the workflow.
%
%   Author(s): C. Dario Bellicoso

%% Generalized coordinates
% We are computing the inverse kinematics for a 3DoF kinematic chain
% composed by revolute joints only.
syms q1 q2 q3 'real'

%% Kinematics
% Parametrize the cartesian components of the relative position of each
% frame expressed in local body coordinates. When only revolute joints are
% employed in the kinematic chain, these quantities are constant.
syms B_r_BH_x B_r_BH_y B_r_BH_z ... % base to hip
     H_r_HT_x H_r_HT_y H_r_HT_z ... % hip to thigh
     T_r_TS_x T_r_TS_y T_r_TS_z ... % thigh to shank
     S_r_SF_x S_r_SF_y S_r_SF_z ... % shank to foot
     'real';

syms ankle_roll ankle_pitch 'real';

% Parametrize the roll-pitch-yaw angles of the relative orientation for
% each pair of successive frames. When only revolute joints are employed,
% these are all function of the joint angles. When a fixed body is
% considered, the associated angles are constant.
syms C_BH_r C_BH_p C_BH_y ... % hip to base orientation
     C_HT_r C_HT_p C_HT_y ... % thigh to hip orientation
     C_TS_r C_TS_p C_TS_y ... % shank to thigh orientation
     C_SF_r C_SF_p C_SF_y ... % foot mount to shank orientation
     'real';


% Full symbolic parameters.
B_r_BH = [B_r_BH_x;  B_r_BH_y;  B_r_BH_z];
H_r_HT = [H_r_HT_x;  H_r_HT_y;  H_r_HT_z];
T_r_TS = [T_r_TS_x;  T_r_TS_y;  T_r_TS_z];
S_r_SF = [S_r_SF_x;  S_r_SF_y;  S_r_SF_z];


% Roll-pitch-yaw angles are equivalent to ZYX Euler angles.
C_BH = rotx(q1);
C_HT = roty(q2);
C_TS = roty(q3);
C_SF = rotx(C_SF_r)*roty(C_SF_p)*rotz(C_SF_y);

%% Homogeneous transformations.
T_BH = [C_BH B_r_BH;
        0 0 0 1];

T_HT = [C_HT H_r_HT;
        0 0 0 1];

T_TS = [C_TS T_r_TS;
        0 0 0 1];

T_SF = [C_SF S_r_SF;
        0 0 0 1];

%% Forward kinematics
T_BF = simplify(T_BH*T_HT*T_TS*T_SF);
T_BT = simplify(T_BH*T_HT);
B_r_BF = T_BF(1:3,4);

B_r_HF = simplify(B_r_BF - B_r_BH);
B_r_BT = T_BT(1:3,4);

%% Inverse kinematics

% Compute the position vector B_r_HF and compensate all the constant terms
% and those which depend only on q1 from it.
% Note: due to the specific kinematic chain (HAA - HFE - KFE), the norm of
% the vector B_r_TF will depend only on q3 = qKFE.
B_r_TF = B_r_BF - B_r_BT;

% Rewrite d in a simpler form.
d = simplify(norm(B_r_TF)^2);
d = collect(expand(d), [cos(q3) sin(q3)]);

% Find the coefficients of an equation in the from:
%       a*sin(q3) + b*cos(q3) + c = || B_r_TF ||^2;
%
% The symbol c3 is a shorthand for cos(q3).
% The symbol s3 is a shorthand for sin(q3).
syms c3 s3 'real';
d = subs(d, cos(q3), c3);
d = subs(d, sin(q3), s3);

a = simplify(jacobian(d, s3));
b = simplify(jacobian(d, c3));
c = (d - (a*s3 + b*c3));

% Test the coefficients.
lhs = a*s3 + b*c3;
rhs = d-c;
if (simplify(lhs - rhs) ~= sym(0))
    disp('Error on qKFE terms: the linear trigonometric equation coefficients are wrong.');
    return;
end

% Generate the matlab files for the coefficients.
matlabFunction(a,'File','generated_fcns/trig_eq_kfe_a','Vars',{T_r_TS, S_r_SF});
matlabFunction(b,'File','generated_fcns/trig_eq_kfe_b','Vars',{T_r_TS, S_r_SF});
matlabFunction(c,'File','generated_fcns/trig_eq_kfe_c','Vars',{T_r_TS, S_r_SF});

% Now repeat the same procedure to get the terms needed to compute qHFE. We
% start from the norm of the vector B_r_HF

% Rewrite d in a simpler form.
d = simplify(norm(B_r_HF)^2);
d = collect(expand(d), [cos(q2) sin(q2)]);

% Find the coefficients of an equation in the from:
%       a*sin(q2) + b*cos(q2) + c = d;
syms c2 s2 'real';
d = simplify(subs(d, cos(q2), c2));
d = simplify(subs(d, sin(q2), s2));

a = simplify(jacobian(d, s2));
b = simplify(jacobian(d, c2));
c = simplify(d - (a*s2 + b*c2));

% Test the coefficients.
lhs = a*s2 + b*c2;
rhs = d-c;
if (simplify(lhs - rhs) ~= sym(0))
    disp('Error on qHFE terms: the linear trigonometric equation coefficients are wrong.');
    return;
end

matlabFunction(a,'File','generated_fcns/trig_eq_hfe_a','Vars',{q3, H_r_HT, T_r_TS, S_r_SF});
matlabFunction(b,'File','generated_fcns/trig_eq_hfe_b','Vars',{q3, H_r_HT, T_r_TS, S_r_SF});
matlabFunction(c,'File','generated_fcns/trig_eq_hfe_c','Vars',{q3, H_r_HT, T_r_TS, S_r_SF});

end