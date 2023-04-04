%%

% Find dw/dal
% al --> Euler angles ZYX
syms ph th ps 'real'
syms dph dth dps 'real'
syms ddph ddth ddps 'real'

syms wx wy wz 'real'
syms dwx dwy dwz 'real'
syms ddwx ddwy ddwz 'real'

syms a 'real'

% al_zyx = [ps;a;a]; %[ps;th;ph];
% dal_zyx = [dps;a;a]; %[dps;dth;dph];
al_zyx = [ps;th;ph];
aldot_zyx = [dps;dth;dph];
alddot_zyx = [ddps;ddth;ddph];


syms B_i11 B_i12 B_i13 B_i22 B_i23 B_i33 'real';
syms I_i11 I_i12 I_i13 I_i22 I_i23 I_i33 'real';
R_PB = getRotationMatrixZ(al_zyx(1))*...
       getRotationMatrixY(al_zyx(2))*...
       getRotationMatrixX(al_zyx(3));

B_inertia = [B_i11 B_i12 B_i13; B_i12 B_i22 B_i23; B_i13 B_i23 B_i33];
% keep I_inertia symbolic for now
%I_inertia = [I_i11 I_i12 I_i13; I_i12 I_i22 I_i23; I_i13 I_i23 I_i33];
I_inertia = simplify( R_PB * B_inertia * R_PB.' );

I_C = getMapEulAngZYXDiffToAngVelInWorldFrame(al_zyx);
I_Cdot = dMATdt(I_C,al_zyx,aldot_zyx);

% angular velocity and acceleration
w = [wx;wy;wz];
wdot = [dwx;dwy;dwz];
w_from_al = I_C*aldot_zyx;
wdot_from_al = I_C*alddot_zyx + I_Cdot*aldot_zyx;

% l dot
ldot_from_omega = simplify(I_inertia*wdot + cross(w, I_inertia*w));
ldot_from_al_dal = simplify(I_inertia*wdot_from_al + cross(w_from_al, I_inertia*w_from_al));

%% ldot / alddot

% from matlab
partial_ldot_alddot_sym = simplify(jacobian(ldot_from_al_dal, alddot_zyx));

% analytic
partial_ldot_wdot = I_inertia;
partial_wdot_alddot = I_C; 

% In conclusion...
partial_ldot_alddot_analytic = simplify(partial_ldot_wdot * partial_wdot_alddot);

%% ldot/aldot

% from matlab
partial_ldot_aldot_sym = simplify(jacobian(ldot_from_al_dal, aldot_zyx));

% analytic
partial_ldot_w = -skew(I_inertia*w_from_al)+skew(w_from_al)*I_inertia;
partial_w_aldot = I_C;
partial_wdot_alddot = I_C;
partial_ldot_wdot = I_inertia;


dim_w = 3;
dim_al = 3;
dim_dal = 3;
                        
partial_I_C_al = sym(zeros(dim_w,dim_dal,dim_al));
for k=1:dim_al
    % Every frontal face of the tensor is the derivative of C(al) w.r.t.
    % al_k
    partial_I_C_al(:,:,k) = diff(I_C,al_zyx(k));
end

partial_I_Ctranspose_al = sym(zeros(dim_w,dim_dal,dim_al));
for k=1:dim_al
    % Every frontal face of the tensor is the derivative of C(al) w.r.t.
    % al_k
    partial_I_Ctranspose_al(:,:,k) = partial_I_C_al(:,k,:);
end

I_Cdot_analytical = sym(zeros(dim_w,dim_dal));
for k=1:dim_dal
    I_Cdot_analytical = I_Cdot_analytical +  reshape(partial_I_C_al(:,:,k),3,3)*aldot_zyx(k);
end

partial_Cdottimesaldot_aldot_analytical = sym(zeros(dim_w,dim_dal));
sumOfPartials = simplify(partial_I_C_al + partial_I_Ctranspose_al);
for k=1:dim_dal
    partial_Cdottimesaldot_aldot_analytical = partial_Cdottimesaldot_aldot_analytical + ...
        sumOfPartials(:,:,k)*aldot_zyx(k);
end

partial_Cdottimesaldot_aldot_sym = ...
    simplify(jacobian(I_Cdot*aldot_zyx, aldot_zyx));

partial_wdot_aldot = partial_Cdottimesaldot_aldot_analytical;

% In conclusion...
partial_ldot_aldot_analytic = partial_ldot_w*partial_w_aldot ...
                            + partial_ldot_wdot*partial_wdot_aldot;

%% ldot / al
partial_ldot_al_sym = simplify(jacobian(ldot_from_al_dal,al_zyx));

% Start searching for the analytical answer
partial_w_al_sym = simplify(jacobian(w_from_al,al_zyx));

% The partial of omega w.r.t. alpha is:
%    dw     dw     dC
%   ---- = ---- * ----
%    da     dC     da
%
% dC/da is computed in the previous section. dw/dC is adot.'
%
partial_w_al_analytical = sym(zeros(dim_w,dim_dal));
for k=1:dim_dal
    partial_w_al_analytical = partial_w_al_analytical + ...
        aldot_zyx(k)*reshape(partial_I_C_al(:,k,:),3,3);
end

% Search for
%    dS(w)
%   -------
%     da
partial_Sw_al_sym = sym(zeros(3,3,3));
for k=1:dim_al
    partial_Sw_al_sym(:,:,k) = simplify(diff(skew(w_from_al),al_zyx(k)));
end
partial_Sw_al_analytic = skewMatSym(partial_w_al_analytical);

% Search for
%    d wdot
%   --------
%      da
partial_wdot_al_sym = simplify(jacobian(wdot_from_al,al_zyx));

hessian_I_C_al = sym(zeros(3,3,3,3));
for k=1:dim_al
    hessian_I_C_al(:,:,:,k) = simplify(diff(partial_I_C_al,al_zyx(k)));
end

reduced1 = sym(zeros(3,3,3));
for k=1:dim_al
    reduced1(:,:,k) = diff(tensorSym3dProduct(partial_I_C_al,aldot_zyx,3),al_zyx(k));
end
partial_wdot_al_analytic = ...
    simplify(tensorSym3dProduct(reduced1, aldot_zyx,2) ...
           + tensorSym3dProduct(partial_I_C_al,alddot_zyx,2));

%% Inertia tensor setup

I_inertia = simplify(R_PB*B_inertia*R_PB.');

partial_Iinertia_al_sym = sym(zeros(3,3,3));
for k=1:3
    partial_Iinertia_al_sym(:,:,k) = simplify(diff(I_inertia,al_zyx(k)));
end
       
%% Inertia tensor

partial_Rpb_dal_sym = sym(zeros(3,3,3));
for k=1:3
    partial_Rpb_dal_sym(:,:,k) = simplify(diff(R_PB,al_zyx(k)));
end
partial_RpbT_dal_sym = sym(zeros(3,3,3));
for k=1:3
    partial_RpbT_dal_sym(:,:,k) = simplify(diff(R_PB.',al_zyx(k)));
end

partial_Rpb_dal_analytic = sym(zeros(3,3,3));
partial_Rpb_dal_analytic(:,:,1) = skew(sym([0;0;1]))*R_PB;
partial_Rpb_dal_analytic(:,:,2) = simplify(skew(getRotationMatrixZ(al_zyx(1))*sym([0;1;0]))*R_PB);
partial_Rpb_dal_analytic(:,:,3) = simplify(skew(...
    getRotationMatrixZ(al_zyx(1))*getRotationMatrixY(al_zyx(2))*sym([1;0;0]))*R_PB);

partial_RpbT_dal_analytic = sym(zeros(3,3,3));
partial_RpbT_dal_analytic(:,:,1) = R_PB.'*skew(sym([0;0;1])).';
partial_RpbT_dal_analytic(:,:,2) = simplify(...
    R_PB.'*skew(getRotationMatrixZ(al_zyx(1))*sym([0;1;0])).'...
    );
partial_RpbT_dal_analytic(:,:,3) = simplify(...
    R_PB.'*skew(getRotationMatrixZ(al_zyx(1))*getRotationMatrixY(al_zyx(2))*sym([1;0;0])).' ...
    );


partialI_test = productTensorMat(partial_Rpb_dal_analytic,B_inertia*R_PB.',2) ...
              + productMatTensor(R_PB*B_inertia,partial_RpbT_dal_analytic,2);
