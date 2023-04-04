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


syms i11 i12 i13 i22 i23 i33 'real';
inertia = [i11 i12 i13;
           i12 i22 i23;
           i13 i23 i33];
       


I_C = getMapEulAngZYXDiffToAngVelInWorldFrame(al_zyx);
I_Cdot = dMATdt(I_C,al_zyx,aldot_zyx);

% angular velocity and acceleration
w = [wx;wy;wz];
wdot = [dwx;dwy;dwz];
w_from_al = I_C*aldot_zyx;
wdot_from_al = I_C*alddot_zyx + I_Cdot*aldot_zyx;

% l dot
ldot_from_omega = inertia*wdot + cross(w, inertia*w);
ldot_from_al_dal = simplify(inertia*wdot_from_al + cross(w_from_al, inertia*w_from_al));
                  

%%

% 
% B_C = getMapEulAngZYXDiffToAngVelInBaseFrame(al_zyx);
% Rx = getRotationMatrixX(al_zyx(3));
% Ry = getRotationMatrixY(al_zyx(2));
% Rz = getRotationMatrixZ(al_zyx(1));
% R_IB = mapEulerAnglesZYXToRotationMatrix(al_zyx);
% dR_IB = getRotationMatrixDerivativeWrtEulerAnglesZyx(al_zyx);
% 
% w_al_dal = I_C*aldot_zyx;
% dwdal = simplify(jacobian(w_al_dal,al_zyx));
% 
% 
%  
% dB_C = simplify((diff(B_C,al_zyx(1))*aldot_zyx(1)+...
%                  diff(B_C,al_zyx(2))*aldot_zyx(2)+...
%                  diff(B_C,al_zyx(3))*aldot_zyx(3)));
% 
% dI_Cdt = dMATdt(I_C,al_zyx,aldot_zyx);

%% ldot / alddot

% from matlab
partial_ldot_alddot_sym = simplify(jacobian(ldot_from_al_dal, alddot_zyx));

% analytic
partial_ldot_wdot = inertia;
partial_wdot_alddot = I_C; 

% In conclusion...
partial_ldot_alddot_analytic = simplify(partial_ldot_wdot * partial_wdot_alddot);

%% ldot/aldot

% from matlab
partial_ldot_aldot_sym = simplify(jacobian(ldot_from_al_dal, aldot_zyx));

% analytic
partial_ldot_w = -skew(inertia*w_from_al)+skew(w_from_al)*inertia;
partial_w_aldot = I_C;
partial_wdot_alddot = I_C;
partial_ldot_wdot = inertia;


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
%     I_Cdot_analytical(:,k) = reshape(partial_I_C_al(:,k,:),3,3)*aldot_zyx;
    I_Cdot_analytical = I_Cdot_analytical +  reshape(partial_I_C_al(:,:,k),3,3)*aldot_zyx(k);
end

% I_Cdot_analytical_other = sym(zeros(dim_w,dim_dal));
% for k=1:dim_dal
% %     I_Cdot_analytical(:,k) = reshape(partial_I_C_al(:,k,:),3,3)*aldot_zyx;
%     I_Cdot_analytical_other = I_Cdot_analytical_other +  reshape(partial_I_C_al(:,k,:),3,3)*aldot_zyx(k);
% end

partial_Cdottimesaldot_aldot_analytical = sym(zeros(dim_w,dim_dal));
sumOfPartials = simplify(partial_I_C_al + partial_I_Ctranspose_al);
for k=1:dim_dal
    partial_Cdottimesaldot_aldot_analytical = partial_Cdottimesaldot_aldot_analytical + ...
        sumOfPartials(:,:,k)*aldot_zyx(k);
end

partial_Cdottimesaldot_aldot_sym = ...
    simplify(jacobian(I_Cdot*aldot_zyx, aldot_zyx));

% dw_da = reshape(partial_I_C_al(:,1,:),3,3)*aldot_zyx(1) + ...
%         reshape(partial_I_C_al(:,2,:),3,3)*aldot_zyx(2) + ...
%         reshape(partial_I_C_al(:,3,:),3,3)*aldot_zyx(3);
% 
% ddal_dal = simplify(ddl_dw*dw_da);

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

%% Inertia tensor

% R_PB = getRotationMatrixZ(alddot_zyx(1))*...
%        getRotationMatrixY(alddot_zyx(2))*...
%        getRotationMatrixX(alddot_zyx(3));
% 
% B_inertia = R_PB*inertia*
       
%% sandbox

% pseudomat = alddot_zyx*alddot_zyx.';
% 
% testmat = sym(zeros(3,3));
% for k=1:dim_al
%     testmat = testmat + ...
%     reshape(partial_I_C_al(k,:,:),3,3)*pseudomat;
% end
% simplify(testmat-I_Cdot)