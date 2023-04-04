%% Setup
close all;
clearvars;

data.g_acc = 9.81;
data.g_vec = [0;0;-data.g_acc];
data.I_r_IG = [2;0;2];
data.I_r_IG_ground = [data.I_r_IG(1:2);0];

data.I_w_IG = [0;0;0];

data.m = 0.1;
data.inertia = eye(3);
data.n = [0;0;1];
data.weight = data.m*data.g_vec;

% External force
data.f_ext = [0;0;0];
data.I_r_GF = [0;0;0];

% Reaction forces
init_force = -data.m*data.g_vec/4 + [0.6;0;0];
data.f_lf = init_force;
data.I_r_IF_lf = data.I_r_IG_ground + [1;1;0];
data.I_r_GF_lf = data.I_r_IF_lf - data.I_r_IG;

data.f_rf = init_force;
data.I_r_IF_rf =  data.I_r_IG_ground +[1;-1;0];
data.I_r_GF_rf =  data.I_r_IF_rf - data.I_r_IG;

data.f_lh = init_force;
data.I_r_IF_lh = data.I_r_IG_ground + [-1;1;0];
data.I_r_GF_lh = data.I_r_IF_lh - data.I_r_IG;

data.f_rh = init_force;
data.I_r_IF_rh = data.I_r_IG_ground + [-1;-1;0];
data.I_r_GF_rh = data.I_r_IF_rh - data.I_r_IG;

% Update data using Newton-Euler equations.
data = update_ne(data);

% Setup visualization
popupmenu_contents = {'I_r_IG', 'I_w_IG', 'f_ext', 'f_lf', 'f_rf', 'f_lh', 'f_rh'};
[figure_handler, data] = setupVisualization(data, popupmenu_contents);