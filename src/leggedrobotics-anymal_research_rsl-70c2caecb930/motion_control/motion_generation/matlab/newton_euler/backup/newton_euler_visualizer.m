%% Setup

close all;
clear all;

g_acc = 9.81;
g_vec = [0;0;-g_acc];

I_r_IG = [2;2;2];
I_ddr_IG = [1;0;0];
I_w_IG = [0;0;3];
I_dw_IG = [0;0;1];
m = 0.1;
inertia = eye(3);
n = [0;0;1];

%% NE equations

ldot = inertia*I_dw_IG + cross(I_w_IG, inertia*I_w_IG);

f_gi = m*(g_vec - I_ddr_IG);
tau_gi_I = cross(I_r_IG, f_gi) - ldot;

I_r_IZ = cross(n,tau_gi_I)/(n.'*f_gi);

%% Visualizer
figure_handler.fh = figure;
fig_lim_min = -4;
fig_lim_max = 5;
xlim([fig_lim_min fig_lim_max]);
ylim([fig_lim_min fig_lim_max]);
zlim([fig_lim_min fig_lim_max]);
hold on; grid on;

% main body
main_body_dim = [1;1;0.5];
main_body_origin = I_r_IG - 0.5*main_body_dim;       % Get the origin of cube so that P is at center 
plotcube(main_body_dim.', main_body_origin.', .3, [1 0 0]);   % use function plotcube 

%% Visualize com
drawArrow = @(arrow_start,arrow_end)quiver3(arrow_start(1),arrow_start(2),arrow_start(3),...
                                            arrow_end(1),arrow_end(2),arrow_end(3),0,...
                                            'linewidth',2);     
drawArrowAtPosition = @(pos,vec)...
    quiver3(pos(1),pos(2),pos(3),...
            vec(1),vec(2),vec(3),0,...
            'linewidth',2);
drawTextAtPos = @(pos, str)...
    text(pos(1)+rand/5,pos(2)+rand/5,pos(3)+rand/5, str, 'interpreter', 'latex');

% f_gi
drawArrowAtPosition(I_r_IG,m*g_vec);
drawTextAtPos(I_r_IG+f_gi, 'mg');

% ddr
drawArrowAtPosition(I_r_IG,I_ddr_IG);
drawTextAtPos(I_r_IG+I_ddr_IG, '$\ddot{x}$');

% Gravito-inertial wrench
drawArrowAtPosition(I_r_IG,f_gi);
drawTextAtPos(I_r_IG+f_gi, '$f^{gi}$');
drawArrowAtPosition(zeros(3,1),tau_gi_I);
drawTextAtPos(tau_gi_I, '$\tau^{gi}_I$');

% zmp
plot3(I_r_IZ(1),I_r_IZ(2),I_r_IZ(3),'r','linewidth',2,'Marker','square');
drawTextAtPos(I_r_IZ, 'ZMP');

