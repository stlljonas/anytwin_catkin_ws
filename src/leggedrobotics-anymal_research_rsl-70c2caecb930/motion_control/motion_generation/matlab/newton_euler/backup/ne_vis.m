%% Setup

close all;
clearvars;

data.g_acc = 9.81;
data.g_vec = [0;0;-data.g_acc];
data.I_r_IG = [2;0;2];
data.I_ddr_IG = [2;0;0];

data.I_w_IG = [0;0;0];
data.I_dw_IG = [0;0;0];

data.m = 0.2;
data.inertia = eye(3);
data.n = [0;0;1];
data.weight = data.m*data.g_vec;

data.f_ext = [0;0;0];
data.I_r_GF = [1;1;0];

%% NE equations

data.ldot = data.inertia*data.I_dw_IG + cross(data.I_w_IG, data.inertia*data.I_w_IG);

data.f_gi = data.m*(data.g_vec - data.I_ddr_IG);
data.tau_gi_I = cross(data.I_r_IG, data.f_gi) - data.ldot;

data.I_r_IZ = cross(data.n,data.tau_gi_I)/(data.n.'*data.f_gi);


%% Visualizer
figure_handler.fh = figure('Position',[100 700 900 700]);
axes('Position',[0.1 0.3 0.7 0.7]);
data.figure_handler = figure_handler;
view(3);
fig_lim_min = -5;
fig_lim_max = 5;
xlim([fig_lim_min fig_lim_max]);
ylim([fig_lim_min fig_lim_max]);
zlim([fig_lim_min fig_lim_max]);
hold on; grid on;

sliders_pos_l = 150;
sliders_pos_b = 100;
sliders_pos_w = 400;
sliders_pos_h = 15;

% drop-down menu
popupmenu_contents = {'I_r_IG', 'I_ddr_IG', 'I_w_IG', 'f_ext'};
figure_handler.dd_menu = ...
    uicontrol('Parent',figure_handler.fh,'Style','popupmenu',...
              'UserData',data,...
              'String', popupmenu_contents,...
              'Callback', {@popupmenu_cb},...
              'Position',[sliders_pos_l-100,sliders_pos_b-25,100,sliders_pos_h]);

% mass slider
figure_handler.mass_slider = ...
    uicontrol('Parent',figure_handler.fh,'Style','slider',...
              'UserData',data,...
              'Position',[sliders_pos_l,sliders_pos_b+10,sliders_pos_w,sliders_pos_h],...
              'SliderStep', [0.005, 0.01],...
              'Callback', {@slider_mass_cb},...
              'value',data.m,'min',0,'max',1);
          
% com sliders
figure_handler.vec_x_slider = ...
    uicontrol('Parent',figure_handler.fh,'Style','slider',...
              'Position',[sliders_pos_l,sliders_pos_b-sliders_pos_h,sliders_pos_w,sliders_pos_h],...
              'SliderStep', [0.005, 0.01],...
              'Callback', {@slider_cb, 1},...
              'value',0,'min',-5,'max',5);
figure_handler.vec_y_slider = ...
    uicontrol('Parent',figure_handler.fh,'Style','slider',...
              'Position',[sliders_pos_l,sliders_pos_b-2*sliders_pos_h,sliders_pos_w,sliders_pos_h],...
              'SliderStep', [0.005, 0.01],...
              'Callback', {@slider_cb, 2},...
              'value',0,'min',-5,'max',5);
figure_handler.vec_z_slider = ...
    uicontrol('Parent',figure_handler.fh,'Style','slider',...
              'Position',[sliders_pos_l,sliders_pos_b-3*sliders_pos_h,sliders_pos_w,sliders_pos_h],...
              'SliderStep', [0.005, 0.01],...
              'Callback', {@slider_cb, 3},...
              'value',0,'min',-5,'max',5);

data = drawVectors(data);
data = update_ne(data);
          
%% Callbacks

function popupmenu_cb(hObject, ~)
  figure_handler = evalin('caller','figure_handler');
  popupmenu_contents = evalin('caller','popupmenu_contents');
  name = popupmenu_contents{get(figure_handler.dd_menu, 'Value')};
  vec = evalin('caller', ['data.' name ';']);
  figure_handler.vec_x_slider.Value = vec(1);
  figure_handler.vec_y_slider.Value = vec(2);
  figure_handler.vec_z_slider.Value = vec(3);
end

function slider_mass_cb(hObject, ~)
  evalin('caller', ['data.m = ' num2str(hObject.Value) ';']);
  evalin('caller', 'data = update_ne(data);');
  evalin('caller', 'updateHandles(data);');
end

function slider_cb(hObject, ~, dim)
  figure_handler = evalin('caller','figure_handler');
  popupmenu_contents = evalin('caller','popupmenu_contents');
  name = popupmenu_contents{get(figure_handler.dd_menu, 'Value')};
  evalin('caller', ['data.' name '(' num2str(dim) ') = ' num2str(hObject.Value) ';']);
  evalin('caller', 'data = update_ne(data);');
  evalin('caller', 'updateHandles(data);');
end

%% Draw
function data = drawVectors(data)
set(0,'CurrentFigure', data.figure_handler.fh);
cla(data.figure_handler.fh);

[x, y, z] = ellipsoid(0, 0, 0, 0.4, 0.2, 0.2);
data.h_mbody.hg_t = hgtransform;
data.h_mbody.h = surface(x, y, z, 'Parent', data.h_mbody.hg_t);
alpha(data.h_mbody.h, 0.3);
data.h_mbody.hg_t.Matrix = makehgtform('translate',data.I_r_IG(1),data.I_r_IG(2),data.I_r_IG(3));

% mg
data.h_weight.h = drawArrowAtPosition(data.I_r_IG, data.weight);
data.h_weight.label = drawTextAtPos(data.I_r_IG + data.weight, 'mg');

% ddr
data.h_ddr.h = drawArrowAtPosition(data.I_r_IG, data.m*data.I_ddr_IG);
data.h_ddr.label = drawTextAtPos(data.I_r_IG + data.m*data.I_ddr_IG, '$m\ddot{x}$');

% Gravito-inertial wrench
data.h_f_gi.h = drawArrowAtPosition(data.I_r_IG, data.f_gi);
data.h_f_gi.label =  drawTextAtPos(data.I_r_IG + data.f_gi, '$f^{gi}$');
data.h_tau_gi.h = drawArrowAtPosition(zeros(3,1), data.tau_gi_I);
data.h_tau_gi.label = drawTextAtPos(data.tau_gi_I, '$\tau^{gi}_I$');

data.h_f_ext.h = drawArrowAtPosition(data.I_r_IG + data.I_r_GF, data.f_ext);
data.h_f_ext.label = drawTextAtPos(data.I_r_IG + data.I_r_GF + data.f_ext, '$f^{ext}$');

% zmp
data.h_zmp.dot = plot3(data.I_r_IZ(1), data.I_r_IZ(2), data.I_r_IZ(3),'r','linewidth',2,'Marker','square');
data.h_zmp.h = drawArrowAtPosition(zeros(3,1), data.I_r_IZ);
data.h_zmp.label = drawTextAtPos(data.I_r_IZ, 'ZMP');

end

function data = update_ne(data)
data.weight = data.m*data.g_vec;

data.ldot = data.inertia*data.I_dw_IG + cross(data.I_w_IG, data.inertia*data.I_w_IG);
data.f_gi = data.m*(data.g_vec - data.I_ddr_IG) ...
          + data.f_ext;

data.tau_gi_I = cross(data.I_r_IG, data.f_gi) ...
              - data.ldot ...
              + cross(data.I_r_GF,data.f_ext);

data.I_r_IZ = cross(data.n,data.tau_gi_I)/(data.n.'*data.f_gi);

end

function updateHandles(data)
%   translate(data.h_mbody.h,data.I_r_IG);
  data.h_mbody.hg_t.Matrix = makehgtform('translate',data.I_r_IG(1),data.I_r_IG(2),data.I_r_IG(3));
  updateDataInHandle(data.h_weight, data.I_r_IG, data.weight);
  updateDataInHandle(data.h_ddr, data.I_r_IG, data.m*data.I_ddr_IG);
  updateDataInHandle(data.h_f_gi, data.I_r_IG, data.f_gi);
  updateDataInHandle(data.h_tau_gi, zeros(3,1), data.tau_gi_I);
  updateDataInHandle(data.h_zmp, zeros(3,1), data.I_r_IZ);
  set(data.h_zmp.dot,'xdata',data.I_r_IZ(1),'ydata',data.I_r_IZ(2),'zdata',data.I_r_IZ(3));
  
  updateDataInHandle(data.h_f_ext, data.I_r_IG + data.I_r_GF, data.f_ext);
end

function handle = drawArrowAtPosition(pos,vec)
   handle = quiver3(pos(1),pos(2),pos(3),...
                    vec(1),vec(2),vec(3),0,...
                    'linewidth',2);
end
function handle = drawTextAtPos(pos, str)
    handle = text(pos(1)+rand/5,pos(2)+rand/5,pos(3)+rand/5, ...
                  str, 'interpreter', 'latex');
end
function updateDataInHandle(dataHandle, pos, vec)
  set(dataHandle.h,...
      'xdata',pos(1),'ydata',pos(2),'zdata',pos(3),...
      'udata',vec(1),'vdata',vec(2),'wdata',vec(3));
  set(dataHandle.label,'Position',pos+vec);
end
