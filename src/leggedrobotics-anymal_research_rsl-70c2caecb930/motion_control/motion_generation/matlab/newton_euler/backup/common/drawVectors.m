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

data.h_tau_gi_Z.h = drawArrowAtPosition(data.I_r_IZ, data.tau_gi_Z);
data.h_tau_gi_Z.label = drawTextAtPos(data.tau_gi_Z, '$\tau^{gi}_Z$');

data.h_f_ext.h = drawArrowAtPosition(data.I_r_IG + data.I_r_GF, data.f_ext);
data.h_f_ext.label = drawTextAtPos(data.I_r_IG + data.I_r_GF + data.f_ext, '$f^{ext}$');

% zmp
data.h_zmp.dot = plot3(data.I_r_IZ(1), data.I_r_IZ(2), data.I_r_IZ(3),'r','linewidth',2,'Marker','square');
data.h_zmp.h = drawArrowAtPosition(zeros(3,1), data.I_r_IZ);
data.h_zmp.label = drawTextAtPos(data.I_r_IZ, 'ZMP');

% reaction forces
data.h_f_lf.h = drawArrowAtPosition(data.I_r_IF_lf, data.f_lf);
data.h_f_lf.label = drawTextAtPos(data.I_r_IF_lf + data.f_lf, '$f_{LF}$');

data.h_f_rf.h = drawArrowAtPosition(data.I_r_IF_rf, data.f_rf);
data.h_f_rf.label = drawTextAtPos(data.I_r_IF_rf + data.f_rf, '$f_{RF}$');

data.h_f_lh.h = drawArrowAtPosition(data.I_r_IF_lh, data.f_lh);
data.h_f_lh.label = drawTextAtPos(data.I_r_IF_lh + data.f_lh, '$f_{LH}$');

data.h_f_rh.h = drawArrowAtPosition(data.I_r_IF_rh, data.f_rh);
data.h_f_rh.label = drawTextAtPos(data.I_r_IF_rh + data.f_rh, '$f_{RH}$');

end