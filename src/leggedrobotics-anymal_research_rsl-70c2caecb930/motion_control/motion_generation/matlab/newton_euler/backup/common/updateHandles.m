function updateHandles(data)
%   translate(data.h_mbody.h,data.I_r_IG);
  data.h_mbody.hg_t.Matrix = makehgtform('translate',data.I_r_IG(1),data.I_r_IG(2),data.I_r_IG(3));
  updateDataInHandle(data.h_weight, data.I_r_IG, data.weight);
  updateDataInHandle(data.h_ddr, data.I_r_IG, data.m*data.I_ddr_IG);
  updateDataInHandle(data.h_f_gi, data.I_r_IG, data.f_gi);
  updateDataInHandle(data.h_tau_gi, zeros(3,1), data.tau_gi_I);
  
  data.I_r_IG_ground = [data.I_r_IG(1:2);0];
  updateDataInHandle(data.h_f_lf, data.I_r_IF_lf, data.f_lf);
  updateDataInHandle(data.h_f_rf, data.I_r_IF_rf, data.f_rf);
  updateDataInHandle(data.h_f_lh, data.I_r_IF_lh, data.f_lh);
  updateDataInHandle(data.h_f_rh, data.I_r_IF_rh, data.f_rh);

  
  updateDataInHandle(data.h_zmp, zeros(3,1), data.I_r_IZ);
  set(data.h_zmp.dot,'xdata',data.I_r_IZ(1),'ydata',data.I_r_IZ(2),'zdata',data.I_r_IZ(3));
  
  updateDataInHandle(data.h_f_ext, data.I_r_IG + data.I_r_GF, data.f_ext);
  
  updateDataInHandle(data.h_tau_gi_Z, data.I_r_IZ, data.tau_gi_Z);
  
end