function c = trig_eq_hfe_c(q3,in2,in3,in4)
%TRIG_EQ_HFE_C
%    C = TRIG_EQ_HFE_C(Q3,IN2,IN3,IN4)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    16-Jul-2017 22:04:29

H_r_HT_x = in2(1,:);
H_r_HT_y = in2(2,:);
H_r_HT_z = in2(3,:);
S_r_SF_x = in4(1,:);
S_r_SF_y = in4(2,:);
S_r_SF_z = in4(3,:);
T_r_TS_x = in3(1,:);
T_r_TS_y = in3(2,:);
T_r_TS_z = in3(3,:);
t2 = cos(q3);
t3 = sin(q3);
c = S_r_SF_y.*T_r_TS_y.*2.0+H_r_HT_x.^2+H_r_HT_y.^2+H_r_HT_z.^2+S_r_SF_x.^2+S_r_SF_y.^2+S_r_SF_z.^2+T_r_TS_x.^2+T_r_TS_y.^2+T_r_TS_z.^2+H_r_HT_y.*S_r_SF_y.*2.0+H_r_HT_y.*T_r_TS_y.*2.0+S_r_SF_x.*T_r_TS_x.*t2.*2.0-S_r_SF_x.*T_r_TS_z.*t3.*2.0+S_r_SF_z.*T_r_TS_x.*t3.*2.0+S_r_SF_z.*T_r_TS_z.*t2.*2.0;
