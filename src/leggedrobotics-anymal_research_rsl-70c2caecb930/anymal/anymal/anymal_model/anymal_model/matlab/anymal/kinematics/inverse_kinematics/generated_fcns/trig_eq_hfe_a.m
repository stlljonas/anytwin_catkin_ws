function a = trig_eq_hfe_a(q3,in2,in3,in4)
%TRIG_EQ_HFE_A
%    A = TRIG_EQ_HFE_A(Q3,IN2,IN3,IN4)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    16-Jul-2017 22:04:28

H_r_HT_x = in2(1,:);
H_r_HT_z = in2(3,:);
S_r_SF_x = in4(1,:);
S_r_SF_z = in4(3,:);
T_r_TS_x = in3(1,:);
T_r_TS_z = in3(3,:);
t2 = cos(q3);
t3 = sin(q3);
a = H_r_HT_x.*T_r_TS_z.*2.0-H_r_HT_z.*T_r_TS_x.*2.0-H_r_HT_x.*S_r_SF_x.*t3.*2.0+H_r_HT_x.*S_r_SF_z.*t2.*2.0-H_r_HT_z.*S_r_SF_x.*t2.*2.0-H_r_HT_z.*S_r_SF_z.*t3.*2.0;
