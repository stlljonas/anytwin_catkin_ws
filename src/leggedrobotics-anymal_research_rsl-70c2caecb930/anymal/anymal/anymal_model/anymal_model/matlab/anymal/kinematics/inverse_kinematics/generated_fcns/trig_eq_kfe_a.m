function a = trig_eq_kfe_a(in1,in2)
%TRIG_EQ_KFE_A
%    A = TRIG_EQ_KFE_A(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    16-Jul-2017 22:04:27

S_r_SF_x = in2(1,:);
S_r_SF_z = in2(3,:);
T_r_TS_x = in1(1,:);
T_r_TS_z = in1(3,:);
a = S_r_SF_x.*T_r_TS_z.*-2.0+S_r_SF_z.*T_r_TS_x.*2.0;
