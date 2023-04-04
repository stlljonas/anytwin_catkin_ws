%% Plot joint positions
h = figure();
set(h, 'Name', 'Measured and desired joint positions');
grid on
subplot(3,4,1)
hold on
plot(logElem(idx_command_jointPosition_LF_HAA).time, logElem(idx_command_jointPosition_LF_HAA).data,'r.-')
plot(logElem(idx_state_jointPos_LF_HAA).time, logElem(idx_state_jointPos_LF_HAA).data,'b.-')
title('LF')
grid on
xlabel('time [s]')
ylabel('HAA [rad]')

subplot(3,4,5)
hold on
plot(logElem(idx_command_jointPosition_LF_HFE).time, logElem(idx_command_jointPosition_LF_HFE).data,'r.-')
plot(logElem(idx_state_jointPos_LF_HFE).time, logElem(idx_state_jointPos_LF_HFE).data,'b.-')
grid on
xlabel('time [s]')
ylabel('HFE [rad]')

subplot(3,4,9)
hold on
plot(logElem(idx_command_jointPosition_LF_KFE).time, logElem(idx_command_jointPosition_LF_KFE).data,'r.-')
plot(logElem(idx_state_jointPos_LF_KFE).time, logElem(idx_state_jointPos_LF_KFE).data,'b.-')
grid on
xlabel('time [s]')
ylabel('KFE [rad]')

subplot(3,4,2)
hold on
plot(logElem(idx_command_jointPosition_RF_HAA).time, logElem(idx_command_jointPosition_RF_HAA).data,'r.-')
plot(logElem(idx_state_jointPos_RF_HAA).time, logElem(idx_state_jointPos_RF_HAA).data,'b.-')
title('RF')
grid on
xlabel('time [s]')
ylabel('HAA [rad]')

subplot(3,4,6)
hold on
plot(logElem(idx_command_jointPosition_RF_HFE).time, logElem(idx_command_jointPosition_RF_HFE).data,'r.-')
plot(logElem(idx_state_jointPos_RF_HFE).time, logElem(idx_state_jointPos_RF_HFE).data,'b.-')
grid on
xlabel('time [s]')
ylabel('HFE [rad]')

subplot(3,4,10)
hold on
plot(logElem(idx_command_jointPosition_RF_KFE).time, logElem(idx_command_jointPosition_RF_KFE).data,'r.-')
plot(logElem(idx_state_jointPos_RF_KFE).time, logElem(idx_state_jointPos_RF_KFE).data,'b.-')
grid on
xlabel('time [s]')
ylabel('KFE [rad]')

subplot(3,4,3)
hold on
plot(logElem(idx_command_jointPosition_LH_HAA).time, logElem(idx_command_jointPosition_LH_HAA).data,'r.-')
plot(logElem(idx_state_jointPos_LH_HAA).time, logElem(idx_state_jointPos_LH_HAA).data,'b.-')
title('LH')
grid on
xlabel('time [s]')
ylabel('HAA [rad]')

subplot(3,4,7)
hold on
plot(logElem(idx_command_jointPosition_LH_HFE).time, logElem(idx_command_jointPosition_LH_HFE).data,'r.-')
plot(logElem(idx_state_jointPos_LH_HFE).time, logElem(idx_state_jointPos_LH_HFE).data,'b.-')
grid on
xlabel('time [s]')
ylabel('HFE [rad]')

subplot(3,4,11)
hold on
plot(logElem(idx_command_jointPosition_LH_KFE).time, logElem(idx_command_jointPosition_LH_KFE).data,'r.-')
plot(logElem(idx_state_jointPos_LH_KFE).time, logElem(idx_state_jointPos_LH_KFE).data,'b.-')
grid on
xlabel('time [s]')
ylabel('KFE [rad]')

subplot(3,4,4)
hold on
plot(logElem(idx_command_jointPosition_RH_HAA).time, logElem(idx_command_jointPosition_RH_HAA).data,'r.-')
plot(logElem(idx_state_jointPos_RH_HAA).time, logElem(idx_state_jointPos_RH_HAA).data,'b.-')
title('RH')
grid on
xlabel('time [s]')
ylabel('HAA [rad]')

subplot(3,4,8)
hold on
plot(logElem(idx_command_jointPosition_RH_HFE).time, logElem(idx_command_jointPosition_RH_HFE).data,'r.-')
plot(logElem(idx_state_jointPos_RH_HFE).time, logElem(idx_state_jointPos_RH_HFE).data,'b.-')
grid on
xlabel('time [s]')
ylabel('HFE [rad]')

subplot(3,4,12)
hold on
plot(logElem(idx_command_jointPosition_RH_KFE).time, logElem(idx_command_jointPosition_RH_KFE).data,'r.-')
plot(logElem(idx_state_jointPos_RH_KFE).time, logElem(idx_state_jointPos_RH_KFE).data,'b.-')
grid on
xlabel('time [s]')
ylabel('KFE [rad]')

%%
h = figure();
set(h, 'Name', 'Measured and desired joint velocities');
grid on

subplot(3,4,1)
hold on
plot(logElem(idx_command_jointVelocity_LF_HAA).time, logElem(idx_command_jointVelocity_LF_HAA).data,'r.-')
plot(logElem(idx_state_jointVel_LF_HAA).time, logElem(idx_state_jointVel_LF_HAA).data,'b.-')
title('LF')
grid on
xlabel('time [s]')
ylabel('HAA [rad/s]')

subplot(3,4,5)
hold on
plot(logElem(idx_command_jointVelocity_LF_HFE).time, logElem(idx_command_jointVelocity_LF_HFE).data,'r.-')
plot(logElem(idx_state_jointVel_LF_HFE).time, logElem(idx_state_jointVel_LF_HFE).data,'b.-')
grid on
xlabel('time [s]')
ylabel('HFE [rad/s]')

subplot(3,4,9)
hold on
plot(logElem(idx_command_jointVelocity_LF_KFE).time, logElem(idx_command_jointVelocity_LF_KFE).data,'r.-')
plot(logElem(idx_state_jointVel_LF_KFE).time, logElem(idx_state_jointVel_LF_KFE).data,'b.-')
grid on
xlabel('time [s]')
ylabel('KFE [rad/s]')

subplot(3,4,2)
hold on
plot(logElem(idx_command_jointVelocity_RF_HAA).time, logElem(idx_command_jointVelocity_RF_HAA).data,'r.-')
plot(logElem(idx_state_jointVel_RF_HAA).time, logElem(idx_state_jointVel_RF_HAA).data,'b.-')
title('RF')
grid on
xlabel('time [s]')
ylabel('HAA [rad/s]')

subplot(3,4,6)
hold on
plot(logElem(idx_command_jointVelocity_RF_HFE).time, logElem(idx_command_jointVelocity_RF_HFE).data,'r.-')
plot(logElem(idx_state_jointVel_RF_HFE).time, logElem(idx_state_jointVel_RF_HFE).data,'b.-')
grid on
xlabel('time [s]')
ylabel('HFE [rad/s]')

subplot(3,4,10)
hold on
plot(logElem(idx_command_jointVelocity_RF_KFE).time, logElem(idx_command_jointVelocity_RF_KFE).data,'r.-')
plot(logElem(idx_state_jointVel_RF_KFE).time, logElem(idx_state_jointVel_RF_KFE).data,'b.-')
grid on
xlabel('time [s]')
ylabel('KFE [rad/s]')

subplot(3,4,3)
hold on
plot(logElem(idx_command_jointVelocity_LH_HAA).time, logElem(idx_command_jointVelocity_LH_HAA).data,'r.-')
plot(logElem(idx_state_jointVel_LH_HAA).time, logElem(idx_state_jointVel_LH_HAA).data,'b.-')
title('LH')
grid on
xlabel('time [s]')
ylabel('HAA [rad/s]')

subplot(3,4,7)
hold on
plot(logElem(idx_command_jointVelocity_LH_HFE).time, logElem(idx_command_jointVelocity_LH_HFE).data,'r.-')
plot(logElem(idx_state_jointVel_LH_HFE).time, logElem(idx_state_jointVel_LH_HFE).data,'b.-')
grid on
xlabel('time [s]')
ylabel('HFE [rad/s]')

subplot(3,4,11)
hold on
plot(logElem(idx_command_jointVelocity_LH_KFE).time, logElem(idx_command_jointVelocity_LH_KFE).data,'r.-')
plot(logElem(idx_state_jointVel_LH_KFE).time, logElem(idx_state_jointVel_LH_KFE).data,'b.-')
grid on
xlabel('time [s]')
ylabel('KFE [rad/s]')

subplot(3,4,4)
hold on
plot(logElem(idx_command_jointVelocity_RH_HAA).time, logElem(idx_command_jointVelocity_RH_HAA).data,'r.-')
plot(logElem(idx_state_jointVel_RH_HAA).time, logElem(idx_state_jointVel_RH_HAA).data,'b.-')
title('RH')
grid on
xlabel('time [s]')
ylabel('HAA [rad/s]')

subplot(3,4,8)
hold on
plot(logElem(idx_command_jointVelocity_RH_HFE).time, logElem(idx_command_jointVelocity_RH_HFE).data,'r.-')
plot(logElem(idx_state_jointVel_RH_HFE).time, logElem(idx_state_jointVel_RH_HFE).data,'b.-')
grid on
xlabel('time [s]')
ylabel('HFE [rad/s]')

subplot(3,4,12)
hold on
plot(logElem(idx_command_jointVelocity_RH_KFE).time, logElem(idx_command_jointVelocity_RH_KFE).data,'r.-')
plot(logElem(idx_state_jointVel_RH_KFE).time, logElem(idx_state_jointVel_RH_KFE).data,'b.-')
grid on
xlabel('time [s]')
ylabel('KFE [rad/s]')


%% Jont torques

h = figure();
set(h, 'Name', 'Measured and desired joint torque');
grid on

subplot(3,4,1)
hold on
plot(logElem(idx_command_jointTorque_LF_HAA).time, logElem(idx_command_jointTorque_LF_HAA).data,'r.-')
plot(logElem(idx_state_jointTor_LF_HAA).time, logElem(idx_state_jointTor_LF_HAA).data,'b.-')
title('LF')
grid on
xlabel('time [s]')
ylabel('HAA [Nm]')

subplot(3,4,5)
hold on
plot(logElem(idx_command_jointTorque_LF_HFE).time, logElem(idx_command_jointTorque_LF_HFE).data,'r.-')
plot(logElem(idx_state_jointTor_LF_HFE).time, logElem(idx_state_jointTor_LF_HFE).data,'b.-')
grid on
xlabel('time [s]')
ylabel('HFE [Nm]')

subplot(3,4,9)
hold on
plot(logElem(idx_command_jointTorque_LF_KFE).time, logElem(idx_command_jointTorque_LF_KFE).data,'r.-')
plot(logElem(idx_state_jointTor_LF_KFE).time, logElem(idx_state_jointTor_LF_KFE).data,'b.-')
grid on
xlabel('time [s]')
ylabel('KFE [Nm]')

subplot(3,4,2)
hold on
plot(logElem(idx_command_jointTorque_RF_HAA).time, logElem(idx_command_jointTorque_RF_HAA).data,'r.-')
plot(logElem(idx_state_jointTor_RF_HAA).time, logElem(idx_state_jointTor_RF_HAA).data,'b.-')
title('RF')
grid on
xlabel('time [s]')
ylabel('HAA [Nm]')

subplot(3,4,6)
hold on
plot(logElem(idx_command_jointTorque_RF_HFE).time, logElem(idx_command_jointTorque_RF_HFE).data,'r.-')
plot(logElem(idx_state_jointTor_RF_HFE).time, logElem(idx_state_jointTor_RF_HFE).data,'b.-')
grid on
xlabel('time [s]')
ylabel('HFE [Nm]')

subplot(3,4,10)
hold on
plot(logElem(idx_command_jointTorque_RF_KFE).time, logElem(idx_command_jointTorque_RF_KFE).data,'r.-')
plot(logElem(idx_state_jointTor_RF_KFE).time, logElem(idx_state_jointTor_RF_KFE).data,'b.-')
grid on
xlabel('time [s]')
ylabel('KFE [Nm]')

subplot(3,4,3)
hold on
plot(logElem(idx_command_jointTorque_LH_HAA).time, logElem(idx_command_jointTorque_LH_HAA).data,'r.-')
plot(logElem(idx_state_jointTor_LH_HAA).time, logElem(idx_state_jointTor_LH_HAA).data,'b.-')
title('LH')
grid on
xlabel('time [s]')
ylabel('HAA [Nm]')

subplot(3,4,7)
hold on
plot(logElem(idx_command_jointTorque_LH_HFE).time, logElem(idx_command_jointTorque_LH_HFE).data,'r.-')
plot(logElem(idx_state_jointTor_LH_HFE).time, logElem(idx_state_jointTor_LH_HFE).data,'b.-')
grid on
xlabel('time [s]')
ylabel('HFE [Nm]')

subplot(3,4,11)
hold on
plot(logElem(idx_command_jointTorque_LH_KFE).time, logElem(idx_command_jointTorque_LH_KFE).data,'r.-')
plot(logElem(idx_state_jointTor_LH_KFE).time, logElem(idx_state_jointTor_LH_KFE).data,'b.-')
grid on
xlabel('time [s]')
ylabel('KFE [Nm]')

subplot(3,4,4)
hold on
plot(logElem(idx_command_jointTorque_RH_HAA).time, logElem(idx_command_jointTorque_RH_HAA).data,'r.-')
plot(logElem(idx_state_jointTor_RH_HAA).time, logElem(idx_state_jointTor_RH_HAA).data,'b.-')
title('RH')
grid on
xlabel('time [s]')
ylabel('HAA [Nm]')

subplot(3,4,8)
hold on
plot(logElem(idx_command_jointTorque_RH_HFE).time, logElem(idx_command_jointTorque_RH_HFE).data,'r.-')
plot(logElem(idx_state_jointTor_RH_HFE).time, logElem(idx_state_jointTor_RH_HFE).data,'b.-')
grid on
xlabel('time [s]')
ylabel('HFE [Nm]')

subplot(3,4,12)
hold on
plot(logElem(idx_command_jointTorque_RH_KFE).time, logElem(idx_command_jointTorque_RH_KFE).data,'r.-')
plot(logElem(idx_state_jointTor_RH_KFE).time, logElem(idx_state_jointTor_RH_KFE).data,'b.-')
grid on
xlabel('time [s]')
ylabel('KFE [Nm]')
