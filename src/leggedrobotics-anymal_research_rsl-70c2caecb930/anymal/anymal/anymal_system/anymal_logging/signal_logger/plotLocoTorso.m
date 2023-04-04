h = figure();
set(h, 'Name', 'torso position expressed in control frame');
subplot(3,1,1)
hold on
plot(logElem(idx_loco_torso_measPositionControlToBaseInControlFrame_x).time, logElem(idx_loco_torso_measPositionControlToBaseInControlFrame_x).data, 'b.-')
%plot(logElem(idx_loco_whole_body_positionControlToComInControlFrame_x).time, logElem(idx_loco_whole_body_positionControlToComInControlFrame_x).data, 'k.-')
plot(logElem(idx_loco_torso_desPositionControlToTargetInControlFrame_x).time, logElem(idx_loco_torso_desPositionControlToTargetInControlFrame_x).data, 'r.-')

grid on
xlabel('time [s]')
ylabel('x [m]')
subplot(3,1,2)
hold on
plot(logElem(idx_loco_torso_measPositionControlToBaseInControlFrame_y).time, logElem(idx_loco_torso_measPositionControlToBaseInControlFrame_y).data, 'b.-')
%plot(logElem(idx_loco_whole_body_positionControlToComInControlFrame_y).time, logElem(idx_loco_whole_body_positionControlToComInControlFrame_y).data, 'k.-')
plot(logElem(idx_loco_torso_desPositionControlToTargetInControlFrame_y).time, logElem(idx_loco_torso_desPositionControlToTargetInControlFrame_y).data, 'r.-')

grid on
xlabel('time [s]')
ylabel('y [m]')
subplot(3,1,3)
hold on
plot(logElem(idx_loco_torso_measPositionControlToBaseInControlFrame_z).time, logElem(idx_loco_torso_measPositionControlToBaseInControlFrame_z).data, 'b.-')
%plot(logElem(idx_loco_whole_body_positionControlToComInControlFrame_z).time, logElem(idx_loco_whole_body_positionControlToComInControlFrame_z).data, 'k.-')
plot(logElem(idx_loco_torso_desPositionControlToTargetInControlFrame_z).time, logElem(idx_loco_torso_desPositionControlToTargetInControlFrame_z).data, 'r.-')

grid on
xlabel('time [s]')
ylabel('z [m]')

legend('base', 'COM', 'des')

%%
h = figure();
set(h, 'Name', 'torso position error');
subplot(3,1,1)
hold on
plot(logElem(idx_loco_torso_positionErrorInControlFrame_x).time, logElem(idx_loco_torso_positionErrorInControlFrame_x).data, 'k.-')
grid on
xlabel('time [s]')
ylabel('x [m]')
title('position error in control frame')
subplot(3,1,2)
hold on
plot(logElem(idx_loco_torso_positionErrorInControlFrame_y).time, logElem(idx_loco_torso_positionErrorInControlFrame_y).data, 'k.-')
grid on
xlabel('time [s]')
ylabel('y [m]')
subplot(3,1,3)
hold on
plot(logElem(idx_loco_torso_positionErrorInControlFrame_z).time, logElem(idx_loco_torso_positionErrorInControlFrame_z).data, 'k.-')
grid on
xlabel('time [s]')
ylabel('z [m]')

%%
h = figure();
set(h, 'Name', 'torso linear velocity');
subplot(3,1,1)
hold on
plot(logElem(idx_loco_torso_measLinearVelocityBaseInControlFrame_x).time, logElem(idx_loco_torso_measLinearVelocityBaseInControlFrame_x).data, 'b')
plot(logElem(idx_loco_torso_desiredLinearVelocityBaseInControlFrame_x).time, logElem(idx_loco_torso_desiredLinearVelocityBaseInControlFrame_x).data, 'r')
grid on
xlabel('time [s]')
ylabel('x [m/s]')
title('linear velocity of torso expressed in control frame')
subplot(3,1,2)
hold on
plot(logElem(idx_loco_torso_measLinearVelocityBaseInControlFrame_y).time, logElem(idx_loco_torso_measLinearVelocityBaseInControlFrame_y).data,'b')
plot(logElem(idx_loco_torso_desiredLinearVelocityBaseInControlFrame_y).time, logElem(idx_loco_torso_desiredLinearVelocityBaseInControlFrame_y).data, 'r')
grid on
xlabel('time [s]')
ylabel('y [m/s]')
subplot(3,1,3)
hold on
plot(logElem(idx_loco_torso_measLinearVelocityBaseInControlFrame_z).time, logElem(idx_loco_torso_measLinearVelocityBaseInControlFrame_z).data, 'b')
plot(logElem(idx_loco_torso_measLinearVelocityBaseInControlFrame_z).time, logElem(idx_loco_torso_desiredLinearVelocityBaseInControlFrame_z).data, 'r')
grid on
xlabel('time [s]')
ylabel('z [m/s]')

legend('meas', 'des')

%%
h = figure();
set(h, 'Name', 'linear velocity torso error');
subplot(3,1,1)
plot(logElem(idx_loco_torso_linearVelocityErrorInControlFrame_x).time, logElem(idx_loco_torso_linearVelocityErrorInControlFrame_x).data, 'k.-')
grid on
xlabel('time [s]')
ylabel('x [m/s]')
title('linear velocity error in control frame')
subplot(3,1,2)
plot(logElem(idx_loco_torso_linearVelocityErrorInControlFrame_y).time, logElem(idx_loco_torso_linearVelocityErrorInControlFrame_y).data, 'k.-')
grid on
xlabel('time [s]')
ylabel('y [m/s]')
subplot(3,1,3)
plot(logElem(idx_loco_torso_linearVelocityErrorInControlFrame_z).time, logElem(idx_loco_torso_linearVelocityErrorInControlFrame_z).data, 'k.-')
grid on
xlabel('time [s]')
ylabel('z [m/s]')


%% Mainbody measured and desired pose
h = figure();
set(h, 'Name', 'torso orientation control to base');

subplot(3,1,1)
hold on
plot(logElem(idx_loco_torso_desEulerAnglesZyxControlToBase_z).time, logElem(idx_loco_torso_desEulerAnglesZyxControlToBase_z).data, 'r');

grid on
xlabel('time [s]')
ylabel('yaw [rad]')
subplot(3,1,2)
    hold on
plot(logElem(idx_loco_torso_desEulerAnglesZyxControlToBase_y).time, logElem(idx_loco_torso_desEulerAnglesZyxControlToBase_y).data, 'r')

grid on
xlabel('time [s]')
ylabel('pitch [rad]')
subplot(3,1,3)
    hold on
plot(logElem(idx_loco_torso_desEulerAnglesZyxControlToBase_x).time, logElem(idx_loco_torso_desEulerAnglesZyxControlToBase_x).data, 'r')

grid on
xlabel('time [s]')
ylabel('roll [rad]')


%%
h = figure();
set(h, 'Name', 'torso angular velocity expressed in control frame');

subplot(3,1,1)
hold on
plot(logElem(idx_loco_torso_measAngularVelocityBaseInControlFrame_x).time, logElem(idx_loco_torso_measAngularVelocityBaseInControlFrame_x).data, 'b')
plot(logElem(idx_loco_torso_desiredAngularVelocityBaseInControlFrame_x).time, logElem(idx_loco_torso_desiredAngularVelocityBaseInControlFrame_x).data, 'r')
grid on
xlabel('time [s]')
ylabel('x [rad/s]')
subplot(3,1,2)
hold on
plot(logElem(idx_loco_torso_measAngularVelocityBaseInControlFrame_y).time, logElem(idx_loco_torso_measAngularVelocityBaseInControlFrame_y).data, 'b')
plot(logElem(idx_loco_torso_desiredAngularVelocityBaseInControlFrame_y).time, logElem(idx_loco_torso_desiredAngularVelocityBaseInControlFrame_y).data, 'r')
grid on
xlabel('time [s]')
ylabel('y [rad/s]')
subplot(3,1,3)
hold on
plot(logElem(idx_loco_torso_measAngularVelocityBaseInControlFrame_z).time, logElem(idx_loco_torso_measAngularVelocityBaseInControlFrame_z).data, 'b')
plot(logElem(idx_loco_torso_desiredAngularVelocityBaseInControlFrame_z).time, logElem(idx_loco_torso_desiredAngularVelocityBaseInControlFrame_z).data, 'r')
grid on
xlabel('time [s]')
ylabel('z [rad/s]')

%%
h = figure();
set(h, 'Name', 'torso linear acceleration expressed in control frame');

subplot(3,1,1)
hold on
plot(logElem(idx_loco_torso_desLinearAccelerationTargetInControlFrame_x).time, logElem(idx_loco_torso_desLinearAccelerationTargetInControlFrame_x).data, 'r')
grid on
xlabel('time [s]')
ylabel('x [m/s^2]')
title('lin. acc. of torso expressed in control frame')
subplot(3,1,2)
hold on
plot(logElem(idx_loco_torso_desLinearAccelerationTargetInControlFrame_y).time, logElem(idx_loco_torso_desLinearAccelerationTargetInControlFrame_y).data, 'r')
grid on
xlabel('time [s]')
ylabel('y [m/s^2]')
subplot(3,1,3)
hold on
plot(logElem(idx_loco_torso_desLinearAccelerationTargetInControlFrame_z).time, logElem(idx_loco_torso_desLinearAccelerationTargetInControlFrame_z).data, 'r')
grid on
xlabel('time [s]')
ylabel('z [m/s^2]')

%%
h = figure();
set(h, 'Name', 'torso world to base position expressed in world frame');
subplot(3,1,1)
hold on
plot(logElem(idx_loco_torso_desPositionWorldToBaseInWorldFrame_x).time, logElem(idx_loco_torso_desPositionWorldToBaseInWorldFrame_x).data, 'b')
grid on
xlabel('time [s]')
ylabel('x [m]')
subplot(3,1,2)
hold on
plot(logElem(idx_loco_torso_desPositionWorldToBaseInWorldFrame_y).time, logElem(idx_loco_torso_desPositionWorldToBaseInWorldFrame_y).data, 'b')
grid on
xlabel('time [s]')
ylabel('y [m]')
subplot(3,1,3)
hold on
plot(logElem(idx_loco_torso_desPositionWorldToBaseInWorldFrame_z).time, logElem(idx_loco_torso_desPositionWorldToBaseInWorldFrame_z).data, 'b')
grid on
xlabel('time [s]')
ylabel('z [m]')