%% Plot desired and estimated contact forces
h = figure();
set(h, 'Name', 'Estimated and desired contact forces');
set(gcf, 'units', 'points', 'position', [plotX0, plotY0, plotWidth, plotHeight]);

% LF
% x
subplot(3,4,1);
title('LF - x', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_leftFore_forceDesiredAtEEInWorldFrame_x).time, logElem(idx_loco_leftFore_forceDesiredAtEEInWorldFrame_x).data, 'r', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('$$ f_x [N]$$','Interpreter','Latex');
% y
subplot(3,4,5);
title('y', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_leftFore_forceDesiredAtEEInWorldFrame_y).time, logElem(idx_loco_leftFore_forceDesiredAtEEInWorldFrame_y).data, 'r', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('$$ f_y [N]$$','Interpreter','Latex');
% z
subplot(3,4,9);
title('z', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_leftFore_forceDesiredAtEEInWorldFrame_z).time, -logElem(idx_loco_leftFore_forceDesiredAtEEInWorldFrame_z).data, 'r', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('$$ f_z [N]$$','Interpreter','Latex');

% RF
% x
subplot(3,4,2);
title('RF - x', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_rightFore_forceDesiredAtEEInWorldFrame_x).time, logElem(idx_loco_rightFore_forceDesiredAtEEInWorldFrame_x).data, 'r', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('$$ f_x [N]$$','Interpreter','Latex');
% y
subplot(3,4,6);
title('y', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_rightFore_forceDesiredAtEEInWorldFrame_y).time, logElem(idx_loco_rightFore_forceDesiredAtEEInWorldFrame_y).data, 'r', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('$$ f_y [N]$$','Interpreter','Latex');
% z
subplot(3,4,10);
title('z', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_rightFore_forceDesiredAtEEInWorldFrame_z).time, -logElem(idx_loco_rightFore_forceDesiredAtEEInWorldFrame_z).data, 'r', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('$$ f_z [N]$$','Interpreter','Latex');

% LH
% x
subplot(3,4,3);
title('LH - x', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_leftHind_forceDesiredAtEEInWorldFrame_x).time, logElem(idx_loco_leftHind_forceDesiredAtEEInWorldFrame_x).data, 'r', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('$$ f_x [N]$$','Interpreter','Latex');
% y
subplot(3,4,7);
title('y', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_leftHind_forceDesiredAtEEInWorldFrame_y).time, logElem(idx_loco_leftHind_forceDesiredAtEEInWorldFrame_y).data, 'r', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('$$ f_y [N]$$','Interpreter','Latex');
% z
subplot(3,4,11);
title('z', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_leftHind_forceDesiredAtEEInWorldFrame_z).time, -logElem(idx_loco_leftHind_forceDesiredAtEEInWorldFrame_z).data, 'r', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('$$ f_z [N]$$','Interpreter','Latex');

% RH
% x
subplot(3,4,4);
title('RH - x', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_rightHind_forceDesiredAtEEInWorldFrame_x).time, logElem(idx_loco_rightHind_forceDesiredAtEEInWorldFrame_x).data, 'r', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('$$ f_x [N]$$','Interpreter','Latex');
% y
subplot(3,4,8);
title('y', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_rightHind_forceDesiredAtEEInWorldFrame_y).time, logElem(idx_loco_rightHind_forceDesiredAtEEInWorldFrame_y).data, 'r', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('$$ f_y [N]$$','Interpreter','Latex');
% z
subplot(3,4,12);
title('z', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_rightHind_forceDesiredAtEEInWorldFrame_z).time, -logElem(idx_loco_rightHind_forceDesiredAtEEInWorldFrame_z).data, 'r', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('$$ f_z [N]$$','Interpreter','Latex');


%% Plot desired and estimated feet positions
h = figure();
set(h, 'Name', 'Estimated and desired feet positions in world frame');
set(gcf, 'units', 'points', 'position', [plotX0, plotY0, plotWidth, plotHeight]);

% LF
% x
subplot(3,4,1);
title('LF - x', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_leftFore_positionWorldToDesiredEEOriginInWorldFrame_x).time, logElem(idx_loco_leftFore_positionWorldToDesiredEEOriginInWorldFrame_x).data, 'r', 'LineWidth', lineWidth);
plot(logElem(idx_loco_leftFore_positionWorldToEEOriginInWorldFrame_x).time, logElem(idx_loco_leftFore_positionWorldToEEOriginInWorldFrame_x).data, 'b', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('position [m]');
% y
subplot(3,4,5);
title('y', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_leftFore_positionWorldToDesiredEEOriginInWorldFrame_y).time, logElem(idx_loco_leftFore_positionWorldToDesiredEEOriginInWorldFrame_y).data, 'r', 'LineWidth', lineWidth);
plot(logElem(idx_loco_leftFore_positionWorldToEEOriginInWorldFrame_y).time, logElem(idx_loco_leftFore_positionWorldToEEOriginInWorldFrame_y).data, 'b', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('position [m]');
% z
subplot(3,4,9);
title('z', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_leftFore_positionWorldToDesiredEEOriginInWorldFrame_z).time, logElem(idx_loco_leftFore_positionWorldToDesiredEEOriginInWorldFrame_z).data, 'r', 'LineWidth', lineWidth);
plot(logElem(idx_loco_leftFore_positionWorldToEEOriginInWorldFrame_z).time, logElem(idx_loco_leftFore_positionWorldToEEOriginInWorldFrame_z).data, 'b', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('position [m]');

% RF
% x
subplot(3,4,2);
title('RF - x', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_rightFore_positionWorldToDesiredEEOriginInWorldFrame_x).time, logElem(idx_loco_rightFore_positionWorldToDesiredEEOriginInWorldFrame_x).data, 'r', 'LineWidth', lineWidth);
plot(logElem(idx_loco_rightFore_positionWorldToEEOriginInWorldFrame_x).time, logElem(idx_loco_rightFore_positionWorldToEEOriginInWorldFrame_x).data, 'b', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('position [m]');
% y
subplot(3,4,6);
title('y', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_rightFore_positionWorldToDesiredEEOriginInWorldFrame_y).time, logElem(idx_loco_rightFore_positionWorldToDesiredEEOriginInWorldFrame_y).data, 'r', 'LineWidth', lineWidth);
plot(logElem(idx_loco_rightFore_positionWorldToEEOriginInWorldFrame_y).time, logElem(idx_loco_rightFore_positionWorldToEEOriginInWorldFrame_y).data, 'b', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('position [m]');
% z
subplot(3,4,10);
title('z', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_rightFore_positionWorldToDesiredEEOriginInWorldFrame_z).time, logElem(idx_loco_rightFore_positionWorldToDesiredEEOriginInWorldFrame_z).data, 'r', 'LineWidth', lineWidth);
plot(logElem(idx_loco_rightFore_positionWorldToEEOriginInWorldFrame_z).time, logElem(idx_loco_rightFore_positionWorldToEEOriginInWorldFrame_z).data, 'b', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('position [m]');

% LH
% x
subplot(3,4,3);
title('LH - x', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_leftHind_positionWorldToDesiredEEOriginInWorldFrame_x).time, logElem(idx_loco_leftHind_positionWorldToDesiredEEOriginInWorldFrame_x).data, 'r', 'LineWidth', lineWidth);
plot(logElem(idx_loco_leftHind_positionWorldToEEOriginInWorldFrame_x).time, logElem(idx_loco_leftHind_positionWorldToEEOriginInWorldFrame_x).data, 'b', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('position [m]');
% y
subplot(3,4,7);
title('y', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_leftHind_positionWorldToDesiredEEOriginInWorldFrame_y).time, logElem(idx_loco_leftHind_positionWorldToDesiredEEOriginInWorldFrame_y).data, 'r', 'LineWidth', lineWidth);
plot(logElem(idx_loco_leftHind_positionWorldToEEOriginInWorldFrame_y).time, logElem(idx_loco_leftHind_positionWorldToEEOriginInWorldFrame_y).data, 'b', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('position [m]');
% z
subplot(3,4,11);
title('z', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_leftHind_positionWorldToDesiredEEOriginInWorldFrame_z).time, logElem(idx_loco_leftHind_positionWorldToDesiredEEOriginInWorldFrame_z).data, 'r', 'LineWidth', lineWidth);
plot(logElem(idx_loco_leftHind_positionWorldToEEOriginInWorldFrame_z).time, logElem(idx_loco_leftHind_positionWorldToEEOriginInWorldFrame_z).data, 'b', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('position [m]');

% RH
% x
subplot(3,4,4);
title('RH - x', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_rightHind_positionWorldToDesiredEEOriginInWorldFrame_x).time, logElem(idx_loco_rightHind_positionWorldToDesiredEEOriginInWorldFrame_x).data, 'r', 'LineWidth', lineWidth);
plot(logElem(idx_loco_rightHind_positionWorldToEEOriginInWorldFrame_x).time, logElem(idx_loco_rightHind_positionWorldToEEOriginInWorldFrame_x).data, 'b', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('position [m]');
% y
subplot(3,4,8);
title('y', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_rightHind_positionWorldToDesiredEEOriginInWorldFrame_y).time, logElem(idx_loco_rightHind_positionWorldToDesiredEEOriginInWorldFrame_y).data, 'r', 'LineWidth', lineWidth);
plot(logElem(idx_loco_rightHind_positionWorldToEEOriginInWorldFrame_y).time, logElem(idx_loco_rightHind_positionWorldToEEOriginInWorldFrame_y).data, 'b', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('position [m]');
% z
subplot(3,4,12);
title('z', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_rightHind_positionWorldToDesiredEEOriginInWorldFrame_z).time, logElem(idx_loco_rightHind_positionWorldToDesiredEEOriginInWorldFrame_z).data, 'r', 'LineWidth', lineWidth);
plot(logElem(idx_loco_rightHind_positionWorldToEEOriginInWorldFrame_z).time, logElem(idx_loco_rightHind_positionWorldToEEOriginInWorldFrame_z).data, 'b', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('position [m]');

%% Plot desired and estimated feet velocities
h = figure();
set(h, 'Name', 'Estimated and desired feet velocities');
set(gcf, 'units', 'points', 'position', [plotX0, plotY0, plotWidth, plotHeight]);

% LF
% x
subplot(3,4,1);
title('LF - x', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_leftFore_linearVelocityEEOriginInWorldFrame_x).time, logElem(idx_loco_leftFore_linearVelocityEEOriginInWorldFrame_x).data, 'b', 'LineWidth', lineWidth);
plot(logElem(idx_loco_leftFore_linearVelocityDesiredEEOriginInWorldFrame_x).time, logElem(idx_loco_leftFore_linearVelocityDesiredEEOriginInWorldFrame_x).data, 'r', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('velocity [m/s]');
% y
subplot(3,4,5);
title('y', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_leftFore_linearVelocityEEOriginInWorldFrame_y).time, logElem(idx_loco_leftFore_linearVelocityEEOriginInWorldFrame_y).data, 'b', 'LineWidth', lineWidth);
plot(logElem(idx_loco_leftFore_linearVelocityDesiredEEOriginInWorldFrame_y).time, logElem(idx_loco_leftFore_linearVelocityDesiredEEOriginInWorldFrame_y).data, 'r', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('velocity [m/s]');
% z
subplot(3,4,9);
title('z', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_leftFore_linearVelocityEEOriginInWorldFrame_z).time, logElem(idx_loco_leftFore_linearVelocityEEOriginInWorldFrame_z).data, 'b', 'LineWidth', lineWidth);
plot(logElem(idx_loco_leftFore_linearVelocityDesiredEEOriginInWorldFrame_z).time, logElem(idx_loco_leftFore_linearVelocityDesiredEEOriginInWorldFrame_z).data, 'r', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('velocity [m/s]');

% RF
% x
subplot(3,4,2);
title('RF - x', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_rightFore_linearVelocityEEOriginInWorldFrame_x).time, logElem(idx_loco_rightFore_linearVelocityEEOriginInWorldFrame_x).data, 'b', 'LineWidth', lineWidth);
plot(logElem(idx_loco_rightFore_linearVelocityDesiredEEOriginInWorldFrame_x).time, logElem(idx_loco_rightFore_linearVelocityDesiredEEOriginInWorldFrame_x).data, 'r', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('velocity [m/s]');
% y
subplot(3,4,6);
title('y', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_rightFore_linearVelocityEEOriginInWorldFrame_y).time, logElem(idx_loco_rightFore_linearVelocityEEOriginInWorldFrame_y).data, 'b', 'LineWidth', lineWidth);
plot(logElem(idx_loco_rightFore_linearVelocityDesiredEEOriginInWorldFrame_y).time, logElem(idx_loco_rightFore_linearVelocityDesiredEEOriginInWorldFrame_y).data, 'r', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('velocity [m/s]');
% z
subplot(3,4,10);
title('z', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_rightFore_linearVelocityEEOriginInWorldFrame_z).time, logElem(idx_loco_rightFore_linearVelocityEEOriginInWorldFrame_z).data, 'b', 'LineWidth', lineWidth);
plot(logElem(idx_loco_rightFore_linearVelocityDesiredEEOriginInWorldFrame_z).time, logElem(idx_loco_rightFore_linearVelocityDesiredEEOriginInWorldFrame_z).data, 'r', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('velocity [m/s]');

% LH
% x
subplot(3,4,3);
title('LH - x', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_leftHind_linearVelocityEEOriginInWorldFrame_x).time, logElem(idx_loco_leftHind_linearVelocityEEOriginInWorldFrame_x).data, 'b', 'LineWidth', lineWidth);
plot(logElem(idx_loco_leftHind_linearVelocityDesiredEEOriginInWorldFrame_x).time, logElem(idx_loco_leftHind_linearVelocityDesiredEEOriginInWorldFrame_x).data, 'r', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('velocity [m/s]');
% y
subplot(3,4,7);
title('y', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_leftHind_linearVelocityEEOriginInWorldFrame_y).time, logElem(idx_loco_leftHind_linearVelocityEEOriginInWorldFrame_y).data, 'b', 'LineWidth', lineWidth);
plot(logElem(idx_loco_leftHind_linearVelocityDesiredEEOriginInWorldFrame_y).time, logElem(idx_loco_leftHind_linearVelocityDesiredEEOriginInWorldFrame_y).data, 'r', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('velocity [m/s]');
% z
subplot(3,4,11);
title('z', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_leftHind_linearVelocityEEOriginInWorldFrame_z).time, logElem(idx_loco_leftHind_linearVelocityEEOriginInWorldFrame_z).data, 'b', 'LineWidth', lineWidth);
plot(logElem(idx_loco_leftHind_linearVelocityDesiredEEOriginInWorldFrame_z).time, logElem(idx_loco_leftHind_linearVelocityDesiredEEOriginInWorldFrame_z).data, 'r', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('velocity [m/s]');

% RH
% x
subplot(3,4,4);
title('RH - x', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_rightHind_linearVelocityEEOriginInWorldFrame_x).time, logElem(idx_loco_rightHind_linearVelocityEEOriginInWorldFrame_x).data, 'b', 'LineWidth', lineWidth);
plot(logElem(idx_loco_rightHind_linearVelocityDesiredEEOriginInWorldFrame_x).time, logElem(idx_loco_rightHind_linearVelocityDesiredEEOriginInWorldFrame_x).data, 'r', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('velocity [m/s]');
% y
subplot(3,4,8);
title('y', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_rightHind_linearVelocityEEOriginInWorldFrame_y).time, logElem(idx_loco_rightHind_linearVelocityEEOriginInWorldFrame_y).data, 'b', 'LineWidth', lineWidth);
plot(logElem(idx_loco_rightHind_linearVelocityDesiredEEOriginInWorldFrame_y).time, logElem(idx_loco_rightHind_linearVelocityDesiredEEOriginInWorldFrame_y).data, 'r', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('velocity [m/s]');
% z
subplot(3,4,12);
title('z', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_rightHind_linearVelocityEEOriginInWorldFrame_z).time, logElem(idx_loco_rightHind_linearVelocityEEOriginInWorldFrame_z).data, 'b', 'LineWidth', lineWidth);
plot(logElem(idx_loco_rightHind_linearVelocityDesiredEEOriginInWorldFrame_z).time, logElem(idx_loco_rightHind_linearVelocityDesiredEEOriginInWorldFrame_z).data, 'r', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('velocity [m/s]');

%% Plot desired feet accelerations
h = figure();
set(h, 'Name', 'Desired feet accelerations');
set(gcf, 'units', 'points', 'position', [plotX0, plotY0, plotWidth, plotHeight]);

% LF
% x
subplot(3,4,1);
title('LF - x', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_leftFore_linearAccelerationDesiredEEOriginInWorldFrame_x).time, logElem(idx_loco_leftFore_linearAccelerationDesiredEEOriginInWorldFrame_x).data, 'r', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('acceleration [m/s^2]');
% y
subplot(3,4,5);
title('y', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_leftFore_linearAccelerationDesiredEEOriginInWorldFrame_y).time, logElem(idx_loco_leftFore_linearAccelerationDesiredEEOriginInWorldFrame_y).data, 'r', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('acceleration [m/s^2]');
% z
subplot(3,4,9);
title('z', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_leftFore_linearAccelerationDesiredEEOriginInWorldFrame_z).time, logElem(idx_loco_leftFore_linearAccelerationDesiredEEOriginInWorldFrame_z).data, 'r', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('acceleration [m/s^2]');

% RF
% x
subplot(3,4,2);
title('RF - x', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_rightFore_linearAccelerationDesiredEEOriginInWorldFrame_x).time, logElem(idx_loco_rightFore_linearAccelerationDesiredEEOriginInWorldFrame_x).data, 'r', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('acceleration [m/s^2]');
% y
subplot(3,4,6);
title('y', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_rightFore_linearAccelerationDesiredEEOriginInWorldFrame_y).time, logElem(idx_loco_rightFore_linearAccelerationDesiredEEOriginInWorldFrame_y).data, 'r', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('acceleration [m/s^2]');
% z
subplot(3,4,10);
title('z', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_rightFore_linearAccelerationDesiredEEOriginInWorldFrame_z).time, logElem(idx_loco_rightFore_linearAccelerationDesiredEEOriginInWorldFrame_z).data, 'r', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('acceleration [m/s^2]');
% LH
% x
subplot(3,4,3);
title('LH - x', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_leftHind_linearAccelerationDesiredEEOriginInWorldFrame_x).time, logElem(idx_loco_leftHind_linearAccelerationDesiredEEOriginInWorldFrame_x).data, 'r', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('acceleration [m/s^2]');
% y
subplot(3,4,7);
title('y', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_leftHind_linearAccelerationDesiredEEOriginInWorldFrame_y).time, logElem(idx_loco_leftHind_linearAccelerationDesiredEEOriginInWorldFrame_y).data, 'r', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('acceleration [m/s^2]');
% z
subplot(3,4,11);
title('z', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_leftHind_linearAccelerationDesiredEEOriginInWorldFrame_z).time, logElem(idx_loco_leftHind_linearAccelerationDesiredEEOriginInWorldFrame_z).data, 'r', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('acceleration [m/s^2]');

% RH
% x
subplot(3,4,4);
title('RH - x', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_rightHind_linearAccelerationDesiredEEOriginInWorldFrame_x).time, logElem(idx_loco_rightHind_linearAccelerationDesiredEEOriginInWorldFrame_x).data, 'r', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('acceleration [m/s^2]');
% y
subplot(3,4,8);
title('y', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_rightHind_linearAccelerationDesiredEEOriginInWorldFrame_y).time, logElem(idx_loco_rightHind_linearAccelerationDesiredEEOriginInWorldFrame_y).data, 'r', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('acceleration [m/s^2]');
% z
subplot(3,4,12);
title('z', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_rightHind_linearAccelerationDesiredEEOriginInWorldFrame_z).time, logElem(idx_loco_rightHind_linearAccelerationDesiredEEOriginInWorldFrame_z).data, 'r', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('acceleration [m/s^2]');
