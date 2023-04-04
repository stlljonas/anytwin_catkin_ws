%% Loco - Contact Force Distribution

%% Desired contact forces
h = figure();
set(h, 'Name', 'Desired contact forces');
grid on
subplot(3,4,1)
hold on
plot(logElem(idx_loco_leftForeLeg_desiredContactForceAtFootInWorldFrame_x).time, logElem(idx_loco_leftForeLeg_desiredContactForceAtFootInWorldFrame_x).data,'r')
title('LF')
grid on
xlabel('time [s]')
ylabel('x')
subplot(3,4,5)
hold on
plot(logElem(idx_loco_leftForeLeg_desiredContactForceAtFootInWorldFrame_y).time, logElem(idx_loco_leftForeLeg_desiredContactForceAtFootInWorldFrame_y).data,'r')
grid on
xlabel('time [s]')
ylabel('y')
subplot(3,4,9)
hold on
plot(logElem(idx_loco_leftForeLeg_desiredContactForceAtFootInWorldFrame_z).time, logElem(idx_loco_leftForeLeg_desiredContactForceAtFootInWorldFrame_z).data,'r')
grid on
xlabel('time [s]')
ylabel('z')

subplot(3,4,2)
hold on
plot(logElem(idx_loco_rightForeLeg_desiredContactForceAtFootInWorldFrame_x).time, logElem(idx_loco_rightForeLeg_desiredContactForceAtFootInWorldFrame_x).data,'r')
title('RF')
grid on
xlabel('time [s]')
ylabel('x')
subplot(3,4,6)
hold on
plot(logElem(idx_loco_rightForeLeg_desiredContactForceAtFootInWorldFrame_y).time, logElem(idx_loco_rightForeLeg_desiredContactForceAtFootInWorldFrame_y).data,'r')
grid on
xlabel('time [s]')
ylabel('y]')
subplot(3,4,10)
hold on
plot(logElem(idx_loco_rightForeLeg_desiredContactForceAtFootInWorldFrame_z).time, logElem(idx_loco_rightForeLeg_desiredContactForceAtFootInWorldFrame_z).data,'r')
grid on
xlabel('time [s]')
ylabel('z')

subplot(3,4,3)
hold on
plot(logElem(idx_loco_leftHindLeg_desiredContactForceAtFootInWorldFrame_x).time, logElem(idx_loco_leftHindLeg_desiredContactForceAtFootInWorldFrame_x).data,'r')
title('LH')
grid on
xlabel('time [s]')
ylabel('x')
subplot(3,4,7)
hold on
plot(logElem(idx_loco_leftHindLeg_desiredContactForceAtFootInWorldFrame_y).time, logElem(idx_loco_leftHindLeg_desiredContactForceAtFootInWorldFrame_y).data,'r')
grid on
xlabel('time [s]')
ylabel('y')
subplot(3,4,11)
hold on
plot(logElem(idx_loco_leftHindLeg_desiredContactForceAtFootInWorldFrame_z).time, logElem(idx_loco_leftHindLeg_desiredContactForceAtFootInWorldFrame_z).data,'r')
grid on
xlabel('time [s]')
ylabel('z')

subplot(3,4,4)
hold on
plot(logElem(idx_loco_rightHindLeg_desiredContactForceAtFootInWorldFrame_x).time, logElem(idx_loco_rightHindLeg_desiredContactForceAtFootInWorldFrame_x).data,'r')
title('RH')
grid on
xlabel('time [s]')
ylabel('x')
subplot(3,4,8)
hold on
plot(logElem(idx_loco_rightHindLeg_desiredContactForceAtFootInWorldFrame_y).time, logElem(idx_loco_rightHindLeg_desiredContactForceAtFootInWorldFrame_y).data,'r')
grid on
xlabel('time [s]')
ylabel('y')
subplot(3,4,12)
hold on
plot(logElem(idx_loco_rightHindLeg_desiredContactForceAtFootInWorldFrame_z).time, logElem(idx_loco_rightHindLeg_desiredContactForceAtFootInWorldFrame_z).data,'r')
grid on
xlabel('time [s]')
ylabel('z')