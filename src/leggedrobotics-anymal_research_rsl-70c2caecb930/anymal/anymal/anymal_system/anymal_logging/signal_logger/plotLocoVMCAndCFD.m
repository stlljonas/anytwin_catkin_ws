%%  Loco - Virtual Model Controller and Contact Force Distribution

%% Desired Virtual and Distributed Forces and Torques
h = figure();
set(h, 'Name', 'VMC/CFD - Virtual Net Force');

supportFlag=1;
contactInvariant=2;

nData = length(logElem(idx_loco_leftForeLeg_limbStrategy).data);
legsMC = zeros(nData, 1);
idxLF = find(logElem(idx_loco_leftForeLeg_limbStrategy).data==supportFlag);
legsMC(idxLF) = legsMC(idxLF) + 1;
idxRF = find(logElem(idx_loco_rightForeLeg_limbStrategy).data==supportFlag);
legsMC(idxRF) = legsMC(idxRF) + 1;
idxLH = find(logElem(idx_loco_leftHindLeg_limbStrategy).data==supportFlag);
legsMC(idxLH) = legsMC(idxLH) + 1;
idxRH = find(logElem(idx_loco_rightHindLeg_limbStrategy).data==supportFlag);
legsMC(idxRH) = legsMC(idxRH) + 1;

idxLF = find(logElem(idx_loco_leftForeLeg_limbStrategy).data==contactInvariant);
legsMC(idxLF) = legsMC(idxLF) + 1;
idxRF = find(logElem(idx_loco_rightForeLeg_limbStrategy).data==contactInvariant);
legsMC(idxRF) = legsMC(idxRF) + 1;
idxLH = find(logElem(idx_loco_leftHindLeg_limbStrategy).data==contactInvariant);
legsMC(idxLH) = legsMC(idxLH) + 1;
idxRH = find(logElem(idx_loco_rightHindLeg_limbStrategy).data==contactInvariant);
legsMC(idxRH) = legsMC(idxRH) + 1;

timeMC = logElem(idx_loco_leftForeLeg_limbStrategy).time;
subplot(4,1,1)
hold on
plot(timeMC, legsMC,'k.-')
plot(logElem(idx_loco_leftForeLeg_isGrounded).time, logElem(idx_loco_leftForeLeg_isGrounded).data+logElem(idx_loco_rightForeLeg_isGrounded).data+logElem(idx_loco_leftHindLeg_isGrounded).data+logElem(idx_loco_rightHindLeg_isGrounded).data,'b.-');
legend('# support', '# contacts')
grid on
xlabel('time [s]')
ylabel('-')
title('# contacts')

subplot(4,1,2)
hold on
plot(logElem(idx_loco_vmc_desVirtualForceInBaseFrame_x).time, logElem(idx_loco_vmc_desVirtualForceInBaseFrame_x).data,'r.-');
plot(logElem(idx_loco_cfd_distVirtualForceInBaseFrame_x).time, logElem(idx_loco_cfd_distVirtualForceInBaseFrame_x).data,'b.-');
grid on
xlabel('time [s]')
ylabel('x')

legend('desired', 'distributed')
title('Virtual Net Force')

subplot(4,1,3)
hold on
plot(logElem(idx_loco_vmc_desVirtualForceInBaseFrame_y).time, logElem(idx_loco_vmc_desVirtualForceInBaseFrame_y).data,'r.-');
plot(logElem(idx_loco_cfd_distVirtualForceInBaseFrame_y).time, logElem(idx_loco_cfd_distVirtualForceInBaseFrame_y).data,'b.-');
grid on
xlabel('time [s]')
ylabel('y')

subplot(4,1,4)
hold on
plot(logElem(idx_loco_vmc_desVirtualForceInBaseFrame_z).time, logElem(idx_loco_vmc_desVirtualForceInBaseFrame_z).data,'r.-');
plot(logElem(idx_loco_cfd_distVirtualForceInBaseFrame_z).time, logElem(idx_loco_cfd_distVirtualForceInBaseFrame_z).data,'b.-');
grid on
xlabel('time [s]')
ylabel('z') 



%%

h = figure();
set(h, 'Name', 'VMC/CFD - Virtual Net Torque');

subplot(3,1,1)
hold on
plot(logElem(idx_loco_vmc_desVirtualTorqueInBaseFrame_x).time, logElem(idx_loco_vmc_desVirtualTorqueInBaseFrame_x).data,'r');
plot(logElem(idx_loco_cfd_distVirtualTorqueInBaseFrame_x).time, logElem(idx_loco_cfd_distVirtualTorqueInBaseFrame_x).data,'b');
grid on
xlabel('time [s]')
ylabel('x')

legend('desired', 'distributed')
title('Virtual Net Torque')

subplot(3,1,2)
hold on
plot(logElem(idx_loco_vmc_desVirtualTorqueInBaseFrame_y).time, logElem(idx_loco_vmc_desVirtualTorqueInBaseFrame_y).data,'r');
plot(logElem(idx_loco_cfd_distVirtualTorqueInBaseFrame_y).time, logElem(idx_loco_cfd_distVirtualTorqueInBaseFrame_y).data,'b');
grid on
xlabel('time [s]')
ylabel('y')

subplot(3,1,3)
hold on
plot(logElem(idx_loco_vmc_desVirtualTorqueInBaseFrame_z).time, logElem(idx_loco_vmc_desVirtualTorqueInBaseFrame_z).data,'r');
plot(logElem(idx_loco_cfd_distVirtualTorqueInBaseFrame_z).time, logElem(idx_loco_cfd_distVirtualTorqueInBaseFrame_z).data,'b');
grid on
xlabel('time [s]')
ylabel('z') 