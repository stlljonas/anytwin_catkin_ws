% h = figure();
% set(h, 'Name', 'stride phase');
% subplot(2,1,1)
% hold on
% plot(logElem(idx_gait_pattern_cycle_phase).time, logElem(idx_gait_pattern_cycle_phase).data, 'k')
% plot(logElem(idx_loco_torso_stridePhase).time, logElem(idx_loco_torso_stridePhase).data, 'b')
% title('stride phase')
% xlabel('time [s]')
% subplot(2,1,2)
% hold on
% plot(logElem(idx_gait_pattern_num_gait_cycles).time, logElem(idx_gait_pattern_num_gait_cycles).data, 'k')
% title('# cycles')
% xlabel('time [s]')

%%
h = figure();
set(h, 'Name', 'LF Leg Timing');
subplot(2,1,1)
hold on
plot(logElem(idx_loco_torso_stridePhase).time, logElem(idx_loco_torso_stridePhase).data, 'r')
plot(logElem(idx_loco_leftFore_stancePhase).time, logElem(idx_loco_leftFore_stancePhase).data, 'k')
plot(logElem(idx_loco_leftFore_swingPhase).time, logElem(idx_loco_leftFore_swingPhase).data, 'b')
legend('stride', 'stance', 'swing')
grid on
title('phase')
xlabel('time [s]')
ylabel('phase [%]')
subplot(2,1,2)
hold on
plot(logElem(idx_loco_torso_strideDuration).time, logElem(idx_loco_torso_strideDuration).data, 'r')
plot(logElem(idx_loco_leftFore_stanceDuration).time, logElem(idx_loco_leftFore_stanceDuration).data, 'k')
plot(logElem(idx_loco_leftFore_swingDuration).time, logElem(idx_loco_leftFore_swingDuration).data, 'b')
legend('stride', 'stance', 'swing')
grid on
title('duration')
xlabel('time [s]')
ylabel('duration [s]')

%%
h = figure();
set(h, 'Name', 'RF Leg Timing');
subplot(2,1,1)
hold on
plot(logElem(idx_loco_torso_stridePhase).time, logElem(idx_loco_torso_stridePhase).data, 'r')
plot(logElem(idx_loco_rightFore_stancePhase).time, logElem(idx_loco_rightFore_stancePhase).data, 'k')
plot(logElem(idx_loco_rightFore_swingPhase).time, logElem(idx_loco_rightFore_swingPhase).data, 'b')
legend('stride', 'stance', 'swing')
grid on
title('phase')
xlabel('time [s]')
ylabel('phase [%]')
subplot(2,1,2)
hold on
plot(logElem(idx_loco_torso_strideDuration).time, logElem(idx_loco_torso_strideDuration).data, 'r')
plot(logElem(idx_loco_rightFore_stanceDuration).time, logElem(idx_loco_rightFore_stanceDuration).data, 'k')
plot(logElem(idx_loco_rightFore_swingDuration).time, logElem(idx_loco_rightFore_swingDuration).data, 'b')
legend('stride', 'stance', 'swing')
grid on
title('duration')
xlabel('time [s]')
ylabel('duration [s]')

%%
h = figure();
set(h, 'Name', 'LH Leg Timing');
subplot(2,1,1)
hold on
plot(logElem(idx_loco_torso_stridePhase).time, logElem(idx_loco_torso_stridePhase).data, 'r')
plot(logElem(idx_loco_leftHind_stancePhase).time, logElem(idx_loco_leftHind_stancePhase).data, 'k')
plot(logElem(idx_loco_leftHind_swingPhase).time, logElem(idx_loco_leftHind_swingPhase).data, 'b')
legend('stride', 'stance', 'swing')
grid on
title('phase')
xlabel('time [s]')
ylabel('phase [%]')
subplot(2,1,2)
hold on
plot(logElem(idx_loco_torso_strideDuration).time, logElem(idx_loco_torso_strideDuration).data, 'r')
plot(logElem(idx_loco_leftHind_stanceDuration).time, logElem(idx_loco_leftHind_stanceDuration).data, 'k')
plot(logElem(idx_loco_leftHind_swingDuration).time, logElem(idx_loco_leftHind_swingDuration).data, 'b')
legend('stride', 'stance', 'swing')
grid on
title('duration')
xlabel('time [s]')
ylabel('duration [s]')

%%
h = figure();
set(h, 'Name', 'RH Leg Timing');
subplot(2,1,1)
hold on
plot(logElem(idx_loco_torso_stridePhase).time, logElem(idx_loco_torso_stridePhase).data, 'r')
plot(logElem(idx_loco_rightHind_stancePhase).time, logElem(idx_loco_rightHind_stancePhase).data, 'k')
plot(logElem(idx_loco_rightHind_swingPhase).time, logElem(idx_loco_rightHind_swingPhase).data, 'b')
legend('stride', 'stance', 'swing')
grid on
title('phase')
xlabel('time [s]')
ylabel('phase [%]')
subplot(2,1,2)
hold on
plot(logElem(idx_loco_torso_strideDuration).time, logElem(idx_loco_torso_strideDuration).data, 'r')
plot(logElem(idx_loco_rightHind_stanceDuration).time, logElem(idx_loco_rightHind_stanceDuration).data, 'k')
plot(logElem(idx_loco_rightHind_swingDuration).time, logElem(idx_loco_rightHind_swingDuration).data, 'b')
legend('stride', 'stance', 'swing')
grid on
title('duration')
xlabel('time [s]')
ylabel('duration [s]')