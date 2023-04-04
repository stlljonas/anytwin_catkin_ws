%% Plot limb state flags
h = figure();
set(h, 'Name', 'Leg state flags');
set(gcf, 'units', 'points', 'position', [plotX0, plotY0, plotWidth, plotHeight]);
limbStateTicks = [-3 -2 -1 1 2 3 4 5];
limbStateLabels = {'SwingBumpedIntoObstacle = -3', 'StanceLostContact = -2', 'StanceSlipping = -1', ...
    'StanceNormal = 1',  'SwingNormal = 2', 'SwingLateLiftOff = 3', ...
    'SwingEarlyTouchDown = 4', 'SwingExpectingContact = 5', };

% LF
subplot(2,2,1);
title('LF', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_leftFore_limbState).time, logElem(idx_loco_leftFore_limbState).data, 'r.-', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('state');
ax = gca;
set(ax,'YTick', limbStateTicks);
set(ax,'YTick', limbStateTicks, 'YTickLabel', limbStateLabels, 'FontSize', 6);

subplot(2,2,2);
title('RF', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_rightFore_limbState).time, logElem(idx_loco_rightFore_limbState).data, 'r.-', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('state');
ax = gca;
set(ax,'YTick', limbStateTicks);
set(ax,'YTickLabel', limbStateLabels, 'FontSize', 6);

subplot(2,2,3);
title('LH', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_leftHind_limbState).time, logElem(idx_loco_leftHind_limbState).data, 'r.-', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('state');
ax = gca;
set(ax,'YTick', limbStateTicks);
set(ax,'YTickLabel', limbStateLabels, 'FontSize', 6);

subplot(2,2,4);
title('RH', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_rightHind_limbState).time, logElem(idx_loco_rightHind_limbState).data, 'r.-', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('state');
ax = gca;
set(ax,'YTick', limbStateTicks);
set(ax,'YTickLabel', limbStateLabels, 'FontSize', 6);


%% Plot contact state flags
h = figure();
set(h, 'Name', 'Leg contact state flags');
set(gcf, 'units', 'points', 'position', [plotX0, plotY0, plotWidth, plotHeight]);
limbStateTicks = [0 1];
limbStateLabels = {'off = 0', 'on = 1'};

% LF
subplot(2,2,1);
title('LF', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_leftFore_isGrounded).time, logElem(idx_loco_leftFore_isGrounded).data, 'r.-', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('contact');
ax = gca;
set(ax,'YTick', limbStateTicks);
set(ax,'YTickLabel', limbStateLabels);

% RF
subplot(2,2,2);
title('RF', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_rightFore_isGrounded).time, logElem(idx_loco_rightFore_isGrounded).data, 'r.-', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('contact');
ax = gca;
set(ax,'YTick', limbStateTicks);
set(ax,'YTickLabel', limbStateLabels);

% LH
subplot(2,2,3);
title('LH', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_leftHind_isGrounded).time, logElem(idx_loco_leftHind_isGrounded).data, 'r.-', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('contact');
ax = gca;
set(ax,'YTick', limbStateTicks);
set(ax,'YTickLabel', limbStateLabels);

% RH
subplot(2,2,4);
title('RH', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_rightHind_isGrounded).time, logElem(idx_loco_rightHind_isGrounded).data, 'r.-', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('contact');
ax = gca;
set(ax,'YTick', limbStateTicks);
set(ax,'YTickLabel', limbStateLabels);


%% Limb strategy

h = figure();
set(h, 'Name', 'Leg strategy');
set(gcf, 'units', 'points', 'position', [plotX0, plotY0, plotWidth, plotHeight]);
limbStrategyTicks = [0 1 2 3 4];
limbStrategyLabels = {'Undefined = 0', 'Support = 1', 'ContactInvariant = 2', 'Motion = 3', ...
    'ContactRecovery = 4',    };

% LF
subplot(2,2,1);
title('LF', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_leftFore_limbStrategy).time, logElem(idx_loco_leftFore_limbStrategy).data, 'r.-', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('state');
ax = gca;
set(ax,'YTick', limbStrategyTicks);
set(ax,'YTick', limbStrategyTicks, 'YTickLabel', limbStrategyLabels, 'FontSize', 6);

subplot(2,2,2);
title('RF', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_rightFore_limbStrategy).time, logElem(idx_loco_rightFore_limbStrategy).data, 'r.-', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('state');
ax = gca;
set(ax,'YTick', limbStrategyTicks);
set(ax,'YTickLabel', limbStrategyLabels, 'FontSize', 6);

subplot(2,2,3);
title('LH', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_leftHind_limbStrategy).time, logElem(idx_loco_leftHind_limbStrategy).data, 'r.-', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('state');
ax = gca;
set(ax,'YTick', limbStrategyTicks);
set(ax,'YTickLabel', limbStrategyLabels, 'FontSize', 6);

subplot(2,2,4);
title('RH', 'FontSize', 9);
hold on; grid on;
plot(logElem(idx_loco_rightHind_limbStrategy).time, logElem(idx_loco_rightHind_limbStrategy).data, 'r.-', 'LineWidth', lineWidth);
xlabel('time [s]');
ylabel('state');
ax = gca;
set(ax,'YTick', limbStrategyTicks);
set(ax,'YTickLabel', limbStrategyLabels, 'FontSize', 6);
