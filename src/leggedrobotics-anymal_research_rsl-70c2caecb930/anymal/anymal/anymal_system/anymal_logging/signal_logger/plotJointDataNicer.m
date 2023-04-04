%% ANYmal signal_logger joint actuator plots
%
% This file plots the measured and desired joint positions (i.e. angles),
% velocities, and torques from binary highlevel_controller files recorded
% with the signal_logger.
%
% Author: Yvain de Viragh
% Date: November 14, 2018
%
%--------------------------------------------------------------------------

clear variables;
close all;
clc;

% Path to the signal_logger matlab files (without trailing /)
addpath(genpath('~/git/signal_logger/signal_logger_std/matlab'));

% Path to the directory containing the file to be plotted (without trailing /)
filePath = '~/.ros';

% Number of file to be plotted
fileNumber = 256;

%% Get the data

% Get filename from directory
fileName = getFilenameFromNumber(fileNumber, filePath);
fprintf(['\nGot filename: ', fileName, ' from number: ', num2str(fileNumber)]);

% Read data
logElements = loadLogFile(fileName);
fprintf(['\n\nLoaded data from binary file:', fileName]);

% Generate Index Variables
verbose = false;
genIndexVariables(logElements, verbose);
fprintf('\n\nGenerated indices for the log elements!\n');

% Increase precision in data cursor and show index of data point
set(0,'defaultFigureCreateFcn',@(s,e)datacursorextra(s))

%% Settings

% Samples to be plotted
startSample = 1;
endSample = length(logElements(idx_state_jointTor_LF_HAA).data);

% Line width of plots
lineWidth = 1.5;

% Color of plots
colorMeasured = 'b';
colorDesired = 'r';

%% Preprocessing

% Enumeration of legs
LF = 1; RF = 2; LH = 3; RH = 4;
enumLegs = [1 2 3 4];
nameLegs = {'LF' 'RF' 'LH' 'RH'};

% Position indices
ind_pos_meas_HAA = [idx_state_jointPos_LF_HAA...
                    idx_state_jointPos_RF_HAA...
                    idx_state_jointPos_LH_HAA...
                    idx_state_jointPos_RH_HAA];
ind_pos_meas_HFE = [idx_state_jointPos_LF_HFE...
                    idx_state_jointPos_RF_HFE...
                    idx_state_jointPos_LH_HFE...
                    idx_state_jointPos_RH_HFE];
ind_pos_meas_KFE = [idx_state_jointPos_LF_KFE...
                    idx_state_jointPos_RF_KFE...
                    idx_state_jointPos_LH_KFE...
                    idx_state_jointPos_RH_KFE];
ind_pos_des_HAA = [idx_command_jointPosition_LF_HAA...
                   idx_command_jointPosition_RF_HAA...
                   idx_command_jointPosition_LH_HAA...
                   idx_command_jointPosition_RH_HAA];
ind_pos_des_HFE = [idx_command_jointPosition_LF_HFE...
                   idx_command_jointPosition_RF_HFE...
                   idx_command_jointPosition_LH_HFE...
                   idx_command_jointPosition_RH_HFE];
ind_pos_des_KFE = [idx_command_jointPosition_LF_KFE...
                   idx_command_jointPosition_RF_KFE...
                   idx_command_jointPosition_LH_KFE...
                   idx_command_jointPosition_RH_KFE];

% Velocity indices
ind_vel_meas_HAA = [idx_state_jointVel_LF_HAA...
                    idx_state_jointVel_RF_HAA...
                    idx_state_jointVel_LH_HAA...
                    idx_state_jointVel_RH_HAA];
ind_vel_meas_HFE = [idx_state_jointVel_LF_HFE...
                    idx_state_jointVel_RF_HFE...
                    idx_state_jointVel_LH_HFE...
                    idx_state_jointVel_RH_HFE];
ind_vel_meas_KFE = [idx_state_jointVel_LF_KFE...
                    idx_state_jointVel_RF_KFE...
                    idx_state_jointVel_LH_KFE...
                    idx_state_jointVel_RH_KFE];
ind_vel_des_HAA = [idx_command_jointVelocity_LF_HAA...
                   idx_command_jointVelocity_RF_HAA...
                   idx_command_jointVelocity_LH_HAA...
                   idx_command_jointVelocity_RH_HAA];
ind_vel_des_HFE = [idx_command_jointVelocity_LF_HFE...
                   idx_command_jointVelocity_RF_HFE...
                   idx_command_jointVelocity_LH_HFE...
                   idx_command_jointVelocity_RH_HFE];
ind_vel_des_KFE = [idx_command_jointVelocity_LF_KFE...
                   idx_command_jointVelocity_RF_KFE...
                   idx_command_jointVelocity_LH_KFE...
                   idx_command_jointVelocity_RH_KFE];

% Torque indices
ind_tor_meas_HAA = [idx_state_jointTor_LF_HAA...
                    idx_state_jointTor_RF_HAA...
                    idx_state_jointTor_LH_HAA...
                    idx_state_jointTor_RH_HAA];
ind_tor_meas_HFE = [idx_state_jointTor_LF_HFE...
                    idx_state_jointTor_RF_HFE...
                    idx_state_jointTor_LH_HFE...
                    idx_state_jointTor_RH_HFE];
ind_tor_meas_KFE = [idx_state_jointTor_LF_KFE...
                    idx_state_jointTor_RF_KFE...
                    idx_state_jointTor_LH_KFE...
                    idx_state_jointTor_RH_KFE];
ind_tor_des_HAA = [idx_command_jointTorque_LF_HAA...
                   idx_command_jointTorque_RF_HAA...
                   idx_command_jointTorque_LH_HAA...
                   idx_command_jointTorque_RH_HAA];
ind_tor_des_HFE = [idx_command_jointTorque_LF_HFE...
                   idx_command_jointTorque_RF_HFE...
                   idx_command_jointTorque_LH_HFE...
                   idx_command_jointTorque_RH_HFE];
ind_tor_des_KFE = [idx_command_jointTorque_LF_KFE...
                   idx_command_jointTorque_RF_KFE...
                   idx_command_jointTorque_LH_KFE...
                   idx_command_jointTorque_RH_KFE];

% Data samples to be plotted
samples = startSample:endSample;

% Extract time samples
time = logElements(idx_state_jointTor_LF_HAA).time(samples);

% Helper lambda to make things less cluttered
data = @(idx) logElements(idx).data(samples);

%% Joint position plots

h_fig_vel = figure('name', 'Joint Positions', 'DefaultAxesFontSize', 10);
set(0, 'CurrentFigure', h_fig_vel)

for leg = enumLegs
    % Hip abduction/adduction
    subplot(3, 4, leg);
    hold on
    plot(time, rad2deg(data(ind_pos_meas_HAA(leg))), colorMeasured, 'LineWidth', lineWidth);
    plot(time, rad2deg(data(ind_pos_des_HAA(leg))), colorDesired, 'LineWidth', lineWidth);
    grid on
    xlabel('Time [s]')
    ylabel('Position [deg]')
    xlim([time(1),time(end)])
    title([nameLegs{leg} '\_HAA'])
    legend('meas', 'des')
    hold off
    % Hip flexion/extension
    subplot(3, 4, 4+leg);
    hold on
    plot(time, rad2deg(data(ind_pos_meas_HFE(leg))), colorMeasured, 'LineWidth', lineWidth);
    plot(time, rad2deg(data(ind_pos_des_HFE(leg))), colorDesired, 'LineWidth', lineWidth);
    grid on
    xlabel('Time [s]')
    ylabel('Postion [deg]')
    xlim([time(1),time(end)])
    title([nameLegs{leg} '\_HFE'])
    legend('meas', 'des')
    hold off
    % Knee flexion/extension
    subplot(3, 4, 8 + leg);
    hold on
    plot(time, rad2deg(data(ind_pos_meas_KFE(leg))), colorMeasured, 'LineWidth', lineWidth);
    plot(time, rad2deg(data(ind_pos_des_KFE(leg))), colorDesired, 'LineWidth', lineWidth);
    grid on
    xlabel('Time [s]')
    ylabel('Position [deg]')
    xlim([time(1),time(end)])
    title([nameLegs{leg} '\_KFE'])
    legend('meas', 'des')
    hold off
end

%% Joint velocity plots

h_fig_vel = figure('name', 'Joint Velocities', 'DefaultAxesFontSize', 10);
set(0, 'CurrentFigure', h_fig_vel)

for leg = enumLegs
    % Hip abduction/adduction
    subplot(3, 4, leg);
    hold on
    plot(time, data(ind_vel_meas_HAA(leg)), colorMeasured, 'LineWidth', lineWidth);
    plot(time, data(ind_vel_des_HAA(leg)), colorDesired, 'LineWidth', lineWidth);
    grid on
    xlabel('Time [s]')
    ylabel('Velocity [rad/s]')
    xlim([time(1),time(end)])
    title([nameLegs{leg} '\_HAA'])
    legend('meas', 'des')
    hold off
    % Hip flexion/extension
    subplot(3, 4, 4+leg);
    hold on
    plot(time, data(ind_vel_meas_HFE(leg)), colorMeasured, 'LineWidth', lineWidth);
    plot(time, data(ind_vel_des_HFE(leg)), colorDesired, 'LineWidth', lineWidth);
    grid on
    xlabel('Time [s]')
    ylabel('Velocity [rad/s]')
    xlim([time(1),time(end)])
    title([nameLegs{leg} '\_HFE'])
    legend('meas', 'des')
    hold off
    % Knee flexion/extension
    subplot(3, 4, 8 + leg);
    hold on
    plot(time, data(ind_vel_meas_KFE(leg)), colorMeasured, 'LineWidth', lineWidth);
    plot(time, data(ind_vel_des_KFE(leg)), colorDesired, 'LineWidth', lineWidth);
    grid on
    xlabel('Time [s]')
    ylabel('Velocity [rad/s]')
    xlim([time(1),time(end)])
    title([nameLegs{leg} '\_KFE'])
    legend('meas', 'des')
    hold off
end

%% Joint torque plots

h_fig_tor = figure('name', 'Joint Torques', 'DefaultAxesFontSize', 10);
set(0, 'CurrentFigure', h_fig_tor)

for leg = enumLegs
    % Hip abduction/adduction
    subplot(3, 4, leg);
    hold on
    plot(time, data(ind_tor_meas_HAA(leg)), colorMeasured, 'LineWidth', lineWidth);
    plot(time, data(ind_tor_des_HAA(leg)), colorDesired, 'LineWidth', lineWidth);
    grid on
    xlabel('Time [s]')
    ylabel('Torque [Nm]')
    xlim([time(1),time(end)])
    title([nameLegs{leg} '\_HAA'])
    legend('meas', 'des')
    hold off
    % Hip flexion/extension
    subplot(3, 4, 4+leg);
    hold on
    plot(time, data(ind_tor_meas_HFE(leg)), colorMeasured, 'LineWidth', lineWidth);
    plot(time, data(ind_tor_des_HFE(leg)), colorDesired, 'LineWidth', lineWidth);
    grid on
    xlabel('Time [s]')
    ylabel('Torque [Nm]')
    xlim([time(1),time(end)])
    title([nameLegs{leg} '\_HFE'])
    legend('meas', 'des')
    hold off
    % Knee flexion/extension
    subplot(3, 4, 8 + leg);
    hold on
    plot(time, data(ind_tor_meas_KFE(leg)), colorMeasured, 'LineWidth', lineWidth);
    plot(time, data(ind_tor_des_KFE(leg)), colorDesired, 'LineWidth', lineWidth);
    grid on
    xlabel('Time [s]')
    ylabel('Torque [Nm]')
    xlim([time(1),time(end)])
    title([nameLegs{leg} '\_KFE'])
    legend('meas', 'des')
    hold off
end
