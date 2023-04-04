%% Initizalize

close all; clear; clc;
format long


%% Load

file = fopen('~/gravity_compensation_100.000000_3_negative.txt','r');
dataNegC = textscan(file, '%f %f %f %f %f', 'Delimiter', ',');
dataNeg = cell2mat(dataNegC);

file = fopen('~/gravity_compensation_100.000000_3_positive.txt','r');
dataPosC = textscan(file, '%f %f %f %f %f', 'Delimiter', ',');
dataPos = cell2mat(dataPosC);

data = [dataNeg; dataPos];


%% Fit

% ticks = 131072;
% omega = 2.0*pi/ticks;
% 
% fit = @(param,joint_position) param(1) + param(2)*(sin(omega*joint_position + param(3)));
% fcn = @(param) sum((fit(param,data(:,1)) - data(:,2)).^2);
% 
% param0 = [0 0 0]';
% param0(1) = mean(data(:,2))
% param1 = fminsearch(fcn, param0)


%% Plot

subplot(2,1,1)
hold on
grid on
title('Gravity Compensation')
plot(dataNeg(:,1), dataNeg(:,2), 'b', ...
     dataPos(:,1), dataPos(:,2), 'g', ...
     dataNeg(:,1), dataNeg(:,3), 'r', ...
     dataPos(:,1), dataPos(:,3), 'r')

subplot(2,1,2)
hold on
grid on
title('Temperature')
plot(1:size(dataNeg,1), dataNeg(:,4), 'b', ...
     1:size(dataPos,1), dataPos(:,4), 'g')


%% Debug Plot

% plot(dataNeg(:,2), dataNeg(:,3), 'b', ...
%      dataPos(:,2), dataPos(:,3), 'b', ...
%      data(:,2), data(:,4), 'g', ...
%      dataNeg(:,2), fit(param1, dataNeg(:,2)), 'r', ...
%      dataPos(:,2), fit(param1, dataPos(:,2)), 'r')

