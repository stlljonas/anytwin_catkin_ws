%% Initizalize

close all; clear; clc;
format long


%% Load

duration = '5.000000';
motorVelocities = ...
    {'-480.000000'
     '-420.000000'
     '-360.000000'
     '-300.000000'
     '-240.000000'
     '-180.000000'
     '-120.000000'
%      '-50.000000'
%      '-30.000000'
%      '-20.000000'
%      '-10.000000'
%      '-5.000000'
%      '-3.000000'
%      '-2.000000'
%      '-1.000000'
%      '1.000000'
%      '2.000000'
%      '3.000000'
%      '5.000000'
%      '10.000000'
%      '20.000000'
%      '30.000000'
%      '50.000000'
     '120.000000'
     '180.000000'
     '240.000000'
     '300.000000'
     '360.000000'
     '420.000000'
     '480.000000'};

for i = 1:length(motorVelocities)
    file = fopen(char(strcat('~/friction_estimation_',duration,'_',motorVelocities(i),'.txt')), 'r');
    dataC = textscan(file, '%f %f %f %f %f %f', 'Delimiter', ',');
    data{i} = cell2mat(dataC);
    
    average(i,:) = [mean(data{i}(:,1)), mean(data{i}(:,2)), mean(data{i}(:,3)), mean(data{i}(:,4)), mean(data{i}(:,5))];
end

% Prepare feed forward friction estimation.
break_away_friction_Nm = 0.299276;
break_away_friction_band_RPM = 20.0;
viscous_friction_coeff_neg_Nm_per_RPM = 0.00235119;
viscous_friction_coeff_pos_Nm_per_RPM = 0.00241778;
joint_velocity_RPM = -110.0:0.01:110.0;
for i = 1:length(joint_velocity_RPM)
    if (joint_velocity_RPM(i) < 0)
        friction_estimation(i) = ...
           break_away_friction_Nm * smooth_sign(joint_velocity_RPM(i), break_away_friction_band_RPM) + ...
           viscous_friction_coeff_neg_Nm_per_RPM * joint_velocity_RPM(i);
    else
        friction_estimation(i) = ...
           break_away_friction_Nm * smooth_sign(joint_velocity_RPM(i), break_away_friction_band_RPM) + ...
           viscous_friction_coeff_pos_Nm_per_RPM * joint_velocity_RPM(i);
    end
end

%% Fit

% ticks = 131072;
% omega = 2.0*pi/ticks;
% 
% fit = @(param,joint_position) param(1) + param(2)*(sin(omega*joint_position + param(3))) + param(4)*(sin(2*omega*joint_position + param(5)));
% fcn = @(param) sum((fit(param,data(:,2)) - data(:,3)).^2);
% 
% param0 = [0 0 0 0 0]';
% param0(1) = mean(data(:,3))
% param1 = fminsearch(fcn, param0)


%% Plot

hold on
grid on
title('Velocity -> Friction')
xlabel('Joint Velocity [rad/s]')
ylabel('Joint Torque [Nm]')
plot(average(:,2), average(:,4), '-x')
plot(average(:,2), average(:,5), '-x')
plot(joint_velocity_RPM/60*2*pi, friction_estimation)
for i = 1:length(data)
    plot(data{i}(:,2), data{i}(:,4), '.')
end
legend('Average measurement','Fit from calibration routine','Fit with Matlab params (update these manually!)')


