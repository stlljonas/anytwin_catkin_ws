clear all
close all
clc

%% ANYdrive Parameters

radPerTick = 2.0*pi/(2^17);


%% Measurements
% 1st column: Distance [m]
% 2nd column: Weight [kg]
% 3rd column: Expected joint torque offset (Pendulum) [Nm]
% 4th column: Expected joint torque, to be computed [Nm]
% 5th column: Measured joint torque [Nm]
% 6th column: Measured spring deflection, to be computed [rad]

% AD119E20
old_spring_stiffness = 145.4;
data = [
    -0.0          0.0         -1.127 0.0   1.09 0.0
     0.0          0.0          1.127 0.0  -0.84 0.0
    
    -0.029-0.3410 2.150       -1.127 0.0   8.21 0.0
    -0.029-0.2485 2.150       -1.127 0.0   6.42 0.0
    -0.029-0.1560 2.150       -1.127 0.0   4.70 0.0
    -0.029-0.0635 2.150       -1.127 0.0   2.93 0.0
     0.029+0.0635 2.150        1.127 0.0  -2.62 0.0
     0.029+0.1560 2.150        1.127 0.0  -4.40 0.0
     0.029+0.2485 2.150        1.127 0.0  -6.15 0.0
     0.029+0.3410 2.150        1.127 0.0  -7.89 0.0
     
    -0.029-0.3410 2.150+1.600 -1.127 0.0  13.60 0.0
    -0.029-0.2485 2.150+1.600 -1.127 0.0  10.53 0.0
    -0.029-0.1560 2.150+1.600 -1.127 0.0   7.43 0.0
    -0.029-0.0635 2.150+1.600 -1.127 0.0   4.26 0.0
     0.029+0.0635 2.150+1.600  1.127 0.0  -3.98 0.0
     0.029+0.1560 2.150+1.600  1.127 0.0  -7.03 0.0
     0.029+0.2485 2.150+1.600  1.127 0.0 -10.03 0.0
     0.029+0.3410 2.150+1.600  1.127 0.0 -12.99 0.0
];

% AD119E20 after ecoder offset calibration
old_spring_stiffness = 160.8;
data = [
    -0.0          0.0         -1.127 0.0   1.20 0.0
     0.0          0.0          1.127 0.0  -1.03 0.0
    
    -0.029-0.3410 2.150       -1.127 0.0   9.07 0.0
    -0.029-0.2485 2.150       -1.127 0.0   7.11 0.0
    -0.029-0.1560 2.150       -1.127 0.0   5.15 0.0
    -0.029-0.0635 2.150       -1.127 0.0   3.24 0.0
     0.029+0.0635 2.150        1.127 0.0  -3.06 0.0
     0.029+0.1560 2.150        1.127 0.0  -4.98 0.0
     0.029+0.2485 2.150        1.127 0.0  -6.92 0.0
     0.029+0.3410 2.150        1.127 0.0  -8.83 0.0
     
    -0.029-0.3410 2.150+1.600 -1.127 0.0  15.15 0.0
    -0.029-0.2485 2.150+1.600 -1.127 0.0  11.63 0.0
    -0.029-0.1560 2.150+1.600 -1.127 0.0   8.12 0.0
    -0.029-0.0635 2.150+1.600 -1.127 0.0   4.70 0.0
     0.029+0.0635 2.150+1.600  1.127 0.0  -4.46 0.0
     0.029+0.1560 2.150+1.600  1.127 0.0  -7.83 0.0
     0.029+0.2485 2.150+1.600  1.127 0.0 -11.18 0.0
     0.029+0.3410 2.150+1.600  1.127 0.0 -14.52 0.0
];

% Compute expected joint torque
data(:,4) = -9.81*data(:,1).*data(:,2)-data(:,3);

% Compute spring deflection
% data(:,5) = data(:,5) - mean(data(:,5)); % Compensate for symmetric joint torque offset.
data(:,6) = data(:,5)/old_spring_stiffness;

% Sort rows
data = sortrows(data,6);


%% Plotting

figure(1)
hold on
grid on
title('Spring Stiffness')
xlabel('Deflection [rad]')
ylabel('Joint Torque [Nm]')
plot(data(:,6), data(:,5), 'x-')
plot(data(:,6), data(:,4), 'x-')
fit = polyfit(data(:,6), data(:,4), 1)
fit_eval = polyval(fit, data(:,6));
new_spring_stiffness = fit(1)
plot(data(:,6), fit_eval)
legend('Measured', 'Expected', 'Linear Fit')

figure(2)
hold on
grid on
title('Spring Stiffness')
xlabel('Deflection [rad]')
ylabel('Spring Stiffness [Nm/rad]')
plot(data(:,6), data(:,5)./data(:,6), 'x-')
plot(data(:,6), data(:,4)./data(:,6), 'x-')
legend('Measured', 'Expected')






