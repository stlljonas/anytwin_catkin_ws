clear all
close all
clc

%% WARNING

% The ATI sensor is badly calibrated.
% This data is not useful. Check out the manual calibration file.


%% ANYdrive Parameters

old_spring_stiffness = 162.3;


%% Measurements
% 1st column: Desired joint torque
% 2nd column: Measured joint torque (ANYdrive)
% 3rd column: Measured joint torque (ATI FT15581)

meas_AD305E17 = [
    0.0     0.03    0.0
    0.5     0.50    0.42
    1.0     0.98    0.86
    2.0     1.98    1.78
    5.0     4.94    4.42
    10.0    9.94    8.87
    15.0    14.85   13.23
    20.0    19.76   17.68
    25.0    24.74   22.04
    40.0    36.10   32.08
];

meas_AD119E20 = [
    0.0     0.02    0.0
    0.5     0.46    0.39
    1.0     0.98    0.86
    2.0     2.00    1.77
    3.0     2.96    2.63
    5.0     4.97    4.39
    7.0     6.94    6.13
    10.0    9.85    8.72
    15.0    14.93   13.25
    20.0    19.67   17.46
    25.0    24.80   21.96
    30.0    29.58   26.21
    35.0    32.85   29.10
    30.0    29.83   26.51
    25.0    24.62   21.95
    20.0    19.88   17.82
    15.0    14.80   13.31
    10.0    9.95    8.99
    7.0     6.95    6.30
    5.0     4.97    4.54
    3.0     2.98    2.74
    2.0     2.00    1.87
    1.0     1.03    0.98
    0.5     0.53    0.53
    0.0     0.03    0.08
    -0.5    -0.45   -0.41
    -1.0    -0.96   -0.84
    -2.0    -1.96   -1.62
    -3.0    -2.94   -2.47
    -5.0    -4.94   -4.22
    -7.0    -6.90   -5.97
    -10.0   -9.88   -8.69
    -15.0   -14.60  -13.10
    -20.0   -19.55  -17.82
    -25.0   -24.52  -22.50
    -30.0   -25.77  -23.71
];

% Add deflection
meas_AD119E20(:,4) = meas_AD119E20(:,2)/old_spring_stiffness;


%% Plotting

figure(1)
hold on
grid on
axis equal
title('Joint Torque')
xlabel('Desired [Nm]')
ylabel('Measured [Nm]')
plot(meas_AD119E20(:,1), meas_AD119E20(:,2), 'x-')
plot(meas_AD119E20(:,1), meas_AD119E20(:,3), 'x-')
plot(meas_AD119E20(:,1), meas_AD119E20(:,2)-meas_AD119E20(:,3), 'x-')

figure(2)
hold on
grid on
title('Joint Torque Errors')
xlabel('Desired Joint Torque [Nm]')
ylabel('Measured Joint Torque Relative Error [-]')
plot(meas_AD119E20(:,1), meas_AD119E20(:,2)./meas_AD119E20(:,3), 'x-')

figure(3)
hold on
grid on
title('Spring Stiffness')
xlabel('Deflection [rad]')
ylabel('Joint Torque [Nm]')
plot(meas_AD119E20(:,4), meas_AD119E20(:,3), 'x-')
fit = polyfit(meas_AD119E20(:,4), meas_AD119E20(:,3), 1);
fit_eval = polyval(fit, meas_AD119E20(:,4));
new_spring_stiffness = fit(1)
plot(meas_AD119E20(:,4), fit_eval)

figure(4)
hold on
grid on
title('Spring Stiffness')
xlabel('Joint Torque [Nm]')
ylabel('Spring Stiffness [Nm/rad]')
plot(meas_AD119E20(:,3), meas_AD119E20(:,3)./meas_AD119E20(:,4), 'x-')





