%% Initizalize

close all; clear; clc;
format long

anydrive_model = 2; % 1,2


%% Load

file = fopen('~/encoder_offset_100.000000_3_negative.txt','r');
dataNegC = textscan(file, '%f %f %f %f %f %f', 'Delimiter', ',');
dataNeg = cell2mat(dataNegC);

file = fopen('~/encoder_offset_100.000000_3_positive.txt','r');
dataPosC = textscan(file, '%f %f %f %f %f %f', 'Delimiter', ',');
dataPos = cell2mat(dataPosC);

data = [dataNeg; dataPos];


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


%% Cut data in periods

cut = false;
lastI = 1;
j = 1;
for i = 2:size(dataNeg, 1)
    if anydrive_model == 1
        cut = dataNeg(i,2) > dataNeg(i-1,2);
    elseif anydrive_model == 2
        cut = dataNeg(i,2) > dataNeg(i-1,2);
    end
    if cut
        dataNegCut{j} = dataNeg(lastI:i-1,:);
        lastI = i;
        j = j+1;
    end
end
dataNegCut{j} = dataNeg(lastI:end,:);

cut = false;
lastI = 1;
j = 1;
for i = 2:size(dataPos, 1)
    if anydrive_model == 1
        cut = dataPos(i,2) < dataPos(i-1,2);
    elseif anydrive_model == 2
        cut = dataPos(i,2) < dataPos(i-1,2);
    end
    if cut
        dataPosCut{j} = dataPos(lastI:i-1,:);
        lastI = i;
        j = j+1;
    end
end
dataPosCut{j} = dataPos(lastI:end,:);


%% Plot

subplot(3,1,1)
hold on
grid on
title('Encoder offset')
for i = 1:size(dataNegCut,2)
    plot(dataNegCut{i}(:,2), dataNegCut{i}(:,4), 'b', ...
         dataNegCut{i}(:,2), dataNegCut{i}(:,5), 'r')
end
for i = 1:size(dataPosCut,2)
    plot(dataPosCut{i}(:,2), dataPosCut{i}(:,4), 'g', ...
         dataPosCut{i}(:,2), dataPosCut{i}(:,5), 'r')
end
 
subplot(3,1,2)
hold on
grid on
title('Encoder offset residual')
for i = 1:size(dataNegCut,2)
    plot(dataNegCut{i}(:,2), dataNegCut{i}(:,6), 'r')
end
for i = 1:size(dataPosCut,2)
    plot(dataPosCut{i}(:,2), dataPosCut{i}(:,6), 'r')
end

subplot(3,1,3)
hold on
grid on
title('Temperature')
plot(1:size(dataNeg,1), dataNeg(:,3), 'b', ...
     1:size(dataPos,1), dataPos(:,3), 'g')


%% Debug Plot

% plot(dataNeg(:,2), dataNeg(:,3), 'b', ...
%      dataPos(:,2), dataPos(:,3), 'b', ...
%      data(:,2), data(:,4), 'g', ...
%      dataNeg(:,2), fit(param1, dataNeg(:,2)), 'r', ...
%      dataPos(:,2), fit(param1, dataPos(:,2)), 'r')

