
clear all, clc, close all

% number of the data file (silo_20170207_15-25-55_02495 => fileNumber = 2495)
fileNumber = 2624; 

% name of folder where the data file is stored (without trailing /)
fileDir = '~/logs';        

% Setup paths to Matlab scripts of signal_logger
setupPath

% Get filename
fname = getFilenameFromNumber(fileNumber, fileDir);

% Read data represented as a uint64
[logElem] = loadLogFile(fname);

% Generate index variables
genIndexVariables(logElem);

% Increase precision in data cursor and show index of data point
set(0,'defaultFigureCreateFcn',@(s,e)datacursorextra(s))

%% Setup plots data
plotWidth  = 330;
plotHeight = 200;
plotX0 = 10;
plotY0 = 10;
lineWidth = 1;

%% Plot data
plotJointData;

%% Plot loco data
plotLimbStateAndStrategy;
plotLegData;
plotFootData;
plotLocoTorso;
plotLocoVMCAndCFD;
plotLocoTiming;
plotLocoCFD;

% plotTorsoPositionData;
% plotTorsoOrientationData;

% plotManipulability;
% plotHipToFootData;




% plotLocoWholeBody;
% plotLocoTerrainModelFreePlane;
% plotImuData
