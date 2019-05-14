% This script reads logs composed by xSens or other loggers

close all;
clear, clc;

%% ===================== PATHES SETTINGS ========================= %
curDir = pwd;
cd('..\logs');
logsDir = pwd;

cd('..\classes');
classesDir = pwd;

cd('..\functions');
funcDir = pwd;

cd('..\libraries');
libDir = pwd;

addpath(logsDir, classesDir, funcDir, libDir);
cd(curDir)


%% ====================== READING CONFIG ======================== %
% Read 'config.ini'
INI = LOGS_READER.fReadSettings('config.ini');


%% ======================= READING LOGS ========================= %
% Samples time - in seconds
fname = [logsDir '\xSens_AGM_GT_testcase1.' INI.general.logFilesExt];
[time, accData, gyroData, magnData, anglesData] = LOGS_READER.read(fname, INI);

% Calculate averaged sample period of input data
TS = 0;
for t=2:length(time)
    TS = TS + time(t) - time(t-1);
end
TS = TS / (length(time)-1);
str = sprintf('Averaged sample period TS = %5.2f', TS);
disp(str);


%% ===================== PLOTTING SIGNALS ======================= %
if INI.visualize.rawSensorPlots
    
end







