% This script allows to calibrate Magnetometer using
% the Matlab code provided by STM in DT0059:
%  "Ellipsoid or sphere fitting for sensor calibration"

close all;
clear, clc;

%% ===================== PATHES SETTINGS ========================= %
curDir = pwd;
cd('..\classes');
classesDir = pwd;

cd('..\functions');
funcDir = pwd;

cd('..\libraries');
libDir = pwd;

cd(curDir)
cd('..\logs\Magnet calibration logs');
logsDir = pwd;

addpath(classesDir, funcDir, libDir, logsDir);
cd(curDir)


%% ====================== READING CONFIG ======================== %
% Read 'config.ini'
INI = LOGS_READER.fReadSettings('..\config-files\config_stm.ini');


%% ============== SETTING LOGS NAMES AND TESTCASES ============== %
logFilename = 'xSens_magnet_calibSample.mat';


%% ======================= READING LOGS ========================= %
% Samples time - in seconds, magnData      - in Gauss
load([logsDir '\' logFilename]);
mainData = savingData;
clear savingData
TS = INI.general.samplePeriod;


%% ================ DEPICT 3D PLOT FOR ANALYSIS ================= %
if INI.visualization.plot3DRawMagnData
    figure('IntegerHandle', 'off', 'Name', 'Raw Magnetometer Data Distribution');
    plot3(mainData(:,1), mainData(:,2), mainData(:,3));
    xlim([-1 1]);   ylim([-1 1]);   zlim([-1 1]);
    grid on,  axis square,  hold on
    xlabel('X axis, Gauss'),  ylabel('Y axis, Gauss'),  zlabel('Z axis, Gauss')
end


%% ======================= PROCESSING ========================= %
[ofs, gain, rotM] = ellipsoid_fit(mainData);
X = mainData(:,1);   Y = mainData(:,2);  Z = mainData(:,3);

% optional refinement
[gain,rotM] = refine_3D_fit(gain, rotM);

% translate to (0,0,0)
XC = X - ofs(1);
YC = Y - ofs(2);
ZC = Z - ofs(3);

% rotate to XYZ axes
XYZC = [XC,YC,ZC] * rotM;

% reference radius
refr = 1;
% scale to sphere
XC = XYZC(:,1) / gain(1) * refr;
YC = XYZC(:,2) / gain(2) * refr;
ZC = XYZC(:,3) / gain(3) * refr; 


%% ================ DEPICT PLOTS FOR ANALYSIS ================= %
figure('IntegerHandle', 'off', 'Name', 'STM calibration of raw Magnetometer Data');
subplot(2,2,1); hold on; 
plot(XC,YC,'ro', X,Y,'kx');
xlabel('X'); ylabel('Y'); axis equal; grid on;

subplot(2,2,2); hold on;
plot(ZC,YC,'go', Z,Y,'kx');
xlabel('Z'); ylabel('Y'); axis equal; grid on;

subplot(2,2,3); hold on;
plot(XC,ZC,'bo', X,Z,'kx');
xlabel('X'); ylabel('Z'); axis equal; grid on;

subplot(2,2,4); hold on;
plot(0,0,'o', .5,.5, 'kx');
legend('Calibrated Data', 'Raw data');










