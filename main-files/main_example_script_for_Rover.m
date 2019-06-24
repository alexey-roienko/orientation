% This script solves the main task of tracking the object
close all;
clear, clc;


%% ===================== PATHES SETTINGS ========================= %
curDir = pwd;

cd('..\classes');
classesDir = pwd;
cd(curDir)

cd('..\logs\Kalman logs');
logsDir = pwd;
cd(curDir)

cd('..\functions');
funcDir = pwd;
cd(curDir)

addpath(classesDir, logsDir, funcDir);

% Names of log-files
logFNameCase1 = [logsDir '\' 'Rover coords (std4).txt'];
logFNameCase2 = [logsDir '\' 'Rover coords 2 (std4).txt'];


%% ====================== CONFIG VARIABLES ======================== %
% Test case 1 - Simple Kalman Filter; test case 2 - Extended version
testCase      = 2;
% Sample period of the measurements
samplePeriod  = 60;     % sec.
stdR_XY       = 0.5;    % meters
newStd_XY     = 4;      % meters
velValue      = 0.01;   % m/sec, constant velocity of the rover
accValue      = 5e-6;   % m/sec^2, constant acceleration value for the model
stdRng        = 100;    % STD of Range measurements noise, default - 1 m
stdTeta       = 1e-1;   % STD of Teta measurements noise, default - 1e-4 rad


%% ======================= READING LOGS ========================= %
if testCase == 1
    iData = loadLogForKalman(logFNameCase1, false);
elseif testCase == 2
    iData = loadLogForKalman(logFNameCase2, false);
end    
mTime = iData(:,1);


%% =============== Kalman Filter 1 APPROACH ===================== %
if testCase == 1
    disp('Appling Kalman Filter for the test case 1...');
    
    %% Create Kalman Filter object 
    KF1           = RoverKF1('TS', samplePeriod, 'sigmaR', newStd_XY, 'acc', accValue);
    % Initialization step
    KF1.InitFilterState([iData(1,2:3) velValue velValue]);
    % Initialize matrix which will contain the filter output
    stateVec      = zeros(length(mTime), 5);
    stateVec(1,:) = [mTime(1) KF1.X'];
    KalmanGain = zeros(length(mTime),2);
    residuals = zeros(length(mTime),2);
    
    %% Main cycle over all measurements
    for t = 2:length(mTime)
        % Update Kalman filter with new measurements
        KF1.Update(iData(t,2:3));
        % Get filtered position
        stateVec(t,:) = [mTime(t) KF1.X'];
        % Get Kalman Gain coefficients for coordinates
        KalmanGain(t,1) = KF1.K(1,1);
        KalmanGain(t,2) = KF1.K(2,2);
        % Get residuals for Chi^2 test
        residuals(t-1,:) = KF1.resid';
    end
    
%% =============== Kalman Filter 2 APPROACH ===================== %
elseif testCase == 2
    disp('Appling Kalman Filter for the test case 2...');
    
    %% Create Kalman Filter object 
    KF2           = RoverKF2('TS', samplePeriod, 'sigmaR', stdR_XY, 'acc', accValue, ...
                                  'sigmaRng', stdRng, 'sigmaTheta', stdTeta);
    % Initialization step
    KF2.InitFilterState([iData(1,2:3) velValue velValue]);
    % Initialize matrix which will contain the filter output
    stateVec      = zeros(length(mTime), 5);
    stateVec(1,:) = [mTime(1) KF2.X'];
    KalmanGain    = zeros(length(mTime),2);
    residuals     = zeros(length(mTime),4);
    
    %% Main cycle over all measurements
    for t = 2:length(mTime)
        % Update Kalman filter with new measurements
        KF2.Update(iData(t,2:5));
        % Get filtered position
        stateVec(t,:) = [mTime(t) KF2.X'];
        % Get Kalman Gain coefficients for coordinates
        KalmanGain(t,1) = KF2.K(1,1);
        KalmanGain(t,2) = KF2.K(2,2);
        % Get residuals for Chi^2 test
        residuals(t-1,:) = KF2.resid';
    end
    
    
    
end


%% ========================== PLOTS Section ===============================
%% Depict raw and filtered trajectories
figure('IntegerHandle', 'off', 'Name', 'Trajectories comparison', ...
    'units', 'normalized', 'outerposition', [0.05 0.05 .9 .9]);    
subplot(1,2,1);
hold on;
plot(iData(:,2), iData(:,3));
plot(stateVec(:,2), stateVec(:,3));
legend('Init trajectory', 'KF trajectory');
xlabel('X coordinate, m');
ylabel('Y coordinate, m');

%% Depict raw and filtered coordinates
subplot(1,2,2);
hold on;
N = length(iData(:,1));
plot(iData(:,1), iData(:,2), iData(:,1), iData(:,3));
plot(iData(:,1), stateVec(:,2), iData(:,1), stateVec(:,3));
legend('Raw coord X', 'Raw coord Y', 'Filtered X', 'Filtered Y');
xlabel('Time, sec.');
ylabel('Coordinate values, m');

%% Depict Kalman Gains for coordinates
figure('IntegerHandle', 'off', 'Name', 'Kalman Gains', ...
    'units', 'normalized', 'outerposition', [0.05 0.05 .5 .9]);
hold on;
plot(iData(:,1), KalmanGain(:,1));
plot(iData(:,1), KalmanGain(:,2));
legend('For X coord', 'For Y coord');
xlabel('Time, sec.');
ylabel('Kalman Gain Value');



%% For the EKF case it is possible to compare distance and angle values 
%  before and after filtering
if testCase == 2
    figure('IntegerHandle', 'off', 'Name', 'Range and Angle Comparison', ...
        'units', 'normalized', 'outerposition', [0.05 0.05 .9 .9]);
    subplot(1,2,1);
    hold on;
    filtRange = sqrt(stateVec(:,2).^2 + stateVec(:,3).^2);
    plot(iData(:,1), iData(:,4), iData(:,1), filtRange);
    legend('Raw Range', 'Filtered Range');
    xlabel('Time, sec.');
    ylabel('Range, m');
    
    subplot(1,2,2);
    hold on;
    filtTheta = atan2(stateVec(:,3), stateVec(:,2));
    plot(iData(:,1), iData(:,5), iData(:,1), filtTheta);
    legend('Raw Theta', 'Filtered Theta');
    xlabel('Time, sec.');
    ylabel('Angle, rad');
end




