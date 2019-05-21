%% Function reads the defined log and output the data
function output = loadLogForKalman(logFName, varargin)

% Read data from the log
data = dlmread(logFName, ' ', 3, 0);

% If there is 'true' (or another flag) for the second arg, ...
if nargin == 2
    %Plot data
    figure(1)
    subplot(1, 2, 1);  plot(data(:,1), data(:,2:3));
    title('Measured coordinates of the rover vs. time')
    xlabel('Time [m]')
    ylabel('X and Y coordinates [m]')

    subplot(1, 2, 2);  plot(data(:,2), data(:,3));
    title('Measured position of the rover')
    xlabel('X coordinate [m]')
    ylabel('Y coordinate [m]')
end

output = data;