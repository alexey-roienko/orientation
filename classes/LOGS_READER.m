classdef LOGS_READER < handle
    
    methods (Access = public, Static)
                
        %% Public method for reading the settings from the INI-config file
        function INI = fReadSettings(fileSettings)
            INI = LOGS_READER.fReadINI(fileSettings);
        end
        
        
        %% Public method for reading different logs
        function [output, varargout] = read(file, varargin)
            [~, name, ext] = fileparts(file);
            ini = varargin{1};
            
            %% xSens logs
            if strcmpi(ini.general.logsSource, 'xSens')
                if contains(name, 'xSens_') && strcmpi(ext, '.txt')
                    [output, varargout{1}, varargout{2}, varargout{3}, varargout{4}] = ...
                        LOGS_READER.fReadXSensImuTXT(file);
                else
                    error ('Read function cannot read the specified file. Please check your code.');
                end
            end
        end
        
    end
    
    
    %% PRIVATE methods
    methods (Access = private, Static)
        
        %% Method reads settings from INI-file using open-source library
        function output = fReadINI(file)
            output = INI('File', file).read();        
        end   
        
        
        %% Method for reading xSens log with Accel, Gyro, Magnet and Ground Truth Angles (from xSens board)        
        % INPUT:
        %    filename     - string, name of the log-file
        % OUTPUT:
        %    time         - sec., time samples of the log-file
        %    varargout{1} - double, matrix of [length(time) x 3], each row is [acc_x acc_y acc_z]
        %    varargout{2} - double, matrix of [length(time) x 3], each row is [gyro_x gyro_y gyro_z]
        %    varargout{3} - double, matrix of [length(time) x 3], each row is [magn_x magn_y magn_z]
        %    varargout{4} - double, matrix of [length(time) x 3], each row is [roll pitch yaw] (deg.)
        function [time, varargout] = fReadXSensImuTXT(filename)            
            %% Initialize variables
            delimiter = ',';
            if nargin<=2
                startRow = 14;
                endRow = inf;
            end

            %% Open the text file.
            fileID = fopen(filename,'r');

            %% Reads header line
            for i=1:startRow
                headerLine = fgetl(fileID);
            end
            headerCells = textscan(headerLine, '%s', 'Delimiter', delimiter, ....
                                   'TextType', 'string', 'EmptyValue', NaN, ...
                                   'ReturnOnError', false, 'EndOfLine', '\r\n');
            clear headerLine
            
            % Look for UTC_time
            cellNmbs = zeros(4*3+1, 1);
            for i=1:length(headerCells{1})
                if strcmpi(headerCells{1}(i), 'UTC_Nano')
                    cellNmbs(1) = i;
                    break;
                end
            end
            
            %Look for Accel_X etc
            stN = 1;
            if cellNmbs(1)~= 0,  stN = cellNmbs(1);  end
            for i=stN:length(headerCells{1})
                % Accel
                if strcmpi(headerCells{1}(i), 'Acc_X'),  cellNmbs(2) = i;  end
                if strcmpi(headerCells{1}(i), 'Acc_Y'),  cellNmbs(3) = i;  end
                if strcmpi(headerCells{1}(i), 'Acc_Z'),  cellNmbs(4) = i;  end
                % Gyro
                if strcmpi(headerCells{1}(i), 'Gyr_X'),  cellNmbs(5) = i;  end
                if strcmpi(headerCells{1}(i), 'Gyr_Y'),  cellNmbs(6) = i;  end
                if strcmpi(headerCells{1}(i), 'Gyr_Z'),  cellNmbs(7) = i;  end
                % Magnet
                if strcmpi(headerCells{1}(i), 'Mag_X'),  cellNmbs(8) = i;  end
                if strcmpi(headerCells{1}(i), 'Mag_Y'),  cellNmbs(9) = i;  end
                if strcmpi(headerCells{1}(i), 'Mag_Z'),  cellNmbs(10) = i;  end
                % Euler angles
                if strcmpi(headerCells{1}(i), 'Roll'),   cellNmbs(11) = i;  end
                if strcmpi(headerCells{1}(i), 'Pitch'),  cellNmbs(12) = i;  end
                if strcmpi(headerCells{1}(i), 'Yaw'),    cellNmbs(13) = i;  end
            end
            
            % Creating the 'formatSpec' using numbers found
            % If there are timesamples,...
            formatSpec = '';
            if cellNmbs(1) ~= 0
                for i = 1:cellNmbs(1)-1
                    formatSpec = [formatSpec '%s'];
                end
                % Add UTC_Nano values
                formatSpec = [formatSpec '%d'];
            end
            
            % Step to the Accel values
            for i = cellNmbs(1)+1:cellNmbs(2)-1
                formatSpec = [formatSpec '%s'];
            end
            
            % Add Accel values
            formatSpec = [formatSpec '%f%f%f'];

            % Look for Gyro values
            if (cellNmbs(4)+1 ~= cellNmbs(5))
                for i = cellNmbs(4)+1:cellNmbs(5)-1
                    formatSpec = [formatSpec '%s'];
                end
            end
            % Add Gyro values
            formatSpec = [formatSpec '%f%f%f'];

            % Look for Magnet values
            if (cellNmbs(7)+1 ~= cellNmbs(8))
                for i = cellNmbs(7)+1:cellNmbs(8)-1
                    formatSpec = [formatSpec '%s'];
                end
            end
            % Add Magnet values
            formatSpec = [formatSpec '%f%f%f'];

            % Look for Euler angle values
            if (cellNmbs(10)+1 ~= cellNmbs(11))
                for i = cellNmbs(10)+1:cellNmbs(11)-1
                    formatSpec = [formatSpec '%s'];
                end
            end
            % Add Euler angle values
            formatSpec = [formatSpec '%f%f%f'];

            % Add %s for remained values
            for i = cellNmbs(13)+1:length(headerCells{1})
                formatSpec = [formatSpec '%s'];
            end
            formatSpec = [formatSpec '%[^\n\r]'];

            
            %% Read columns of data according to the format.
            % This call is based on the structure of the file used to generate this
            % code. If an error occurs for a different file, try regenerating the code
            % from the Import Tool.
            dataArray = textscan(fileID, formatSpec, endRow-startRow, 'Delimiter', delimiter, ....
                                 'TextType', 'string', 'EmptyValue', NaN, 'HeaderLines', 0, ...
                                 'ReturnOnError', false, 'EndOfLine', '\r\n');
            for block=2:length(startRow)
                frewind(fileID);
                dataArrayBlock = textscan(fileID, formatSpec, endRow(block)-startRow(block)+1, ...
                    'Delimiter', delimiter, 'TextType', 'string', 'EmptyValue', NaN, ...
                    'HeaderLines', startRow(block)-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
                for col=1:length(dataArray)
                    dataArray{col} = [dataArray{col}; dataArrayBlock{col}];
                end
            end

            %% Close the text file.
            fclose(fileID);

            %% Create output variable
            % If there is no UTC_Nano field, just output zeros for the 'time'
            if cellNmbs(1) == 0
                time = zeros(length(dataArray{1, cellNmbs(2)}), 1);
            else
                time = double(dataArray{1, cellNmbs(1)}) / 1e+9;
                
                time(1,1) = 0;
                for t=2:length(time)
                    % if the there is a new second, add 1 sec. to the current time sample
                    if (time(t-1) > time(t))                        
                        time(t:end) = 1 + time(t:end);
                    end                    
                end
            end
            
            % Accel data
            varargout{1}(:,1) = dataArray{1,cellNmbs(2)};
            varargout{1}(:,2) = dataArray{1,cellNmbs(3)};
            varargout{1}(:,3) = dataArray{1,cellNmbs(4)};
            % Gyro data
            varargout{2}(:,1) = dataArray{1,cellNmbs(5)};
            varargout{2}(:,2) = dataArray{1,cellNmbs(6)};
            varargout{2}(:,3) = dataArray{1,cellNmbs(7)};
            % Magnet data
            varargout{3}(:,1) = dataArray{1,cellNmbs(8)};
            varargout{3}(:,2) = dataArray{1,cellNmbs(9)};
            varargout{3}(:,3) = dataArray{1,cellNmbs(10)};
            % Angles data
            varargout{4}(:,1) = dataArray{1,cellNmbs(11)};
            varargout{4}(:,2) = dataArray{1,cellNmbs(12)};
            varargout{4}(:,3) = dataArray{1,cellNmbs(13)};
        end

        
        
        %% Method for reading an image file
        function im = fReadImage(path, imName)
            imPath = [path '\' imName];
            if (~isnan(imPath))
                im  = imread(imPath);
            end
        end
        
    end
end