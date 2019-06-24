classdef KF2D_ver1 < handle
    %KF2D_ver1 Kalman Filter implementation for estimation of two orientation
    % angles, Roll and Pitch, from the corresponding Gyro components and
    % angle estimates obtained from Accelerometer measurements
    
    properties (Access = public)        
        X = zeros(2,1);       % State vector, equals to two angles
        P = 10*eye(2);        % P covariance matrix;
        K = zeros(2,1);       % Kalman Gain 
        H = eye(2);           % H matrix
        Q = zeros(2);         % Process noise covariance matrix
        R = zeros(2);         % Measurement noise covariance matrix
        Z = zeros(2,1);       % Measurement vector, equals to angles from Accel
        F = eye(2);           % State transition matrix
        B = zeros(2);         % Control-input Model Matrix
        U = zeros(2,1);         
    end
    
    properties (Access = private)        
        TS = 0.01;             % Sampling interval, default - 10 msec. (from xSens 100 Hz logs)
        varQ = [0.0003 0.0003];% STD of Gyro noise component
        varR = [0.03 0.03];    % STD of Accel noise component
    end
    
    
    methods  (Access = public)        
        %% Constructor
        function obj = KF2D_ver1(varargin)
            for i = 1:2:nargin
                if     strcmpi(varargin{i}, 'PInit'), obj.P = varargin{i+1};
                elseif strcmpi(varargin{i}, 'varQ'), obj.setVarQ(varargin{i+1});
                elseif strcmpi(varargin{i}, 'varR'), obj.setVarR(varargin{i+1});
                elseif strcmpi(varargin{i}, 'TS'), obj.setTS(varargin{i+1});
                else
                    error('KF2D_ver1.m: Invalid argument!');
                end
            end          
        end
        
        
        %% Function for initialization the first sample of the filter output
        function obj = InitFilterState(obj, meas)
            obj.X = meas';
            obj.K = ones(2);
            obj.P = 10*eye(2); 
            obj.B(1,1) = obj.TS;  obj.B(2,2) = obj.TS;
            obj.Q = obj.CalcQ();
            obj.R = obj.CalcR();
        end
        
        
        %% Main function where new sensor measurements (G and A) are passed to the
        %  Kalman Filter
        function obj = Update(obj, acc, gyro)
            % Update KF State Update Equation            
            obj.U = gyro;
            obj.X = obj.F * obj.X + obj.B * obj.U;
            
            % Update KF Covariance Matrix Equation
            obj.P = obj.F * obj.P * obj.F' + obj.Q;
            
            % Update KF Gain coefficient
            obj.K = obj.P * obj.H' / (obj.H * obj.P * obj.H' + obj.R);          
            
            % Correction step
            obj.Z = acc;
            obj.X = obj.X + obj.K * (obj.Z - obj.H * obj.X);
            obj.P = (eye(size(obj.X,1)) - obj.K * obj.H) * obj.P;
        end
        
        
        
        %% Setter for sampling interval value
        function setTS(obj, newTS)
            obj.TS = newTS;
        end
        
        
        %% Getter for sampling interval value
        function output = getTS(obj)
            output = obj.TS;
        end
              
        %% Setter for STD of Gyro noise component
        function setVarQ(obj, newSQ)
            obj.varQ = newSQ;
        end
        
        
        %% Getter for STD of Gyro noise component
        function output = getVarQ(obj)
            output = obj.varQ;
        end
        
        
        %% Setter for STD of Accel noise component
        function setVarR(obj, newSR)
            obj.varR = newSR;            
        end
        
        
        %% Getter for STD of Accel noise component
        function output = getVarR(obj)
            output = obj.varR;
        end
               
    end    
    
    
    methods (Access = private)
        
        %% Function calculates the Q matrix for the KF Covariance Matrix Update Equation
        function output = CalcQ(obj)
            temp = zeros(2);
            temp(1,1) = obj.varQ(1);
            temp(2,2) = obj.varQ(2);
            output = temp;
        end      
        
        
        %% Function calculates the Q matrix for the KF Covariance Matrix Update Equation
        function output = CalcR(obj)
            temp = zeros(2);
            temp(1,1) = obj.varR(1);
            temp(2,2) = obj.varR(2);
            output = temp;
        end  
        
    end
end








