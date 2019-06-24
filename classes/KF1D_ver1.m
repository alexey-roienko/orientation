classdef KF1D_ver1 < handle
    %KF1_1D Kalman Filter implementation for estimation of one orientation
    %angle like Roll or Pitch
    
    properties (Access = public)        
        X = 0;                % State vector, equals to one angle
        P = 10;               % P covariance matrix;
        K;                    % Kalman Gain 
        H = 1;                % H matrix
        Q;                    % Process noise covariance matrix
        R;                    % Measurement noise covariance matrix
        Z = 0;                % Measurement vector, equals to quaternion
        F = 1;                % State transition matrix
        B = 0;                % Control-input Model Matrix
        U = 0;  
    end
    
    properties (Access = private)        
        TS = 0.01;            % Sampling interval, default - 10 msec. (from xSens 100 Hz logs)
        varQ = 0.1;         % STD of Gyro noise component, default - 1 rad./sec.
        varR = 5e-5;        % STD of Accel noise component, default - 5e-5 g
                      % Control vector
    end
    
    methods  (Access = public)        
        %% Constructor
        function obj = KF1D_ver1(varargin)
            for i = 1:2:nargin
                if     strcmpi(varargin{i}, 'PInit'), obj.P = varargin{i+1};
                elseif strcmpi(varargin{i}, 'varQ'), obj.setVarQ(varargin{i+1});
                elseif strcmpi(varargin{i}, 'varR'), obj.setVarR(varargin{i+1});
                elseif strcmpi(varargin{i}, 'TS'), obj.setTS(varargin{i+1});
                else
                    error('KF1D_ver1.m: Invalid argument!');
                end
            end            
        end
        
        
        %% Function for initialization the first sample of the filter output
        function obj = InitFilterState(obj, meas)
            obj.X = meas;
            obj.K = 1;
            obj.P = 10;
            obj.Q = obj.CalcQ();
            obj.R = obj.CalcR();
        end
        
        
        %% Main function where new sensor measurements (G and A) are passed to the
        %  Kalman Filter
        function obj = Update(obj, acc, gyro)
            % Update KF State Update Equation
            obj.B = obj.TS;
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
            output = obj.varQ;
        end      
        
        
        %% Function calculates the Q matrix for the KF Covariance Matrix Update Equation
        function output = CalcR(obj)
            output = obj.varR;
        end  
        
    end
end








