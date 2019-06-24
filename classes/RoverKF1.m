classdef RoverKF1 < handle
    % RoverKF1
    % The first Kalman Filter for Rover task - case 1
    
    properties (Access = public)        
        X = zeros(4,1);       % State vector (x,y, vx, vy)
        P = eye(4);           % P covariance matrix; 
        F = eye(4);           % State Transition matrix
        H = eye(2,4);         % H matrix
        Q;                    % Process noise covariance matrix
        R;                    % Measurement noise covariance matrix
        K;                    % Kalman Gain
        Z;                    % Measurement vector; (x, y)
        resid = zeros(2,1);   % matrix for Hi^2 test
    end
    
    properties (Access = private)   
        %% Default values of filter parameters for the case if user doesn't
        %  set them up
        TS     = 60;          % Sampling interval, default - 60 sec.
        acc    = 5e-6;        % Rover Acceleration, m/sec.^2
        sigmaR = 0.5;         % STD of Measurements noise, default - 0.5 m
    end
    
    methods  (Access = public)        
        %% Constructor
        function obj = RoverKF1(varargin)
            for i = 1:2:nargin
                if     strcmpi(varargin{i}, 'PInit'), obj.P = varargin{i+1};
                elseif strcmpi(varargin{i}, 'sigmaR'), obj.setSigmaR(varargin{i+1});
                elseif strcmpi(varargin{i}, 'TS'), obj.setTS(varargin{i+1});
                elseif strcmpi(varargin{i}, 'acc'), obj.setAcc(varargin{i+1});
                else
                    error('RoverKF1.m: Invalid argument!');
                end
            end
            % Define A matrix
            obj.F(1,3) = obj.TS;    obj.F(2,4) = obj.TS;
            % Define R matrix
            obj.R = obj.CalcR();
            % Define Q matrix
            obj.Q = obj.CalcQ();
        end
        
        
        %% Filter Initialization with the first known position of the rover
        function obj = InitFilterState(obj, meas)
            obj.X = meas';
        end
        
        
        %% Main function where new measurements are passed to the Filter
        function obj = Update(obj, meas)
            % Update State Equation
            obj.X = obj.F * obj.X;
            
            % Update Covariance Matrix Equation
            obj.P = obj.F * obj.P * obj.F' + obj.Q;
            
            % Update Gain coefficient  
            numer = obj.P * obj.H';
            denom = obj.H * obj.P * obj.H' + obj.R;
            obj.K = numer / denom;
            
            % Add measurements
            obj.Z = meas';
            obj.resid = (obj.Z - obj.H * obj.X);
            
            % Correction step
            obj.X = obj.X + obj.K * obj.resid;
            
            % Update Covariance Matrix
            obj.P = (eye(4) - obj.K * obj.H) * obj.P;
        end
        
        
        
        %% Setter for sampling interval value
        function setTS(obj, newTS)
            obj.TS = newTS;
        end
              
        %% Setter for STD of Model noise component
        function setSigmaQ(obj, newSQ)
            obj.sigmaQ = newSQ;
        end

        %% Setter for STD of Measurement noise component
        function setSigmaR(obj, newSR)
            obj.sigmaR = newSR;
        end
        
        %% Setter for Acceleration value
        function setAcc(obj, newAcc)
            obj.acc = newAcc;
        end        
    end
    
    
    
    methods (Access = private)
        
        %% Function calculates the Q matrix for the KF Covariance Matrix Update Equation
        function output = CalcQ(obj)
            tempQ = eye(4) * obj.acc * obj.TS;
            tempQ(1,1) = tempQ(1,1) * 0.5 * obj.TS;
            tempQ(2,2) = tempQ(2,2) * 0.5 * obj.TS;
            output = tempQ;
        end      
        
        
        %% Function calculates the R matrix for the KF Covariance Matrix Update Equation
        function output = CalcR(obj)
            output = eye(2) * obj.sigmaR^2;
        end      
       
    end
end








