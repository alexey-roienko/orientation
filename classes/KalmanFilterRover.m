classdef KalmanFilterRover < handle
    % KalmanFilterRover
    % The Kalman Filter for filtering the Rover's coordinates on Mars
    
    % Matrices initialization by Default values
    properties (Access = public)        
        X = ???;              % State vector
        P = ???;              % P covariance matrix; 
        F = ???;              % State Transition matrix
        H = ???;              % H matrix
        Q;                    % Process noise covariance matrix
        R;                    % Measurement noise covariance matrix
        K;                    % Kalman Gain
        Z;                    % Measurement vector
        innov = ???;          % Matrix of Innovation terms
    end
    
    properties (Access = private)   
        %% Default values of filter parameters for the case if user doesn't
        %  set them up
        TS     = ???;         % Sampling interval
        acc    = ???;         % Rover Acceleration typical magnitude
        sigmaR = ???;         % STD of Measurements noise, for matrix R
    end
    
    methods  (Access = public)        
        %% Class constructor - describes how user should initialize the class
        %  What arguments they should substitute while calling the Class Constructor
        function obj = KalmanFilterRover(varargin)
            for i = 1:2:nargin
                if     strcmpi(varargin{i}, 'PInit'), obj.P = varargin{i+1};
                elseif strcmpi(varargin{i}, 'sigmaR'), obj.setSigmaR(varargin{i+1});
                elseif strcmpi(varargin{i}, 'TS'), obj.setTS(varargin{i+1});
                elseif strcmpi(varargin{i}, 'acc'), obj.setAcc(varargin{i+1});
                else
                    error('KalmanFilterRover.m: Invalid argument!');
                end
            end
            % Define matrix F
            obj.F = ???;
            % Define matrix R
            obj.R = obj.CalcR();
            % Define matrix Q
            obj.Q = obj.CalcQ();
        end
        
        
        %% Filter State Initialization with the first known position of the Rover
        function obj = InitFilterState(obj, meas)
            obj.X = meas';
        end
        
        
        %% Main function - new measurements are passed to the Filter using vector argument 'meas'
        function obj = Update(obj, meas)
            % Update State Equation - Equation 1
            obj.X =
            
            % Update Covariance Matrix Equation - Equation 2
            obj.P =    
            
            % Update Gain coefficient - Equation 3
            obj.K =
            
            % Add measurements - Equation 4.1 (innovation)
            obj.Z = 
            obj.innov = 
            
            % Correction step - Equation 4.2 (State Update Equation)
            obj.X =
            
            % Update Covariance Matrix - Equation 5
            obj.P =
            
        end       
        
    end
    
    
    
    methods (Access = private)
        
        %% Function calculates the Q matrix for the KF Covariance Extrapolation Equation (#2)
        function output = CalcQ(obj)            
            output = ???;
        end      
        
        
        %% Function calculates the R matrix for the KF Kalman Gain Equation (#3)
        function output = CalcR(obj)
            output = ???;
        end      
       
    end
end








