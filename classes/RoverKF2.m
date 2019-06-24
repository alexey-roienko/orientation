classdef RoverKF2 < handle
    % RoverKF2
    % The first Kalman Filter for Rover task - case 1
    
    properties (Access = public)        
        X = zeros(4,1);       % State vector (x,y, vx, vy)
        P = eye(4);           % P covariance matrix; 
        F = eye(4);           % State Transition matrix
        H = eye(4);           % H matrix
        Q;                    % Process noise covariance matrix
        R = eye(4);           % Measurement noise covariance matrix
        K;                    % Kalman Gain
        Z;                    % Measurement vector; (x, y, r, teta)
        resid = zeros(4,1);   % matrix for Hi^2 test
    end
    
    properties (Access = private)   
        %% Default values of filter parameters for the case if user doesn't
        %  set them up
        TS     = 60;          % Sampling interval, default - 60 sec.
        acc    = 5e-6;        % Rover Acceleration, m/sec.^2
        sigmaR = 0.5;         % STD of XY measurements noise, default - 0.5 m
        sigmaRng = 1;         % STD of Range measurements noise, default - 1 m
        sigmaTheta = 1e-4;     % STD of Teta measurements noise, default - 1e-4 rad
        beacCenter = [0 0];   % Coordinates of the Beacon position
    end
    
    methods  (Access = public)        
        %% Constructor
        function obj = RoverKF2(varargin)
            for i = 1:2:nargin
                if     strcmpi(varargin{i}, 'PInit'), obj.P = varargin{i+1};
                elseif strcmpi(varargin{i}, 'sigmaR'), obj.setSigmaR(varargin{i+1});
                elseif strcmpi(varargin{i}, 'sigmaRng'), obj.setSigmaRng(varargin{i+1});
                elseif strcmpi(varargin{i}, 'sigmaTheta'), obj.setSigmaTheta(varargin{i+1});
                elseif strcmpi(varargin{i}, 'TS'), obj.setTS(varargin{i+1});
                elseif strcmpi(varargin{i}, 'acc'), obj.setAcc(varargin{i+1});
                else
                    error('RoverKF2.m: Invalid argument!');
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
            
            % Update H matrix
            obj.H = obj.CalcH(obj.X(1,1), obj.X(2,1));
            
            % Update Gain coefficient
            numer = obj.P * obj.H';
            denom = obj.H * obj.P * obj.H' + obj.R;
            obj.K = numer / denom;
            
            % Add measurements
            obj.Z = meas';
            predict = obj.H * obj.X;
            obj.resid = (obj.Z - predict);
            
            % Correction step
            add = obj.K * obj.resid;
            obj.X = obj.X + add;
            
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
        
        %% Setter for STD of Range measurement noise component
        function setSigmaRng(obj, newSRng)
            obj.sigmaRng = newSRng;
        end
        
        %% Setter for STD of Teta measurement noise component
        function setSigmaTheta(obj, newSTheta)
            obj.sigmaTheta = newSTheta;
        end
        
        %% Setter for Acceleration value
        function setAcc(obj, newAcc)
            obj.acc = newAcc;
        end        
    end
    
    
    
    methods (Access = private)
        
        %% Function calculates the H matrix for the EKF Update Equation
        function output = CalcH(obj, x, y)
            tempH = zeros(4);
            tempH(1,1) = 1;               tempH(2,2) = 1;
            
            % Initialization of H for XY
%             x = obj.X(1);                 y = obj.X(2);            
            bC_x = obj.beacCenter(1);     bC_y = obj.beacCenter(2);
            
            % Row for "r" parameter
            % dr/dx
            tempH(3,1) = (x - bC_x) / sqrt((x - bC_x)^2 + (y - bC_y)^2);
            % dr/dy
            tempH(3,2) = (y - bC_y) / sqrt((x - bC_x)^2 + (y - bC_y)^2);
            % dr/dvx = 0 , dr/dvy = 0 
            
            % Row for "teta" parameter
            % dteta/dx
            tempH(4,1) = -(y - bC_y) / ((x - bC_x)^2 + (y - bC_y)^2);
            % dteta/dy
            tempH(4,2) =  (x - bC_x) / ((x - bC_x)^2 + (y - bC_y)^2);
            % dteta/dvx = 0 , dteta/dvy = 0 
            
            output = tempH;
        end      
        
        
        %% Function calculates the Q matrix for the KF Covariance Matrix Update Equation
        function output = CalcQ(obj)
            tempQ = eye(4) * obj.acc * obj.TS;
            tempQ(1,1) = tempQ(1,1) * 0.5 * obj.TS;
            tempQ(2,2) = tempQ(2,2) * 0.5 * obj.TS;
            output = tempQ;
        end      
        
        
        %% Function calculates the R matrix for the KF Covariance Matrix Update Equation
        function output = CalcR(obj)
            tempR = eye(4);
            % Set Variance of the XY measurement components
            tempR(1,1) = tempR(1,1) * obj.sigmaR^2;
            tempR(2,2) = tempR(2,2) * obj.sigmaR^2;
            % Set Variance of the Range measurement components
            tempR(3,3) = tempR(3,3) * obj.sigmaRng^2;
            % Set Variance of the Teta measurement components
            tempR(4,4) = tempR(4,4) * obj.sigmaTheta^2;
            output = tempR;
        end      
       
    end
end








