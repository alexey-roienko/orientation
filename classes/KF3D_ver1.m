classdef KF3D_ver1 < handle
    %KF1INITIAL My first Kalman Filter implementation for orientation
    %   According to the article by Li Wang, Zheng Zhang and Ping Sun 
	%  "Quaternion-based Kalman Filter for AHRS using an Adaptive-step Gradient Descent Algorithm”
    
    properties (Access = public)        
        X = [1 0 0 0]';       % State vector, equals to quaternion
        P = eye(4);           % P covariance matrix; 
        F = eye(4);           % State transition matrix
        H = eye(4);           % H matrix
        Q;                    % Process noise covariance matrix
        R;                    % Measurement noise covariance matrix
        K;                    % Kalman Gain
        Z = [1 0 0 0]';       % Measurement vector, equals to quaternion
    end
    
    properties (Access = private)        
        TS = 0.01;            % Sampling interval, default - 20 msec.
        varQ = 1e-6;          % STD of Gyro noise component, default - 1 rad./sec.
        varR = 5e-5;          % STD of Accel noise component, default - 5e-5 g
    end
    
    methods  (Access = public)        
        %% Constructor
        function obj = KF3D_ver1(varargin)
            for i = 1:2:nargin
                if     strcmpi(varargin{i}, 'PInit'), obj.P = varargin{i+1};
                elseif strcmpi(varargin{i}, 'varQ'), obj.setVarQ(varargin{i+1});
                elseif strcmpi(varargin{i}, 'varR'), obj.setVarR(varargin{i+1});
                elseif strcmpi(varargin{i}, 'TS'), obj.setTS(varargin{i+1});
                else
                    error('KF3D_ver1.m: Invalid argument!');
                end
            end
        end
        
        
        %% Function for initialization the first sample of the filter output
        function obj = InitFilterState(obj)
            obj.K = ones(4); 
            obj.Q = obj.CalcQ();
            obj.R = obj.CalcR();
        end
        
        
        %% Main function where new sensor values (A, M, G) are passed to the Kalman Filter
        function obj = Update(obj, acc, gyro)
            % Update State Interpolation Equation
            obj.F = obj.UpdateF(gyro);            
            obj.X = obj.F * obj.X;      
            
            % Update KF Covariance Interpolation Equation            
            obj.P = obj.F * obj.P * obj.F' + obj.Q;
            
            % Update KF Gain coefficient
            obj.K = obj.P * obj.H' / (obj.H * obj.P * obj.H' + obj.R);
            
            % Gradient Descent method for computing the quaternion based on A
            obj.Z = obj.ApplyGDA(acc);
            
            % Correction step
            obj.X = obj.X + obj.K * (obj.Z - obj.H * obj.X);
            obj.P = (eye(4) - obj.K * obj.H) * obj.P;
            
            obj.X = obj.X / norm(obj.X);
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
        
        %% Function for quaternion Gradient Descent Algorithm based on A only
        function output = ApplyGDA(obj, acc)
            q = obj.X;
            % Normalize accelerometer measurement
            if (norm(acc) == 0), return; end
            acc = acc / norm(acc);

            % Apply GDA            
            objFunc = [2*(q(2)*q(4) - q(1)*q(3)) - acc(1)
                       2*(q(1)*q(2) + q(3)*q(4)) - acc(2)
                       2*(0.5 - q(2)^2 - q(3)^2) - acc(3)];
            J = [-2*q(3),	2*q(4),    -2*q(1),	2*q(2)
                 2*q(2),     2*q(1),     2*q(4),	2*q(3)
                 0,         -4*q(2),    -4*q(3),	0    ];
%             J = 2 * [-q(3),	 q(4),  -q(1),	q(2)    % expressions from the article
%                      q(2),   q(1),   q(4),	q(3)
%                      q(1),   -q(2),  -q(3),	q(4)];
            step = (J'*objFunc);
            step = step / norm(step);

            % Quaternion determination wrt the A only
            alphaCoef = 2;
            muCoef = alphaCoef * obj.TS;
            qRes = q - muCoef * step;
            output = qRes / norm(qRes);
        end
        
        
        %% Function updates the values of matrix F in State Interpolation Equation
        function newF = UpdateF(obj, gyro)
            g = [gyro(1) gyro(2) gyro(3)];
            % Calculate deltaTetaSquared variable
            dTeta2 = sum(g.^2) * obj.TS^2;
            
            % Fill in the Omega matrix
            Omega = zeros(4);
            Omega(1,2:4) = -g;
            Omega(2:4,1) = g';
            Omega(2,3:4) = [g(3) -g(2)];
            Omega(3,2) = -g(3);              Omega(3,4) = g(1);
            Omega(4,2:3) = [g(2) -g(1)];
            
            newF = eye(4)*(1 - dTeta2/8) + Omega * obj.TS / 2;
        end
        
       
        
        %% Function calculates the Q matrix for the KF Covariance Interpolation Equation
        function output = CalcQ(obj)            
            temp = eye(4);
            if (size(obj.varQ, 1) == 1) && (size(obj.varQ, 2) == 1)
                temp = temp * obj.varQ;
            else
                temp(1,1) = obj.varQ(1);
                temp(2,2) = obj.varQ(2);
                temp(3,3) = obj.varQ(3);
                temp(4,4) = obj.varQ(4);
            end
            output = temp;
        end      
        
        
        %% Function calculates the R matrix for the Covariance Update Equation
        function output = CalcR(obj)
            temp = eye(4);
            if (size(obj.varR, 1) == 1) && (size(obj.varR, 2) == 1)
                temp = temp * obj.varR;
            else
                temp(1,1) = obj.varR(1);
                temp(2,2) = obj.varR(2);
                temp(3,3) = obj.varR(3);
                temp(4,4) = obj.varR(4);
            end
            output = temp;
        end 

    end
end








